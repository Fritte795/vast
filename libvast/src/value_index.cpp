//    _   _____   __________
//   | | / / _ | / __/_  __/     Visibility
//   | |/ / __ |_\ \  / /          Across
//   |___/_/ |_/___/ /_/       Space and Time
//
// SPDX-FileCopyrightText: (c) 2016 The VAST Contributors
// SPDX-License-Identifier: BSD-3-Clause

#include "vast/value_index.hpp"

#include "vast/chunk.hpp"
#include "vast/data.hpp"
#include "vast/detail/legacy_deserialize.hpp"
#include "vast/fbs/value_index.hpp"
#include "vast/legacy_type.hpp"
#include "vast/value_index_factory.hpp"

#include <caf/binary_serializer.hpp>
#include <caf/deserializer.hpp>
#include <caf/sec.hpp>

namespace vast {

value_index::value_index(vast::type t, caf::settings opts)
  : type_{std::move(t)}, opts_{std::move(opts)} {
  // nop
}

value_index::~value_index() {
  // nop
}

caf::expected<void> value_index::append(data_view x) {
  return append(x, offset());
}

caf::expected<void> value_index::append(data_view x, id pos) {
  auto off = offset();
  if (pos < off)
    // Can only append at the end
    return caf::make_error(ec::unspecified, pos, '<', off);
  if (caf::holds_alternative<caf::none_t>(x)) {
    none_.append_bits(false, pos - none_.size());
    none_.append_bit(true);
    return caf::no_error;
  }
  // TODO: let append_impl return caf::error
  if (!append_impl(x, pos))
    return caf::make_error(ec::unspecified, "append_impl");
  mask_.append_bits(false, pos - mask_.size());
  mask_.append_bit(true);
  return caf::no_error;
}

caf::expected<ids>
value_index::lookup(relational_operator op, data_view x) const {
  // When x is nil, we can answer the query right here.
  if (caf::holds_alternative<caf::none_t>(x)) {
    if (!(op == relational_operator::equal
          || op == relational_operator::not_equal))
      return caf::make_error(ec::unsupported_operator, op);
    auto is_equal = op == relational_operator::equal;
    auto result = is_equal ? none_ : ~none_;
    if (result.size() < mask_.size())
      result.append_bits(!is_equal, mask_.size() - result.size());
    return result;
  }
  // If x is not nil, we dispatch to the concrete implementation.
  auto result = lookup_impl(op, x);
  if (!result)
    return result;
  // The result can only have mass (i.e., 1-bits) where actual IDs exist.
  *result &= mask_;
  // Because the value index implementations never see nil values, they need
  // to be handled here. If we have a predicate with a non-nil RHS and `!=` as
  // operator, then we need to add the nils to the result, because the
  // expression `nil != RHS` is true when RHS is not nil.
  auto is_negation = op == relational_operator::not_equal;
  if (is_negation)
    *result |= none_;
  // Finally, the concrete result may be too short, e.g., when the last values
  // have been nils. In this case we need to fill it up. For any operator other
  // than !=, the result of comparing with nil is undefined.
  if (result->size() < offset())
    result->append_bits(is_negation, offset() - result->size());
  return std::move(*result);
}

size_t value_index::memusage() const {
  return mask_.memusage() + none_.memusage() + memusage_impl();
}

value_index::size_type value_index::offset() const {
  return std::max(none_.size(), mask_.size());
}

const type& value_index::type() const {
  return type_;
}

const caf::settings& value_index::options() const {
  return opts_;
}

caf::error value_index::serialize(caf::serializer& sink) const {
  return sink(mask_, none_);
}

caf::error value_index::deserialize(caf::deserializer& source) {
  return source(mask_, none_);
}

bool value_index::deserialize(detail::legacy_deserializer& source) {
  return source(mask_, none_);
}

flatbuffers::Offset<fbs::ValueIndex>
pack(flatbuffers::FlatBufferBuilder& builder, const value_index_ptr& value) {
  const auto mask_offset = pack(builder, value->mask_);
  const auto none_offset = pack(builder, value->none_);
  const auto type_bytes = as_bytes(value->type_);
  const auto type_offset = fbs::CreateTypeBuffer(
    builder,
    builder.CreateVector(reinterpret_cast<const uint8_t*>(type_bytes.data()),
                         type_bytes.size()));
  auto options_data = data{};
  const auto convert_ok = convert(value->opts_, options_data);
  VAST_ASSERT(convert_ok);
  const auto options_offset = pack(builder, options_data);
  const auto base_offset = fbs::value_index::detail::CreateValueIndexBase(
    builder, mask_offset, none_offset, type_offset, options_offset);
  return value->pack_impl(builder, base_offset);
}

caf::error unpack(const fbs::ValueIndex& from, value_index_ptr& to) {
  auto do_unpack
    = [&](const fbs::value_index::detail::ValueIndexBase& base) -> caf::error {
    // Create initial value index by unpacking type and options,
    const auto type = vast::type{chunk::copy(*base.type()->buffer())};
    auto options_data = data{};
    if (auto err = unpack(*base.options(), options_data))
      return err;
    auto options = caf::settings{};
    if (const auto* options_record = caf::get_if<record>(&options_data))
      if (auto err = convert(*options_record, options))
        return err;
    to = factory<value_index>::make(type, options);
    if (!to)
      return caf::make_error(ec::format_error,
                             fmt::format("failed to create value index for "
                                         "type {} with options {}",
                                         type, options_data));
    if (auto err = unpack(*base.mask(), to->mask_))
      return err;
    if (auto err = unpack(*base.none(), to->none_))
      return err;
    return to->unpack_impl(from);
  };
  switch (from.value_index_type()) {
    case fbs::value_index::ValueIndex::NONE:
      return caf::make_error(ec::format_error, "invalid value index type");
    case fbs::value_index::ValueIndex::arithmetic:
      return do_unpack(*from.value_index_as_arithmetic()->base());
    case fbs::value_index::ValueIndex::address:
      return do_unpack(*from.value_index_as_address()->base());
    case fbs::value_index::ValueIndex::enumeration:
      return do_unpack(*from.value_index_as_enumeration()->base());
    case fbs::value_index::ValueIndex::hash:
      return do_unpack(*from.value_index_as_hash()->base());
    case fbs::value_index::ValueIndex::list:
      return do_unpack(*from.value_index_as_list()->base());
    case fbs::value_index::ValueIndex::subnet:
      return do_unpack(*from.value_index_as_subnet()->base());
    case fbs::value_index::ValueIndex::string:
      return do_unpack(*from.value_index_as_string()->base());
  }
  return caf::make_error(ec::format_error, "unexpected value index type");
}

const ewah_bitmap& value_index::mask() const {
  return mask_;
}

const ewah_bitmap& value_index::none() const {
  return none_;
}

caf::error inspect(caf::serializer& sink, const value_index& x) {
  return x.serialize(sink);
}

caf::error inspect(caf::deserializer& source, value_index& x) {
  return x.deserialize(source);
}

bool inspect(detail::legacy_deserializer& source, value_index& x) {
  return x.deserialize(source);
}

caf::error inspect(caf::serializer& sink, const value_index_ptr& x) {
  auto lt = legacy_type{};
  if (x == nullptr)
    return sink(lt);
  lt = x->type().to_legacy_type();
  return caf::error::eval(
    [&] {
      return sink(lt, x->options());
    },
    [&] {
      return x->serialize(sink);
    });
}

caf::error inspect(caf::deserializer& source, value_index_ptr& x) {
  legacy_type lt;
  if (auto err = source(lt))
    return err;
  if (caf::holds_alternative<legacy_none_type>(lt)) {
    x = nullptr;
    return caf::none;
  }
  caf::settings opts;
  if (auto err = source(opts))
    return err;
  x = factory<value_index>::make(type::from_legacy_type(lt), std::move(opts));
  if (x == nullptr)
    return caf::make_error(ec::unspecified, "failed to construct value index");
  return x->deserialize(source);
}

bool inspect(detail::legacy_deserializer& source, value_index_ptr& x) {
  legacy_type t;
  if (!source(t))
    return false;
  if (caf::holds_alternative<legacy_none_type>(t)) {
    x = nullptr;
    return true;
  }
  caf::settings opts;
  if (!source(opts))
    return false;
  auto new_ptr
    = factory<value_index>::make(type::from_legacy_type(t), std::move(opts));
  if (!new_ptr || !new_ptr->deserialize(source))
    return false;
  std::swap(x, new_ptr);
  return true;
}

vast::chunk_ptr chunkify(const value_index_ptr& idx) {
  std::vector<char> buf;
  caf::binary_serializer sink{nullptr, buf};
  auto error = sink(idx);
  if (error)
    return nullptr;
  return chunk::make(std::move(buf));
}

} // namespace vast
