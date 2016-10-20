#ifndef VAST_CODER_HPP
#define VAST_CODER_HPP

#include <algorithm>
#include <array>
#include <limits>
#include <vector>
#include <type_traits>

#include <caf/meta/load_callback.hpp>
#include <caf/meta/save_callback.hpp>

#include "vast/operator.hpp"
#include "vast/detail/assert.hpp"
#include "vast/detail/operators.hpp"

namespace vast {

/// The concept class for bitmap coders. A coder offers two basic primitives:
/// encoding and decoding of (one or more) values into bitmap storage. The
/// decoding step is a function of specific relational operator, as supported
/// by the coder. A coder is an append-only data structure. Users have the
/// ability to control the position/offset where to begin encoding of values.
template <class Bitmap>
struct coder {
  using bitmap_type = Bitmap;
  using size_type = typename Bitmap::size_type;
  using value_type = size_t;

  /// Encodes a single values multiple times.
  /// @tparam An unsigned integral type.
  /// @param x The value to encode.
  /// @param n The number of time to add *x*.
  /// @param skip The number of entries to skip before encoding.
  /// @pre `Bitmap::max_size - size() >= n + skip`
  void encode(value_type x, size_type n = 1, size_type skip = 0);

  /// Decodes a value under a relational operator.
  /// @param x The value to decode.
  /// @param op The relation operator under which to decode *x*.
  /// @returns The bitmap for lookup *? op x* where *?* represents the value in
  ///          the coder.
  Bitmap decode(relational_operator op, value_type x) const;

  /// Appends another coder to this instance.
  /// @param other The coder to append.
  /// @pre `size() + other.size() < Bitmap::max_size`
  void append(coder const& other);

  /// Retrieves the number entries in the coder, i.e., the number of rows.
  /// @returns The size of the coder measured in number of entries.
  size_type size() const;

  /// Retrieves the coder-specific bitmap storage.
  auto& storage() const;
};

/// A coder that wraps a single bitmap (and can thus only stores 2 values).
template <class Bitmap>
class singleton_coder : detail::equality_comparable<singleton_coder<Bitmap>> {
public:
  using bitmap_type = Bitmap;
  using size_type = typename Bitmap::size_type;
  using value_type = bool;

  void encode(value_type x, size_type n = 1, size_type skip = 0) {
    VAST_ASSERT(Bitmap::max_size - size() >= n + skip);
    bitmap_.append_bits(x, n + skip);
  }

  Bitmap decode(relational_operator op, value_type x) const {
    auto result = bitmap_;
    if ((x && op == equal) || (!x && op == not_equal))
      return result;
    result.flip();
    return result;
  }

  void append(singleton_coder const& other) {
    bitmap_.append(other.bitmap_);
  }

  size_type size() const {
    return bitmap_.size();
  }

  Bitmap const& storage() const {
    return bitmap_;
  }

  friend bool operator==(singleton_coder const& x, singleton_coder const& y) {
    return x.bitmap_ == y.bitmap_;
  }

  template <class Inspector>
  friend auto inspect(Inspector&f, singleton_coder& sc) {
    return f(sc.bitmap_);
  }

private:
  Bitmap bitmap_;
};

template <class Bitmap>
class vector_coder : detail::equality_comparable<vector_coder<Bitmap>> {
public:
  using bitmap_type = Bitmap;
  using size_type = typename Bitmap::size_type;
  using value_type = size_t;

  vector_coder() : size_{0} {
  }

  vector_coder(size_t n) : size_{0}, bitmaps_(n) {
  }

  void append(vector_coder const& other) {
    append(other, false);
  }

  auto size() const {
    return size_;
  }

  auto& storage() const {
    return bitmaps_;
  }

  friend bool operator==(vector_coder const& x, vector_coder const& y) {
    return x.size_ == y.size_ && x.bitmaps_ == y.bitmaps_;
  }

  template <class Inspector>
  friend auto inspect(Inspector& f, vector_coder& ec) {
    return f(ec.size_, ec.bitmaps_);
  }

protected:
  void append(vector_coder const& other, bool bit) {
    VAST_ASSERT(bitmaps_.size() == other.bitmaps_.size());
    for (auto i = 0u; i < bitmaps_.size(); ++i) {
      bitmaps_[i].append_bits(bit, this->size() - bitmaps_[i].size());
      bitmaps_[i].append(other.bitmaps_[i]);
    }
    size_ += other.size_;
  }

  size_type size_;
  std::vector<Bitmap> bitmaps_;
};

/// Encodes each value in its own bitmap.
template <class Bitmap>
class equality_coder : public vector_coder<Bitmap> {
public:
  using typename vector_coder<Bitmap>::value_type;
  using typename vector_coder<Bitmap>::size_type;
  using vector_coder<Bitmap>::vector_coder;

  void encode(value_type x, size_type n = 1, size_type skip = 0) {
    VAST_ASSERT(Bitmap::max_size - this->size_ >= n + skip);
    VAST_ASSERT(x < this->bitmaps_.size());
    auto& bm = this->bitmaps_[x];
    bm.append_bits(false, this->size_ + skip - bm.size());
    bm.append_bits(true, n);
    this->size_ += skip + n;
  }

  Bitmap decode(relational_operator op, value_type x) const {
    VAST_ASSERT(op == less || op == less_equal || op == equal || op == not_equal
                || op == greater_equal || op == greater);
    VAST_ASSERT(x < this->bitmaps_.size());
    switch (op) {
      default:
        return {this->size_, false};
      case less: {
        if (x == 0)
          return {this->size_, false};
        auto f = this->bitmaps_.begin();
        auto result = nary_or(f, f + x);
        result.append_bits(false, this->size_ - result.size());
        return result;
      }
      case less_equal: {
        auto f = this->bitmaps_.begin();
        auto result = nary_or(f, f + x + 1);
        result.append_bits(false, this->size_ - result.size());
        return result;
      }
      case equal:
      case not_equal: {
        auto result = this->bitmaps_[x];
        result.append_bits(false, this->size_ - result.size());
        if (op == not_equal)
          result.flip();
        return result;
      }
      case greater_equal: {
        auto result = nary_or(this->bitmaps_.begin() + x, this->bitmaps_.end());
        result.append_bits(false, this->size_ - result.size());
        return result;
      }
      case greater: {
        if (x >= this->bitmaps_.size() - 1)
          return {this->size_, false};
        auto f = this->bitmaps_.begin();
        auto l = this->bitmaps_.end();
        auto result = nary_or(f + x + 1, l);
        result.append_bits(false, this->size_ - result.size());
        return result;
      }
    }
  }
};

/// Encodes a value according to an inequalty. Given a value *x* and an index
/// *i* in *[0,N)*, all bits are 0 for i < x and 1 for i >= x.
template <class Bitmap>
class range_coder : public vector_coder<Bitmap> {
public:
  using typename vector_coder<Bitmap>::value_type;
  using typename vector_coder<Bitmap>::size_type;
  using vector_coder<Bitmap>::vector_coder;

  void encode(value_type x, size_type n = 1, size_type skip = 0) {
    VAST_ASSERT(Bitmap::max_size - this->size_ >= n + skip);
    VAST_ASSERT(x < this->bitmaps_.size() + 1);
    // Lazy append: we only add 0s until we hit index i of value x. The
    // remaining bitmaps are always 1, by definition of the range coding
    // property i >= x for all i in [0,N).
    // TODO: This requires adapating RangeEval-Opt to perform "lazy extension"
    // on complement, which is why it is still commented.
    // for (auto i = 0u; i < x; ++i) {
    //  auto& bm = this->bitmaps_[i];
    //  bm.append_bits(true, this->size_ + skip - bm.size());
    //  bm.append_bits(false, n);
    // }
    for (auto i = 0u; i < this->bitmaps_.size(); ++i) {
      auto& bm = this->bitmaps_[i];
      bm.append_bits(true, this->size_ + skip - bm.size());
      bm.append_bits(i >= x, n);
    }
    this->size_ += n + skip;
  }

  Bitmap decode(relational_operator op, value_type x) const {
    VAST_ASSERT(op == less || op == less_equal || op == equal || op == not_equal
                || op == greater_equal || op == greater);
    VAST_ASSERT(x < this->bitmaps_.size() + 1);
    switch (op) {
      default:
        return {this->size_, false};
      case less: {
        auto result = this->bitmaps_[x > 0 ? x - 1 : 0];
        result.append_bits(true, this->size_ - result.size());
        return result;
      }
      case less_equal: {
        auto result = this->bitmaps_[x];
        result.append_bits(true, this->size_ - result.size());
        return result;
      }
      case equal:
      case not_equal: {
        auto result = this->bitmaps_[x];
        if (x > 0) {
          auto prev = ~this->bitmaps_[x - 1];
          VAST_ASSERT(prev.size() >= result.size());
          result.append_bits(true, prev.size() - result.size());
          result &= prev;
        }
        result.append_bits(false, this->size_ - result.size());
        if (op == not_equal)
          result.flip();
        return result;
      }
      case greater: {
        auto result = ~this->bitmaps_[x];
        result.append_bits(false, this->size_ - result.size());
        return result;
      }
      case greater_equal: {
        auto result = ~this->bitmaps_[x > 0 ? x - 1 : 0];
        result.append_bits(false, this->size_ - result.size());
        return result;
      }
    }
  }

  void append(range_coder const& other) {
    vector_coder<Bitmap>::append(other, true);
  }
};

/// Maintains one bitmap per *bit* of the value to encode.
/// For example, adding the value 4 appends a 1 to the bitmap for 2^2 and a
/// 0 to to all other bitmaps.
template <class Bitmap>
class bitslice_coder : public vector_coder<Bitmap> {
public:
  using typename vector_coder<Bitmap>::value_type;
  using typename vector_coder<Bitmap>::size_type;
  using vector_coder<Bitmap>::vector_coder;

  void encode(value_type x, size_type n = 1, size_type skip = 0) {
    VAST_ASSERT(Bitmap::max_size - this->size_ >= n + skip);
    for (auto i = 0u; i < this->bitmaps_.size(); ++i) {
      auto& bm = this->bitmaps_[i];
      bm.append_bits(false, this->size_ + skip - bm.size());
      bm.append_bits(((x >> i) & 1) == 0, n);
    }
    this->size_ += n + skip;
  }

  // RangeEval-Opt for the special case with uniform base 2.
  Bitmap decode(relational_operator op, value_type x) const {
    switch (op) {
      default:
        break;
      case less:
      case less_equal:
      case greater:
      case greater_equal: {
        if (x == std::numeric_limits<value_type>::min()) {
          if (op == less)
            return {this->size_, false};
          else if (op == greater_equal)
            return {this->size_, true};
        } else if (op == less || op == greater_equal) {
          --x;
        }
        auto result = x & 1 ? Bitmap{this->size_, true} : this->bitmaps_[0];
        for (auto i = 1u; i < this->bitmaps_.size(); ++i)
          if ((x >> i) & 1)
            result |= this->bitmaps_[i];
          else
            result &= this->bitmaps_[i];
        if (op == greater || op == greater_equal || op == not_equal)
          result.flip();
        return result;
      }
      case equal:
      case not_equal: {
        auto result = Bitmap{this->size_, true};
        for (auto i = 0u; i < this->bitmaps_.size(); ++i) {
          auto& bm = this->bitmaps_[i];
          result &= (((x >> i) & 1) ? ~bm : bm);
        }
        if (op == not_equal)
          result.flip();
        return result;
      }
      case in:
      case not_in: {
        if (x == 0)
          break;
        x = ~x;
        auto result = Bitmap{this->size_, false};
        for (auto i = 0u; i < this->bitmaps_.size(); ++i)
          if (((x >> i) & 1) == 0)
            result |= this->bitmaps_[i];
        if (op == in)
          result.flip();
        return result;
      }
    }
    return {this->size_, false};
  }
};

template <class T>
struct is_singleton_coder : std::false_type {};

template <class Bitmap>
struct is_singleton_coder<singleton_coder<Bitmap>> : std::true_type {};

template <class T>
struct is_equality_coder : std::false_type {};

template <class Bitmap>
struct is_equality_coder<equality_coder<Bitmap>> : std::true_type {};

template <class T>
struct is_range_coder : std::false_type {};

template <class Bitmap>
struct is_range_coder<range_coder<Bitmap>> : std::true_type {};

template <class T>
struct is_bitslice_coder : std::false_type {};

template <class Bitmap>
struct is_bitslice_coder<bitslice_coder<Bitmap>> : std::true_type {};

/// A multi-component (or multi-level) coder expresses values as a linear
/// combination according to a base vector. The literature refers to this
/// represenation as *attribute value decomposition*.
template <class Coder>
class multi_level_coder
  : detail::equality_comparable<multi_level_coder<Coder>> {
public:
  using coder_type = Coder;
  using bitmap_type = typename coder_type::bitmap_type;
  using size_type = typename coder_type::size_type;
  using value_type = typename coder_type::value_type;

  // Default-constructs an unusable multi-level coder. This function exists
  // only to make this type serializable.
  multi_level_coder() = default;

  /// Constructs a multi-level coder from a given base.
  /// @param b The base to initialize this coder with.
  multi_level_coder(base b)
    : base_{std::move(b)},
      xs_(base_.size()),
      coders_(base_.size()) {
    VAST_ASSERT(base_.well_defined());
    init(coders_); // dispatch on coder_type
    VAST_ASSERT(coders_.size() == base_.size());
  }

  void encode(value_type x, size_type n = 1, size_type skip = 0) {
    base_.decompose(x, xs_);
    for (auto i = 0u; i < base_.size(); ++i)
      coders_[i].encode(xs_[i], n, skip);
  }

  auto decode(relational_operator op, value_type x) const {
    return decode(coders_, op, x); // dispatch on coder_type
  }

  void append(multi_level_coder const& other) {
    VAST_ASSERT(coders_.size() == other.coders_.size());
    for (auto i = 0u; i < coders_.size(); ++i)
      coders_[i].append(other.coders_[i]);
  }

  size_type size() const {
    return coders_[0].size();
  }

  auto& storage() const {
    return coders_;
  }

  friend bool operator==(multi_level_coder const& x,
                         multi_level_coder const& y) {
    return x.base_ == y.base_ && x.coders_ == y.coders_;
  }

  template <class Inspector>
  friend auto inspect(Inspector& f, multi_level_coder& mlc) {
    return f(mlc.base_, mlc.xs_, mlc.coders_);
  }

private:
  // TODO
  // We could further optimze the number of bitmaps per coder: any base b
  // requires only b-1 bitmaps because one can obtain any bitmap through
  // conjunction/disjunction of the others. While this decreases space
  // requirements by a factor of 1/b, it increases query time by b-1.

  void init(std::vector<singleton_coder<bitmap_type>>&) {
    // Nothing to for singleton coders.
  }

  void init(std::vector<range_coder<bitmap_type>>& coders) {
    // For range coders it suffices to use b-1 bitmaps because the last
    // bitmap always consists of all 1s and is hence superfluous.
    for (auto i = 0u; i < base_.size(); ++i)
      coders[i] = range_coder<bitmap_type>{base_[i] - 1};
  }

  template <class C>
  void init(std::vector<C>& coders) {
    // All other multi-bitmap coders use one bitmap per unique value.
    for (auto i = 0u; i < base_.size(); ++i)
      coders[i] = C{base_[i]};
  }

  // Range-Eval-Opt
  auto decode(std::vector<range_coder<bitmap_type>> const& coders,
              relational_operator op, value_type x) const {
    VAST_ASSERT(!(op == in || op == not_in));
    // All coders must have the same number of elements.
    VAST_ASSERT(std::all_of(coders.begin(), coders.end(),
                            [=](auto c) { return c.size() == size(); }));
    // Check boundaries first.
    if (x == 0) {
      if (op == less) // A < min => false
        return bitmap_type{size(), false};
      else if (op == greater_equal) // A >= min => true
        return bitmap_type{size(), true};
    } else if (op == less || op == greater_equal) {
      --x;
    }
    base_.decompose(x, xs_);
    bitmap_type result{size(), true};
    auto bitmaps = [&](auto i) -> auto& { return coders[i].storage(); };
    switch (op) {
      default:
        return bitmap_type{size(), false};
      case less:
      case less_equal:
      case greater:
      case greater_equal: {
        if (xs_[0] < base_[0] - 1) // && bitmap != all_ones
          result = bitmaps(0)[xs_[0]];
        for (auto i = 1u; i < base_.size(); ++i) {
          if (xs_[i] != base_[i] - 1) // && bitmap != all_ones
            result &= bitmaps(i)[xs_[i]];
          if (xs_[i] != 0) // && bitmap != all_ones
            result |= bitmaps(i)[xs_[i] - 1];
        }
      } break;
      case equal:
      case not_equal: {
        for (auto i = 0u; i < base_.size(); ++i) {
          if (xs_[i] == 0) // && bitmap != all_ones
            result &= bitmaps(i)[0];
          else if (xs_[i] == base_[i] - 1)
            result &= ~bitmaps(i)[base_[i] - 2];
          else
            result &= bitmaps(i)[xs_[i]] ^ bitmaps(i)[xs_[i] - 1];
        }
      } break;
    }
    if (op == greater || op == greater_equal || op == not_equal)
      result.flip();
    return result;
  }

  // If we don't have a range_coder, we only support simple equality queries at
  // this point.
  template <class C>
  auto decode(std::vector<C> const& coders, relational_operator op,
              value_type x) const
  -> std::enable_if_t<
    is_equality_coder<C>{} || is_bitslice_coder<C>{},
    bitmap_type
  > {
    VAST_ASSERT(op == equal || op == not_equal);
    base_.decompose(x, xs_);
    auto result = coders[0].decode(equal, xs_[0]);
    for (auto i = 1u; i < base_.size(); ++i)
      result &= coders[i].decode(equal, xs_[i]);
    if (op == not_equal || op == not_in)
      result.flip();
    return result;
  }

  base base_;
  mutable std::vector<value_type> xs_;
  std::vector<coder_type> coders_;
};

template <class T>
struct is_multi_level_coder : std::false_type {};

template <class C>
struct is_multi_level_coder<multi_level_coder<C>> : std::true_type {};

} // namespace vast

#endif
