{ stdenv
, lib
, vast-source
, nix-gitignore
, cmake
, cmake-format
, pkgconfig
, git
, pandoc
, caf
, libpcap
, arrow-cpp
, fast_float
, flatbuffers
, spdlog
, libyamlcpp
, simdjson
, robin-map
, jemalloc
, libunwind
, xxHash
, zeek-broker
, python3
, jq
, tcpdump
, utillinux
, dpkg
, versionOverride ? null
, withPlugins ? []
, extraCmakeFlags ? []
, disableTests ? true
, buildType ? "Release"
}:
let
  inherit (stdenv.hostPlatform) isStatic;
  isCross = stdenv.buildPlatform != stdenv.hostPlatform;

  py3 = (python3.withPackages(ps: with ps; [
    coloredlogs
    jsondiff
    pyarrow
    pyyaml
    schema
  ]));

  src = vast-source;

  version = if (versionOverride != null) then versionOverride else "v2.3.0-rc2";
in

stdenv.mkDerivation rec {
  inherit src version;
  pname = "vast";

  outputs = ["out"] ++ lib.optionals isStatic ["debian"];

  preConfigure = ''
    substituteInPlace plugins/pcap/cmake/FindPCAP.cmake \
      --replace /bin/sh "${stdenv.shell}" \
      --replace nm "''${NM}"
  '';

  nativeBuildInputs = [ cmake cmake-format dpkg ];
  propagatedNativeBuildInputs = [ pkgconfig pandoc ];
  buildInputs = [
    fast_float
    jemalloc
    libpcap
    libunwind
    libyamlcpp
    robin-map
    simdjson
    spdlog
    zeek-broker
  ];
  propagatedBuildInputs = [
    arrow-cpp
    caf
    flatbuffers
    xxHash
  ];

  cmakeFlags = [
    "-DCMAKE_BUILD_TYPE:STRING=${buildType}"
    "-DCMAKE_FIND_PACKAGE_PREFER_CONFIG=ON"
    "-DVAST_VERSION_TAG=${version}"
    "-DVAST_ENABLE_RELOCATABLE_INSTALLATIONS=${if isStatic then "ON" else "OFF"}"
    "-DVAST_ENABLE_BACKTRACE=ON"
    "-DVAST_ENABLE_JEMALLOC=ON"
    "-DVAST_ENABLE_LSVAST=ON"
    "-DVAST_ENABLE_VAST_REGENERATE=OFF"
    "-DCAF_ROOT_DIR=${caf}"
  ] ++ lib.optionals (buildType == "CI") [
    "-DVAST_ENABLE_ASSERTIONS=ON"
  ] ++ lib.optionals isStatic [
    "-DBUILD_SHARED_LIBS:BOOL=OFF"
    "-DVAST_ENABLE_STATIC_EXECUTABLE:BOOL=ON"
    "-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON"
  ] ++ lib.optional disableTests "-DVAST_ENABLE_UNIT_TESTS=OFF"
    # Plugin Section
    ++ lib.optional (withPlugins != [])
       "-DVAST_PLUGINS=${lib.concatStringsSep ";" withPlugins}"
    ++ extraCmakeFlags;

  # The executable is run to generate the man page as part of the build phase.
  # libvast.{dyld,so} is put into the libvast subdir if relocatable installation
  # is off, which is the case here.
  preBuild = lib.optionalString (!isStatic) ''
    export LD_LIBRARY_PATH="$PWD/libvast''${LD_LIBRARY_PATH:+:}$LD_LIBRARY_PATH"
    export DYLD_LIBRARY_PATH="$PWD/libvast''${DYLD_LIBRARY_PATH:+:}$DYLD_LIBRARY_PATH"
  '';

  hardeningDisable = lib.optional isStatic "pic";

  doCheck = false;
  checkTarget = "test";

  dontStrip = true;

  doInstallCheck = true;
  installCheckInputs = [ py3 jq tcpdump ];
  # TODO: Investigate why the disk monitor test fails in the build sandbox.
  installCheckPhase = ''
    python ../vast/integration/integration.py \
      --app ${placeholder "out"}/bin/vast \
      --disable "Disk Monitor"
  '';

  postInstall = "" + lib.optionalString isStatic ''
    rm CMakeCache.txt
    cmake ''${cmakeDir:-.} $cmakeFlags "''${cmakeFlagsArray[@]}" \
      -UCMAKE_INSTALL_BINDIR \
      -UCMAKE_INSTALL_SBINDIR \
      -UCMAKE_INSTALL_INCLUDEDIR \
      -UCMAKE_INSTALL_OLDINCLUDEDIR \
      -UCMAKE_INSTALL_MANDIR \
      -UCMAKE_INSTALL_INFODIR \
      -UCMAKE_INSTALL_DOCDIR \
      -UCMAKE_INSTALL_LIBDIR \
      -UCMAKE_INSTALL_LIBEXECDIR \
      -UCMAKE_INSTALL_LOCALEDIR \
      -DCMAKE_INSTALL_PREFIX="/usr" ..
    cmake --build . --target package -j $NIX_BUILD_CORES
    install -m 644 -Dt $debian *.deb
  '';

  meta = with lib; {
    description = "Visibility Across Space and Time";
    homepage = http://vast.io/;
    license = licenses.bsd3;
    platforms = platforms.unix;
    maintainers = with maintainers; [ tobim ];
  };
}
