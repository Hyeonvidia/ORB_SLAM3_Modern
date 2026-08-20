cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM3_Modern VERSION 0.1.0 LANGUAGES C CXX)

# =============================================================================
# Modern CMake replacement for the upstream flat build (Phase 1).
# Behavior parity with the golden baseline (benchmark/golden/BASELINE.md):
#   -O3, NO -march=native anywhere, BOOST_BIND_GLOBAL_PLACEHOLDERS.
# R1 (2026-08-20): our targets moved to C++20 on the gcc-13 / Ubuntu 24.04
# toolchain (docs/REFACTOR_PLAN.md); ATE re-validated by the 6-mode band check.
# R2 (2026-08-20): Eigen comes from the third_party/eigen submodule (5.0.1)
# via cmake/FindEigen3.cmake — the single Eigen for core, g2o, and the
# image-baked Pangolin (v0.9.6 submodule); apt libeigen3-dev is gone.
# P6-2: the backend now builds against the pinned upstream g2o submodule
# (third_party/g2o @ 20241228_git); Sophus remains vendored Thirdparty.
# R3 (2026-08-20): DBoW2 builds from the pinned upstream dorian3d submodule
# (third_party/DBoW2 @ master 3924753); the ORB-SLAM fork deltas live in
# src/recognition/OrbVocabulary.cpp + include/recognition/BowTypes.hpp, and
# DUtils::Random moved to src/util/. The vendored Thirdparty/DBoW2 is gone.
# The vendored Thirdparty/g2o stays in-tree only for the frozen reference
# variant of tests/backend_equiv (equivalence harness, docs/P6_DESIGN.md §B).
# =============================================================================

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# C++ standard is applied PER TARGET below, not globally: Thirdparty
# (DBoW2) and third_party/g2o keep their own project defaults — historically,
# forcing strict c++17 (__STRICT_ANSI__) onto libg2o measurably degraded
# stereo-inertial ATE (P1 gate finding, p=0.017). Our targets are C++20
# since R1.

option(BUILD_EXAMPLES "Build dataset example executables" ON)
option(BUILD_TESTS "Build bit-identity test harness" ON)

add_compile_options(-Wall -O3)

# R2: cmake/ is PREPENDed so our FindEigen3.cmake (hard pin to the
# third_party/eigen submodule) shadows every other Eigen lookup in this
# configure — g2o's bundled module and Pangolin's find_dependency(Eigen3)
# included. MODULE keyword below = never fall back to a system config.
list(PREPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.4 REQUIRED MODULE)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

# ---- upstream DBoW2 submodule (pinned dorian3d master 3924753) --------------
# Compiled as our own small static target instead of add_subdirectory: the
# upstream CMakeLists builds a demo and pulls DLib via ExternalProject at
# configure time — unusable here. Only the four sources the ORB vocabulary
# and BoW containers need are built; the submodule is never patched (the
# OrbLevenberg precedent — ORB-SLAM-specific behavior lives in
# src/recognition/ + include/recognition/ instead).
add_library(DBoW2 STATIC
  third_party/DBoW2/src/BowVector.cpp
  third_party/DBoW2/src/FeatureVector.cpp
  third_party/DBoW2/src/ScoringObject.cpp
  third_party/DBoW2/src/FORB.cpp
)
target_include_directories(DBoW2 PUBLIC
  ${PROJECT_SOURCE_DIR}/third_party/DBoW2/include          # "DBoW2/..." includes
  PRIVATE
  ${PROJECT_SOURCE_DIR}/third_party/DBoW2/include/DBoW2    # its own bare includes
)
target_link_libraries(DBoW2 PUBLIC ${OpenCV_LIBS})
# Linked into the SHARED core; upstream default C++ dialect (no forced -std).
set_target_properties(DBoW2 PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- upstream g2o submodule (pinned 20241228_git) ---------------------------
# Option matrix per docs/P6_DESIGN.md §A(5): no apps/examples, no external
# solver/GL deps, logging off (blocks the quiet spdlog fallback), and the
# numeric-determinism pins (march-native/fast-math OFF) made explicit.
set(G2O_BUILD_APPS OFF CACHE BOOL "" FORCE)
set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(G2O_USE_LOGGING OFF CACHE BOOL "" FORCE)
set(G2O_USE_CHOLMOD OFF CACHE BOOL "" FORCE)
set(G2O_USE_CSPARSE OFF CACHE BOOL "" FORCE)
set(G2O_USE_OPENGL OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "" FORCE)
set(G2O_FAST_MATH OFF CACHE BOOL "" FORCE)
# DO_SSE_AUTODETECT reads the BUILD HOST's /proc/cpuinfo to pick -msse* flags
# at g2o's directory scope only — host-dependent and asymmetric with our core.
# Inert on arm64 but pinned so an x86 build host cannot change the numerics
# (docs/DIVERGENCES.md #17).
set(DO_SSE_AUTODETECT OFF CACHE BOOL "" FORCE)
# R2: g2o's internal find_package(Eigen3 3.3) resolves to cmake/FindEigen3.cmake
# (module-path shadowing, see above) => same third_party/eigen submodule as core.
add_subdirectory(third_party/g2o)

# ---- core library -----------------------------------------------------------
add_library(orbslam3_core SHARED
  src/core/System.cpp
  src/core/Settings.cpp
  src/core/FixFlags.cpp
  src/camera/CameraFactory.cpp
  src/camera/Pinhole.cpp
  src/camera/KannalaBrandt8.cpp
  src/features/ORBextractor.cpp
  src/features/ORBmatcher.cpp
  src/map/Atlas.cpp
  src/map/Map.cpp
  src/map/KeyFrame.cpp
  src/map/MapPoint.cpp
  src/map/Frame.cpp
  src/backend/Optimizer.cpp
  src/backend/OrbLevenberg.cpp
  src/backend/G2oBackend.cpp
  src/backend/G2oTypes.cpp
  src/backend/OptimizableTypes.cpp
  src/backend/ImuTypes.cpp
  src/geometry/TwoViewReconstruction.cpp
  src/geometry/TwoViewInitializer.cpp
  src/geometry/Sim3Solver.cpp
  src/geometry/MLPnPsolver.cpp
  src/geometry/GeometricTools.cpp
  src/tracking/Tracking.cpp
  src/mapping/LocalMapping.cpp
  src/mapping/ImuInitializer.cpp
  src/recognition/KeyFrameDatabase.cpp
  src/recognition/OrbVocabulary.cpp
  src/util/Random.cpp
  src/closing/LoopClosing.cpp
  src/closing/PlaceRecognition.cpp
  src/io/Converter.cpp
  src/viz/Viewer.cpp
  src/viz/FrameDrawer.cpp
  src/viz/MapDrawer.cpp
)
add_library(ORB_SLAM3::core ALIAS orbslam3_core)
set_target_properties(orbslam3_core PROPERTIES
  OUTPUT_NAME orbslam3
  CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

target_include_directories(orbslam3_core PUBLIC
  ${PROJECT_SOURCE_DIR}                    # "Thirdparty/..." relative includes
  ${PROJECT_SOURCE_DIR}/include            # module-qualified: "map/KeyFrame.hpp"
  ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus  # bare "sophus/..." includes
  ${EIGEN3_INCLUDE_DIR}
)

target_compile_definitions(orbslam3_core PUBLIC
  COMPILEDWITHC11                    # steady_clock branch in examples/lib
  BOOST_BIND_GLOBAL_PLACEHOLDERS    # silence deprecated Boost.Bind placeholders
)

# Upstream g2o targets export their include dirs (submodule root + generated
# config.h dir) PUBLIC, so no extra include wiring is needed for "g2o/..."
# includes. core self-manages its C++20 standard per target (compatible with
# the thirdparty-defaults note above — the feature is a floor, not a dialect).
target_link_libraries(orbslam3_core PUBLIC
  DBoW2
  core stuff solver_eigen solver_dense types_sba types_sim3 types_slam3d
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Boost::serialization
  OpenSSL::Crypto
)

# ---- dataset examples (golden-gate binaries) --------------------------------
if(BUILD_EXAMPLES)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
  # TUM/RealSense/RGB-D targets are intentionally not built: outside the
  # golden-gate scope (see docs/PROJECT_PLAN.md §4). Examples_old/ (old-API
  # examples, never referenced by any target) was deleted post-phase-11;
  # recover from git history if ever needed.
  add_executable(mono_euroc Examples/Monocular/mono_euroc.cc)
  add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
  add_executable(stereo_euroc Examples/Stereo/stereo_euroc.cc)
  add_executable(stereo_kitti Examples/Stereo/stereo_kitti.cc)
  add_executable(mono_inertial_euroc Examples/Monocular-Inertial/mono_inertial_euroc.cc)
  add_executable(stereo_inertial_euroc Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
  foreach(tgt mono_euroc mono_kitti stereo_euroc stereo_kitti mono_inertial_euroc stereo_inertial_euroc)
    target_link_libraries(${tgt} PRIVATE ORB_SLAM3::core)
    set_target_properties(${tgt} PROPERTIES
      CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
  endforeach()
endif()

# ---- bit-identity test harness (P4) -----------------------------------------
if(BUILD_TESTS)
  add_executable(extract_hash tests/bit_identity/extract_hash.cpp)
  target_link_libraries(extract_hash PRIVATE ORB_SLAM3::core OpenSSL::Crypto)
  set_target_properties(extract_hash PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

  # ---- threading protocol checks (P10-4) ------------------------------------
  # Dependency-free assert-style binary (extract_hash mold): bounded-wait
  # checks of the LocalMapping queue+CV+flag protocol shape. See the file
  # header for the fixture-vs-class scope note.
  find_package(Threads REQUIRED)
  add_executable(threading_checks tests/threading/threading_checks.cpp)
  target_link_libraries(threading_checks PRIVATE Threads::Threads)
  set_target_properties(threading_checks PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

  # ---- vocabulary binary-cache round trip (P11-V) ---------------------------
  # extract_hash mold: text load -> saveToBinaryFile -> loadFromBinaryFile
  # into a second instance -> node-by-node bit-exact comparison (the
  # DIVERGENCES #27 "identical by construction" evidence). OpenCV comes
  # transitively through the DBoW2 target. R3: exercises the OrbVocabulary
  # wrapper (compiled in directly — no core dependency).
  add_executable(vocab_roundtrip
    tests/vocab/vocab_roundtrip.cpp
    src/recognition/OrbVocabulary.cpp)
  target_include_directories(vocab_roundtrip PRIVATE ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(vocab_roundtrip PRIVATE DBoW2)
  set_target_properties(vocab_roundtrip PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

  # ---- FixLevel flag parse/preset checks (P11-F0) ---------------------------
  # extract_hash mold: asserts the all-false level-0 default (the permanent
  # gate contract), the FixLevel:1/2 preset sets, individual-key override
  # precedence in both directions, and absent-keys silence, on temp yamls
  # written by the test itself. Runs well under 1 s.
  add_executable(fixflags_checks tests/fixflags/fixflags_checks.cpp)
  target_link_libraries(fixflags_checks PRIVATE ORB_SLAM3::core)
  set_target_properties(fixflags_checks PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

  # ---- FL-9 scale-convergence property test (P11-F1) ------------------------
  # extract_hash mold; reuses the backend_equiv imu_chain fixture (with the
  # P11-F1 s_true override) against the LIVE backend in this build tree.
  # OFF phase reproduces the DIVERGENCES #9 stall at s_true=2.0; the ON phase
  # arms Fix.ScaleJacobian through the public write-once FixFlags::Set
  # (single-threaded binary, no System — the Set-before-threads contract
  # holds trivially) and asserts convergence. The frozen reference backend
  # cannot arm the flag, so the ON property lives here, not in backend_equiv.
  add_executable(fl9_scale_convergence
    tests/fixlevel/fl9_scale_convergence.cpp
    tests/backend_equiv/common/EquivFixtures.cpp)
  target_include_directories(fl9_scale_convergence PRIVATE
    ${PROJECT_SOURCE_DIR}/tests/backend_equiv/common)
  target_link_libraries(fl9_scale_convergence PRIVATE ORB_SLAM3::core)
  set_target_properties(fl9_scale_convergence PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

  # ---- P11-F3..F5 fix-level checks -----------------------------------------
  foreach(tgt detection_machine_checks fl13_ldlt_posegraph fl5_stereo_mb_checks)
    add_executable(${tgt} tests/fixlevel/${tgt}.cpp)
    target_link_libraries(${tgt} PRIVATE ORB_SLAM3::core)
    set_target_properties(${tgt} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
      CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
  endforeach()
endif()
