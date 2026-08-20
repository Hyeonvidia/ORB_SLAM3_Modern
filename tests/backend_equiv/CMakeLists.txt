# =============================================================================
# P6 backend-equivalence harness (docs/P6_DESIGN.md §B) — standalone project.
#
#   -DEQUIV_BACKEND=vendored : frozen reference_backend/ snapshot + vendored
#                              Thirdparty/g2o (golden pre-migration backend)
#   -DEQUIV_BACKEND=modern   : live src/backend/ + third_party/g2o submodule
#                              (P6-2)
#
# Two independent build trees, one binary each (ODR: the two g2o's share
# symbols and cannot coexist in one binary — design §B.3).
#
# Flag parity with the root build / golden baseline: -Wall -O3, per-target
# C++17, NO -march=native anywhere.
# =============================================================================
cmake_minimum_required(VERSION 3.16)
project(backend_equiv LANGUAGES C CXX)

set(EQUIV_BACKEND "vendored" CACHE STRING "backend variant: vendored | modern")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "backend_equiv: EQUIV_BACKEND=${EQUIV_BACKEND}, build type ${CMAKE_BUILD_TYPE}")

add_compile_options(-Wall -O3)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." REALPATH)

# Same find_package set as the root CMakeLists.
# R3 repair of R2 fallout: apt libeigen3-dev is gone since R2 — Eigen must
# resolve through the repo's FindEigen3.cmake pin (third_party/eigen), same
# module-path shadowing as the root build.
list(PREPEND CMAKE_MODULE_PATH "${REPO_ROOT}/cmake")
find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED MODULE)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

# R3: DBoW2 comes from the pinned upstream submodule for BOTH variants — the
# live shared sources (Frame/KeyFrame/ORBmatcher/...) now include the
# recognition wrapper layer, and the vendored Thirdparty/DBoW2 is retired.
# Same 4-source static target as the root CMakeLists (upstream's own CMake
# pulls DLib via ExternalProject and is not usable via add_subdirectory).
add_library(DBoW2 STATIC
  ${REPO_ROOT}/third_party/DBoW2/src/BowVector.cpp
  ${REPO_ROOT}/third_party/DBoW2/src/FeatureVector.cpp
  ${REPO_ROOT}/third_party/DBoW2/src/ScoringObject.cpp
  ${REPO_ROOT}/third_party/DBoW2/src/FORB.cpp
)
target_include_directories(DBoW2 PUBLIC
  ${REPO_ROOT}/third_party/DBoW2/include
  PRIVATE
  ${REPO_ROOT}/third_party/DBoW2/include/DBoW2
)
target_link_libraries(DBoW2 PUBLIC ${OpenCV_LIBS})

if(EQUIV_BACKEND STREQUAL "vendored")
  # Vendored (pre-migration) g2o, exactly as the pre-P6-2 root build used it.
  add_subdirectory(${REPO_ROOT}/Thirdparty/g2o    ${CMAKE_BINARY_DIR}/Thirdparty/g2o)

  set(EQUIV_BACKEND_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/reference_backend/Optimizer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/reference_backend/G2oTypes.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/reference_backend/OptimizableTypes.cpp
  )
  # Frozen snapshot headers shadow the live include/ tree (priority include
  # dir, see target_include_directories BEFORE below): every TU of this
  # variant — including live shared sources such as src/map/Frame.cpp, which
  # includes backend/G2oTypes.hpp — must see the golden headers, not the
  # migrated ones. Since P6-2 the snapshot also freezes the pre-migration
  # io/Converter.hpp, closing/LoopClosing.hpp and backend/ILoopOptimizer.hpp
  # (their only change was the g2o include path; the live copies now point at
  # the upstream submodule, which this variant must not see).
  set(EQUIV_BACKEND_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/reference_backend/include)
  # P7-1b repair (2026-08-11, see docs/DIVERGENCES.md 게이트 방법론 이력): the
  # frozen Optimizer.cpp uses Verbose but never includes it — the declaration
  # arrived transitively via frozen closing/LoopClosing.hpp -> live
  # tracking/Tracking.hpp -> core/System.hpp -> core/Verbose.hpp, a chain
  # P7-1b severed. The live TU gained an explicit #include in that commit; the
  # byte-frozen snapshot cannot. Re-inject the declaration at the compile
  # command instead of freezing pre-P7-1b Tracking.hpp: that header's change
  # was an API refactor, outside the P6-2 include-drift precedent, and
  # freezing it would re-couple the snapshot to the still-drifting
  # System/Viewer closure. core/Verbose.hpp is unchanged since P2-3, so the
  # TU sees byte-identical content to the pre-P7-1b build; Verbose::th's
  # definition already lives in common/EquivMain.cpp.
  set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/reference_backend/Optimizer.cpp
    PROPERTIES COMPILE_OPTIONS "-include;${REPO_ROOT}/include/core/Verbose.hpp")
  set(EQUIV_G2O_TARGETS g2o)
elseif(EQUIV_BACKEND STREQUAL "modern")
  # Live migrated backend + pinned upstream g2o submodule (20241228_git).
  # Same option matrix as the root CMakeLists (docs/P6_DESIGN.md §A(5)).
  set(G2O_BUILD_APPS OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(G2O_USE_LOGGING OFF CACHE BOOL "" FORCE)
  set(G2O_USE_CHOLMOD OFF CACHE BOOL "" FORCE)
  set(G2O_USE_CSPARSE OFF CACHE BOOL "" FORCE)
  set(G2O_USE_OPENGL OFF CACHE BOOL "" FORCE)
  set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "" FORCE)
  set(G2O_FAST_MATH OFF CACHE BOOL "" FORCE)
  add_subdirectory(${REPO_ROOT}/third_party/g2o ${CMAKE_BINARY_DIR}/third_party/g2o)

  set(EQUIV_BACKEND_SOURCES
    ${REPO_ROOT}/src/backend/Optimizer.cpp
    # g2o-fork parity restoration (docs/DIVERGENCES.md 10): modern-only, the
    # vendored variant already gets the criterion from its own Thirdparty/g2o.
    ${REPO_ROOT}/src/backend/OrbLevenberg.cpp
    ${REPO_ROOT}/src/backend/G2oTypes.cpp
    ${REPO_ROOT}/src/backend/OptimizableTypes.cpp
  )
  # No shadow include dir: the live include/ tree is the modern contract.
  set(EQUIV_BACKEND_INCLUDE "")
  # Upstream g2o exports its include dirs on these targets (submodule root +
  # generated config.h build dir), so "g2o/..." includes need no extra wiring.
  set(EQUIV_G2O_TARGETS core stuff solver_eigen solver_dense
                        types_sba types_sim3 types_slam3d)
else()
  message(FATAL_ERROR "EQUIV_BACKEND must be 'vendored' or 'modern' (got '${EQUIV_BACKEND}')")
endif()

# Shared LIVE sources (g2o-symbol-free, but Frame.cpp #includes G2oTypes.hpp,
# so they are recompiled per variant against that variant's headers/g2o —
# design §B.0/§B.5). ImuTypes is deliberately NOT snapshotted: shared float
# path used live by both variants.
set(EQUIV_SHARED_SOURCES
  # R4a: the FixLevel flag machinery (formerly a shared TU here) was retired
  # when the eight flagged fixes were promoted to unconditional. NOTE: the live
  # backend now carries the #9 scale-jacobian chain rule unconditionally
  # while the frozen reference twin stays unfixed, so the cross-variant
  # equivalence contract for scale-jacobian-sensitive fixtures is stale by
  # design (the convergence property lives in tests/fixlevel/
  # fl9_scale_convergence.cpp).
  ${REPO_ROOT}/src/backend/ImuTypes.cpp
  ${REPO_ROOT}/src/map/Frame.cpp
  ${REPO_ROOT}/src/map/KeyFrame.cpp
  ${REPO_ROOT}/src/map/Map.cpp
  ${REPO_ROOT}/src/map/MapPoint.cpp
  ${REPO_ROOT}/src/map/Atlas.cpp
  ${REPO_ROOT}/src/camera/CameraFactory.cpp
  ${REPO_ROOT}/src/camera/Pinhole.cpp
  ${REPO_ROOT}/src/camera/KannalaBrandt8.cpp
  ${REPO_ROOT}/src/features/ORBextractor.cpp
  ${REPO_ROOT}/src/features/ORBmatcher.cpp
  ${REPO_ROOT}/src/recognition/KeyFrameDatabase.cpp
  # R3: OrbVocabulary wrapper (one-TU template instantiation the live
  # headers reference) + relocated DUtils::Random (previously linked in via
  # the vendored DBoW2 library, now a plain shared TU).
  ${REPO_ROOT}/src/recognition/OrbVocabulary.cpp
  ${REPO_ROOT}/src/util/Random.cpp
  ${REPO_ROOT}/src/io/Converter.cpp
  ${REPO_ROOT}/src/geometry/GeometricTools.cpp
  ${REPO_ROOT}/src/geometry/TwoViewReconstruction.cpp
  ${REPO_ROOT}/src/geometry/TwoViewInitializer.cpp
  ${REPO_ROOT}/src/geometry/Sim3Solver.cpp
  ${REPO_ROOT}/src/geometry/MLPnPsolver.cpp
)

add_executable(equiv_runner
  common/EquivMain.cpp
  common/EquivFixtures.cpp
  common/EquivSerialize.cpp
  ${EQUIV_BACKEND_SOURCES}
  ${EQUIV_SHARED_SOURCES}
)

# R1: the modern variant follows the core's C++20 bump; the vendored variant
# stays C++17 — it compiles the byte-frozen reference_backend snapshot against
# the vendored (pre-migration) g2o fork, whose contract predates the bump.
if(EQUIV_BACKEND STREQUAL "modern")
  set_target_properties(equiv_runner PROPERTIES
    CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
else()
  set_target_properties(equiv_runner PROPERTIES
    CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
endif()

# BEFORE: the backend snapshot headers MUST win over ${REPO_ROOT}/include
# (vendored variant only; the modern variant sets no shadow dir).
if(EQUIV_BACKEND_INCLUDE)
  target_include_directories(equiv_runner BEFORE PRIVATE ${EQUIV_BACKEND_INCLUDE})
endif()
target_include_directories(equiv_runner PRIVATE
  ${REPO_ROOT}                     # "Thirdparty/..." relative includes
  ${REPO_ROOT}/include             # module-qualified: "map/Frame.hpp"
  ${REPO_ROOT}/Thirdparty/Sophus   # bare "sophus/..." includes
  ${EIGEN3_INCLUDE_DIR}
)

target_compile_definitions(equiv_runner PRIVATE
  COMPILEDWITHC11
  BOOST_BIND_GLOBAL_PLACEHOLDERS
)

target_link_libraries(equiv_runner PRIVATE
  DBoW2
  ${EQUIV_G2O_TARGETS}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Boost::serialization
  OpenSSL::Crypto
)
