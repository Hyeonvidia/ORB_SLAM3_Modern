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
find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

if(EQUIV_BACKEND STREQUAL "vendored")
  # Vendored (pre-migration) g2o + DBoW2, exactly as the root build uses them.
  add_subdirectory(${REPO_ROOT}/Thirdparty/g2o    ${CMAKE_BINARY_DIR}/Thirdparty/g2o)
  add_subdirectory(${REPO_ROOT}/Thirdparty/DBoW2  ${CMAKE_BINARY_DIR}/Thirdparty/DBoW2)

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
  add_subdirectory(${REPO_ROOT}/Thirdparty/DBoW2 ${CMAKE_BINARY_DIR}/Thirdparty/DBoW2)

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

set_target_properties(equiv_runner PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

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
