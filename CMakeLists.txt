cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM3_Modern VERSION 0.1.0 LANGUAGES C CXX)

# =============================================================================
# Modern CMake replacement for the upstream flat build (Phase 1).
# Behavior parity with the golden baseline (benchmark/golden/BASELINE.md):
#   -O3, -std=c++17, NO -march=native anywhere, BOOST_BIND_GLOBAL_PLACEHOLDERS.
# Vendored Thirdparty (DBoW2/g2o/Sophus) is still the backend at this phase;
# submodules under third_party/ take over in later phases (backend swap = P6).
# =============================================================================

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# C++ standard is applied PER TARGET below, not globally: the golden baseline
# built Thirdparty (DBoW2/g2o) as standalone projects with the compiler
# default (gnu++17), and forcing strict c++17 (__STRICT_ANSI__) onto libg2o
# measurably degrades stereo-inertial ATE (P1 gate finding, p=0.017 —
# the same flag-sensitivity class as the Remastered mono-inertial regression).

option(BUILD_EXAMPLES "Build dataset example executables" ON)

add_compile_options(-Wall -O3)

find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

# ---- core library -----------------------------------------------------------
add_library(orbslam3_core SHARED
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Atlas.cc
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Viewer.cc
  src/ImuTypes.cc
  src/G2oTypes.cc
  src/CameraModels/Pinhole.cpp
  src/CameraModels/KannalaBrandt8.cpp
  src/OptimizableTypes.cpp
  src/MLPnPsolver.cpp
  src/GeometricTools.cc
  src/TwoViewReconstruction.cc
  src/Config.cc
  src/Settings.cc
)
add_library(ORB_SLAM3::core ALIAS orbslam3_core)
set_target_properties(orbslam3_core PROPERTIES
  OUTPUT_NAME orbslam3
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

target_include_directories(orbslam3_core PUBLIC
  ${PROJECT_SOURCE_DIR}                    # "Thirdparty/..." relative includes
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/CameraModels
  ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus  # bare "sophus/..." includes
  ${EIGEN3_INCLUDE_DIR}
)

target_compile_definitions(orbslam3_core PUBLIC
  COMPILEDWITHC11                    # steady_clock branch in examples/lib
  BOOST_BIND_GLOBAL_PLACEHOLDERS    # silence deprecated Boost.Bind placeholders
)

target_link_libraries(orbslam3_core PUBLIC
  DBoW2
  g2o
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Boost::serialization
  OpenSSL::Crypto
)

# ---- dataset examples (golden-gate binaries) --------------------------------
if(BUILD_EXAMPLES)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
  # TUM/RealSense/RGB-D and Examples_old targets are intentionally not built:
  # outside the golden-gate scope (see docs/PROJECT_PLAN.md §4).
  add_executable(mono_euroc Examples/Monocular/mono_euroc.cc)
  add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
  add_executable(stereo_euroc Examples/Stereo/stereo_euroc.cc)
  add_executable(stereo_kitti Examples/Stereo/stereo_kitti.cc)
  add_executable(mono_inertial_euroc Examples/Monocular-Inertial/mono_inertial_euroc.cc)
  add_executable(stereo_inertial_euroc Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
  foreach(tgt mono_euroc mono_kitti stereo_euroc stereo_kitti mono_inertial_euroc stereo_inertial_euroc)
    target_link_libraries(${tgt} PRIVATE ORB_SLAM3::core)
    set_target_properties(${tgt} PROPERTIES
      CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
  endforeach()
endif()
