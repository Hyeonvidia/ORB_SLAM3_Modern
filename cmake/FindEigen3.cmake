# R2 (2026-08-20): pin module for the third_party/eigen submodule (5.0.1).
#
# Every Eigen consumer in this build resolves through this file:
#   - the top-level find_package(Eigen3 ... MODULE),
#   - g2o's internal find_package(Eigen3 3.3 REQUIRED): this file shadows
#     g2o's bundled cmake_modules/FindEigen3.cmake via CMAKE_MODULE_PATH
#     order (ours is PREPENDed at the top level, g2o only APPENDs its own).
#     The shadowing is load-bearing, not just hygiene: Eigen >= 5 moved the
#     version macros from Eigen/src/Core/util/Macros.h to Eigen/Version, so
#     g2o's bundled module would mis-parse the version and hard-fail.
#   - Pangolin's installed PangolinConfig.cmake (plain find_dependency(Eigen3)
#     -> module mode first -> this file).
# The dev image ships no other Eigen (apt libeigen3-dev dropped in R2; the
# copy Pangolin was baked against lives only in a discarded build stage), so
# any consumer escaping this pin fails loudly at configure time instead of
# silently drifting to a second Eigen.

set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../third_party/eigen")

# Version parse: Eigen >= 5 keeps EIGEN_VERSION_STRING in Eigen/Version;
# Eigen <= 3.4 keeps WORLD/MAJOR/MINOR in Eigen/src/Core/util/Macros.h.
if(EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Version")
  set(_eigen3_version_header "${EIGEN3_INCLUDE_DIR}/Eigen/Version")
else()
  set(_eigen3_version_header "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h")
endif()

set(EIGEN3_VERSION "")
if(EXISTS "${_eigen3_version_header}")
  file(READ "${_eigen3_version_header}" _eigen3_header)
  string(REGEX MATCH "define[ \t]+EIGEN_VERSION_STRING[ \t]+\"([0-9.]+)" _eigen3_m "${_eigen3_header}")
  if(CMAKE_MATCH_1)
    set(EIGEN3_VERSION "${CMAKE_MATCH_1}")  # semver scheme (>= 5.0)
  else()
    string(REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)" _eigen3_m "${_eigen3_header}")
    set(_eigen3_w "${CMAKE_MATCH_1}")
    string(REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _eigen3_m "${_eigen3_header}")
    set(_eigen3_ma "${CMAKE_MATCH_1}")
    string(REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _eigen3_m "${_eigen3_header}")
    set(EIGEN3_VERSION "${_eigen3_w}.${_eigen3_ma}.${CMAKE_MATCH_1}")  # classic 3.x scheme
  endif()
endif()
set(EIGEN3_VERSION_STRING "${EIGEN3_VERSION}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3
  REQUIRED_VARS EIGEN3_INCLUDE_DIR
  VERSION_VAR EIGEN3_VERSION)

if(Eigen3_FOUND AND NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()
