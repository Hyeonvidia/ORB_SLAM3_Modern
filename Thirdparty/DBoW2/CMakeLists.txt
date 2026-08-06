cmake_minimum_required(VERSION 3.16)
project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Flag parity with the golden baseline (benchmark/golden/BASELINE.md):
# -Wall -O3, NO -march=native (breaks arm64 portability and silently changes
# numerics — the ORB_SLAM3_Remastered mono-inertial regression suspect).
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

find_package(OpenCV 4 REQUIRED)

add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_include_directories(DBoW2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})
