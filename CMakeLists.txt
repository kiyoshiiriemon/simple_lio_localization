CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

project(simple_lio_localization)

add_subdirectory(thirdparty/small_gicp)

if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  thirdparty/small_gicp/include
  thirdparty/bag_rdr
  thirdparty/bag_rdr/deps/common_cxx
)

add_library(simple_lio_localization
    src/simple_lio_loc.cpp
    src/map_matcher.cpp
)

target_link_libraries(simple_lio_localization
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    Eigen3::Eigen
    small_gicp
)

add_executable(test_simple_lio_localization
    src/simple_lio_loc.cpp
    src/test_main.cpp
    src/map_matcher.cpp
)

target_link_libraries(test_simple_lio_localization
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    pthread
    Eigen3::Eigen
    small_gicp
)

install(TARGETS simple_lio_localization
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

