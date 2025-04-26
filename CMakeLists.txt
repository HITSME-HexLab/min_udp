cmake_minimum_required(VERSION 3.0.2)
project(min_udp)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

set(SRC_FILES
  src/basic_math.cpp
  src/udp_comm.cpp
  src/udp_basic.cpp
)

add_library(min_udp STATIC ${SRC_FILES})
# add_library(air_interface SHARED ${SRC_FILES})
add_executable(test_udp test/test_udp.cpp)

target_link_libraries(test_udp min_udp
  pthread ${catkin_LIBRARIES}
)

install(TARGETS min_udp test_udp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)