cmake_minimum_required(VERSION 3.0.2)
project(min_udp)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

set(SRC_FILES
  src/basic_math.cpp
  src/udp_comm.cpp
  src/udp_basic.cpp
  src/motor_init.cpp
)

add_library(min_udp STATIC ${SRC_FILES})
# add_library(air_interface SHARED ${SRC_FILES})
add_executable(test_udp test/test_udp.cpp)
add_executable(protect_motor test/protect_motor.cpp)

target_link_libraries(test_udp min_udp
  pthread
)

add_executable(test_udp2 test/test_udp2.cpp)
target_link_libraries(test_udp2 min_udp
  pthread
)

target_link_libraries(protect_motor min_udp
  pthread
)

install(TARGETS min_udp test_udp protect_motor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)