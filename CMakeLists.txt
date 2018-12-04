cmake_minimum_required(VERSION 3.0)

project(franka_utils CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(
    common
    STATIC
    src/examples_common.cpp
    src/quintic_interpolator.cpp
)
target_link_libraries(common PUBLIC Franka::Franka Eigen3::Eigen3)

add_executable(noise src/noise.cpp)
target_link_libraries(noise Franka::Franka common Eigen3::Eigen3)

#add_executable(sine_wave src/sine_wave.cpp)
#target_link_libraries(sine_wave Franka::Franka common Eigen3::Eigen3)

add_executable(playback src/playback.cpp)
target_link_libraries(playback Franka::Franka common Eigen3::Eigen3)

add_executable(record src/record.cpp)
target_link_libraries(record Franka::Franka common Eigen3::Eigen3)

add_executable(trajectory_test src/trajectory_test.cpp src/trajectory.cpp)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
