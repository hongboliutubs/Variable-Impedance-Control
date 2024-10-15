cmake_minimum_required(VERSION 3.1)
project(Masterarbeit)

find_package(Franka REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main Franka::Franka)