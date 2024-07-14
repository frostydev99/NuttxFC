cmake_minimum_required(VERSION 3.5)
project(FlightComputer)

set(CMAKE_CXX_STANDARD 14)

include_directories(
        ${CMAKE_SOURCE_DIR}/FlightComputer/StateMachine
        ${CMAKE_SOURCE_DIR}/FlightComputer/StateMachine/sensors
        ${CMAKE_SOURCE_DIR}/FlightComputer/StateMachine/sensors/LPS25
        ${CMAKE_SOURCE_DIR}/FlightComputer/StateMachine/sensors/MMC5983MA
)

file(GLOB_RECURSE SOURCES
        ${CMAKE_SOURCE_DIR}/FlightComputer/StateMachine/*.cxx
        ${CMAKE_SOURCE_DIR}/FlightComputer/StateMachine/sensors/LPS25/*.cxx
        ${CMAKE_SOURCE_DIR}/FlightComputer/StateMachine/sensors/MMC5983MA/*.cxx
)

add_executable(FlightComputer ${SOURCES})
