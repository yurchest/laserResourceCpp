cmake_minimum_required(VERSION 3.5)

project(test VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON )

set(SOURCES
        main.cpp
        ReadThread.cpp
        )

set(LIBS
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets)

add_executable(test
        ${SOURCES}
        )

target_link_libraries(test PRIVATE ${LIBS}
        ${CMAKE_SOURCE_DIR}/ftd2xx.a
        ${CMAKE_SOURCE_DIR}/libchai.so)