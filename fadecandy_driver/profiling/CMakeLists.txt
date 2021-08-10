cmake_minimum_required(VERSION 3.0.0)
project(FadeCandyProfiling VERSION 0.1.0)
include_directories($(root_SOURCE_DIR)../src)

file(GLOB Sources "../src/util.cpp")

include(CTest)
enable_testing()

add_executable(FadeCandyProfiling main.cpp ${Sources})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)