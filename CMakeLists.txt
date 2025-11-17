cmake_minimum_required(VERSION 3.10)
project(sti_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting default build type to 'Release'")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

add_subdirectory(deps/polyscope)
add_subdirectory(deps/geometry-central)

add_executable(sti_example
    src/sample.cpp
)

target_include_directories(sti_example PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/deps/polyscope/include
    ${PROJECT_SOURCE_DIR}/deps/geometry-central/include
)

if (EXISTS "${PROJECT_SOURCE_DIR}/deps/geometry-central/deps/eigen")
    target_include_directories(sti_example PRIVATE
        ${PROJECT_SOURCE_DIR}/deps/geometry-central/deps/eigen
    )
endif()

target_compile_options(sti_example PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-O3 -march=native>
)

target_compile_options(sti_example PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/O2>
)

target_link_options(sti_example PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-Wl,-O1>
)

target_link_libraries(sti_example
    polyscope
    geometry-central
)

target_compile_definitions(sti_example PRIVATE GLM_ENABLE_EXPERIMENTAL)
