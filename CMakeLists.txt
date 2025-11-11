cmake_minimum_required(VERSION 3.10)
project(sti_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(deps/polyscope)
add_subdirectory(deps/geometry-central)

add_executable(sti_example src/sample.cpp)

target_include_directories(sti_example PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/deps/polyscope/include
    ${PROJECT_SOURCE_DIR}/deps/geometry-central/include
    ${PROJECT_SOURCE_DIR}/deps/geometry-central/deps/eigen
)

target_link_libraries(sti_example
    polyscope
    geometry-central
)

target_compile_definitions(sti_example PRIVATE GLM_ENABLE_EXPERIMENTAL)
