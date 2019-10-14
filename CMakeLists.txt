cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
project(benchmark VERSION 0.0.1 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
add_library(eigen3-dep INTERFACE IMPORTED)
target_compile_features(eigen3-dep INTERFACE cxx_std_11)
target_link_libraries(eigen3-dep INTERFACE Eigen3::Eigen)

find_package(yaml-cpp REQUIRED)
add_library(yaml-cpp-dep INTERFACE IMPORTED)
target_compile_features(yaml-cpp-dep INTERFACE cxx_std_11)
target_include_directories(yaml-cpp-dep INTERFACE ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(yaml-cpp-dep INTERFACE ${YAML_CPP_LIBRARIES})

add_library(benchmark-dep INTERFACE IMPORTED)
target_sources(benchmark-dep INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src/benchmark.h)
target_compile_features(benchmark-dep INTERFACE cxx_std_17)
target_link_libraries(benchmark-dep INTERFACE eigen3-dep yaml-cpp-dep)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(accuracy ${CMAKE_CURRENT_SOURCE_DIR}/src/accuracy.cpp)
target_link_libraries(accuracy PUBLIC benchmark-dep)

add_executable(initialization ${CMAKE_CURRENT_SOURCE_DIR}/src/initialization.cpp)
target_link_libraries(initialization PUBLIC benchmark-dep)

add_executable(robustness ${CMAKE_CURRENT_SOURCE_DIR}/src/robustness.cpp)
target_link_libraries(robustness PUBLIC benchmark-dep)

add_executable(relocalization ${CMAKE_CURRENT_SOURCE_DIR}/src/relocalization.cpp)
target_link_libraries(relocalization PUBLIC benchmark-dep)

add_executable(relocalization_runtime ${CMAKE_CURRENT_SOURCE_DIR}/src/relocalization_runtime.cpp)
target_link_libraries(relocalization_runtime PUBLIC benchmark-dep)

add_executable(extract_time ${CMAKE_CURRENT_SOURCE_DIR}/src/extract_time.cpp)
target_link_libraries(extract_time PUBLIC benchmark-dep)
