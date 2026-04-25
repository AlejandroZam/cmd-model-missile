cmake_minimum_required(VERSION 3.15)
project(cmd-model-missile VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT TARGET osk)
    find_package(osk 1.0.0 REQUIRED)
endif()

add_library(missile STATIC missile.cpp)
target_include_directories(missile PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(missile PUBLIC osk)

install(TARGETS missile EXPORT missile-targets ARCHIVE DESTINATION lib)
install(FILES missile.h           DESTINATION include)
install(FILES default_params.yaml DESTINATION share/cmd-model-missile)
