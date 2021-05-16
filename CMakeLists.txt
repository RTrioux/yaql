cmake_minimum_required(VERSION 3.17.3)
get_filename_component(ROOT ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(PROJECT_NAME ${ROOT})
project(${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(Colors)
message(STATUS "${Green}Processing "  ${ROOT} ${ColourReset})
include_directories(inc)


file(GLOB SOURCES "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# Gtest related
file(GLOB TEST_SOURCES "test/src/*.cpp")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test/lib")
add_library(${PROJECT_NAME}_lib STATIC src/quat.cpp src/vector3d.cpp)
add_executable(${PROJECT_NAME}_test ${TEST_SOURCES})
set_target_properties(${PROJECT_NAME}_test PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_test)
# If gtest installed on the system
target_link_libraries(${PROJECT_NAME}_test PUBLIC ${PROJECT_NAME}_lib gtest)
# else
# ??? gtest should be cloned, compiled
set_target_properties(${PROJECT_NAME}_test PROPERTIES EXCLUDE_FROM_ALL 1)

#add_subdirectory(test)