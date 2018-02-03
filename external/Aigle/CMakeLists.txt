cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0054 NEW)

if (NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "install prefix" FORCE)
endif()

message(STATUS "install-prefix: ${CMAKE_INSTALL_PREFIX}")

project( Aigle VERSION 1.0.0)
include(GNUInstallDirs)

#######################
## Find Dependencies ##
#######################

# Add search directory for CMake on OS X
list(APPEND CMAKE_MODULE_PATH /usr/local/share/cmake/Modules)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# find MAVLink
find_package(MAVLink)

OPTION(AIGLE_SITL_MODE "Enable SITL mode " ON)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)
# find_package(gazebo REQUIRED)

# include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -std=c99 -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")
set (CMAKE_BUILD_TYPE Release)

set(${PROJECT_NAME}_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

message(STATUS "Building ${PROJECT_NAME} .... ")

set (Aigle_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${MAVLINK_INCLUDE_DIRS}
)

include_directories(${Aigle_INCLUDE_DIRS})

# link_libraries(
# 	${GAZEBO_LIBRARIES}
# )

# link_directories(
# 	${GAZEBO_LIBRARY_DIRS}
# 	${CMAKE_CURRENT_BINARY_DIR}
# )

set(SRCS
	src/aigle.cpp
	src/io_aigle_sitl.cpp
	src/strategy.cpp
)

add_library(Aigle SHARED ${SRCS})
target_include_directories(Aigle PUBLIC include)
if (AIGLE_SITL_MODE)
    message(STATUS "COMPILING IN SITL MODE ...")
    target_compile_options(Aigle PUBLIC -DAIGLE_SITL_MODE)
else()
    message(STATUS "COMPILING IN NON SITL MODE ...")
endif()

# add_executable(main ${SRCS})
# target_compile_options(main PUBLIC -DAIGLE_SITL_MODE)

# arget_link_libraries(main ${GAZEBO_LIBRARIES})

# Generate the Config file from the .in file, doing @VAR@ substitution only
configure_file(AigleConfig.cmake.in FindAigle.cmake @ONLY)

# INSTALL STEPS
INSTALL(
  TARGETS Aigle
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib)

INSTALL(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include)

INSTALL(
  FILES ${CMAKE_BINARY_DIR}/FindAigle.cmake
  DESTINATION ${CMAKE_INSTALL_PREFIX})
