cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0054 NEW)

if (NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "install prefix" FORCE)
endif()


OPTION(AIGLE_SITL_MODE "Enable SITL mode " ON)

project( Aigle VERSION 1.0.0)
include(GNUInstallDirs)

#######################
## Find Dependencies ##
#######################

# Add search directory for CMake on OS X
list(APPEND CMAKE_MODULE_PATH /usr/local/share/cmake/Modules)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")


## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)

# find MAVLink
find_package(MAVLink)

# find thread package
find_package (Threads REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -std=c99 -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")
set (CMAKE_BUILD_TYPE Release)

set(${PROJECT_NAME}_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

set(FREERTOS_SRC_DIR 
  ${CMAKE_CURRENT_SOURCE_DIR}/external/FreeRTOS/Source
)

set(FREERTOS_SRCS
   ${FREERTOS_SRC_DIR}/event_groups.c
   ${FREERTOS_SRC_DIR}/list.c
   ${FREERTOS_SRC_DIR}/queue.c
   ${FREERTOS_SRC_DIR}/tasks.c
   ${FREERTOS_SRC_DIR}/timers.c
   ${FREERTOS_SRC_DIR}/croutine.c
   ${FREERTOS_SRC_DIR}/stream_buffer.c
   ${FREERTOS_SRC_DIR}/portable/GCC/Linux/port.c
   ${FREERTOS_SRC_DIR}/portable/MemMang/heap_3.c
)

set (Aigle_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/Matrix
  ${CMAKE_CURRENT_BINARY_DIR}
  ${MAVLINK_INCLUDE_DIRS}
  ${FREERTOS_SRC_DIR}/include
  ${FREERTOS_SRC_DIR}/portable/GCC/Linux
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
	src/AigleIoInterfaceSITL.cpp
	src/StrategyManager.cpp
  src/Estimator.cpp
  src/RectangleArea.cpp
  src/OutOfZoneFault.cpp
  src/InsideSoftZoneCorrector.cpp
  src/InsideHardZoneCorrector.cpp
  src/LowBatteryFault.cpp
  src/NoMotorCommandFault.cpp
  src/NoMotorLowBatteryCorrector.cpp
  src/StrategyDecisionMaker.cpp
  ${FREERTOS_SRCS}
)

# create .so
# add_library(Aigle SHARED ${SRCS})
# target_include_directories(Aigle PUBLIC include)

# Create an executable
add_executable(main ${SRCS})
target_link_libraries(main ${CMAKE_THREAD_LIBS_INIT})

if (AIGLE_SITL_MODE)
    message(STATUS "COMPILING IN SITL MODE ...")
    # target_compile_options(Aigle PUBLIC -DAIGLE_SITL_MODE)
    target_compile_options(main PUBLIC -DAIGLE_SITL_MODE -DSUPPORT_STDIOSTREAM)
else()
    message(STATUS "COMPILING IN NON SITL MODE ...")
endif()

# # Generate the Config file from the .in file, doing @VAR@ substitution only
# configure_file(AigleConfig.cmake.in FindAigle.cmake @ONLY)

# # INSTALL STEPS
# INSTALL(
#   TARGETS Aigle
#   RUNTIME DESTINATION lib
#   LIBRARY DESTINATION lib)

# INSTALL(
#   DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
#   DESTINATION include)

# INSTALL(
#   FILES ${CMAKE_BINARY_DIR}/FindAigle.cmake
#   DESTINATION ${CMAKE_INSTALL_PREFIX})
