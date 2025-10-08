cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME VDCode)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
	${CMAKE_SOURCE_DIR}/Core/Inc/
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
	Core/Inc/asm330lhh_reg.h
	Core/Src/asm330lhh_reg.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
	${CMAKE_CURRENT_BINARY_DIR}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Add variables for configuration
# These can be overridden from command line using -D<VAR>=<value>
# Example: cmake -DCANBUS_ID_1=0x34 -DCANBUS_ID_2=0x35 -DIS_COG=1 ..
set(CANBUS_ID_1 0x34)
set(CANBUS_ID_2 0x35)
set(CANBUS_ID_3 0x36)
set(CANBUS_ID_4 0x37)
set(IS_COG 0)

if (NOT DEFINED IS_COG)
	message(FATAL_ERROR "IS_COG is not defined. Please define it using -DIS_COG=<value>")
endif()
if (NOT DEFINED CANBUS_ID_1)
	message(FATAL_ERROR "CANBUS_ID_1 is not defined. Please define it using -DCANBUS_ID_1=<value>")
endif()
if (NOT DEFINED CANBUS_ID_2)
	message(FATAL_ERROR "CANBUS_ID_2 is not defined. Please define it using -DCANBUS_ID_2=<value>")
endif()
if(IS_COG)
	message(STATUS "Building COG variant")
	if (NOT DEFINED CANBUS_ID_3)
		message(FATAL_ERROR "CANBUS_ID_3 is not defined. Please define it using -DCANBUS_ID_3=<value>")
	endif()
	if (NOT DEFINED CANBUS_ID_4)
		message(FATAL_ERROR "CANBUS_ID_4 is not defined. Please define it using -DCANBUS_ID_4=<value>")
	endif()
endif(IS_COG)

message(STATUS "CANBUS_ID_1 = ${CANBUS_ID_1}")
message(STATUS "CANBUS_ID_2 = ${CANBUS_ID_2}")
if(IS_COG)
	message(STATUS "CANBUS_ID_3 = ${CANBUS_ID_3}")
	message(STATUS "CANBUS_ID_4 = ${CANBUS_ID_4}")
endif(IS_COG)

configure_file(
    ${CMAKE_SOURCE_DIR}/Core/Inc/config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
	@ONLY
)
