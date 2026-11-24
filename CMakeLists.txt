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
set(CMAKE_PROJECT_NAME H7_prepared_Development)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Hardware/Power_contral.c
        Core/Hardware/Power_contral.h
        Core/bsp/My_Tim.c
        Core/bsp/My_Tim.h
        Core/Hardware/HW_Config.h
        Core/Hardware/Motor_Contral.c
        Core/Hardware/Motor_Contral.h
        Core/Hardware/Buzzer.h
        Core/bsp/Overall_Struct.c
        Core/bsp/bsp_Can_Callback.c
        Core/bsp/bsp_Can_Callback.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# ---------------------------------------------------------------------------
# User application sources (anything you add under Core/)
# ---------------------------------------------------------------------------
# Auto-scan the whole Core/ tree for .c/.cpp, so any module folder you create
# (e.g. Core/<new_module>/) is compiled automatically. CONFIGURE_DEPENDS makes
# CMake re-scan when files are added or removed.
# Core/Src is excluded: those CubeMX-generated files are already added by
# cmake/stm32cubemx/CMakeLists.txt (adding them again would double-link).
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/Core/*.c
    ${CMAKE_SOURCE_DIR}/Core/*.cpp
)
list(FILTER USER_SOURCES EXCLUDE REGEX "Core/Src")

# Every top-level folder under Core/ becomes an include directory, so headers
# in new module folders resolve without touching this file.
file(GLOB CORE_INCLUDE_DIRS LIST_DIRECTORIES true CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/Core/*
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CORE_INCLUDE_DIRS}
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
