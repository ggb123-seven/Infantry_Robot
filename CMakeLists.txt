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
set(CMAKE_PROJECT_NAME Infantry_Robot)

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
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # LwRB v3.2.0 第三方源码
    Middlewares/Third_Party/RingBuffer/src/ringbuffer/ringbuffer.c

    # User/bsp 源文件
    User/bsp/can.c
    User/bsp/gpio.c
    User/bsp/mm.c
    User/bsp/time.c
    User/bsp/uart.c

    # User/component 源文件
    User/component/filter.c
    User/component/pid.c
    User/component/user_math.c

    # User/device 源文件
    User/device/can_devices.c
    User/device/dr16.c
    User/device/motor.c
    User/device/motor_rm.c

    # User/module 源文件
    User/module/chassis.c
    User/module/motor_speed_control.c

    # User/task 源文件
    User/task/can_task.c
    User/task/dr16_task.c
    User/task/init.c
    User/task/motor_chassis.c
    User/task/ozone_debug.c
    User/task/user_task.c

    # 现有 LED 任务
    applications/blue_led_task.c
    applications/green_led_task.c
    applications/red_led_task.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Middlewares/Third_Party/RingBuffer/src/include
    User
    applications
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
