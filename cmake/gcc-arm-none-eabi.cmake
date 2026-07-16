set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# GNU Arm 工具链查找顺序：当前环境、显式根目录、STM32Cube Bundle。
set(TOOLCHAIN_PREFIX                arm-none-eabi-)
set(_TOOLCHAIN_SEARCH_DIRS)

if(DEFINED ARM_GNU_TOOLCHAIN_ROOT)
    list(APPEND _TOOLCHAIN_SEARCH_DIRS "${ARM_GNU_TOOLCHAIN_ROOT}/bin")
endif()

if(DEFINED ENV{CUBE_BUNDLE_PATH})
    file(GLOB _CUBE_ENV_TOOLCHAIN_DIRS LIST_DIRECTORIES true
        "$ENV{CUBE_BUNDLE_PATH}/gnu-tools-for-stm32/*/bin"
    )
    list(SORT _CUBE_ENV_TOOLCHAIN_DIRS COMPARE NATURAL ORDER DESCENDING)
    list(APPEND _TOOLCHAIN_SEARCH_DIRS ${_CUBE_ENV_TOOLCHAIN_DIRS})
endif()

if(WIN32 AND DEFINED ENV{LOCALAPPDATA})
    file(GLOB _CUBE_LOCAL_TOOLCHAIN_DIRS LIST_DIRECTORIES true
        "$ENV{LOCALAPPDATA}/stm32cube/bundles/gnu-tools-for-stm32/*/bin"
    )
    list(SORT _CUBE_LOCAL_TOOLCHAIN_DIRS COMPARE NATURAL ORDER DESCENDING)
    list(APPEND _TOOLCHAIN_SEARCH_DIRS ${_CUBE_LOCAL_TOOLCHAIN_DIRS})
endif()

find_program(_ARM_GCC_FROM_ENV NAMES ${TOOLCHAIN_PREFIX}gcc NO_CACHE)
if(_ARM_GCC_FROM_ENV)
    set(_ARM_GCC_EXECUTABLE "${_ARM_GCC_FROM_ENV}")
else()
    find_program(_ARM_GCC_EXECUTABLE
        NAMES ${TOOLCHAIN_PREFIX}gcc
        HINTS ${_TOOLCHAIN_SEARCH_DIRS}
        NO_DEFAULT_PATH
        NO_CACHE
        REQUIRED
    )
endif()

get_filename_component(_TOOLCHAIN_BIN_DIR "${_ARM_GCC_EXECUTABLE}" DIRECTORY)

function(_find_arm_tool output_variable tool_name)
    find_program(_TOOL_EXECUTABLE
        NAMES ${TOOLCHAIN_PREFIX}${tool_name}
        HINTS "${_TOOLCHAIN_BIN_DIR}"
        NO_DEFAULT_PATH
        NO_CACHE
        REQUIRED
    )
    set(${output_variable} "${_TOOL_EXECUTABLE}" PARENT_SCOPE)
endfunction()

_find_arm_tool(_ARM_GXX_EXECUTABLE g++)
_find_arm_tool(_ARM_OBJCOPY_EXECUTABLE objcopy)
_find_arm_tool(_ARM_SIZE_EXECUTABLE size)

set(CMAKE_C_COMPILER                ${_ARM_GCC_EXECUTABLE})
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${_ARM_GXX_EXECUTABLE})
set(CMAKE_LINKER                    ${_ARM_GXX_EXECUTABLE})
set(CMAKE_OBJCOPY                   ${_ARM_OBJCOPY_EXECUTABLE})
set(CMAKE_SIZE                      ${_ARM_SIZE_EXECUTABLE})

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
