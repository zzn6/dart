# ##############################################################################
# #################        CMake Template (CUSTOM)       ######################
# #################    Copyright (c) 2023 Hello World    ######################
# ##############################################################################

# Set the system name and version
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.22)

# Specify the cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)

# Specify the type of libraries to try to compile
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Set the project name and the languages used
project(dartrack1 C CXX ASM) # TODO: change project name here

# Set the C++ and C standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set the library path
set(CMAKE_LIBRARY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Lib")

# Set the linker script
file(GLOB LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32*_FLASH.ld)

# ########################## USER CONFIG SECTION ##############################
# TODO: Enable or disable software floating point
set(enable_soft_fp
    OFF
    CACHE INTERNAL "Enable software floating point")

# TODO: Enable or disable hardware floating point ! This option is required by
# the HW-Components/Third_Party/DSP library
set(enable_hard_fp
    ON
    CACHE INTERNAL "Enable hardware floating point")

# ! You can only choose one floating point mode
if(enable_soft_fp AND enable_hard_fp)
  message(FATAL_ERROR "You can only choose one floating point mode")
endif()

# TODO: Modify according to the chip model used, 
# you can find the required macro definition in the stm32f4xx.h file 
set(STM32_DEVICE STM32F427xx)

# Specify user folders
set(user_folders  "tasks" ) # TODO Add your own user folders here

# Specify the path to the HW-Components directory
set(HWC_DIR "HW-Components") # TODO: Set your own HW-Components path here

# Include utility functions and default configuration
include("${HWC_DIR}/cmake/utils/function_tools.cmake")
include("${HWC_DIR}/config.cmake") # Default configuration

# TODO: Overwrite default configuration instead of changing it in file

# TODO: Add your own `config.cmake` file or set your own configuration here

# ###################### COMPILE AND LINK OPTIONS SECTION ######################
# If hardware floating point is enabled, add the corresponding compile and link
# options
if(enable_hard_fp)
  message(STATUS "Enable hard floating point")
  add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
  add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
endif()

# If software floating point is enabled, add the corresponding compile options
if(enable_soft_fp)
  message(STATUS "Enable soft floating point")
  add_compile_options(-mfloat-abi=soft)
endif()

# Add compile options for the target CPU and instruction set,function and data
# sections
add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common
  -fmessage-length=0)

# Enable preprocessing for assembler files
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# Set optimization level based on the build type
if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(STATUS "Maximum optimization for speed")
  add_compile_options(-Ofast)
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
  message(STATUS "Maximum optimization for speed, debug info included")
  add_compile_options(-Ofast -g)
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
  message(STATUS "Maximum optimization for size")
  add_compile_options(-Os)
else()
  message(STATUS "Minimal optimization, debug info included")
  add_compile_options(-Og -g)
endif()

# Add definitions for the preprocessor
add_definitions(-DDEBUG -DUSE_HAL_DRIVER -D${STM32_DEVICE} -D__CC_ASM)

# Add link options for garbage collection, memory usage printing, map file
# generation, the target CPU and instruction set, the linker script to the link
# options
add_link_options(
  -Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

# #################### ADD LIBRARIES AND EXECUTABLE SECTION ####################
# Initialize source and include lists
set(project_srcs)
set(project_incs)

# Search for include files and source files in the Core directory
search_incs_recurse("${CMAKE_CURRENT_SOURCE_DIR}/Core" core_incs)
file(GLOB_RECURSE core_srcs "Core/*.*")
list(APPEND project_srcs ${core_srcs})
list(APPEND project_incs ${core_incs})

# Search for include files and source files in the Drivers directory
search_incs_recurse("${CMAKE_CURRENT_SOURCE_DIR}/Drivers" drivers_incs)
file(GLOB_RECURSE drivers_srcs "Drivers/*.*")
list(APPEND project_incs ${drivers_incs})

# For each user folder, search for include files and source files
foreach(user_folder ${user_folders})
  search_incs_recurse("${CMAKE_CURRENT_SOURCE_DIR}/${user_folder}"
    ${user_folder}_incs)
  file(GLOB_RECURSE ${user_folder}_srcs "${user_folder}/*.*")
  list(APPEND project_incs ${${user_folder}_incs})
  list(APPEND project_srcs ${${user_folder}_srcs})
endforeach()

# Add a static library for the drivers
add_library(drivers STATIC ${drivers_srcs})
target_include_directories(drivers PUBLIC ${core_incs} ${drivers_incs})

# Add the HW-Components directory as a subdirectory
add_subdirectory(${HWC_DIR})

# Add an executable for the project
add_executable(${PROJECT_NAME}.elf ${project_srcs} ${LINKER_SCRIPT})

# Add the project includes to the executable
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${project_incs})
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${${HWC_LIB_PREFIX}_incs})

# Link the drivers library and the HW-Components library to the executable
target_link_libraries(${PROJECT_NAME}.elf PUBLIC drivers)
target_link_libraries(${PROJECT_NAME}.elf PUBLIC ${${HWC_LIB_PREFIX}_libs})

# Define the output hex and bin files
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

# Add a post-build command to generate the hex and bin files
add_custom_command(
  TARGET ${PROJECT_NAME}.elf
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
  COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf>
  ${BIN_FILE}
  COMMENT "Building ${HEX_FILE} Building ${BIN_FILE}")