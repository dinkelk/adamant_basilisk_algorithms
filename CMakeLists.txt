cmake_minimum_required(VERSION 3.15)

project(attTrackingError LANGUAGES CXX)

# 1) Force C++23, PIC
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 2) Ensure CMAKE_BUILD_TYPE is defined (default=Release)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Debug or Release" FORCE)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_UPPER)
message(STATUS "Build type: ${BUILD_TYPE_UPPER}")

# 3) If we're NOT cross-compiling (i.e. native Linux), set native flags here.
#    For RISC-V, the toolchain file will handle all flags.
if(NOT CMAKE_CROSSCOMPILING)
  if(BUILD_TYPE_UPPER STREQUAL "DEBUG")
    message(STATUS "Configuring native Linux Debug flags")
    set(CMAKE_C_FLAGS_DEBUG   "${CMAKE_C_FLAGS_DEBUG}   -O0 -g"        CACHE STRING "" FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g"        CACHE STRING "" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} "    CACHE STRING "" FORCE)
  else()
    message(STATUS "Configuring native Linux Release flags")
    set(CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE}   -O3 -DNDEBUG" CACHE STRING "" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG" CACHE STRING "" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} " CACHE STRING "" FORCE)
  endif()
endif()

# 4) Include Eigen (always /home/user/eigen) and project headers
set(EIGEN3_DIR "/home/user/eigen")
if(NOT EXISTS "${EIGEN3_DIR}/Eigen/Dense")
  message(FATAL_ERROR "Eigen not found under ${EIGEN3_DIR}")
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}                            # root headers
  ${PROJECT_SOURCE_DIR}/architecture/utilities     # avsEigenSupport.h, etc.
  ${PROJECT_SOURCE_DIR}/architecture/msgPayloadDefC # AttGuidMsgPayload.h, etc.
  ${EIGEN3_DIR}                                    # Eigen headers
)

# 5) Force Eigen freestanding
add_compile_definitions(EIGEN_FREESTANDING=1)

# 6) List source files
set(SRC_FILES
  attTrackingErrorAlgorithm.cpp
  architecture/utilities/avsEigenSupport.cpp
)

# 7) Build static library
add_library(attTrackingError STATIC ${SRC_FILES})

# 8) Put the archive in build/lib/
set_target_properties(attTrackingError PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
)

# 9) Usage notes (omit flags here—toolchain file handles RISC-V):
#    - Native Linux (Debug):
#        mkdir build_linux_debug && cd build_linux_debug
#        cmake -DCMAKE_BUILD_TYPE=Debug ..
#        make
#
#    - Native Linux (Release):
#        mkdir build_linux_release && cd build_linux_release
#        cmake -DCMAKE_BUILD_TYPE=Release ..
#        make
#
#    - RISC-V 32 (Debug):
#        mkdir build_riscv_debug && cd build_riscv_debug
#        cmake -DCMAKE_BUILD_TYPE=Debug \
#              -DCMAKE_TOOLCHAIN_FILE=../riscv32-toolchain.cmake \
#              ..
#        make
#
#    - RISC-V 32 (Release):
#        mkdir build_riscv_release && cd build_riscv_release
#        cmake -DCMAKE_BUILD_TYPE=Release \
#              -DCMAKE_TOOLCHAIN_FILE=../riscv32-toolchain.cmake \
#              ..
#        make
