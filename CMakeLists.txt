cmake_minimum_required(VERSION 3.15)

project(attTrackingError LANGUAGES CXX)

# 1) Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 2) RISC-V target flags (adjust -march/-mabi as needed)
set(RISCV_ARCH_FLAGS "-march=rv32imac" "-mabi=ilp32")
add_compile_options(${RISCV_ARCH_FLAGS})
add_link_options(${RISCV_ARCH_FLAGS})

# 3) Always include /home/user/eigen for Eigen headers
set(EIGEN3_DIR "/home/user/eigen")
if(NOT EXISTS "${EIGEN3_DIR}/Eigen/Dense")
    message(FATAL_ERROR "Eigen headers not found under ${EIGEN3_DIR}. Make sure /home/user/eigen/Eigen/Dense exists.")
endif()

# 4) Include directories for all needed headers
#    - PROJECT_SOURCE_DIR covers headers at the repo root
#    - architecture/utilities covers avsEigenSupport.h, etc.
#    - architecture/msgPayloadDefC covers AttGuidMsgPayload.h, etc.
#    - EIGEN3_DIR covers <Eigen/...>
include_directories(
  ${PROJECT_SOURCE_DIR}                             # root for attTrackingErrorAlgorithm.h, etc.
  ${PROJECT_SOURCE_DIR}/architecture/utilities      # for avsEigenSupport.h, etc.
  ${PROJECT_SOURCE_DIR}/architecture/msgPayloadDefC  # for AttGuidMsgPayload.h, AttRefMsgPayload.h, NavAttMsgPayload.h
  ${EIGEN3_DIR}                                     # for <Eigen/...>
)

# 5) Force the Eigen “freestanding” define
add_compile_definitions(EIGEN_FREESTANDING=1)

# 6) Specify source files
set(SRC_FILES
    attTrackingErrorAlgorithm.cpp
    architecture/utilities/avsEigenSupport.cpp
)

# 7) Declare the static library target (produces libattTrackingError.a)
add_library(attTrackingError STATIC ${SRC_FILES})

# 8) Place the resulting .a into build/lib/
set_target_properties(attTrackingError PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
)

# 9) Usage instructions:
#
#    mkdir build
#    cd build
#    cmake -DCMAKE_BUILD_TYPE=Release \
#          -DCMAKE_TOOLCHAIN_FILE=../riscv32-toolchain.cmake \
#          ..
#    make
#
# After building, you will have:
#    build/lib/libattTrackingError.a
#
# Point your Ada project’s Library_Dir to "build/lib" and link "-l:libattTrackingError.a"
# plus "-lstdc++", "-lm", "-lgcc" as needed.
