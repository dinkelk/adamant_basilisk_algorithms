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

# 3) Native Linux flags (only when not cross-compiling)
if(NOT CMAKE_CROSSCOMPILING)
  if(BUILD_TYPE_UPPER STREQUAL "DEBUG")
    message(STATUS "Configuring native Linux Debug flags")
    set(CMAKE_C_FLAGS_DEBUG   "${CMAKE_C_FLAGS_DEBUG}   -O0 -g -ffreestanding" CACHE STRING "" FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -ffreestanding"  CACHE STRING "" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} " CACHE STRING "" FORCE)
  else()
    message(STATUS "Configuring native Linux Release flags")
    set(CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE}   -O3 -DNDEBUG" CACHE STRING "" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG" CACHE STRING "" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} " CACHE STRING "" FORCE)
  endif()
endif()

# 4) Eigen include (always /home/user/eigen) and project headers
set(EIGEN3_DIR "/home/user/eigen")
if(NOT EXISTS "${EIGEN3_DIR}/Eigen")
  message(FATAL_ERROR "Eigen not found under ${EIGEN3_DIR}")
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}                             # root headers
  ${PROJECT_SOURCE_DIR}/attTrackingError            # attitude tracking files
  ${PROJECT_SOURCE_DIR}/rateDamp                    # rate damping algorithm files
  ${PROJECT_SOURCE_DIR}/architecture/utilities      # avsEigenSupport.h, etc.
  ${PROJECT_SOURCE_DIR}/architecture/msgPayloadDefC  # messaging headers
  ${EIGEN3_DIR}                                     # Eigen headers
)

# 5) Force Eigen freestanding
add_compile_definitions(EIGEN_FREESTANDING=1)

# 6) Source files for static library
set(SRC_FILES
  attTrackingError/attTrackingErrorAlgorithm.cpp
  rateDamp/rateDampAlgorithm.cpp
  architecture/utilities/avsEigenSupport.cpp
  attTrackingError/attTrackingErrorAlgorithm_c.cpp   # attitude shim
  rateDamp/rateDampAlgorithm_c.cpp                   # rate damp shim
)

# 7) Build the static library (libgncAlgorithms.a)
add_library(gncAlgorithms STATIC ${SRC_FILES})

# Place the archive in build/lib/
set_target_properties(gncAlgorithms PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
)

target_compile_options(gncAlgorithms PRIVATE
  -include ${EIGEN3_DIR}/Eigen/src/Freestanding/all_freestanding.hpp
)

# 8) Generate Ada bindings (always, for both Linux and RISC-V)
set(BINDINGS_DIR "${CMAKE_BINARY_DIR}/bindings")
file(MAKE_DIRECTORY "${BINDINGS_DIR}")

set(ADA_HEADER1 "${PROJECT_SOURCE_DIR}/attTrackingError/attTrackingErrorAlgorithm_c.h")
set(ADA_SPEC1   "${BINDINGS_DIR}/attTrackingErrorAlgorithm_c.ads")

set(ADA_HEADER2 "${PROJECT_SOURCE_DIR}/rateDamp/rateDampAlgorithm_c.h")
set(ADA_SPEC2   "${BINDINGS_DIR}/rateDampAlgorithm_c.ads")

set(INCLUDE_FLAGS
  "-I${PROJECT_SOURCE_DIR}"
  "-I${PROJECT_SOURCE_DIR}/attTrackingError"
  "-I${PROJECT_SOURCE_DIR}/rateDamp"
  "-I${PROJECT_SOURCE_DIR}/architecture/utilities"
  "-I${PROJECT_SOURCE_DIR}/architecture/msgPayloadDefC"
  "-I${EIGEN3_DIR}"
)

add_custom_command(
  OUTPUT "${ADA_SPEC1}"
  WORKING_DIRECTORY "${BINDINGS_DIR}"
  COMMAND ${CMAKE_CXX_COMPILER}
          "-c"
          "-fdump-ada-spec"
          "-C" "${ADA_HEADER1}"
          ${INCLUDE_FLAGS}
          "-DEIGEN_FREESTANDING=1"
  COMMENT "Generating Ada spec: ${ADA_SPEC1}"
  VERBATIM
)

add_custom_command(
  OUTPUT "${ADA_SPEC2}"
  WORKING_DIRECTORY "${BINDINGS_DIR}"
  COMMAND ${CMAKE_CXX_COMPILER}
          "-c"
          "-fdump-ada-spec"
          "-C" "${ADA_HEADER2}"
          ${INCLUDE_FLAGS}
          "-DEIGEN_FREESTANDING=1"
  COMMENT "Generating Ada spec: ${ADA_SPEC2}"
  VERBATIM
)

add_custom_target(generate_ada_bindings ALL
  DEPENDS "${ADA_SPEC1}" "${ADA_SPEC2}"
)

add_dependencies(gncAlgorithms generate_ada_bindings)

# 9) Usage summary:
# Native Linux (Debug):
#   mkdir build_linux_debug && cd build_linux_debug
#   cmake -DCMAKE_BUILD_TYPE=Debug ..
#   make
#
# Native Linux (Release):
#   mkdir build_linux_release && cd build_linux_release
#   cmake -DCMAKE_BUILD_TYPE=Release ..
#   make
#
# RISC-V 32 (Debug, Ada bindings):
#   mkdir build_riscv_debug && cd build_riscv_debug
#   cmake -DCMAKE_BUILD_TYPE=Debug \
#         -DCMAKE_TOOLCHAIN_FILE=../riscv32-toolchain.cmake \
#         ..
#   make
#
# RISC-V 32 (Release, Ada bindings):
#   mkdir build_riscv_release && cd build_riscv_release
#   cmake -DCMAKE_BUILD_TYPE=Release \
#         -DCMAKE_TOOLCHAIN_FILE=../riscv32-toolchain.cmake \
#         ..
#   make
#
# The generated Ada specs are in <build_dir>/bindings/*.ads
# The static archive is in <build_dir>/lib/libgncAlgorithms.a
