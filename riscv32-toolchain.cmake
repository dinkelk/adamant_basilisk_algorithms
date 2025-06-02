# Toolchain file to tell CMake to use riscv32-elf-gcc/g++
# from GNAT Pro (or any other riscv32-elf‐GCC installation) for cross-compiling.
#
# We also force CMake’s “try_compile” to build a STATIC_LIBRARY
# (so it never tries to link a full executable and fail on missing crt0.o).

# 1) Indicate that we are cross-compiling for a generic (no-OS) target.
set(CMAKE_SYSTEM_NAME Generic   CACHE STRING "Cross compile for RISC-V 32" FORCE)

# 2) Point at the RISC-V compiler wrappers (assumed to come from GNAT Pro).
#    If your riscv32-elf-gcc / riscv32-elf-g++ are in a custom folder,
#    replace these with absolute paths.
set(CMAKE_C_COMPILER   riscv32-elf-gcc   CACHE STRING "" FORCE)
set(CMAKE_CXX_COMPILER riscv32-elf-g++   CACHE STRING "" FORCE)

# 3) Instruct CMake to perform its try_compile() tests by building a STATIC_LIBRARY
#    instead of an executable. That way, it never tries to link against crt0.o or libgloss.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# (Optional) If GNAT Pro installed its own sysroot in a nonstandard place, you can
# hint CMake to look there. For example, if GNAT Pro’s cross-runtime lives in
# /opt/gnatpro/riscv32-elf/lib, you might do:
#
#   set(CMAKE_FIND_ROOT_PATH /opt/gnatpro/riscv32-elf)
#
# But often GNAT Pro’s riscv32-elf-g++ already knows its own search paths.
