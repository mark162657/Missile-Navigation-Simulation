cmake_minimum_required(VERSION 3.15)
project(missile_pathfinder)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 1. GENERIC PYTHON LOOKUP ---
# We do NOT set Python3_EXECUTABLE here.
# We let the local machine provide it via CLI or Environment.
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# --- 2. Smart Locator ---
# Uses whatever Python was found above
execute_process(
        COMMAND "${Python3_EXECUTABLE}" -c "import pybind11; print(pybind11.get_cmake_dir())"
        OUTPUT_VARIABLE _pybind11_path
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _pybind11_status
)

if("${_pybind11_status}" EQUAL 0 AND NOT "${_pybind11_path}" STREQUAL "")
    list(APPEND CMAKE_PREFIX_PATH "${_pybind11_path}")
else()
    message(WARNING "Could not auto-detect pybind11.")
endif()

find_package(pybind11 REQUIRED)
pybind11_add_module(missile_backend pathfinder.cpp)

if(MSVC)
    target_compile_options(missile_backend PRIVATE /O2 /fp:fast)
else()
    target_compile_options(missile_backend PRIVATE -O3 -march=native)
endif()

# Move the output file to src/guidance/ for Python import
add_custom_command(TARGET missile_backend POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:missile_backend>
        ${CMAKE_CURRENT_SOURCE_DIR}/../
)