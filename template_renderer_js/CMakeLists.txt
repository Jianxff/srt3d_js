cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0072 NEW)
cmake_policy(SET CMP0079 NEW)

project(renderer LANGUAGES CXX C)

# CXX Standard
# =============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")


# Emscripten
# =============================================================================
## emscripten settings
set(COMPILE_FLAG_TEXT "-O3 -fexceptions")
set(LINK_FLAG_TEXT " \
    -s USE_ZLIB=1 \
    -s DYNAMIC_EXECUTION=0 \
    -s INITIAL_MEMORY=256MB \
    -s NO_DISABLE_EXCEPTION_CATCHING=1 \
    -s ERROR_ON_UNDEFINED_SYMBOLS=0 \
    -s ALLOW_MEMORY_GROWTH=1 \
    -s ALLOW_TABLE_GROWTH=1 \
    -s MODULARIZE=1 \
    -s EXPORT_NAME=initRenderer \
    -s SINGLE_FILE=1 \
    -s ENVIRONMENT=web,worker,shell \
    -s USE_PTHREADS=0 \
    -s SHARED_MEMORY=0 \
    -s FORCE_FILESYSTEM=1 \
    -s EXPORTED_RUNTIME_METHODS=['FS'] \
    -fexceptions \
    -O3 --no-entry --no-check-features --bind \
    -s MIN_WEBGL_VERSION=2 \
    -lGL --js-library /data/jxf/shared/emscripten-glfw/src/js/lib_emscripten_glfw3.js \
    --extern-post-js ${CMAKE_CURRENT_SOURCE_DIR}/src/export.js"
)

## emscripten debug
# debug on
set(CMAKE_BUILD_TYPE "Debug")
set(LINK_FLAG_TEXT "${LINK_FLAG_TEXT} -s ASSERTIONS=1")
# debug off
# set(CMAKE_BUILD_TYPE "Release")
# set(COMPILE_FLAG_TEXT "${COMPILE_FLAG_TEXT} -g0")
# set(LINK_FLAG_TEXT "${LINK_FLAG_TEXT} -g0 -s ASSERTIONS=0")


# Libraries
# =============================================================================
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
# find_package(GLEW REQUIRED)
# find_package(glfw3 3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(OpenMP)
if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


# Compiler Settings
# =============================================================================
# Default settings
# set(CMAKE_CXX_VISIBILITY_PRESET hidden)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-march HAS_MARCH)
check_cxx_compiler_flag(-mtune HAS_MTUNE)
if(HAS_MARCH)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()
if(HAS_MTUNE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native")
endif()


# Compiling
# =============================================================================
FILE(GLOB_RECURSE FILES_CPP "${PROJECT_SOURCE_DIR}/src/*.cpp")

## executable
add_executable(${PROJECT_NAME}
    ${FILES_CPP}
    ${PROJECT_SOURCE_DIR}/main.cpp
)

## include
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/third_party
        /data/jxf/shared/emscripten-glfw/include
        ${OPENGL_INCLUDE_DIRS}
)

## linking
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        ${OpenCV_LIBS}
        ${OPENGL_LIBRARIES}
        /data/jxf/shared/emscripten-glfw/build/libglfw3.a
)

## output
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "${LINK_FLAG_TEXT}")
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${COMPILE_FLAG_TEXT}")


# add export
# add_custom_command(TARGET ${PROJECT_NAME}
#     POST_BUILD
#     COMMAND cp ${CMAKE_BINARY_DIR}/${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/../example/
#     VERBATIM
# )