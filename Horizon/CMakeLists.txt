cmake_minimum_required(VERSION 3.20)
project(Horizon VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(HR_BUILD_GAME "Build the Horizon driving demo" ON)
option(HR_BUILD_TESTS "Build unit tests" ON)
option(HR_BUILD_BENCHMARKS "Build benchmarks" ON)
option(HR_ENABLE_ASAN "Enable AddressSanitizer (debug only)" OFF)
option(HR_ENABLE_VALIDATION "Enable Validation Layers" OFF)

if(UNIX)
  add_compile_definitions(_POSIX_C_SOURCE=200809L)
endif()

find_package(Threads REQUIRED)
find_package(Vulkan REQUIRED)

if(HR_ENABLE_ASAN)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

set(HR_WARNINGS -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion -Wno-unused-parameter
  -Wno-missing-braces)
if(MSVC)
  set(HR_WARNINGS /W4 /permissive-)
else()
  set(HR_WARNINGS ${HR_WARNINGS} -Wno-missing-field-initializers)
endif()

set(HR_SHADERS
  scene.vert scene.frag shadow.vert sky.vert sky.frag
  fullscreen.vert bright.frag blur.frag post.frag)

set(HR_SPV_DIR ${CMAKE_BINARY_DIR}/shaders)
file(MAKE_DIRECTORY ${HR_SPV_DIR})
set(HR_SPV_FILES)
foreach(s ${HR_SHADERS})
  get_filename_component(name ${s} NAME_WE)
  get_filename_component(ext ${s} EXT)
  set(out ${HR_SPV_DIR}/${name}${ext}.spv)
  add_custom_command(
    OUTPUT ${out}
    COMMAND glslangValidator -V --target-env vulkan1.1 -o ${out} ${CMAKE_CURRENT_SOURCE_DIR}/shaders/${s}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/${s}
    COMMENT "GLSL -> SPIR-V: ${s}"
    VERBATIM)
  list(APPEND HR_SPV_FILES ${out})
endforeach()

set(HR_SHADER_EMBED ${CMAKE_BINARY_DIR}/generated/shaders_embedded.c)
set(HR_SHADER_EMBED_H ${CMAKE_BINARY_DIR}/generated/shaders_embedded.h)
add_custom_command(
  OUTPUT ${HR_SHADER_EMBED} ${HR_SHADER_EMBED_H}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/generated
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/embed_shaders.py
          ${CMAKE_BINARY_DIR}/generated ${HR_SPV_FILES}
  DEPENDS ${HR_SPV_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/tools/embed_shaders.py
  COMMENT "Embedding SPIR-V blobs"
  VERBATIM)

set(HR_ENGINE_SOURCES
  engine/src/core/core.c
  engine/src/core/math.c
  engine/src/core/memory.c
  engine/src/core/jobs.c
  engine/src/core/ecs.c
  engine/src/core/scene.c
  engine/src/animation/animation.c
  engine/src/physics/physics.c
  engine/src/platform/platform_linux.c
  engine/src/platform/platform_win32.c
  engine/src/renderer/vk_common.c
  engine/src/renderer/vk_device.c
  engine/src/renderer/vk_targets.c
  engine/src/renderer/vk_frame.c
  engine/src/renderer/mesh.c
  engine/src/renderer/textures.c
  ${HR_SHADER_EMBED})

add_library(horizon STATIC ${HR_ENGINE_SOURCES})
target_include_directories(horizon PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/engine/include
  ${CMAKE_BINARY_DIR}/generated)
target_compile_options(horizon PRIVATE ${HR_WARNINGS})
target_link_libraries(horizon PUBLIC Threads::Threads Vulkan::Vulkan)

if(UNIX AND NOT APPLE)
  find_package(X11 REQUIRED)
  target_include_directories(horizon PRIVATE ${X11_INCLUDE_DIR})
  target_link_libraries(horizon PRIVATE ${X11_LIBRARIES} m dl)
  target_compile_definitions(horizon PRIVATE HR_PLATFORM_LINUX=1)
elseif(WIN32)
  target_compile_definitions(horizon PRIVATE HR_PLATFORM_WINDOWS=1)
  target_link_libraries(horizon PRIVATE user32 gdi32 shell32)
else()
  message(FATAL_ERROR "Unsupported platform for this build")
endif()

if(HR_BUILD_GAME)
  add_executable(horizon_demo game/src/demo.c game/src/world.c game/src/world_mesh.c game/src/car.c)
  target_include_directories(horizon_demo PRIVATE engine/include)
  target_compile_options(horizon_demo PRIVATE ${HR_WARNINGS})
  target_link_libraries(horizon_demo PRIVATE horizon)
endif()

if(HR_BUILD_TESTS)
  enable_testing()
  add_executable(horizon_tests tests/test_core.c)
  target_link_libraries(horizon_tests PRIVATE horizon)
  target_compile_options(horizon_tests PRIVATE ${HR_WARNINGS})
  add_test(NAME horizon_tests COMMAND horizon_tests)
endif()

if(HR_BUILD_BENCHMARKS)
  add_executable(horizon_bench benchmarks/bench_core.c)
  target_link_libraries(horizon_bench PRIVATE horizon)
  target_compile_options(horizon_bench PRIVATE ${HR_WARNINGS})
endif()
