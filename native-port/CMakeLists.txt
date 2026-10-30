cmake_minimum_required(VERSION 3.25)
project(ac5_native VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(FILES LICENSE
    DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/ac5_native")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(AC5_BUILD_RENDERER "Build the SDL3/Vulkan native renderer" ON)

add_library(ac5_asset_core STATIC
    src/adpcm.cpp
    src/data_archive.cpp
    src/gs_image.cpp
    src/ulz.cpp)
add_library(ac5::asset_core ALIAS ac5_asset_core)
set_target_properties(ac5_asset_core PROPERTIES EXPORT_NAME asset_core)
target_include_directories(ac5_asset_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ac5_native>")
target_compile_features(ac5_asset_core PUBLIC cxx_std_20)

add_library(ac5_game_core STATIC
    src/model_shading.cpp
    src/obj_loader.cpp
    src/texture_loader.cpp)
add_library(ac5::game_core ALIAS ac5_game_core)
set_target_properties(ac5_game_core PROPERTIES EXPORT_NAME game_core)
target_include_directories(ac5_game_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ac5_native>")
target_compile_features(ac5_game_core PUBLIC cxx_std_20)

add_executable(ac5_extract src/extract_main.cpp)

enable_testing()
add_executable(ac5_native_tests tests/native_tests.cpp)
target_link_libraries(ac5_native_tests PRIVATE ac5_asset_core ac5_game_core)
add_test(NAME ac5_native_tests COMMAND ac5_native_tests)
target_link_libraries(ac5_extract PRIVATE ac5_asset_core)
if(WIN32)
    target_link_libraries(ac5_extract PRIVATE shell32)
endif()

install(TARGETS ac5_asset_core ac5_game_core
    EXPORT ac5NativeCoreTargets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(TARGETS ac5_extract
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(FILES
    src/adpcm.hpp
    src/data_archive.hpp
    src/gs_image.hpp
    src/model_shading.hpp
    src/ulz.hpp
    src/math.hpp
    src/scene.hpp
    src/scene_state_machine.hpp
    src/selection.hpp
    src/flight_input.hpp
    src/obj_loader.hpp
    src/texture_loader.hpp
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ac5_native")

set(AC5_NATIVE_CMAKE_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/ac5_native")
configure_package_config_file(
    cmake/ac5_nativeConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/ac5_nativeConfig.cmake"
    INSTALL_DESTINATION "${AC5_NATIVE_CMAKE_DIR}")
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/ac5_nativeConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion)
install(EXPORT ac5NativeCoreTargets
    FILE ac5_nativeCoreTargets.cmake
    NAMESPACE ac5::
    DESTINATION "${AC5_NATIVE_CMAKE_DIR}")
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/ac5_nativeConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/ac5_nativeConfigVersion.cmake"
    DESTINATION "${AC5_NATIVE_CMAKE_DIR}")

if(AC5_BUILD_RENDERER)
    find_package(Vulkan 1.3 REQUIRED)
    find_package(SDL3 CONFIG REQUIRED)
    find_program(GLSLC glslc HINTS "$ENV{VULKAN_SDK}/Bin" "$ENV{VULKAN_SDK}/bin" REQUIRED)

    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/assets")
    configure_file(assets/demo.obj "${CMAKE_CURRENT_BINARY_DIR}/assets/demo.obj" COPYONLY)
    configure_file(assets/demo.mtl "${CMAKE_CURRENT_BINARY_DIR}/assets/demo.mtl" COPYONLY)
    configure_file(assets/demo.ppm "${CMAKE_CURRENT_BINARY_DIR}/assets/demo.ppm" COPYONLY)

    add_library(ac5_vulkan_renderer STATIC src/vulkan_renderer.cpp)
    add_library(ac5::vulkan_renderer ALIAS ac5_vulkan_renderer)
    set_target_properties(ac5_vulkan_renderer PROPERTIES EXPORT_NAME vulkan_renderer)
    target_include_directories(ac5_vulkan_renderer PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ac5_native>")
    target_compile_features(ac5_vulkan_renderer PUBLIC cxx_std_20)
    target_link_libraries(ac5_vulkan_renderer
        PUBLIC Vulkan::Vulkan SDL3::SDL3)
    add_library(ac5_platform STATIC src/input.cpp)
    add_library(ac5::platform ALIAS ac5_platform)
    set_target_properties(ac5_platform PROPERTIES EXPORT_NAME platform)
    target_include_directories(ac5_platform PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ac5_native>")
    target_compile_features(ac5_platform PUBLIC cxx_std_20)
    target_link_libraries(ac5_platform PUBLIC SDL3::SDL3)
    add_executable(ac5_native
        src/main.cpp
        src/application.cpp)

    target_include_directories(ac5_native PRIVATE src)
    target_link_libraries(ac5_native PRIVATE
        ac5_vulkan_renderer ac5_platform ac5_asset_core ac5_game_core)
    if(WIN32)
        target_link_libraries(ac5_native PRIVATE shell32)
    endif()

    install(TARGETS ac5_vulkan_renderer ac5_platform
        EXPORT ac5NativeRendererTargets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
    install(TARGETS ac5_native
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
    install(FILES src/vulkan_renderer.hpp src/input.hpp
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ac5_native")

    set(AC5_NATIVE_RENDERER_CMAKE_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/ac5_native_renderer")
    configure_package_config_file(
        cmake/ac5_nativeRendererConfig.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/ac5_native_rendererConfig.cmake"
        INSTALL_DESTINATION "${AC5_NATIVE_RENDERER_CMAKE_DIR}")
    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/ac5_native_rendererConfigVersion.cmake"
        VERSION "${PROJECT_VERSION}"
        COMPATIBILITY SameMajorVersion)
    install(EXPORT ac5NativeRendererTargets
        FILE ac5_native_rendererTargets.cmake
        NAMESPACE ac5::
        DESTINATION "${AC5_NATIVE_RENDERER_CMAKE_DIR}")
    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/ac5_native_rendererConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/ac5_native_rendererConfigVersion.cmake"
        DESTINATION "${AC5_NATIVE_RENDERER_CMAKE_DIR}")

    set(SHADER_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders")
    file(MAKE_DIRECTORY "${SHADER_OUTPUT_DIR}")
    set(SHADER_OUTPUTS)
    foreach(SHADER mesh.vert mesh.frag)
        set(OUTPUT "${SHADER_OUTPUT_DIR}/${SHADER}.spv")
        add_custom_command(
            OUTPUT "${OUTPUT}"
            COMMAND "${GLSLC}" -O "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${SHADER}"
                    -o "${OUTPUT}"
            DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${SHADER}"
            VERBATIM)
        list(APPEND SHADER_OUTPUTS "${OUTPUT}")
    endforeach()
    add_custom_target(ac5_native_shaders DEPENDS ${SHADER_OUTPUTS})
    add_dependencies(ac5_native ac5_native_shaders)
    install(FILES ${SHADER_OUTPUTS}
        DESTINATION "${CMAKE_INSTALL_BINDIR}/shaders")
    install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/assets/"
        DESTINATION "${CMAKE_INSTALL_BINDIR}/assets")
endif()

if(MSVC)
    target_compile_options(ac5_asset_core PRIVATE /W4 /permissive-)
    target_compile_options(ac5_game_core PRIVATE /W4 /permissive-)
    target_compile_options(ac5_extract PRIVATE /W4 /permissive-)
    if(AC5_BUILD_RENDERER)
        target_compile_options(ac5_vulkan_renderer PRIVATE /W4 /permissive-)
        target_compile_options(ac5_platform PRIVATE /W4 /permissive-)
        target_compile_options(ac5_native PRIVATE /W4 /permissive-)
    endif()
else()
    target_compile_options(ac5_asset_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ac5_game_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ac5_extract PRIVATE -Wall -Wextra -Wpedantic)
    if(AC5_BUILD_RENDERER)
        target_compile_options(ac5_vulkan_renderer PRIVATE -Wall -Wextra -Wpedantic)
        target_compile_options(ac5_platform PRIVATE -Wall -Wextra -Wpedantic)
        target_compile_options(ac5_native PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()
