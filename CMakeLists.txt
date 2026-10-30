cmake_minimum_required(VERSION 3.22)
# Release version: the single place to change it (window titles and .exe file properties use it)
project(BreakoutCNC VERSION 0.2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(PROJECT_VERSION_PATCH STREQUAL "")
    set(BREAKOUTCNC_VERSION_PATCH 0)
else()
    set(BREAKOUTCNC_VERSION_PATCH ${PROJECT_VERSION_PATCH})
endif()
configure_file(cmake/breakout_version.h.in ${CMAKE_CURRENT_BINARY_DIR}/generated/breakout_version.h @ONLY)

# Link the C/C++ runtime statically on MSVC (all targets, including fetched libraries) so the release zip runs
# without the Visual C++ Redistributable installed
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
set(USE_MSVC_RUNTIME_LIBRARY_DLL OFF CACHE BOOL "" FORCE) # GLFW's own runtime switch

include(FetchContent)

# 1. Fetch GLFW 3.4 (Windowing + Input)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(glfw)

# 2. Fetch Dear ImGui (Docking Branch)
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    # Pinned to an exact docking-branch commit (1.93.1 WIP) so builds are reproducible
    GIT_TAG        d88f238d1d363ef387382b2e29ad12cf96f826ac
)
FetchContent_MakeAvailable(imgui)

# 2b. Fetch Lua 5.4 (Embedded Scripting Engine)
FetchContent_Declare(
    lua
    GIT_REPOSITORY https://github.com/lua/lua.git
    GIT_TAG        v5.4.7
    GIT_SHALLOW    TRUE
)
# Lua has no CMakeLists.txt, so this only downloads it; the library is defined below
FetchContent_MakeAvailable(lua)
file(GLOB LUA_SOURCES "${lua_SOURCE_DIR}/*.c")
list(REMOVE_ITEM LUA_SOURCES
    "${lua_SOURCE_DIR}/lua.c"
    "${lua_SOURCE_DIR}/luac.c"
    "${lua_SOURCE_DIR}/onelua.c"
    "${lua_SOURCE_DIR}/ltests.c"
)
# Compile Lua as C++: lua_error()/luaL_error() then throw C++ exceptions instead of longjmp, so
# errors raised inside the cnc.* C++ bindings unwind correctly (destructors run, locks are released).
set_source_files_properties(${LUA_SOURCES} PROPERTIES LANGUAGE CXX)
add_library(lua_lib STATIC ${LUA_SOURCES})
target_include_directories(lua_lib PUBLIC ${lua_SOURCE_DIR})
if(MSVC)
    target_compile_definitions(lua_lib PRIVATE _CRT_SECURE_NO_WARNINGS)
elseif(UNIX)
    target_compile_definitions(lua_lib PRIVATE LUA_USE_POSIX) # mkstemp instead of tmpnam, POSIX popen
endif()

find_package(OpenGL REQUIRED)

# 3. Build ImGui + GLFW + OpenGL3 Static Library
add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw OpenGL::GL)

# 4. Shared Hardware, G-Code & Trajectory Planner Engine (rnr_core)
add_library(rnr_core STATIC
    src/core/hid/rnr_usb_device.cpp
    src/core/gcode/gcode_parser.cpp
    src/core/planner/trajectory_planner.cpp
    src/core/controller/machine_controller.cpp
    src/core/board/rnr_board_driver.cpp
    src/core/config/settings_manager.cpp
    src/core/config/mach3_xml_importer.cpp
    src/core/spindle/huanyang_vfd.cpp
    src/core/scripting/script_engine.cpp
    src/core/scripting/script_api.cpp
    src/core/scripting/script_runner.cpp
    src/core/camera/camera_manager.cpp
    src/core/camera/camera_stream_server.cpp
)
target_include_directories(rnr_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(rnr_core PUBLIC lua_lib)

# Platform-specific dependencies for rnr_core
if(WIN32)
    target_link_libraries(rnr_core PUBLIC hid setupapi ws2_32 mfplat mf mfreadwrite mfuuid ole32)
else()
    # Cross-platform USB HID via hidapi on Linux / macOS
    FetchContent_Declare(
        hidapi
        GIT_REPOSITORY https://github.com/libusb/hidapi.git
        GIT_TAG        hidapi-0.14.0
        GIT_SHALLOW    TRUE
    )
    set(HIDAPI_BUILD_HIDTEST OFF CACHE BOOL "" FORCE)
    # hidapi 0.14 declares cmake_minimum_required(3.1.3), which CMake 4 rejects; let it configure with the
    # 3.5 policies (the variable is ignored by CMake 3.x)
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
    FetchContent_MakeAvailable(hidapi)
    unset(CMAKE_POLICY_VERSION_MINIMUM)
    target_link_libraries(rnr_core PUBLIC hidapi::hidapi)
endif()

if(APPLE)
    find_library(IOKIT_FRAMEWORK IOKit REQUIRED)
    find_library(COREFOUNDATION_FRAMEWORK CoreFoundation REQUIRED)
    find_library(APPKIT_FRAMEWORK AppKit REQUIRED)
    target_link_libraries(rnr_core PUBLIC ${IOKIT_FRAMEWORK} ${COREFOUNDATION_FRAMEWORK})
elseif(UNIX AND NOT WIN32)
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(LIBUDEV libudev)
    endif()
    if(LIBUDEV_FOUND)
        target_link_libraries(rnr_core PUBLIC ${LIBUDEV_LIBRARIES})
        target_include_directories(rnr_core PUBLIC ${LIBUDEV_INCLUDE_DIRS})
    endif()
    target_link_libraries(rnr_core PUBLIC pthread)
endif()

# 5. Shared UI Components (rnr_ui_common)
add_library(rnr_ui_common STATIC
    src/ui_common/app_window.cpp
    src/ui_common/toolpath_viewport.cpp
    src/ui_common/dro_panel.cpp
    src/ui_common/io_status_bar.cpp
    src/ui_common/gcode_panel.cpp
    src/ui_common/scripting_panel.cpp
    src/ui_common/probe_panel.cpp
    src/ui_common/workspace_panel.cpp
    src/ui_common/file_dialog.cpp
    src/ui_common/mach3_migration_dialog.cpp
    src/ui_common/camera_panel.cpp
    src/ui_common/tool_table_panel.cpp
    src/ui_common/lua_guide.cpp
)
target_include_directories(rnr_ui_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(rnr_ui_common PUBLIC rnr_core imgui_lib)
if(WIN32)
    target_link_libraries(rnr_ui_common PUBLIC dwmapi comdlg32 shell32)
elseif(APPLE)
    target_link_libraries(rnr_ui_common PUBLIC ${APPKIT_FRAMEWORK})
endif()

# 6. CLI Live Hardware Diagnostic Tool (rnr_hid_test)
add_executable(rnr_hid_test tests/rnr_hid_test.cpp)
target_link_libraries(rnr_hid_test PRIVATE rnr_core)

# 6b. Huanyang VFD Protocol & Driver Unit Test
add_executable(huanyang_vfd_test tests/huanyang_vfd_test.cpp)
target_link_libraries(huanyang_vfd_test PRIVATE rnr_core)

# 6c. Script Engine & Lua 5.4 API Unit Test
add_executable(script_engine_test tests/script_engine_test.cpp)
target_link_libraries(script_engine_test PRIVATE rnr_core)
target_include_directories(script_engine_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src) # ui_common/lua_guide_examples.h (header only)

# 7. Executable: BreakoutCut (Dual-Mode Plasma & Laser Cutter Controller)
set(BREAKOUT_CUT_SRCS src/app_cut/main_cut.cpp)
if(WIN32)
    list(APPEND BREAKOUT_CUT_SRCS resources/breakout_cut.rc)
endif()
add_executable(BreakoutCut ${BREAKOUT_CUT_SRCS})
target_link_libraries(BreakoutCut PRIVATE rnr_ui_common)
if(WIN32)
    target_link_libraries(BreakoutCut PRIVATE comdlg32 shell32)
endif()
if(MSVC)
    target_link_options(BreakoutCut PRIVATE "$<$<CONFIG:Release>:/SUBSYSTEM:WINDOWS;/ENTRY:mainCRTStartup>")
endif()

# 8. Executable: BreakoutMill (4-Axis CNC Mill Controller)
set(BREAKOUT_MILL_SRCS src/app_mill/main_mill.cpp)
if(WIN32)
    list(APPEND BREAKOUT_MILL_SRCS resources/breakout_mill.rc)
endif()
add_executable(BreakoutMill ${BREAKOUT_MILL_SRCS})
target_link_libraries(BreakoutMill PRIVATE rnr_ui_common)
if(WIN32)
    target_link_libraries(BreakoutMill PRIVATE comdlg32 shell32)
endif()
if(MSVC)
    target_link_options(BreakoutMill PRIVATE "$<$<CONFIG:Release>:/SUBSYSTEM:WINDOWS;/ENTRY:mainCRTStartup>")
endif()

# 9. Deploy scripts & resources directory to build output
add_custom_target(deploy_scripts ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts
        $<IF:$<CONFIG:Debug>,${CMAKE_CURRENT_BINARY_DIR}/Debug/scripts,${CMAKE_CURRENT_BINARY_DIR}/Release/scripts>
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts
        ${CMAKE_CURRENT_BINARY_DIR}/scripts
    COMMENT "Deploying scripts folder to build output"
)

add_custom_target(deploy_resources ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/resources
        $<IF:$<CONFIG:Debug>,${CMAKE_CURRENT_BINARY_DIR}/Debug/resources,${CMAKE_CURRENT_BINARY_DIR}/Release/resources>
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/resources
        ${CMAKE_CURRENT_BINARY_DIR}/resources
    COMMENT "Deploying resources folder to build output"
)
add_dependencies(BreakoutMill deploy_scripts deploy_resources)
add_dependencies(BreakoutCut deploy_scripts deploy_resources)

# 10. Release package: cmake --build build --config Release --target package -> build/BreakoutCNC-<version>-win64.zip
# Only what end users need: no test / diagnostic executables, settings files or libraries.
install(TARGETS BreakoutMill BreakoutCut RUNTIME DESTINATION .)
install(DIRECTORY scripts/ DESTINATION scripts)
install(DIRECTORY resources/icons/ DESTINATION resources/icons)
install(FILES README.md LICENSE CHANGELOG.md THIRD_PARTY_NOTICES.md DESTINATION .)
install(FILES docs/HARDWARE_VALIDATION_CHECKLIST.md DESTINATION docs)

set(CPACK_GENERATOR ZIP)
set(CPACK_PACKAGE_NAME BreakoutCNC)
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
if(WIN32)
    set(CPACK_SYSTEM_NAME win64)
endif()
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE)
include(CPack)
