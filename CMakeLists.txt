cmake_minimum_required(VERSION 3.15)
project(LidarRoomViewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(src)
include_directories(external/glad/include)
include_directories(external/glfw-3.4/include)
include_directories(external/glm)

include_directories(external/imgui)
include_directories(external/imgui/backends)

file(GLOB SOURCES "src/*.cpp" "src/*.h")

set(IMGUI_SOURCES
    external/imgui/imgui.cpp
    external/imgui/imgui_draw.cpp
    external/imgui/imgui_tables.cpp
    external/imgui/imgui_widgets.cpp

    external/imgui/backends/imgui_impl_glfw.cpp
    external/imgui/backends/imgui_impl_opengl3.cpp
)

add_executable(LidarRoomViewer ${SOURCES} ${IMGUI_SOURCES})

add_library(glad STATIC external/glad/src/glad.c)
target_link_libraries(LidarRoomViewer glad)

add_subdirectory(external/glfw-3.4)
target_link_libraries(LidarRoomViewer glfw)

if (WIN32)
    target_link_libraries(LidarRoomViewer opengl32)
elseif (APPLE)
    find_library(OpenGL_LIBRARY OpenGL)
    target_link_libraries(LidarRoomViewer ${OpenGL_LIBRARY})
else()
    target_link_libraries(LidarRoomViewer GL dl pthread X11 Xrandr Xi)
endif()

find_package(Python3 COMPONENTS Interpreter)

if (Python3_FOUND)
    add_custom_target(GenerateTSDFMesh
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/tsdf_range_fusion/tsdf_fuse_organized.py
                --detail-preset ultra
                --fill-holes-size 0.35
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating TSDF mesh from scan_output/room_scan.csv"
    )
endif()
