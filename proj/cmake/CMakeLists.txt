cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(AICPHelper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CINDER_VERBOSE ON)

find_package(PCL 1.8 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(APPLE)
elseif(LINUX)
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
endif()

find_package(OpenCV 3.0 REQUIRED)
find_package(Boost COMPONENTS system filesystem program_options thread REQUIRED)

get_filename_component(CINDER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../cinder/Cinder" ABSOLUTE)
get_filename_component(PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

set(CINDER_IMGUI_DIR ${PROJECT_DIR}/blocks/ImGui)

include_directories(${CINDER_IMGUI_DIR}/lib/imgui)
include_directories(${CINDER_IMGUI_DIR}/include)

set(CINDER_IMGUI_SOURCES
  ${CINDER_IMGUI_DIR}/lib/imgui/imgui.cpp
  ${CINDER_IMGUI_DIR}/lib/imgui/imgui_draw.cpp
  ${CINDER_IMGUI_DIR}/lib/imgui/imgui_demo.cpp
  ${CINDER_IMGUI_DIR}/src/CinderImGui.cpp
)

set(PROJECT_SOURCES
  ${PROJECT_DIR}/src/AICPHelperApp.cpp
  ${PROJECT_DIR}/src/AppGui.cpp
  ${PROJECT_DIR}/src/Clouds.cpp
  ${CINDER_IMGUI_SOURCES}
)

include("${CINDER_PATH}/proj/cmake/modules/cinderMakeApp.cmake")

ci_make_app(
  SOURCES       ${PROJECT_SOURCES}
  INCLUDES      ${PROJECT_DIR}/include ${PCL_INCLUDE_DIRS}
  CINDER_PATH   ${CINDER_PATH}
  LIBRARIES     ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} yaml-cpp
)
