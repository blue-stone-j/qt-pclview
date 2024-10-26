cmake_minimum_required(VERSION 3.10)
project(qt_pclview)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# set(CMAKE_CXX_FLAGS "-O2 -Wall -g ${CMAKE_CXX_FLAGS}")

include(FindPkgConfig)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)

find_package(PCL REQUIRED)

EXECUTE_PROCESS(
  COMMAND pkg-config --variable=host_bins Qt5Core
  OUTPUT_VARIABLE Qt5BIN
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(
  include

  ${PCL_INCLUDE_DIRS}
)

qt5_wrap_cpp(MOC_FILES include/mainwindow.h)

add_executable(qt_pclview
  src/main.cpp
  src/mainwindow.cpp
  viewer/pclview.cpp
  ${MOC_FILES}
)

target_link_libraries(qt_pclview

  ${PCL_LIBRARIES}
  "libvtkGUISupportQt-7.1.so"

  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::OpenGL)
