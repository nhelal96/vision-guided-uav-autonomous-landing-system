cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project(qr_landing)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
)

find_package(PkgConfig REQUIRED)
pkg_search_module(ZBAR REQUIRED zbar)


find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES SafeLanding
  CATKIN_DEPENDS OpenCV cv_bridge roscpp image_transport
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${realsense_INCLUDE_DIR}
  /usr/local/include
)

link_directories(/usr/local/lib)

set(sources
  include/qr_landing/algorithm.h
  include/qr_landing/image_processor.h
  include/qr_landing/helper.h
  include/qr_landing/communicator.h
  src/helper.cpp
  src/algorithm.cpp
  src/image_processor.cpp
  src/communicator.cpp
  src/qrlanding_node.cpp
)

add_library(${PROJECT_NAME}
   ${sources}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node ${sources})

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

target_include_directories(${PROJECT_NAME}
  PRIVATE ${realsense_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS} ${ZBAR_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_node
  realsense2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${ZBAR_LIBRARIES}
)

install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
