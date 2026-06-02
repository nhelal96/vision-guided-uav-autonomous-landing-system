cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project(safe_takeoff)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
)

 #find_package(OpenCV REQUIRED)


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
  include/safe_takeoff/algorithm.h
  include/safe_takeoff/image_processor.h
  include/safe_takeoff/helper.h
  include/safe_takeoff/communicator.h
  src/helper.cpp
  src/algorithm.cpp
  src/image_processor.cpp
  src/communicator.cpp
  src/safetakeoff_node.cpp
)

add_library(${PROJECT_NAME}
   ${sources}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node ${sources})

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

target_include_directories(${PROJECT_NAME}
  PRIVATE ${realsense_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME}_node
  realsense2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
