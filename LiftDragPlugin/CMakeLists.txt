cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)

find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_library(LiftDragPlugin SHARED ${headers_MOC} LiftDragPlugin.cc)
target_link_libraries(LiftDragPlugin ${GAZEBO_LIBRARIES} 
    	${PROTOBUF_LIBRARIES})
