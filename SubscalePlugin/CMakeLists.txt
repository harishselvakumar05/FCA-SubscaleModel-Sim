
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set(CMAKE_AUTOMOC ON)
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

find_package (Qt5Widgets REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(Qt5Core REQUIRED)




include_directories(${GAZEBO_INCLUDE_DIRS} 
			    ${Qt5Core_INCLUDE_DIRS} 
			    ${SDF_INCLUDE_DIRS} 
			    ${Boost_LIBRARIES}
			    
			    ${CMAKE_CURRENT_BINARY_DIR}/msgs)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)



set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
add_library(SubscalePlugin SHARED ${headers_MOC} SubscalePlugin.cc)
target_link_libraries(SubscalePlugin   ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${SDF_LIBRARIES})
target_link_libraries(SubscalePlugin   SubscaleGUIPlugin_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(SubscalePlugin SubscaleGUIPlugin_msgs)

QT5_WRAP_CPP(headers_MOC SubscaleGUIPlugin.hh)


add_library(SubscaleGUIPlugin SHARED  ${headers_MOC} SubscaleGUIPlugin.cc)
target_link_libraries(SubscaleGUIPlugin SubscaleGUIPlugin_msgs
	${GAZEBO_LIBRARIES} 
    	${PROTOBUF_LIBRARIES}
    	${Qt5Core_LIBRARIES}
    	${Qt5Test_LIBRARIES}
    	${Qt5Widgets_LIBRARIES}
    	${Boost_LIBRARIES}
    	${SDF_LIBRARIES})
    	
target_link_libraries(SubscaleGUIPlugin
	${GAZEBO_LIBRARIES} 
    	${PROTOBUF_LIBRARIES}
    	${Qt5Core_LIBRARIES}
    	${Qt5Test_LIBRARIES}
    	${Qt5Widgets_LIBRARIES}
    	${Boost_LIBRARIES}
    	${SDF_LIBRARIES})
add_dependencies(SubscaleGUIPlugin SubscaleGUIPlugin_msgs)




