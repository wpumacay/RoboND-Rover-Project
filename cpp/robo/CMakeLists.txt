cmake_minimum_required( VERSION 3.0 )

project( roboPerceptionCpp C CXX )

set( CMAKE_BUILD_TYPE Debug )

find_package( OpenCV REQUIRED )

include_directories( perception/ )

add_executable( sample_1.out sample_1.cpp )
add_executable( sample_2.out sample_2.cpp )
add_executable( main.out main.cpp )
add_executable( sample_socketio main_socketio.cpp )

target_link_libraries( sample_1.out ${OpenCV_LIBS} )
target_link_libraries( sample_2.out ${OpenCV_LIBS} )
target_link_libraries( main.out ${OpenCV_LIBS} )
