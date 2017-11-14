cmake_minimum_required( VERSION 3.5.0 )

project( robo_io C CXX )

set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_CXX_STANDARD 11 )

link_directories( /usr/local/lib )

add_executable( robo_io
				main.cpp )

target_link_libraries( robo_io
					   sioclient
					   boost_system
					   pthread )