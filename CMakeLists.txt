cmake_minimum_required (VERSION 2.6)
project (ball_trace)


add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	/usr/local/urc_utils/include/opencv2
	)
	
add_executable(ball_trace src/ball_trace.cpp)
target_link_libraries(ball_trace
	${OpenCV_LIBRARIES}
	${OpenCV_LIBS}
	/usr/local/lib
	)


