cmake_minimum_required (VERSION 2.6)
project (RHA C CXX)

SET(LIB_PATH
	/usr/lib/arm-linux-gnueabihf/
	${CMAKE_SOURCE_DIR}/lib/
)

SET(LIB
	pthread
	PiDuino
	wiringPi
)

SET(PIDUIO_PATH
	${CMAKE_SOURCE_DIR}/lib/piduino
	${CMAKE_SOURCE_DIR}/lib/Wire
	${CMAKE_SOURCE_DIR}/lib/Wire/utility
)

SET(PIDUINO_CPP_FILES
	${CMAKE_SOURCE_DIR}/lib/Wire/Wire.cpp
	${CMAKE_SOURCE_DIR}/lib/Wire/utility/twi.c
)

SET(RHA_LIB_PATH
	${CMAKE_SOURCE_DIR}/lib/
	${CMAKE_SOURCE_DIR}/lib/chuck_handler
	${CMAKE_SOURCE_DIR}/lib/cytron_g15_servo
	${CMAKE_SOURCE_DIR}/lib/debug
	${CMAKE_SOURCE_DIR}/lib/joint_handler
	${CMAKE_SOURCE_DIR}/lib/joint_rha
	${CMAKE_SOURCE_DIR}/lib/piduino
	${CMAKE_SOURCE_DIR}/lib/rha_types
	${CMAKE_SOURCE_DIR}/lib/robot_rha
	${CMAKE_SOURCE_DIR}/lib/servo_rha
	${CMAKE_SOURCE_DIR}/lib/utilities
)

SET(RHA_CPP_FILES
	${CMAKE_SOURCE_DIR}/lib/joint_handler/joint_handler.cpp
	${CMAKE_SOURCE_DIR}/lib/utilities/utilities.cpp
	${CMAKE_SOURCE_DIR}/lib/joint_rha/joint_rha.cpp
	${CMAKE_SOURCE_DIR}/lib/debug/debug.cpp
	${CMAKE_SOURCE_DIR}/lib/robot_rha/robot_rha.cpp
	${CMAKE_SOURCE_DIR}/lib/servo_rha/servo_rha.cpp
#	${CMAKE_SOURCE_DIR}/test/b_test_joint_rha/test_joint_rha.cpp
#	${CMAKE_SOURCE_DIR}/test/c_test_joint_handler_mock/test_joint_handler.cpp
#	${CMAKE_SOURCE_DIR}/test/a_test_servo_rha/test_servo_rha.cpp
)

SET(CMKE_BUILD_TYPE DEBUG)
SET(CMAKE_C_FLAGS "-fPIC -MD -Wall -Winline -pipe -g")

include_directories(${RHA_LIB_PATH} ${PIDUIO_PATH} ${LIB_PATH})

link_directories(${RHA_LIB_PATH} ${PIDUIO_PATH} ${LIB_PATH})

add_executable(${PROJECT_NAME}_node ${CMAKE_SOURCE_DIR}/lib/piduino/wiring_main.cpp 
									${CMAKE_SOURCE_DIR}/src/main.cpp 
									${RHA_CPP_FILES} 
									${PIDUINO_CPP_FILES}
) 

target_link_libraries(${PROJECT_NAME}_node ${LIB} "-lc -lm")




