cmake_minimum_required (VERSION 2.6)
project (RHA C CXX)

SET(LIB_PATH
	/usr/lib/arm-linux-gnueabihf/
	${CMAKE_SOURCE_DIR}/lib/
)

SET(LIB
	pthread
	RHASPDUINO
	wiringPi
)

##### Raspberry Pi version sources path #####
SET(RHASPDUINO_PATH
	${CMAKE_SOURCE_DIR}/RHAspberry_lib/external/RHASPDUINO
	${CMAKE_SOURCE_DIR}/RHAspberry_lib/external/Wire
	${CMAKE_SOURCE_DIR}/RHAspberry_lib/external/Wire/utility
	${CMAKE_SOURCE_DIR}/RHAspberry_lib/gci
	${CMAKE_SOURCE_DIR}/RHAspberry_lib/logger
)

##### Raspberry Pi version sources #####
SET(RHASPDUINO_CPP_FILES
	#RHA Raspberry Pi version libs:
	${CMAKE_SOURCE_DIR}/RHAspberry_lib/gci/gci.cpp
	${CMAKE_SOURCE_DIR}/RHAspberry_lib/logger/logger.cpp
	# From External lib:
	${CMAKE_SOURCE_DIR}/RHAspberry_lib/external/Wire/Wire.cpp
	${CMAKE_SOURCE_DIR}/RHAspberry_lib/external/Wire/utility/twi.c
)

##### RHA Arduino version sources path #####
SET(RHA_LIB_PATH
	${CMAKE_SOURCE_DIR}/lib/
	${CMAKE_SOURCE_DIR}/lib/chuck_handler
	${CMAKE_SOURCE_DIR}/lib/cytron_g15_servo
	${CMAKE_SOURCE_DIR}/lib/debug
	${CMAKE_SOURCE_DIR}/lib/joint_handler
	${CMAKE_SOURCE_DIR}/lib/joint_rha
	${CMAKE_SOURCE_DIR}/lib/rha_types
	${CMAKE_SOURCE_DIR}/lib/robot_rha
	${CMAKE_SOURCE_DIR}/lib/servo_rha
	${CMAKE_SOURCE_DIR}/lib/utilities
)

##### RHA Arduino version sources #####
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

include_directories(${RHA_LIB_PATH} ${RHASPDUINO_PATH} ${LIB_PATH})

link_directories(${RHA_LIB_PATH} ${RHASPDUINO_PATH} ${LIB_PATH})

add_executable(${PROJECT_NAME}_node ${CMAKE_SOURCE_DIR}/RHAspberry_lib/external/RHASPDUINO/wiring_main.cpp 
				${CMAKE_SOURCE_DIR}/src/main.cpp 
				${RHA_CPP_FILES} 
				${RHASPDUINO_CPP_FILES}
) 

target_link_libraries(${PROJECT_NAME}_node ${LIB} "-lc -lm")




