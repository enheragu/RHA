/**
 * @file
 * @brief Implements JointHandler functions defined in joint_handler.h
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_handler.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 08_Sep_2017
 */


#include "joint_handler.h"

/**
  * @brief
  */
JointHandler::JointHandler(uint64_t timer){
    timer_ = timer;
}

/**
  * @brief
  */
void JointHandler::init(uint64_t timer){
    timer_ = timer;
    joint_.init(1, CW, A0);
}

/**
  * @brief controlLoop() function handles control loop for servo speed
  */
void JointHandler::controlLoop(){
    if(millis() - time_last_ >= timer_){
        // Firt all info have to be updated
        joint_.updateInfo();

        // Ask each joint torque to update speed
        uint8_t error = joint_.speedError();
        uint16_t torque = joint_.regulatorJoint();

        // Add all servo to the buffer
        uint8_t buffer_add[30], goal[2], buffer_wrap[35];
        uint8_t position = 0;
        uint8_t num_servo = 0;

        goal[0] = torque & 0x00FF;
        goal[1] = torque >> 8;  // Goal pos top 8 bits
        joint_.addToPacket(buffer_add, position, goal, sizeof(goal_3), num_servo);
        joint_.wrapPacket(buffer_wrap, buffer_add, position, MOVING_SPEED_L, num_servo);
        //Send buffer

        joint_.sendPacket(iWRITE_DATA, buffer_wrap, position)

        time_last_ = millis();
    }
}
