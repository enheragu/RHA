#include "joint_handler.h"


JointHandler::JointHandler(uint64_t timer){
    timer_ = timer;
}

void JointHandler::init(uint64_t timer){
    timer_ = timer;
    joint_.init(1, CW, A0);
}

void JointHandler::controlLoop(){
    if(millis() - time_last_ >= timer_){
    // Firt all info have to be updated

    // Ask each joint torque to update speed

    // Add all servo to the buffer

    //Send buffer

        time_last_ = millis();
    }
}
