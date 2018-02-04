/**
 * @file
 * @brief Implements JointRHA functions defined in joint_rha.h
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_rha.cpp
 * @Last modified by:   quique
 * @Last modified time: 29-Sep-2017
 */

#include "joint_rha.h"

JointRHA::JointRHA() {
    time_last_error_ = 0;
    position_pot_ = 0;
    position_pot_last_ = 0;
    position_target_ = 0;
    error_moving_ = 0;
}

/** @brief Cunstructor of JointRHA class.
  * @param {uint8_t} servo_id servo id controlled by this joint
  * @param {uint8_t} up_direction direction in which the servo has to move (CW or CCW) so the joint moves up.
  * @param {uint8_t} potentiometer pin in which the potentiometer for this joint is connected. If there is no realim for this joint value will be 255
  */
JointRHA::JointRHA(uint8_t _servo_id, uint8_t _up_direction, uint8_t _potentiometer) {
    // servo_ = new ServoRHA (servo_id);
    servo_.init(_servo_id);
    up_direction_ = _up_direction;
    potentiometer_pin_ = _potentiometer;
    JointRHA();
}

/** @brief ~JointRHA destructor of JointRHA class.
  */
JointRHA::~JointRHA() {
}

/** @brief Initialization for JointRHA default constructor.
  * @param {uint8_t} servo_id servo id controlled by this joint
  * @param {uint8_t} up_direction direction in which the servo has to move (CW or CCW) so the joint moves up.
  * @param {uint8_t} potentiometer pin in which the potentiometer for this joint is connected. If there is no realim for this joint value will be 255
  */
void JointRHA::init(uint8_t _servo_id, uint8_t _up_direction, float _zero_compensation, uint8_t _potentiometer) {
    DebugSerialJRHALn("init: begin of function");
    DebugSerialJRHALn2("init: initialazing with id: ", _servo_id);
    up_direction_ = _up_direction;
    potentiometer_pin_ = _potentiometer;
    zero_pos_compensation_ = _zero_compensation;
    joint_pot_relation_ = 1;
    DebugSerialJRHALn2("init: potentiometer in pin: ", potentiometer_pin_);

    speed_regulator_.setKRegulator(KP, KI, KD);

    DebugSerialJRHALn2("init: setKRegulator kp: ", KP);
    DebugSerialJRHALn2("init: setKRegulator ki: ", KI);
    DebugSerialJRHALn2("init: setKRegulator kd: ", KD);

    servo_.init(_servo_id);
    if (potentiometer_pin_ != NO_POTENTIOMETER) pinMode(potentiometer_pin_, INPUT);
}

/**
 * @brief Updates position reading from potentiometer if there is a pot to read (not 255). Updates joint angle position
 * @method JointRHA::updatePosition
 * @return returns position value in joint reference
 */
void JointRHA::updatePosition() {
    if (potentiometer_pin_ != NO_POTENTIOMETER) {
        // Serial.print("UpdatePositionPot: ");
        DebugSerialJRHALn2("updatePosition: for potentiometer in pin: ", potentiometer_pin_);
        pot_analog_read_ = analogRead(potentiometer_pin_);
        //Serial.print("Art: "); Serial.print(servo_.getID(), DEC); Serial.print(", pot: "); Serial.println(pot_analog_read_);
        position_pot_last_ = position_pot_;
        position_pot_ = floatMap(pot_analog_read_, 0, 1023, 0, 265) * joint_pot_relation_;  // from 0 to 5V transform to 0-265 degrees (max angle of potentiometer)
        position_pot_ = position_pot_ - zero_pos_compensation_;
        //position_pot_ = position_pot_ - zero_pos_compensation_;

        // Serial.println(position_pot_);
    } else {
        position_pot_ = -1;
        DebugSerialJRHALn2("updatePosition: not updated for potentiometer in pin: ", potentiometer_pin_);
    }
}

/**
 * Sets the relation between the potentiometer angle (in grads) and the joint angle
 * @method JointRHA::setPotRelation
 * @param  _relation            relation between measures. diameter of pot gear / diameter of bar gear
 */
void JointRHA::setPotRelation(float _relation) {
    if (potentiometer_pin_ == NO_POTENTIOMETER ) return;
    joint_pot_relation_ = _relation;
}


/**
 * @brief Updates all the information of servo object information and position feedback of joint  to use it in next control iteration (in control loop)
 * @method JointRHA::updateInfo
 * @param {uint8_t *} data data with servo information to pass to it
 * @param {uint16_t *} error  error in communication with servo
 */
void JointRHA::updateInfo(uint8_t *_data, uint16_t _error) {
    updatePosition();
    servo_.updateInfo(_data, _error);
}

/**
 * @brief Sets a goal position for this joint
 * @method setPositionGoal
 * @param  position        position to go
 */
void JointRHA::setPositionGoal(int _position) {
    if (potentiometer_pin_ == NO_POTENTIOMETER ) return;
    position_target_ = _position;
}

/**
 * @brief Calculates error to send to servo regulator
 */
void JointRHA::posError() {
    if (potentiometer_pin_ == NO_POTENTIOMETER ) return;
    DebugSerialJRHALn("posError: begin of function");
    pos_error_ = (((float)position_target_ - (float)position_pot_));
    pos_derror_ = (pos_error_ - pos_last_error_) / (millis() - time_last_error_);
    pos_ierror_ = pos_error_ * (millis() - time_last_error_);
    pos_last_error_ = pos_error_;
    time_last_error_ = millis();
    DebugSerialJRHALn2("posError: target pos is: ", position_target_);
    DebugSerialJRHALn2("posError: error is: ", pos_error_);
}

/**
 * @brief calculates speed from pos error using regulator. Params are by default 0, it is only used with params for testing pourposes
 * @method JointRHA::calculateTorque
 * @param  error      pos error
 * @param  derror     derivative of pos error
 * @param  ierror     integral of pos error
 */
void JointRHA::calculateSpeed(float _error, float _derror, float _ierror) {
    if (potentiometer_pin_ == NO_POTENTIOMETER ) return;
    DebugSerialJRHALn2("calculateSpeed: begin of function for joint ", servo_.getID());
    // float error = (float)target_speed - (float)speed_;
    float speed;
    speed = speed_regulator_.regulator(pos_error_, pos_derror_, pos_ierror_);
    if (speed > MAX_SPEED_VALUE) speed = MAX_SPEED_VALUE;  // compensate saturation of servos
    else if (abs(speed) > MAX_SPEED_VALUE) speed = -MAX_SPEED_VALUE;
    if (compareAngles(position_pot_, position_target_, ANGLE_TOLERANCE) == ServoRHAConstants::EQUAL) {
        DebugSerialJRHALn2("calculateSpeed: position is: ", position_pot_);
        DebugSerialJRHALn2("calculateSpeed: target is: ", position_target_);
        DebugSerialJRHALn("calculateSpeed: target reached, speed set to 0 ");
        speed = 0;
    }
    DebugSerialJRHALn2("calculateSpeed: speed calculated is: ", (speed));
    DebugSerialJRHALn2("calculateSpeed: speed calculated is (uint16_t cast): ", uint16_t(abs(speed)));
    DebugSerialJRHALn2("calculateSpeed: move joint in: ", (speed > 0)?"UP":"DOWN");
    goal_speed_.speed = uint16_t(abs(speed));
    goal_speed_.servo_id = servo_.getID();
    goal_speed_.direction = (speed < 0)?!up_direction_:up_direction_;
}

/**
 * @brief Updates ServoRHA speed goal
 * @method JointRHA::updateServoSpeedGoal
 */
void JointRHA::updateServoSpeedGoal() {
    if (potentiometer_pin_ == NO_POTENTIOMETER ) return;
    servo_.setSpeedGoal(goal_speed_);
}

/**
 * @brief returns true if goal position is reached
 * @method JointRHA::reachedGoalPosition
 * @return [description]
 */
bool JointRHA::reachedGoalPosition() {
    return (compareAngles(position_pot_, position_target_, ANGLE_TOLERANCE) == ServoRHAConstants::EQUAL)?true:false;
}


/**
 * @brief checks that everithing goes as espected. If not it stops the servo
 * @method checkSecurity
 * @return Returns true when theres no problem, false otherwise
 */
bool JointRHA::checkSecurity() {
    if (abs(servo_.getSpeed()) > 0 && compareAngles(position_pot_, position_pot_last_, ANGLE_TOLERANCE) == ServoRHAConstants::EQUAL )
        error_moving_++;
    else error_moving_ = 0;
    if (error_moving_ > ERROR_MOVING_MARGIN) {
            Serial.print("[Error] Some error ocurred, joint should be moving but its not. Joint: "); Serial.println(servo_.getID());
            return false;
    }
    else return true;
}
