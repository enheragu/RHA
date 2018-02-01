/**
 * @Author: Enrique Heredia Aguado <quique>
 * @Date:   17-Sep-2017
 * @Project: RHA
 * @Last modified by:   quique
 * @Last modified time: 29-Sep-2017
 */

#ifndef RHA_TYPES_H
#define RHA_TYPES_H

#include "debug.h"

#include "pid_regulator.h"
// #include "fuzzy_regulator.h"

#include <stdint.h>
#include <Arduino.h>


namespace RHATypes {

/**
 * @brief Data structure to store speed goal (servo id to send the goal, speed target, speed slope and direction to move).
 * @struct SpeedGoal
 */
struct SpeedGoal {
    uint8_t servo_id;
    int16_t speed;
    uint8_t direction;
    SpeedGoal(): servo_id(0), speed(0), direction(0) {}
    SpeedGoal(uint8_t _id, int16_t _speed, uint8_t _direction): servo_id(_id), speed(_speed), direction(_direction) {}
};

struct Point3 {
    float x;
    float y;
    float z;
    Point3(): x(0), y(0), z(0) {}
    Point3(uint8_t _x, int16_t _y, int16_t _z): x(_x), y(_y), z(_z) {}
};


/**
 * Class which implements a timer
 * @class setTimer
 * @param  time_set time duration
 */
class Timer {
 protected:
    uint32_t time_;
    uint64_t init_time_;

 public:
    void setTimer(uint32_t _time_set) {
        time_ = _time_set;
    }

    /**
     * @brief Activates the timer, it starts counting time
     * @method activateTimer
     */
    virtual void activateTimer() {
      DebugSerialRHATypesLn("Timer::activateTimer:");
      init_time_ = millis();
    }

    /**
     * @brief Checks if time was reached. If no, it pauses the execution and waits for it to finish
     * @method checkWait
     */
    virtual void checkWait() {
        while (millis() - init_time_ < time_) {
            DebugSerialRHATypesLn2("Timer::checkWait: time left: ", time_ - (micros() - init_time_));
            delay(1);
        }
    }

    /**
     * @brief Checks if time was reached. If not it returns a false and does not block the execution.
     * @method checkContinue
     * @return Returns true or false if it reached the time or not
     */
    virtual bool checkContinue() {
        DebugSerialRHATypesLn2("Timer::checkContinue: time left: ", time_ - (micros() - init_time_));
        if (millis() - init_time_ >= time_) return true;
        else
            return false;
    }

    /**
     * @brief Interface method to get time in which timer was activated
     * @method getInitTime
     * @return Returns last activation time
     */
    uint64_t getInitTime() { return init_time_; }
};  // end class Timer

/**
 * Implements the timer but in microseconds
 * @class activateTimer
 */
class TimerMicroseconds : public Timer {
 public:
    /**
     * @method activateTimer
     * @see Timer::activateTimer
     */
    virtual void activateTimer() {
        DebugSerialRHATypesLn("TimerMicroseconds::activateTimer:");
        init_time_ = micros();
    }

    /**
     * @method checkWait
     * @see Timer::activateTimer
     */
    virtual void checkWait() {
        while (micros() - init_time_ < time_) {
            DebugSerialRHATypesLn2("TimerMicroseconds::checkWait: time left: ", time_ - (micros() - init_time_));
            delayMicroseconds(1);
        }
    }

    /**
     * @method checkContinue
     * @see Timer::activateTimer
     */
    virtual bool checkContinue() {
        DebugSerialRHATypesLn2("TimerMicroseconds::checkContinue: time left: ", time_ - (micros() - init_time_));
        if (micros() - init_time_ >= time_) return true;
        else
            return false;
    }
};  // end class TimerMicroseconds

}  // namespace RHATypes
#endif
