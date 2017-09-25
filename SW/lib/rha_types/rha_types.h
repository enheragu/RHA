/**
 * @Author: Enrique Heredia Aguado <quique>
 * @Date:   17-Sep-2017
 * @Project: RHA
 * @Last modified by:   quique
 * @Last modified time: 26-Sep-2017
 */

#ifndef RHA_TYPES_H
#define RHA_TYPES_H

#include "debug.h"

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
        int16_t speed_slope;
        uint8_t direction;
        SpeedGoal();
        SpeedGoal(uint8_t id, int16_t speed, int16_t speed_slope, uint8_t direction): servo_id(id), speed(speed), speed_slope(speed_slope), direction(direction) {}
     } ;


    #define INTEGER_INTERVAL 5  // How many intervals are taken when error is integrated

    /**
     * @brief Implements a standard PID regulator
     * @class Regulator
     */
    class Regulator {
        float kp_, ki_, kd_;
        float ierror_[INTEGER_INTERVAL];
        uint8_t index_;

     public:
        Regulator() {
            index_ = 0;
            for (uint8_t i = 0; i < INTEGER_INTERVAL; i++) ierror_[i] = 0;
        }

        /**
         * @brief Resets all regulator data to 0
         * @method resetRegulator
         */
        void resetRegulator() {
            kp_ = ki_ = kd_ = index_ = 0;
            for (uint8_t i = 0; i < INTEGER_INTERVAL; i++) ierror_[i] = 0;
        }

        /**
         * Sets PID regulator constants
         * @method setKRegulator
         * @param  kp            Proportional K
         * @param  ki            Integral K
         * @param  kd            Derivative K
         */
        void setKRegulator(float kp, float ki = 0, float kd = 0) {
            kp_ = kp; ki_ = ki; kd_ = kd;
        }

        /**
         * @brief Calculates output of regulator to a set error
         * @method regulator
         * @param  error     error
         * @param  derror    derivative error
         * @param  ierror    integral error
         * @return           returns output of regulator 
         */
        float regulator(float error, float derror = 0, float ierror = 0) {
            ierror_[index_] = ierror;
            index_ ++;
            if (index_ > INTEGER_INTERVAL) index_ = 0;

            float sum_i_error = 0;
            for (uint8_t i = 0; i < INTEGER_INTERVAL; i++) sum_i_error += ierror_[i];
            return kp_ * error + kd_ * derror + ki_ * sum_i_error;
        }

        float getKp() { return kp_; }
    };  // end class Regulator

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
        void setTimer(uint32_t time_set) {
            time_ = time_set;
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
            else return false;
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
            else return false;
        }
    };  // end class TimerMicroseconds

}  // end namespace RHATypes
#endif
