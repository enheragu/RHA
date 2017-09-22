/**
 * @Author: Enrique Heredia Aguado <quique>
 * @Date:   17-Sep-2017
 * @Project: RHA
 * @Last modified by:   enheragu
 * @Last modified time: 22_Sep_2017
 */

#ifndef RHA_TYPES_H
#define RHA_TYPES_H

#include "debug.h"

#include <stdint.h>
#include <Arduino.h>

namespace RHATypes {

    struct SpeedGoal {
        uint8_t servo_id;
        int16_t speed;
        int16_t speed_slope;
        uint8_t direction;
        SpeedGoal();
        SpeedGoal(uint8_t id, int16_t speed, int16_t speed_slope, uint8_t direction): servo_id(id), speed(speed), speed_slope(speed_slope), direction(direction) {}
     } ;


    #define INTEGER_INTERVAL 5  // How many intervals are taken when error is integrated

    class Regulator {
        float kp_, ki_, kd_;
        float ierror_[INTEGER_INTERVAL];
        uint8_t index_;

     public:
        Regulator() {
            index_ = 0;
            for (uint8_t i = 0; i < INTEGER_INTERVAL; i++) ierror_[i] = 0;
        }

        void resetRegulator() {
            kp_ = ki_ = kd_ = index_ = 0;
            for (uint8_t i = 0; i < INTEGER_INTERVAL; i++) ierror_[i] = 0;
        }

        void setKRegulator(float kp, float ki = 0, float kd = 0) {
            kp_ = kp; ki_ = ki; kd_ = kd;
        }

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

    class Timer {
     protected:
        uint32_t time_;
        uint64_t init_time_;
     public:
        void setTimer(uint32_t time_set) {
            time_ = time_set;
        }

        virtual void activateTimer() {
          DebugSerialRHATypesLn("Timer::activateTimer:");
          init_time_ = millis();
        }

        virtual void checkWait() {
            while (millis() - init_time_ < time_) {
                DebugSerialRHATypesLn2("Timer::checkWait: time left: ", time_ - (micros() - init_time_));
                delay(1);
            }
        }

        virtual bool checkContinue() {
            DebugSerialRHATypesLn2("Timer::checkContinue: time left: ", time_ - (micros() - init_time_));
            if (millis() - init_time_ >= time_) return true;
            else return false;
        }

        uint64_t getInitTime() { return init_time_; }
    };  // end class Timer

    class TimerMicroseconds : public Timer {
     public:
        virtual void activateTimer() {
            DebugSerialRHATypesLn("TimerMicroseconds::activateTimer:");
            init_time_ = micros();
        }

        virtual void checkWait() {
            while (micros() - init_time_ < time_) {
                DebugSerialRHATypesLn2("TimerMicroseconds::checkWait: time left: ", time_ - (micros() - init_time_));
                delayMicroseconds(1);
            }
        }

        virtual bool checkContinue() {
            DebugSerialRHATypesLn2("TimerMicroseconds::checkContinue: time left: ", time_ - (micros() - init_time_));
            if (micros() - init_time_ >= time_) return true;
            else return false;
        }
    };  // end class TimerMicroseconds

}  // end namespace RHATypes
#endif
