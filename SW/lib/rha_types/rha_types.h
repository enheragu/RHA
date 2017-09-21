/**
 * @Author: Enrique Heredia Aguado <quique>
 * @Date:   17-Sep-2017
 * @Project: RHA
 * @Last modified by:   enheragu
 * @Last modified time: 21_Sep_2017
 */

 #ifndef RHA_TYPES_H
 #define RHA_TYPES_H

#include <stdint.h>
#include <Arduino.h>

namespace RHATypes {

    struct SpeedGoal {
        uint8_t servo_id;
        int16_t speed;
        int16_t speed_slope;
        SpeedGoal();
        SpeedGoal(uint8_t id, int16_t speed, int16_t speed_slope): servo_id(id), speed(speed), speed_slope(speed_slope) {}
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

        float regulator(float error, float derror, float ierror) {
            ierror_[index_] = ierror;
            index_ ++;
            if (index_ > INTEGER_INTERVAL) index_ = 0;

            float sum_i_error = 0;
            for (uint8_t i = 0; i < INTEGER_INTERVAL; i++) sum_i_error += ierror_[i];
            return kp_ * error + kd_ * derror + ki_ * sum_i_error;
        }
    };  // end class Regulator

    class Timer {
        uint32_t time_;
        uint64_t init_time_;
     public:
        void setTimer(uint32_t time_set) {
            time_ = time_set;
        }

        virtual void activateTimer() {
            init_time_ = millis();
        }

        virtual void checkWait() {
            while (millis() - init_time_ < time_) {
              delay(1);
            }
        }

        virtual bool checkContinue() {
            if (millis() - init_time_ > time_) return true;
            else return false;
        }
    };  // end class Timer

    class TimerMicroseconds : public Timer {
     public:
        virtual void activateTimer() {
            init_time_ = micros();
        }

        virtual void checkWait() {
            while (micros() - init_time_ < time_) {
                delayMicroseconds(1);
            }
        }

        virtual bool checkContinue() {
            if (micros() - init_time_ > time_) return true;
            else return false;
        }
    };  // end class TimerMicroseconds

}  // end namespace RHATypes
#endif