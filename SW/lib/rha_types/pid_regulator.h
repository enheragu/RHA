/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   22-Dec-2017
 * @Project: RHA
 * @Last modified by:   enheragu
 * @Last modified time: 22-Dec-2017
 */



#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

#include "debug.h"

#include <stdint.h>
#include <Arduino.h>

namespace RHATypes {
#define INTEGER_INTERVAL 15  // How many intervals are taken when error is integrated

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
        kp_ = ki_ = kd_ = 0;
        for (uint8_t i = 0; i < INTEGER_INTERVAL; i++) ierror_[i] = 0;
    }

    /**
     * @brief Resets all regulator data to 0
     * @method resetRegulator
     */
    virtual void resetRegulator() {
        kp_ = ki_ = kd_ = index_ = 0;
        for (uint8_t i = 0; i < INTEGER_INTERVAL; i++) ierror_[i] = 0;
    }

    /**
     * Sets PID regulator constants
     * @method setKRegulator
     * @param  _kp            Proportional K
     * @param  _ki            Integral K
     * @param  _kd            Derivative K
     */
    virtual void setKRegulator(float _kp, float _ki = 0, float _kd = 0) {
        kp_ = _kp; ki_ = _ki; kd_ = _kd;
    }

    /**
     * @brief Calculates output of regulator to a set error
     * @method regulator
     * @param  _error     error
     * @param  _derror    derivative error
     * @param  _ierror    integral error
     * @return           returns output of regulator
     */
    virtual float regulator(float _error, float _derror = 0, float _ierror = 0) {
        ierror_[index_] = _ierror;
        index_++;
        if (index_ > INTEGER_INTERVAL) index_ = 0;

        float sum_i_error = 0;
        for (uint8_t i = 0; i < INTEGER_INTERVAL; i++) sum_i_error += ierror_[i];
        return (kp_ * _error + kd_ * _derror + ki_ * sum_i_error);
    }

    float getKp() { return kp_; }
    float getKi() { return ki_; }
    float getKd() { return kd_; }
};  // end class Regulator
}  // namespace RHATypes
#endif
