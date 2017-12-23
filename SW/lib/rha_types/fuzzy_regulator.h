/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   22-Dec-2017
 * @Project: RHA
 * @Last modified by:   enheragu
 * @Last modified time: 22-Dec-2017
 */

#ifndef FUZZY_REGULATOR_H
#define FUZZY_REGULATOR_H

#include "debug.h"

#include <stdint.h>
#include <Arduino.h>

#include "pid_regulator.h"
#include "rha_types.h"

namespace RHATypes {

    #define NUM_FUZZY_REGULATOR_NODES 10

    class FuzzyRegulatorNode : public Regulator {
        double aplication_point_;

     public:
         FuzzyRegulatorNode(): Regulator() {
             aplication_point_ = 0;
         }

         /**
          * @brief Resets all regulator data to 0
          * @method resetRegulator
          */
         void resetRegulator() {
             aplication_point_ = 0;
             Regulator::resetRegulator();
         }

         /**
          * Sets Fuzzy regulator node constants and aplication point
          * @method setRegulator
          * @param  _aplication_point Is the point in which this regulator is used
          * @param  _kp            Proportional K
          * @param  _ki            Integral K
          * @param  _kd            Derivative K
          */
         void setRegulator(float _aplication_point, float _kp, float _ki = 0, float _kd = 0) {
             aplication_point_ = _aplication_point;
             Regulator::setKRegulator(_kp, _ki, _kd);
         }

         float regulator(float _error, float _derror = 0, float _ierror = 0) {
             return Regulator::regulator(_error, _derror, _ierror);
         }

         float getAplicationPoint() { return aplication_point_; }
    };  // End class FuzzyRegulatorNode


    class FuzzyRegulator {
        FuzzyRegulatorNode nodes[NUM_FUZZY_REGULATOR_NODES];

        FuzzyRegulator() {
            // They are sorted from (0 to 10) the one with smaller aplication point to the one with the bigger
            nodes[0].setRegulator(0,0,0,0);
            nodes[1].setRegulator(0,0,0,0);
            nodes[2].setRegulator(0,0,0,0);
            nodes[3].setRegulator(0,0,0,0);
            nodes[4].setRegulator(0,0,0,0);
            nodes[5].setRegulator(0,0,0,0);
            nodes[6].setRegulator(0,0,0,0);
            nodes[7].setRegulator(0,0,0,0);
            nodes[8].setRegulator(0,0,0,0);
            nodes[9].setRegulator(0,0,0,0);
        }

        /**
         * It gets the output of the fuzzy regulator.
         * @method regulator
         * @param  _actual_point Actual position to evaluate membership to the nodes
         * @param  _error        error
         * @param  _derror    derivative error
         * @param  _ierror    integral error
         * @return           returns output of regulator
         */
        float regulator(float _actual_point, float _error, float _derror = 0, float _ierror = 0) {
            float result = 0;
            // The evaluation is made between the intervals, that's why the for gets only to NUM_FUZZY_REGULATOR_NODES-1 so it never reaches to evaluate nodes[NUM_FUZZY_REGULATOR_NODES+1], which will cause a segfault
            for (uint8_t i = 0; i < NUM_FUZZY_REGULATOR_NODES-1; i++) {

                if (_actual_point > nodes[i].getAplicationPoint() &&
                    _actual_point <= nodes[i+1].getAplicationPoint()) {
                        // A triangular membership function is applied:
                        // Vertex A is the smaller, vertex B is the bigger, the point to evaluate is between both
                        float vertexA = nodes[i].regulator(_error,_derror,_ierror);
                        float vertexB = nodes[i+1].regulator(_error,_derror,_ierror);
                        result = (_actual_point - vertexA) / (vertexB - vertexA);
                        i++;
                    }
                else {
                    nodes[i].regulator(_error,_derror,_ierror);
                }
            }
            return result;
        }
    };
}  // End RHATypes namespace
#endif
