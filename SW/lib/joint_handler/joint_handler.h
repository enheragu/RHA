/**
 * @file
 * @brief Implements JointHandler class. This object is in charge to sync all joints.
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_handler.h
 * @Last modified by:   enheragu
 * @Last modified time: 08_Sep_2017
 */

#include "joint_rha.h"

class JointHandler {
    uint64_t time_last_, timer_;

    JointRHA joint_;
 public:
    JointHandler();
    JointHandler(uint64_t timer);

    void init(uint64_t timer);

    void controlLoop();
};
