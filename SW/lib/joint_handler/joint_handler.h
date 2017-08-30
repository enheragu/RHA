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
