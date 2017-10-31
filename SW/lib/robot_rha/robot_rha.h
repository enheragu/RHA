/**
 * @Author: Enrique Heredia Aguado <gmv>
 * @Date:   31_Oct_2017
 * @Project: RHA
 * @Filename: robot_rha.h
 * @Last modified by:   enheragu
 * @Last modified time: 31_Oct_2017
 */



class RobotRHA {
 protected:

 public:
    RobotRHA();

    void handleRobot();
    void setCartesianSpeedGoal(float speed_x, float speed_y, float speed_z);
};
