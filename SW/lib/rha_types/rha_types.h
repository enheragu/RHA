/**
 * @Author: Enrique Heredia Aguado <quique>
 * @Date:   17-Sep-2017
 * @Project: RHA
 * @Last modified by:   quique
 * @Last modified time: 19-Sep-2017
 */

 #ifndef RHA_TYPES_H
 #define RHA_TYPES_H

#include <stdint.h>

 struct SpeedGoal {
    uint8_t servo_id;
    int16_t speed;
    int16_t speed_slope;
    SpeedGoal();
    SpeedGoal(uint8_t id, int16_t speed, int16_t speed_slope): servo_id(id), speed(speed), speed_slope(speed_slope) {}
 } ;

#endif
