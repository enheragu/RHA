/**
 * @Author: Enrique Heredia Aguado <quique>
 * @Date:   17-Sep-2017
 * @Project: RHA
 * @Last modified by:   quique
 * @Last modified time: 17-Sep-2017
 */

#include <stdint.h>

 struct SpeedGoal {
    uint8_t servo_id;
    int16_t speed;
    int16_t speed_slope;
} ;
