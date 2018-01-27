#ifndef CHUCK_HANDLER_H
#define CHUCK_HANDLER_H

#include <Arduino.h>
#include "rha_types.h"
#include "WiiChuck.h"

#define CHUCK_MARGIN 40
#define CHUCK_MAX_VALUE 90

// SDA goes to pin A4
// SCL goes to pin A5

struct ChuckReadStruct {
    int X_, Y_, Z_;
    bool updated_;
    explicit ChuckReadStruct(int _x = 0, int _y = 0, int _z = 0, bool _updated = false): X_(_x), Y_(_y), Z_(_z), updated_(_updated) {}
};

class ChuckHandler {
 protected:
    WiiChuck chuck_;
    RHATypes::Timer chuck_refresh_timer_;

 public:
    void begin() {
         chuck_.begin();
    }

    /**
     * @brief Sets period of chuck read axis refresh
     * @method setTimer
     * @param  timer    period in ms
     */
    void setTimer(uint64_t timer) {
        chuck_refresh_timer_.setTimer(timer);
        chuck_refresh_timer_.activateTimer();
    }

    void printChuckValues() {
         chuck_.update();
         Serial.print("Joy X value is: "); Serial.println(chuck_.readJoyX());
         Serial.print("Joy Y value is: "); Serial.println(chuck_.readJoyY());
         Serial.print("Z value is: "); Serial.println(chuck_.buttonZ);
         Serial.print("C value is: "); Serial.println(chuck_.buttonC);
    }

    /**
     * @brief Reads values from chuck and returns an X,Y,Z speed
     * @method readAxis
     * @return ChuckReadStruct is a struct with speed values in it (X,Y,Z) from -100 to 100 (direction and module)
     */
    ChuckReadStruct readAxis() {
        if (chuck_refresh_timer_.checkContinue()) {
            ChuckReadStruct info;

            chuck_.update();
            int x_value = chuck_.readJoyX();
            int y_value = chuck_.readJoyY();
            bool c_value = chuck_.buttonC;
            bool z_value = chuck_.buttonZ;

            /*if ( c_value && z_value) info.Z_ = 0;
            else if ( c_value ) info.Z_ = 50;
            else if ( z_value ) info.Z_ = -50;
            else info.Z_ = 0;*/

            if (x_value > CHUCK_MAX_VALUE || x_value < -CHUCK_MAX_VALUE) x_value = (x_value > 0) ? CHUCK_MAX_VALUE : -CHUCK_MAX_VALUE;
            if (y_value > CHUCK_MAX_VALUE || y_value < -CHUCK_MAX_VALUE) y_value = (y_value > 0) ? CHUCK_MAX_VALUE : -CHUCK_MAX_VALUE;

            if (x_value > CHUCK_MARGIN) info.X_ = map(x_value, CHUCK_MARGIN, CHUCK_MAX_VALUE, 0, 100);
            else if (x_value < -CHUCK_MARGIN) info.X_ = map(x_value, -CHUCK_MAX_VALUE, -CHUCK_MARGIN,  -100, 0);
            else
                info.X_ = 0;

            if (y_value > CHUCK_MARGIN) info.Y_ = map(y_value, CHUCK_MARGIN, CHUCK_MAX_VALUE, 0, 100);
            else if (y_value < -CHUCK_MARGIN) info.Y_ = map(y_value, -CHUCK_MAX_VALUE, -CHUCK_MARGIN,  -100, 0);
            else
                info.Y_ = 0;

            if ( c_value ) {
                info.Z_ = info.X_;
                info.X_ = 0;
                info.Y_ = 0;
            } else
                info.Z_ = 0;

            Serial.print(" X value is: "); Serial.println(info.X_);
            Serial.print(" Y value is: "); Serial.println(info.Y_);
            Serial.print(" Z value is: "); Serial.println(info.Z_);

            info.updated_ = true;

            chuck_refresh_timer_.activateTimer();
            return info;
        }
        return ChuckReadStruct(0, 0, 0, false);
    }
};
#endif
