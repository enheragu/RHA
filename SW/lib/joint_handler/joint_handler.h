/**
 * @file
 * @brief Implements JointHandler class. This object is in charge to sync all joints.
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_handler.h
 * @Last modified by:   enheragu
 * @Last modified time: 14-Sep-2017
 */

#include "joint_rha.h"
#include <SoftwareSerial.h>



#define G15_BAUDRATE 57600

class JointHandler {
    uint64_t time_last_, timer_;

    JointRHA joint_;

    boolean hardwareSerial_;
    SoftwareSerial* G15Serial_;
    uint8_t txpin_shield_, rxpin_shield_, ctrlpin_shield_;
 public:
    JointHandler();
    JointHandler(uint64_t timer);
    /**************************************
     *        Serial Port Handling        *
     **************************************/
    JointHandler(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin);
    JointHandler(uint8_t ctrlpin);
    void init(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin);
    void init(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin, uint32_t baudrate);

    void begin(uint32_t baudrate);
    void end(void);

    void setTxMode(void);
    void setRxMode(void);
    uint16_t sendPacket(uint8_t id, uint8_t instruction, uint8_t* data, uint8_t parameterLength);

    void init(uint64_t timer);

    void controlLoop();
};
