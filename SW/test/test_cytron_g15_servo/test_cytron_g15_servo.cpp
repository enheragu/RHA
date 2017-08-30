#include "debug.h"
#include "cytron_g15_servo.h"
#include "unity.h"

#ifdef UNIT_TEST

#define SERVO_ID 1
#define DELAY_MOVE 5000
#define POSITION 180
#define SPEED 300
#define BAUDRATE 19200.
#define MARGIN 5

class TestCytron : public Cytron_G15_Servo {
 public:
  TestCytron(uint8_t servo_id, uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin):Cytron_G15_Servo(servo_id, rxpin, txpin, ctrlpin) {  }
  uint8_t getServoID() {  return servo_id_; }
  uint8_t getTxPin() {  return txpin_shield; }
  uint8_t getRxPin() {  return rxpin_shield; }
  uint8_t getCtrlPin() {  return ctrlpin_shield; }
};

void checkStatus(word status) {
  switch (status) {
    case SERROR_PING:  TEST_ASSERT_EQUAL_MESSAGE(0, status, "[!]   Ping error in servo"); break;
    case SERROR_INPUTVOLTAGE: TEST_ASSERT_EQUAL_MESSAGE(0, status, "[!]   Input voltage error in servo"); break;             // bit 0
    case SERROR_ANGLELIMIT: TEST_ASSERT_EQUAL_MESSAGE(0, status, "[!]   Angle limit error in servo"); break;                 // bit 1
    case SERROR_OVERHEATING: TEST_ASSERT_EQUAL_MESSAGE(0, status, "[!]   Overheating error in servo"); break;                // bit 2
    case SERROR_RANGE: TEST_ASSERT_EQUAL_MESSAGE(0, status, "[!]   Range error in servo"); break;                            // bit 3
    case SERROR_CHECKSUM: TEST_ASSERT_EQUAL_MESSAGE(0, status, "[!]   Checksum error in servo"); break;                      // bit 4
    case SERROR_OVERLOAD: TEST_ASSERT_EQUAL_MESSAGE(0, status, "[!]   Overload error in servo"); break;                      // bit 5
    case SERROR_INSTRUCTION: TEST_ASSERT_EQUAL_MESSAGE(0, status, "[!]   Instruction error in servo"); break;                // bit 7
    case SERROR_PACKETLOST: TEST_ASSERT_EQUAL_MESSAGE(0, status, "[!]   Packet lost or receive time out in servo"); break;    // bit 8
    case SERROR_WRONGHEADER: TEST_ASSERT_EQUAL_MESSAGE(0, status, "[!]   Wrong header in servo");  break;                    // bit 9
    case SERROR_IDMISMATCH: TEST_ASSERT_EQUAL_MESSAGE(0, status, "[!]   ID mismatch in servo"); break;                       // bit 10
    case SERROR_CHECKSUMERROR: TEST_ASSERT_EQUAL_MESSAGE(0, status, "[!]   Checksum error in servo"); break;                 // bit 13
    default: TEST_ASSERT_EQUAL_MESSAGE(0, status, "Expected 0, unknown error");
  }
}  // End of checkStatus function


void test_comunication_Cytron_G15_Servo(void) {
    TestCytron g15(SERVO_ID, 2, 3, 8);
    g15.begin(BAUDRATE);
    word status, id;
    uint8_t data[10];
    status = g15.ping(data);
    checkStatus(status);
    id = data[0];
    TEST_ASSERT_EQUAL(SERVO_ID, id);
}

void test_positioningMode_functioning(void) {
  TestCytron g15(SERVO_ID, 2, 3, 8);
  g15.begin(BAUDRATE);
  word status, pos;
  uint8_t data[10];
  status = g15.exitWheelMode();
  checkStatus(status);
  status = g15.setSpeed(SPEED, iWRITE_DATA);
  checkStatus(status);
  status = g15.setPosAngle(POSITION, iWRITE_DATA);
  checkStatus(status);
  delay(DELAY_MOVE);
  status = g15.getPos(data);
  checkStatus(status);
  pos = data[0];
  pos = pos | (data[1] << 8);
  DebugSerialTG15("Data recieved is: "); DebugSerialTG15(data[0]); DebugSerialTG15(" and "); DebugSerialTG15(data[1]);
  DebugSerialTG15(" --- "); DebugSerialTG15("Pos is:");  DebugSerialTG15(pos); DebugSerialTG15(". In degrees:");
  DebugSerialTG15Ln(ConvertPosToAngle(pos));
  pos = ConvertPosToAngle(pos);
  TEST_ASSERT_TRUE(POSITION< pos+MARGIN && POSITION > pos-MARGIN);
}

void test_setWheelMode_functioning(void) {
  TestCytron g15(SERVO_ID, 2, 3, 8);
  g15.begin(BAUDRATE);
  word status, speed;
  uint8_t data[10];
  status = g15.setWheelMode();
  checkStatus(status);
  status = g15.setWheelSpeed(SPEED, CW, iWRITE_DATA);
  checkStatus(status);
  delay(DELAY_MOVE);
  status = g15.getSpeed(data);
  checkStatus(status);
  speed = data[0];
  speed |=  word(data[1]) << 8;
  // if (speed > 1000) speed = speed - 1000; ¿? saw in cytron example
  // with no load:
  /* setSpeed -> 90;      getSpeed -> 1059
   * setSpeed -> 10;      getSpeed -> 1067
   * setSpeed -> 900;      getSpeed -> 1124
   * setSpeed -> 200;      getSpeed -> 1304
   * setSpeed -> 512;      getSpeed -> 1421
   * setSpeed -> 1023;      getSpeed -> 1603
   * An increment of 100 in torque increment 60 the speed
   */
  // speed read is the actual speed, not the one sended. It depends on the load it has
  DebugSerialTG15("Data recieved is: "); DebugSerialTG15(data[0]); DebugSerialTG15(" and "); DebugSerialTG15(data[1]);
  DebugSerialTG15(" --- "); DebugSerialTG15("Speed is:");
  DebugSerialTG15Ln(speed);
  TEST_ASSERT_FALSE(SPEED< speed+MARGIN && SPEED > speed-MARGIN);
  status = g15.exitWheelMode();
  checkStatus(status);
}

void test_function_isMovingPositioningMode(void) {
  TestCytron g15(SERVO_ID, 2, 3, 8);
  g15.begin(BAUDRATE);
  word status, pos;
  uint8_t data[10];
  status = g15.exitWheelMode();
  checkStatus(status);
  status = g15.setSpeed(SPEED, iWRITE_DATA);
  checkStatus(status);
  status = g15.setPosAngle(POSITION, iWRITE_DATA);
  checkStatus(status);
  // delay(DELAY_MOVE);
  status = g15.isMoving(data);
  checkStatus(status);
  TEST_ASSERT_EQUAL(ON, data[0]);
  while (data[0] == ON) {
    g15.isMoving(data);
  }
  status = g15.isMoving(data);
  checkStatus(status);
  TEST_ASSERT_EQUAL(OFF, data[0]);  status = g15.getPos(data);
  checkStatus(status);
  pos = data[0];
  pos = pos | (data[1] << 8);
  pos = ConvertPosToAngle(pos);
  TEST_ASSERT_TRUE(POSITION< pos+MARGIN && POSITION > pos-MARGIN);
}

/*
---> Looks like isMoving only works when positioning <---

void test_function_isMovingWheelMode(void) {
  TestCytron g15(SERVO_ID, 2, 3, 8);
  g15.begin(BAUDRATE);
  word status, speed;
  uint8_t data[10];
  status = g15.setWheelMode();
  checkStatus(status);
  status = g15.setWheelSpeed(SPEED, CW, iWRITE_DATA);
  checkStatus(status);
  delay(DELAY_MOVE);
  status = g15.isMoving(data);
  checkStatus(status);
  TEST_ASSERT_EQUAL(ON, data[0]);
  status = g15.exitWheelMode();
}*/

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_comunication_Cytron_G15_Servo);
  RUN_TEST(test_positioningMode_functioning);
  RUN_TEST(test_setWheelMode_functioning);
  RUN_TEST(test_function_isMovingPositioningMode);
  UNITY_END();
}


void setup() {
  Serial.begin(9600);
  delay(3000);
  process();
}

void loop() {
  // some code...
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(500);
}

#endif
