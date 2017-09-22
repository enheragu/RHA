/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Aug_31
 * @Project: RHA
 * @Filename: test_cytron_g15_servo.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 22_Sep_2017
 */



#include "debug.h"
#include "cytron_g15_servo.h"
#include "unity.h"

#ifdef UNIT_TEST

#define SERVO_ID 4
#define DELAY_MOVE 5000
#define POSITION 180
#define SPEED 300
#define BAUDRATE 19200.
#define MARGIN 5

class TestCytron : public Cytron_G15_Servo {
 public:
  TestCytron(uint8_t servo_id, uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin):Cytron_G15_Servo(servo_id, rxpin, txpin, ctrlpin) {  }
  uint8_t getServoID() {  return servo_id_; }
};

void testStatus(word status) {
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
}  // End of testStatus function


// These goals are used in test_function_addToPacket and test_function_warpPacket
uint8_t goal_0[4] = {  0x10, 0x00, 0x50, 0x01};
uint8_t goal_1[4] = {  0x20, 0x02, 0x60, 0x03};
uint8_t goal_2[4] = {  0x30, 0x00, 0x70, 0x01};
uint8_t goal_3[4] = {  0x20, 0x02, 0x80, 0x03};

void test_function_warpPacket(void) {
    TestServoRHA servo_test0(0, 2, 3, 8);
    servo_test0.initServo();
    TestServoRHA servo_test1(1, 2, 3, 8);
    servo_test1.initServo();
    TestServoRHA servo_test2(2, 2, 3, 8);
    servo_test2.initServo();
    TestServoRHA servo_test3(3, 2, 3, 8);
    servo_test3.initServo();

    uint8_t data[30];
    uint8_t buffer[30];
    uint8_t buffer_test[30] = {   0xFF, 0xFF, 0xFE, 0x18, 0x83, 0x1E, 0x04,  // header, ID, length, instruction, adress, data length
                            0x00, 0x10, 0x00, 0x50, 0x01,  // servo ID 0 to position 0x010 with speed of 0x150
                            0x01, 0x20, 0x02, 0x60, 0x03,  // servo ID 1 to position 0x220 with speed of 0x360.
                            0x02, 0x30, 0x00, 0x70, 0x01,  // servo ID 2 to position 0x030 with speed of 0x170
                            0x03, 0x20, 0x02, 0x80, 0x03,  // servo ID 3 to position 0x220 with speed of 0x380
                            0x12 };  // checksum

    uint8_t position = 0;
    uint8_t num_servo = 0;

    servo_test0.addToPacket(data, position, goal_0, sizeof(goal_0), num_servo);
    servo_test1.addToPacket(data, position, goal_1, sizeof(goal_1), num_servo);
    servo_test2.addToPacket(data, position, goal_2, sizeof(goal_2), num_servo);
    servo_test3.addToPacket(data, position, goal_3, sizeof(goal_3), num_servo);
    TEST_ASSERT_EQUAL_UINT8(4, num_servo);
    TEST_ASSERT_EQUAL_UINT8(20, position);
    uint8_t buffer_len = 0;

    buffer_len = servo_test1.wrapPacket(buffer, data, position, GOAL_POSITION_L, num_servo);

    for (int i = 0; i < buffer_len; i++) {
      DebugSerialTSRHAMock("0x"); DebugSerialTSRHAMock((buffer[i], HEX)); DebugSerialTSRHAMock(", ");
    } DebugSerialTSRHAMockLn(" ");

    TEST_ASSERT_EQUAL_UINT8(28, buffer_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test, buffer, buffer_len);
}

void test_comunication_Cytron_G15_Servo(void) {
    TestCytron g15(SERVO_ID, 2, 3, 8);
    g15.begin(BAUDRATE);
    word status, id;
    uint8_t data[10];
    status = g15.ping(data);
    testStatus(status);
    id = data[0];
    TEST_ASSERT_EQUAL(SERVO_ID, id);
}

void test_positioningMode_functioning(void) {
  TestCytron g15(SERVO_ID, 2, 3, 8);
  g15.begin(BAUDRATE);
  word status, pos;
  uint8_t data[10];
  status = g15.exitWheelMode();
  testStatus(status);
  status = g15.setSpeed(SPEED, iWRITE_DATA);
  testStatus(status);
  status = g15.setPosAngle(POSITION, iWRITE_DATA);
  testStatus(status);
  delay(DELAY_MOVE);
  status = g15.getPos(data);
  testStatus(status);
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
  testStatus(status);
  status = g15.setWheelSpeed(SPEED, CW, iWRITE_DATA);
  testStatus(status);
  delay(DELAY_MOVE);
  status = g15.getSpeed(data);
  testStatus(status);
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
  testStatus(status);
}

void test_function_isMovingPositioningMode(void) {
  TestCytron g15(SERVO_ID, 2, 3, 8);
  g15.begin(BAUDRATE);
  word status, pos;
  uint8_t data[10];
  status = g15.exitWheelMode();
  testStatus(status);
  status = g15.setSpeed(SPEED, iWRITE_DATA);
  testStatus(status);
  status = g15.setPosAngle(POSITION, iWRITE_DATA);
  testStatus(status);
  // delay(DELAY_MOVE);
  status = g15.isMoving(data);
  testStatus(status);
  TEST_ASSERT_EQUAL(ON, data[0]);
  while (data[0] == ON) {
    g15.isMoving(data);
  }
  status = g15.isMoving(data);
  testStatus(status);
  TEST_ASSERT_EQUAL(OFF, data[0]);  status = g15.getPos(data);
  testStatus(status);
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
  testStatus(status);
  status = g15.setWheelSpeed(SPEED, CW, iWRITE_DATA);
  testStatus(status);
  delay(DELAY_MOVE);
  status = g15.isMoving(data);
  testStatus(status);
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
