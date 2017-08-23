#ifndef G15_h
#define G15_h

#include "Arduino.h"


//definitions
//******************************************************************
//*	INSTRUCTIONS
//******************************************************************
#define iPING 		0x01 //obtain a status packet
#define iREAD_DATA	0x02 //read Control Table values
#define iWRITE_DATA	0x03 //write Control Table values
#define iREG_WRITE 	0x04 //write and wait for ACTION instruction
#define iACTION 	0x05 //triggers REG_WRITE instruction
#define iRESET 		0x06 //set factory defaults
#define iSYNC_WRITE     0x83 //simultaneously control multiple actuators

#define SerialTimeOut 100L
#define TxMode LOW
#define RxMode HIGH
#define ConvertAngle2Pos(Angle) uint16_t(uint16_t(Angle)*1088UL/360UL)
#define ConvertPos2Angle(Pos) float((Pos)*360.0/1088.0)
#define ConvertTime(Time) uint16_t(Time*10UL)
#define CW 1
#define CCW 0
#define ON 1
#define OFF 0
//Alarm Mask	1111 1111
#define ALARM_INST			0x40
#define ALARM_OVERLOAD		0x20
#define ALARM_CHECKSUM		0x10
#define ALARM_RANGE			0x08
#define ALARM_OVERHEAT		0x04
#define ALARM_ANGLELIMIT 	0x02
#define ALARM_VOLTAGE		0x01

enum{
			  MODEL_NUMBER_L, 		// 0x00
			  MODEL_NUMBER_H, 		// 0x01
			  VERSION, 			// 0x02
			  ID, 				// 0x03
			  BAUD_RATE, 			// 0x04
			  RETURN_DELAY_TIME, 		// 0x05
			  CW_ANGLE_LIMIT_L, 		// 0x06
			  CW_ANGLE_LIMIT_H, 		// 0x07
			  CCW_ANGLE_LIMIT_L, 		// 0x08
			  CCW_ANGLE_LIMIT_H, 		// 0x09
			  RESERVED1, 			// 0x0A
			  LIMIT_TEMPERATURE, 		// 0x0B
			  DOWN_LIMIT_VOLTAGE, 		// 0x0C
			  UP_LIMIT_VOLTAGE, 		// 0x0D
			  MAX_TORQUE_L, 		// 0x0E
			  MAX_TORQUE_H, 		// 0x0F
			  STATUS_RETURN_LEVEL, 		// 0x10
			  ALARM_LED, 			// 0x11
			  ALARM_SHUTDOWN, 		// 0x12
			  RESERVED2, 			// 0x13
			  DOWN_CALIBRATION_L, 		// 0x14
			  DOWN_CALIBRATION_H, 		// 0x15
			  UP_CALIBRATION_L, 		// 0x16
			  UP_CALIBRATION_H, 		// 0x17
			  TORQUE_ENABLE, 		// 0x18
			  LED, 				// 0x19
			  CW_COMPLIANCE_MARGIN, 	// 0x1A
			  CCW_COMPLIANCE_MARGIN, 	// 0x1B
			  CW_COMPLIANCE_SLOPE, 		// 0x1C
			  CCW_COMPLIANCE_SLOPE, 	// 0x1D
			  GOAL_POSITION_L, 		// 0x1E
			  GOAL_POSITION_H, 		// 0x1F
			  MOVING_SPEED_L, 		// 0x20
			  MOVING_SPEED_H,		// 0x21
			  TORQUE_LIMIT_L, 		// 0x22
			  TORQUE_LIMIT_H, 		// 0x23
			  PRESENT_POSITION_L, 		// 0x24
			  PRESENT_POSITION_H, 		// 0x25
			  PRESENT_SPEED_L, 		// 0x26
			  PRESENT_SPEED_H, 		// 0x27
			  PRESENT_LOAD_L, 		// 0x28
			  PRESENT_LOAD_H, 		// 0x29
			  PRESENT_VOLTAGE, 		// 0x2A
			  PRESENT_TEMPERATURE, 		// 0x2B
			  REGISTERED_INSTRUCTION, 	// 0x2C
			  RESERVE3, 			// 0x2D
			  MOVING, 			// 0x2E
			  LOCK, 			// 0x2F
			  PUNCH_L, 			// 0x30
			  PUNCH_H			// 0x31
			};


//function protos
void G15ShieldInit(long baud, char G15_CTRL, char AX12_CTRL);
void set_act(char ctrl);
//void waitTXC(void);


class G15
{
	public:
		uint8_t ServoID;

		G15(uint8_t ID) ; //, char ctrl);
		void init(void);

		//*=========Wheel Mode=====================================================================================
		//360 degree continous rotation. change CW and CCW Angle Limits to same value
		uint16_t SetWheelMode(void);
		uint16_t ExitWheelMode(void);
		virtual uint16_t SetWheelSpeed(uint16_t Speed, uint8_t CW_CCW);


		//*=========Normal Positioning Mode========================================================================
		//(Rotation limited by Angle Limit and Direction of Rotation determined by operation section of Angle Limit)
		uint16_t SetPos(uint16_t Position, uint8_t Write_Reg);
		//uint8_t SetPosAngle(uint16_t Angle, uint8_t Write_Reg); 	//replaced with ConvertAngle()

		//*========Direction Positioning Mode======================================================================
		//(Rotation direction and angle is NOT limited by Angle Limit Control Register value)
		uint16_t RotateCW (uint16_t Position, uint8_t Write_Reg);
		uint16_t RotateCCW (uint16_t Position, uint8_t Write_Reg);

		//*=======Torque Enable and Speed Control==================================================================
		uint16_t SetTorqueOnOff(uint8_t on_off, uint8_t Write_Reg);
		uint16_t SetSpeed(uint16_t Speed, uint8_t Write_Reg);
		uint16_t SetTimetoGoal(uint16_t Time,uint8_t Write_Reg);

		//*=======Set Maximum Limits===============================================================================
		uint16_t SetAngleLimit(uint16_t CW_angle, uint16_t CCW_angle);
		uint16_t SetTorqueLimit(uint16_t TorqueLimit); //in RAM area
		uint16_t SetTemperatureLimit(uint8_t Temperature);
		uint16_t SetVoltageLimit(uint8_t VoltageLow, uint8_t VoltageHigh);

		uint16_t SetID(uint8_t NewID);

		uint16_t SetLED(uint8_t on_off, uint8_t Write_Reg);
		uint16_t SetAlarmLED(uint8_t AlarmLED);
		uint16_t SetAlarmShutDown(uint8_t Alarm);


		//*========Servo Positioning Control Parameters============================================================
		uint16_t SetMarginSlopePunch(uint8_t CWMargin, uint8_t CCWMargin, uint8_t CWSlope, uint8_t CCWSlope, uint16_t Punch);



		uint16_t SetBaudRate(long bps);

		uint16_t FactoryReset(void);

		uint16_t Ping(uint8_t* data);

		uint16_t GetPos(uint8_t* data);
		uint16_t GetSpeed(uint8_t* data);
		uint16_t GetLoad(uint8_t* data);
		uint16_t GetVoltage(uint8_t* data);
		uint16_t GetTemperature(uint8_t* data);
		uint16_t GetTorqueOnOff(uint8_t* data);
		uint16_t IsMoving(uint8_t* data);

		static void SetAction(void);




	protected:

		char TxRx;

		void setRX(void);
		void setTX(void);
		uint16_t send_packet(uint8_t ID, uint8_t inst, uint8_t* data, uint8_t param_len);
		uint8_t read_data(uint8_t id, uint8_t* data);



};

class AX12:public G15{

public:
	  AX12(uint8_t ID) ;//, char ctrl);
	  uint16_t SetBaudRate(long bps);
	  void init(void);
	  static void SetAction(void);

protected:



};




#endif
