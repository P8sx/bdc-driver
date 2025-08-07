/*
 * modbus.h
 *
 *  Created on: Jun 20, 2025
 *      Author: phsx
 */

#ifndef SHARED_MODBUS_H_
#define SHARED_MODBUS_H_


/*
 * com.h
 *
 *  Created on: Jun 15, 2025
 *      Author: phsx
 */

#ifndef COM_H_
#define COM_H_

#include "microtbx.h"
#include "microtbxmodbus.h"
#include "main.h"


extern uint8_t rxByte;
extern tTbxMbTp modbusTp;
extern tTbxMbServer modbusServer;


#define DEFAULT_MODBUS_ADDRESS		  		 		0x0A
#define DEFAULT_MODBUS_BOOTLOADER_ADDRESS	 		0x0A

#define COIL_REG_JUMP								1000U		// Read/Write coil REG, if 1 written jump to APP, if 0 - jump to BOOTLOADER

#define INPUT_REG_BUILD_DATE						39000U	// string (30 registers)
#define INPUT_REG_BUILD_DATE_SIZE					30U
#define INPUT_REG_VERSION							39050U  // string (30 registers)
#define INPUT_REG_VERSION_SIZE						30U

#ifdef BOOTLOADER

	#define COIL_REG_OTW_PERFORM_UPDATE  			1001U	// uint8_t



	#define INPUT_REG_FLASH_PAGE_CRC32				30000U	// uint32_t (2 registers)



	#define HOLDING_REG_FLASH_PAGE_NUMBER  			40000U	// uint8_t
	#define HOLDING_REG_FLASH_PAGE_START_ADDR		41000U	// uint8_t[2048] (1024 registers)
	#define HOLDING_REG_FLASH_PAGE_SIZE	    		1024U


#elif defined(APP)

	#define COIL_REG_VALIDATE_UPDATE 				1002U
	#define COIL_REG_MOTOR_INVERT_CONTROL         	1U      // uint8_t
	#define COIL_REG_MOTOR_NTC_ENABLED            	2U      // uint8_t
	#define COIL_REG_POSITION_STOP_ON_TARGET      	3U      // uint8_t
	#define COIL_REG_ENDSTOP_CW_ENABLED           	4U      // uint8_t
	#define COIL_REG_ENDSTOP_CCW_ENABLED          	5U      // uint8_t
	#define COIL_REG_ESTOP				          	6U      // uint8_t

	#define INPUT_CONTACTS_MOTOR_OCLN				10000U  // uint8_t

	#define INPUT_REG_MOTOR_TEMP					30000U	// float (2 registers)
	#define INPUT_REG_BRIDGE_TEMP					30002U	// float (2 registers)
	#define INPUT_REG_VOLTAGE						30004U  // float (2 registers)

	#define INPUT_REG_ENC_RPS						30006U	// int16_t
	#define INPUT_REG_ENC_ABSOLUTE_POSITION			30008U	// int64_t (4 registers)


	#define HOLDING_REG_SET_MOTOR_RPS		       	40002U  // int16_t

	#define HOLDING_REG_SET_POSITION				40004U	// int64_t (4 registers)

	#define HOLDING_REG_SET_MOTOR_PWMA		       	40008U  // uint16_t
	#define HOLDING_REG_SET_MOTOR_PWMB		       	40010U  // uint16_t


	#define HOLDING_REG_MOTOR_TEMP_HYS            	40012U  // float (2 registers)
	#define HOLDING_REG_MOTOR_OVER_TEMP_LIMIT     	40014U  // float (2 registers)

	#define HOLDING_REG_BRIDGE_CURRENT_LIMIT      	40016U  // float (2 registers)
	#define HOLDING_REG_BRIDGE_OVERCURRENT_LIMIT 	40018U  // float (2 registers)

	#define HOLDING_REG_BRIDGE_OVERCURRENT_TIME  	40020U  // uint16_t

	#define HOLDING_REG_PID_KP                    	40022U  // float (2 registers)
	#define HOLDING_REG_PID_KD                    	40024U  // float (2 registers)
	#define HOLDING_REG_PID_KI                    	40026U  // float (2 registers)

	#define HOLDING_REG_POSITION_TOLERANCE       	40028U  // uint16_t




#endif


void COM_Init(void);
void COM_MainLoop(void);
void COM_DeInit(void);


#ifdef BOOTLOADER

	void    COM_SetFlashPageTransferReady();
	uint8_t COM_GetFlashPageTransferStatus();
	uint8_t COM_GetFlashPage(uint8_t* destBuffer);
	uint8_t COM_PerformUpdate();

#elif defined(APP)

	uint8_t COM_GetOTWRequest();

#endif


tTbxMbServerResult AppWriteCoil(tTbxMbServer channel, uint16_t addr, uint8_t value);
tTbxMbServerResult AppReadCoil(tTbxMbServer channel, uint16_t addr, uint8_t* value);

tTbxMbServerResult AppWriteReg(tTbxMbServer channel, uint16_t addr, uint16_t value);
tTbxMbServerResult AppReadReg(tTbxMbServer channel, uint16_t addr, uint16_t *value);



#endif /* COM_H_ */










#endif /* SHARED_MODBUS_H_ */
