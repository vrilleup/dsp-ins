/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	ADIS16350/16355 Data Read
	By VrilleUp (vrilleup.pu@gmail.com), 2008.08.14
*/

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#include "ADIS16350.h"
#include "INS_Init.h"

#pragma CODE_SECTION(ADIS_Command, "xintfloadfuncs");
#pragma CODE_SECTION(ADIS_Init, "xintfloadfuncs");
#pragma CODE_SECTION(ADIS_ReadData, "xintfloadfuncs");


#define SPI_READ(x) SPIA_xmit(x)
#define TWOCOMP14(x) (((int16)(x)<0x2000)?(int16)(x):-(0x4000-(int16)(x)))
#define TWOCOMP12(x) (((int16)(x)<0x0800)?(int16)(x):-(0x1000-(int16)(x)))

#define SUPPLY_OUT	0x02
#define XGYRO_OUT	0x04
#define YGYRO_OUT	0x06
#define ZGYRO_OUT	0x08
#define XACCL_OUT	0x0A
#define YACCL_OUT	0x0C
#define ZACCL_OUT	0x0E
#define XTEMP_OUT	0x10
#define YTEMP_OUT	0x12
#define ZTEMP_OUT	0x14
#define AUX_ADC		0x16

#define ENDURANCE	0x00
#define XGYRO_OFF	0x1A
#define YGYRO_OFF	0x1C
#define ZGYRO_OFF	0x1E
#define XACCL_OFF	0x20
#define YACCL_OFF	0x22
#define ZACCL_OFF	0x24
#define ALM_MAG1	0x26
#define ALM_MAG2	0x28
#define ALM_SMPL1	0x2A
#define ALM_SMPL2	0x2C
#define ALM_CTRL	0x2E
#define AUX_DAC		0x30
#define GPIO_CTRL	0x32
#define MSC_CTRL	0x34
#define SMPL_PRD	0x36
#define SENS_AVG	0x38
#define SLP_CNT		0x3A
#define STATUS		0x3C
#define COMMAND		0x3E

Uint16 ADIS_Command(unsigned char addr, unsigned char data, BOOL IsWrite);

Uint16 ADIS_Command(unsigned char addr, unsigned char data, BOOL IsWrite)
{
	Uint16 result = 0;
	Uint16 tmp = 0;

	SPIA_Mode(1);
	
	CS_IMU();

	if (IsWrite)
		tmp = SPI_READ(0x80 | (addr & 0x3F));
	else
		tmp = SPI_READ(0x00 | (addr & 0x3F));	

	result = tmp<<8;
	tmp = 0;
	tmp = SPI_READ(data);
	result = result | tmp;
	
	CS_NONE();
	DELAY_US(15);
	return result;
}

void ADIS_Init(Uint16 d)
{
	Uint16 t;
	/*
	SMPL_PRD Register
	[15:8] Not used
	[7] Time base, 0 = 0.61035 ms, 1 = 18.921 ms
	[6:0] Multiplier (add 1 before multiplying by the time base)
	*/
	ADIS_Command(SMPL_PRD  , 0x01, TRUE);
	ADIS_Command(SMPL_PRD+1, 0x00, TRUE);	
	
	/*
	SENS_AVG = 00000 100 00000 100
	                 [1]       [2]
	[1] 100 = 300 deg/s [*]
	    010 = 150 deg/s
	    001 =  75 deg/s
	[2] Filter tap = N = 2^M (M=[2])
	    N = 64 = 8Hz			0x06
	    N = 16 = 30Hz   [*]		0x04
	    N = 8  = 160Hz			0x03
	*/
	ADIS_Command(SENS_AVG  , 0x04, TRUE);
	ADIS_Command(SENS_AVG+1, 0x04, TRUE);

	/*
	MSC_CTRL Register
	[15:11] Not used
	[10] Internal self-test enable (clears on completion) 1 = enabled, 0 = disabled 
	[9] Manual self-test, negative stimulus 1 = enabled, 0 = disabled 
	[8] Manual self-test, positive stimulus 1 = enabled, 0 = disabled 
	[7] Linear acceleration bias compensation for gyroscopes 1 = enabled, 0 = disabled 
	[6] Linear accelerometer origin alignment 1 = enabled, 0 = disabled 
	[5:3] Not used 
	[2] Data-ready enable 1 = enabled, 0 = disabled 
	[1] Data-ready polarity 1 = active high, 0 = active low 
	[0] Data-ready line select 1 = DIO2, 0 = DIO1
	*/
	ADIS_Command(MSC_CTRL  , 0xC0, TRUE);
	ADIS_Command(MSC_CTRL+1, 0x00, TRUE);

	/*
	t = ADIS_Command(MSC_CTRL  , 0xC0, FALSE);
	t = ADIS_Command(MSC_CTRL  , 0xC0, FALSE);
	t = ADIS_Command(MSC_CTRL  , 0xC0, FALSE);
	*/

	/*
	ALM_SMPL1 and ALM_SIMPL2 Register
	[15:8] Not used
	[7:0] Data bits
	*/
	ADIS_Command(ALM_SMPL1  , d & 0xff,   TRUE);
	ADIS_Command(ALM_SMPL1+1, 0x00, TRUE);
	ADIS_Command(ALM_SMPL2  , (d>>8) & 0xff,   TRUE);
	ADIS_Command(ALM_SMPL2+1, 0x00, TRUE);

	/*
	COMMAND Register
	[15:8] Not used
	[7] Software reset command 
	[6:5] Not used 
	[4] Precision autonull command
	[3] Flash update command
	[2] Auxiliary DAC data latch 
	[1] Factory calibration restore command 
	[0] Autonull command

	Save the Registers above
	require 100ms
	*/
	// ADIS_Command(COMMAND, 0x08, TRUE);

}

/*
Uint16 ADIS_ReadParameter()
{
	Uint16 result = 0;
	
	//ALM_SMPL1 and ALM_SIMPL2 Register
	//[15:8] Not used
	//[7:0] Data bits
	
	ADIS_Command(ALM_SMPL2  , 0,   FALSE);
	result = ADIS_Command(ALM_SMPL1  , 0,   FALSE);
	result = result << 8;
	result = result | (ADIS_Command(ALM_SMPL1  , 0,   FALSE) & 0xff);

	return result;
}

void ADIS_AutoNull()
{
	// for gyros auto null
	ADIS_Command(COMMAND, 0x01, TRUE);
}

void ADIS_PreciseAutoNull()
{
	// for gyros precise auto null
	// require about 35s
	ADIS_Command(COMMAND, 0x10, TRUE);
}
*/

void ADIS_ReadData(float32* data)
{
	Uint16 result = 0;
	
	// read gyro and acceleration data
	// wx, wy, wz in rad/s
	// ax, ay, az in g

	// GYRO 300deg/s : 0.07326  deg/s = 1LSB
	// ACCL          : 0.002522 g     = 1LSB

			 ADIS_Command(XGYRO_OUT, 0, FALSE);
	result = ADIS_Command(YGYRO_OUT, 0, FALSE) & 0x3FFF;
	data[0] = -0.07326*TWOCOMP14(result);
	result = ADIS_Command(ZGYRO_OUT, 0, FALSE) & 0x3FFF;
	data[1] =  0.07326*TWOCOMP14(result);
	result = ADIS_Command(XACCL_OUT, 0, FALSE) & 0x3FFF;
	data[2] = -0.07326*TWOCOMP14(result);
	result = ADIS_Command(YACCL_OUT, 0, FALSE) & 0x3FFF;
	data[3] = -0.002522*TWOCOMP14(result);
	result = ADIS_Command(ZACCL_OUT, 0, FALSE) & 0x3FFF;
	data[4] =  0.002522*TWOCOMP14(result);
	result = ADIS_Command(ZACCL_OUT, 0, FALSE) & 0x3FFF;
	data[5] = -0.002522*TWOCOMP14(result);

}

/*
int16 ADIS_ReadTemperature(Uint16 axis)
{
	// return Temperature in 0.1¡ãC
	// 1LSB = 0.1453¡ãC
	
	Uint16 result = 0;
	Uint16 addr;

	if (axis == 0)
		addr = XTEMP_OUT;
	else if (axis == 1)
		addr = YTEMP_OUT;
	else if (axis == 3)
		addr = ZTEMP_OUT;
	else
		return 0;

			 ADIS_Command(addr, 0, FALSE);
	result = ADIS_Command(addr, 0, FALSE);
	
	return (int16)(0.1453*TWOCOMP12(result)*10.0);

}

Uint16 ADIS_ReadAUX_ADC()
{
	// return AUX_ADC in 0.1mV
	// 1LSB = 0.6105mV

	Uint16 result = 0;
	
			 ADIS_Command(AUX_ADC, 0, FALSE);
	result = ADIS_Command(AUX_ADC, 0, FALSE);

	return (Uint16)(6.105*result);
}

*/

