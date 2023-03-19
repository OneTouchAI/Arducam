#pragma once

#include "Arduino.h"
#include "ov2640.h"
#include <SPI.h>
#include <Wire.h>
#include "HardwareSerial.h"

//Fifo
#define MAX_FIFO_SIZE		0x5FFFF

//Device Specific Definitions
#define regtype volatile uint8_t
#define regsize uint8_t

//ID
#define OV2640_CHIPID_HIGH 	0x0A
#define OV2640_CHIPID_LOW 	0x0B

//Hardware Platform
#define OV2640_MINI_2MP

//Hardware Type
#define OV2640_CAM

//Sensor Related Definitions
#define BMP 	0
#define JPEG	1
#define RAW	  2

#define OV2640  	5

#define OV2640_160x120 		0	//160x120
#define OV2640_176x144 		1	//176x144
#define OV2640_320x240 		2	//320x240
#define OV2640_352x288 		3	//352x288
#define OV2640_640x480		4	//640x480
#define OV2640_800x600 		5	//800x600
#define OV2640_1024x768		6	//1024x768
#define OV2640_1280x1024	7	//1280x1024
#define OV2640_1600x1200	8	//1600x1200

//Arduchip Register Definitions
#define RWBIT									0x80  //READ AND WRITE BIT IS BIT[7]

#define ARDUCHIP_TEST1       	0x00  //TEST register

#define ARDUCHIP_FRAMES			  0x01  //FRAME control register, Bit[2:0] = Number of frames to be captured																		//On 5MP_Plus platforms bit[2:0] = 7 means continuous capture until frame buffer is full

#define ARDUCHIP_MODE      		0x02  //Mode register
#define MCU2LCD_MODE       		0x00
#define CAM2LCD_MODE       		0x01
#define LCD2MCU_MODE       		0x02

#define ARDUCHIP_TIM       		0x03  //Timming control

#define HREF_LEVEL_MASK    		0x01  //0 = High active , 		1 = Low active
#define VSYNC_LEVEL_MASK   		0x02  //0 = High active , 		1 = Low active
#define LCD_BKEN_MASK      		0x04  //0 = Enable, 					1 = Disable

#define ARDUCHIP_FIFO      		0x04  //FIFO and I2C control
#define FIFO_CLEAR_MASK    		0x01
#define FIFO_START_MASK    		0x02
#define FIFO_RDPTR_RST_MASK     0x10
#define FIFO_WRPTR_RST_MASK     0x20

#define ARDUCHIP_GPIO			  0x06  //GPIO Write Register

#define GPIO_RESET_MASK			0x01  //0 = Sensor reset,							1 =  Sensor normal operation
#define GPIO_PWDN_MASK			0x02  //0 = Sensor normal operation, 	1 = Sensor standby
#define GPIO_PWREN_MASK			0x04	//0 = Sensor LDO disable, 			1 = sensor LDO enable

#define BURST_FIFO_READ			0x3C  //Burst FIFO read operation
#define SINGLE_FIFO_READ		0x3D  //Single FIFO read operation

#define ARDUCHIP_REV       		0x40  //ArduCHIP revision
#define VER_LOW_MASK       		0x3F
#define VER_HIGH_MASK      		0xC0

#define ARDUCHIP_TRIG      		0x41  //Trigger source
#define VSYNC_MASK         		0x01
#define SHUTTER_MASK       		0x02
#define CAP_DONE_MASK      		0x08

#define FIFO_SIZE1				0x42  //Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2				0x43  //Camera write FIFO size[15:8]
#define FIFO_SIZE3				0x44  //Camera write FIFO size[18:16]

class ArduTest
{
    public:
	    ArduTest(int CS);
        void InitCAM(void);

        void flush_fifo(void);
        void start_capture(void);
        void clear_fifo_flag(void);
        uint8_t read_fifo(void);
        uint32_t read_fifo_length(void);
        void set_fifo_burst(void);

        byte wrSensorReg8_8(int regID, int regDat);
	    byte rdSensorReg8_8(uint8_t regID, uint8_t* regDat);

        void spi_write(uint16_t address, uint16_t data);
        uint8_t spi_read(uint16_t address);

        byte wrSensorReg8_16(int regID, int regDat);
        byte rdSensorReg8_16(uint8_t regID, uint16_t* regDat);

        uint8_t get_bit(uint8_t addr, uint8_t bit);

        void CS_HIGH(void);
	    void CS_LOW(void);

        void set_format(byte fmt);

        void OV2640_set_JPEG_size(uint8_t size);

        int wrSensorRegs8_8(const struct sensor_reg*);
	
    private:
        int CS;
        byte sensor_addr = 0x60;
        byte m_fmt;

        regtype *P_CS;
	    regsize B_CS;
};
