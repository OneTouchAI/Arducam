#include "ArduTest.h"

ArduTest::ArduTest(int CS_)
{
	CS = CS_;
    pinMode(CS, OUTPUT);
}

void ArduTest::clear_fifo_flag(void )
{
	spi_write(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void ArduTest::flush_fifo(void)
{
	spi_write(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint8_t ArduTest::read_fifo(void)
{
	uint8_t data;
	data = spi_read(SINGLE_FIFO_READ);
	return data;
}

void ArduTest::start_capture(void)
{
	spi_write(ARDUCHIP_FIFO, FIFO_START_MASK);
}

uint32_t ArduTest::read_fifo_length(void)
{
	uint32_t len1,len2,len3,length=0;
	len1 = spi_read(FIFO_SIZE1);
    len2 = spi_read(FIFO_SIZE2);
    len3 = spi_read(FIFO_SIZE3) & 0x7f;
    length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
	return length;	
}

byte ArduTest::wrSensorReg8_8(int regID, int regDat)
{
    Wire.beginTransmission(sensor_addr >> 1);
    Wire.write(regID & 0x00FF);
    Wire.write(regDat & 0x00FF);
    if (Wire.endTransmission())
    {
    return 0;
    }
    delay(1);

	return 1;
}

byte ArduTest::rdSensorReg8_8(uint8_t regID, uint8_t *regDat)
{
    Wire.beginTransmission(sensor_addr >> 1);
    Wire.write(regID & 0x00FF);
    Wire.endTransmission();

    Wire.requestFrom((sensor_addr >> 1), 1);
    if (Wire.available())
    *regDat = Wire.read();
    delay(1);

	return 1;
}

void ArduTest::spi_write(uint16_t address, uint16_t data)
{
    digitalWrite(CS, LOW);

    SPI.transfer(address | 0x80);
    SPI.transfer(data);

    digitalWrite(CS, HIGH);
}

uint8_t ArduTest::spi_read(uint16_t address)
{
    digitalWrite(CS, LOW);

    SPI.transfer(address & 0x7F);
    uint8_t value = SPI.transfer(0x00);

    digitalWrite(CS, HIGH);

    return value;
}

byte ArduTest::wrSensorReg8_16(int regID, int regDat)
{
    Wire.beginTransmission(sensor_addr >> 1);
    Wire.write(regID & 0x00FF);

    Wire.write(regDat >> 8);            // sends data byte, MSB first
    Wire.write(regDat & 0x00FF);

    if (Wire.endTransmission())
    {
        return 0;
    }	

    delay(1);
    return 1;
}

byte ArduTest::rdSensorReg8_16(uint8_t regID, uint16_t *regDat)
{
    uint8_t temp;
    Wire.beginTransmission(sensor_addr >> 1);
    Wire.write(regID);
    Wire.endTransmission();

    Wire.requestFrom((sensor_addr >> 1), 2);

    if (Wire.available())
    {
    temp = Wire.read();
    *regDat = (temp << 8) | Wire.read();
    }

    delay(1);
    return 1;
}

uint8_t ArduTest::get_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = spi_read(addr);
  temp = temp & bit;
  return temp;
}

void ArduTest::CS_HIGH(void)
{
    digitalWrite(CS, HIGH);
}

void ArduTest::CS_LOW(void)
{
    digitalWrite(CS, LOW);
}

void ArduTest::set_fifo_burst()
{
    SPI.transfer(BURST_FIFO_READ);
}

void ArduTest::set_format(byte fmt)
{
  if (fmt == BMP)
    m_fmt = BMP;
  else if(fmt == RAW)
    m_fmt = RAW;
  else
    m_fmt = JPEG;
}

void ArduTest::InitCAM()
{
    wrSensorReg8_8(0xff, 0x01);
    wrSensorReg8_8(0x12, 0x80);

    delay(100);

    if (m_fmt == JPEG)
    {
        wrSensorRegs8_8(OV2640_JPEG_INIT);
        wrSensorRegs8_8(OV2640_YUV422);
        wrSensorRegs8_8(OV2640_JPEG);
        wrSensorReg8_8(0xff, 0x01);
        wrSensorReg8_8(0x15, 0x00);
        wrSensorRegs8_8(OV2640_320x240_JPEG);
        //wrSensorReg8_8(0xff, 0x00);
        //wrSensorReg8_8(0x44, 0x32);
    }
    else
    {
        wrSensorRegs8_8(OV2640_QVGA);
    }
}

void ArduTest::OV2640_set_JPEG_size(uint8_t size)
{
    switch(size)
	{
		case OV2640_160x120:
			wrSensorRegs8_8(OV2640_160x120_JPEG);
			break;
		case OV2640_176x144:
			wrSensorRegs8_8(OV2640_176x144_JPEG);
			break;
		case OV2640_320x240:
			wrSensorRegs8_8(OV2640_320x240_JPEG);
			break;
		case OV2640_352x288:
	  	wrSensorRegs8_8(OV2640_352x288_JPEG);
			break;
		case OV2640_640x480:
			wrSensorRegs8_8(OV2640_640x480_JPEG);
			break;
		case OV2640_800x600:
			wrSensorRegs8_8(OV2640_800x600_JPEG);
			break;
		case OV2640_1024x768:
			wrSensorRegs8_8(OV2640_1024x768_JPEG);
			break;
		case OV2640_1280x1024:
			wrSensorRegs8_8(OV2640_1280x1024_JPEG);
			break;
		case OV2640_1600x1200:
			wrSensorRegs8_8(OV2640_1600x1200_JPEG);
			break;
		default:
			wrSensorRegs8_8(OV2640_320x240_JPEG);
			break;
	}
}

int ArduTest::wrSensorRegs8_8(const struct sensor_reg reglist[])
{
    int err = 0;
    uint16_t reg_addr = 0;
    uint16_t reg_val = 0;
    const struct sensor_reg *next = reglist;
    while ((reg_addr != 0xff) | (reg_val != 0xff))
    {
    reg_addr = pgm_read_word(&next->reg);
    reg_val = pgm_read_word(&next->val);
    err = wrSensorReg8_8(reg_addr, reg_val);
    next++;
    }

	return 1;
}