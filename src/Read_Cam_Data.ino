#include <Wire.h>
#include <ArduTest.h>
#include <SPI.h>

#include <JPEGDecoder.h>
//#include "memorysaver.h"

#define JPEGIMAGEOFFSET 625

//#include "ov2640_regs.h"

// Global Var
const int CS = 17;
bool is_header = false;
uint8_t vid, pid;
uint8_t temp;
ArduTest myCAM(CS);

const uint16_t width = 1600;
const uint16_t height = 1200;

static const size_t bufferSize = 2*2048;
static uint8_t buffer[bufferSize] = {0xFF};

const char JPEG_header[625] PROGMEM =
{
  0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 0x4a, 0x46, 0x49, 0x46, 0x00, 0x01, 0x02, 0x00, 0x00, 0x01,
  0x00, 0x01, 0x00, 0x00, 0xff, 0xdb, 0x00, 0x84, 0x00, 0x03, 0x02, 0x02, 0x03, 0x02, 0x02, 0x03,
  0x03, 0x02, 0x03, 0x03, 0x03, 0x03, 0x04, 0x05, 0x08, 0x05, 0x05, 0x04, 0x04, 0x05, 0x09, 0x07,
  0x07, 0x05, 0x08, 0x0b, 0x0a, 0x0b, 0x0b, 0x0b, 0x0a, 0x0b, 0x0a, 0x0c, 0x0e, 0x11, 0x0f, 0x0c,
  0x0d, 0x10, 0x0d, 0x0a, 0x0b, 0x0f, 0x14, 0x0f, 0x10, 0x12, 0x12, 0x13, 0x14, 0x13, 0x0c, 0x0e,
  0x15, 0x17, 0x15, 0x13, 0x17, 0x11, 0x13, 0x13, 0x13, 0x01, 0x03, 0x03, 0x03, 0x05, 0x04, 0x05,
  0x09, 0x05, 0x05, 0x09, 0x13, 0x0c, 0x0b, 0x0c, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13,
  0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13,
  0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13,
  0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0xff, 0xc0, 0x00, 0x11, 0x08, 0x04,
  0x80, 0x06, 0x40, 0x03, 0x00, 0x21, 0x00, 0x01, 0x11, 0x01, 0x02, 0x11, 0x01, 0xff, 0xc4, 0x00,
  0x1f, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0xff, 0xc4,
  0x00, 0xb5, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00,
  0x00, 0x01, 0x7d, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13,
  0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15,
  0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25,
  0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46,
  0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66,
  0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86,
  0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4,
  0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2,
  0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9,
  0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5,
  0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xff, 0xc4, 0x00, 0x1f, 0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
  0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0xff, 0xc4, 0x00, 0xb5, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04,
  0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11,
  0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08,
  0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a,
  0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35,
  0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55,
  0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75,
  0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93,
  0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa,
  0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8,
  0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6,
  0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xff, 0xdd, 0x00,
  0x04, 0x00, 0x20, 0xff, 0xda, 0x00, 0x0c, 0x03, 0x00, 0x00, 0x01, 0x11, 0x02, 0x11, 0x00, 0x3f,
  0x00
};

void setup() {
  // put your setup code here, to run once:
  // Code Works on UNO...  But dont understand format

  Wire.begin();
  Serial.begin(9600);

  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  SPI.begin();
  //Reset the CPLD
  myCAM.spi_write(0x07, 0x80);
  delay(100);
  myCAM.spi_write(0x07, 0x00);
  delay(100);

  while(1){
  //Check if the ArduCAM SPI bus is OK
  myCAM.spi_write(ARDUCHIP_TEST1, 0x55);
  temp = myCAM.spi_read(ARDUCHIP_TEST1);
    if (temp != 0x55){
      Serial.println(F("ACK CMD SPI interface Error! END"));
      delay(1000);continue;
    }else{
      Serial.println(F("ACK CMD SPI interface OK. END"));break;
    }
  }

  while(1){
    //Check if the camera module type is OV2640
    myCAM.wrSensorReg8_8(0xff, 0x01);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))){
      Serial.println(F("ACK CMD Can't find OV2640 module! END"));
      delay(1000);continue;
    }
    else{
      Serial.println(F("ACK CMD OV2640 detected. END"));break;
    } 
  }

  Serial.println(F("Test"));

  myCAM.set_format(JPEG);
  myCAM.InitCAM();

  myCAM.OV2640_set_JPEG_size(OV2640_176x144);

  delay(1000);
  myCAM.clear_fifo_flag();

  //myCAM.write_reg(ARDUCHIP_FRAMES,0x00);  #define ARDUCHIP_FRAMES			  0x01

  while (1) {
    myCAMSendViaSerial(myCAM);
    delay(1000);
  }

  while (1) {
    buffer[bufferSize] = {0xFF};

    delay(1000);

    myCAM.flush_fifo();
    delay(30);
    myCAM.clear_fifo_flag();
    delay(30);
    //Start capture
    myCAM.start_capture();
    delay(30);

    //camCapture(myCAM);

   //if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
   //{
   //  read_fifo_burst(myCAM);
   //  myCAM.clear_fifo_flag();
   //}

    delay(3000);
  }

  while (1) {
  // put your main code here, to run repeatedly:

  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  //Start capture
  myCAM.start_capture();

    if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
    {
      delay(50);

      uint8_t temp = 0, temp_last = 0;
      uint32_t length = 0;
      length = myCAM.read_fifo_length();
      Serial.println(length, DEC);

      //Clear the capture done flag
      myCAM.clear_fifo_flag();

      myCAM.CS_LOW();
      myCAM.set_fifo_burst();//Set fifo burst mode
      temp =  SPI.transfer(0x00);
      length --;
      while ( length-- )
      {
        temp_last = temp;
        temp =  SPI.transfer(0x00);
        if (is_header == true)
        {
          Serial.print(temp, HEX);
        }
        else if ((temp == 0xD8) & (temp_last == 0xFF))
        {
          is_header = true;
          Serial.println(F("ACK IMG END"));
          Serial.println(temp_last, HEX);
          Serial.println(temp, HEX);
        }
        if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
        break;
        delayMicroseconds(15);
      }
      myCAM.CS_HIGH();
      is_header = false;
        }

      }
}

void loop() {

}

uint8_t read_fifo_burst(ArduTest myCAM)
{
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  length = myCAM.read_fifo_length();
  Serial.println(length, DEC);

  if (length >= MAX_FIFO_SIZE) //512 kb
  {
    Serial.println("ACK CMD Over size. END");
    return 0;
  }
  if (length == 0 ) //0 kb
  {
    Serial.println("ACK CMD Size is 0. END");
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();//Set fifo burst mode
  
  temp =  SPI.transfer(0x00);
  length --;
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);

    if (is_header == true)
    {
      Serial.print(temp, HEX);
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      Serial.println("ACK IMG END");
      Serial.print(temp_last, HEX);
      Serial.print(temp, HEX);
    }
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    break;
    delayMicroseconds(15);
  }
  myCAM.CS_HIGH();
  is_header = false;
  return 1;
}

void myCAMSendViaSerial(ArduTest myCam) {
  byte buf[256];
  static int i = 0;
  static int k = 0;
  static int n = 0;
  uint8_t temp, temp_last;

  //Flush the FIFO
  myCAM.flush_fifo();

  //Clear the capture done flag
  myCAM.clear_fifo_flag();

  //Start capture
  myCAM.start_capture();
  Serial.println("Start Capture");

  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  Serial.println("Capture Done!");

  i = 0;
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  #if !(defined (ARDUCAM_SHIELD_V2) && defined (OV2640_CAM))
    SPI.transfer(0xFF);
  #endif

  //Read JPEG data from FIFO and send via serial
  while ((temp != 0xD9) | (temp_last != 0xFF)) {
    temp_last = temp;
    temp = SPI.transfer(0x00);

    //Write image data to buffer if not full
    if (i < 256)
      buf[i++] = temp;
    else {
      //Write 256 bytes image data to serial in hexadecimal format
      for (int j = 0; j < i; j++) {
        if (buf[j] <= 15){
          Serial.print(0);
          Serial.print(buf[j], HEX);
        } else {
          Serial.print(buf[j], HEX);
        }
      }
      i = 0;
      buf[i++] = temp;
    }
    delay(0);
  }

  //Write the remaining bytes in the buffer
  if (i > 0) {
    for (int j = 0; j < i; j++) {
      if (buf[j] <= 15){
          Serial.print(0);
          Serial.print(buf[j], HEX);
      } else {
          Serial.print(buf[j], HEX);
      }
    }
  }

  myCAM.CS_HIGH();
  Serial.println("CAM Send Done!");
}