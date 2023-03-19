## What was changed:
* "sensor_addr = 0x60", Using I2C discovery script we found the sensor address has to be 0x60 to work.
* Test register for SPI and others were incorrect in the provided library.  Those were changed in the ArduTest libary.
* The initialization of the sensor with a Pico using the arduino IDE was not an option.  Had to rebuild this to what it is now in ArduTest.
* For actually storing the image, the function myCAMSendViaSerial() correctly sends a jpeg encoded image.  Check to make sure it is not missing the last 2 bytes.  You can replace 
* To initialize the camera you need the following:
    > ArduTest myCAM(CS);
    >
    > myCAM.set_format(JPEG);
    > myCAM.InitCAM();
    >
    > myCAM.OV2640_set_JPEG_size(OV2640_176x144);
    >
    > delay(1000);
    > myCAM.clear_fifo_flag();
* When printing 2 bytes, if the decimal number is less than 16, there is no leading zero... lol