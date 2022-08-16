///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for getting the altitude calibration data from the barometer
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_barometer() {
  //Setup the bmp280 chip
  Serial.println("Start BMP"); 
  bmp.begin();                                     //Begin connection with the BMP280 chip
  //Configure the BMP280 sampling settings.  This will only be used to calculate the calibration values.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode. 
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling 
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering
                  Adafruit_BMP280::STANDBY_MS_1);   // Standby time set to 0.5 ms
                  
  //Let's take 2000 pressure data samples on the ground so that we can determine the average base pressure reading (calibration).
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){
    altitude_cal += bmp.readPressure();
  }
  //Divide by number of cycles * 100 to get to the correct units
  //Pascals to Hectopascals
  altitude_cal /= 200000;     
 
  Serial.print("Alt cal: ");
  Serial.println(altitude_cal);
  Serial.println("bmp280 ready");
}
