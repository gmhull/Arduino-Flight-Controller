///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(){
  //Read the MPU-6050
  Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
  Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                                 //End the transmission.
  Wire.requestFrom(gyro_address,14);                                      //Request 14 bytes from the gyro.

  receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiver_input_channel_3 = convert_throttle_receiver_channel(3);        //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

  while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
  acc_axis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_x variable.
  acc_axis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_y variable.
  acc_axis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_z variable.
  temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
  gyro_axis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  gyro_axis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  gyro_axis[3] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.

  //Compensate for gyro offsets once the calibration has been completed.
  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                                     //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                     //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                     //Only compensate after the calibration.
  }

  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                    //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;                        //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                   //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                       //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                     //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                         //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                         //Set acc_x to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                            //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                         //Set acc_y to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                            //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                         //Set acc_z to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                            //Invert acc_z if the MSB of EEPROM bit 30 is set.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for setting up the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_gyro_registers(){
  //Setup the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                                 //Start communication with the address found during search.
    Wire.write(0x6B);                                                     //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                     //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                               //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                 //Start communication with the address found during search.
    Wire.write(0x1B);                                                     //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                     //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                               //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                 //Start communication with the address found during search.
    Wire.write(0x1C);                                                     //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                     //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                               //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                                 //Start communication with the address found during search
    Wire.write(0x1B);                                                     //Start reading @ register 0x1B
    Wire.endTransmission();                                               //End the transmission
    Wire.requestFrom(gyro_address, 1);                                    //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                          //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){                                              //Check if the value is 0x08
      digitalWrite(12,HIGH);                                              //Turn on the warning led
      while(1)delay(10);                                                  //Stay in this loop for ever
    }

    Wire.beginTransmission(gyro_address);                                 //Start communication with the address found during search
    Wire.write(0x1A);                                                     //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                     //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                               //End the transmission with the gyro
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calibrating the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(void) {
  //Lets take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){                         //Take 2000 readings for calibration.
    if(cal_int % 15 == 0)digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));    //Change the led status to indicate calibration.
    gyro_signalen();                                                      //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];                                     //Add pitch value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];                                     //Add roll value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];                                     //Add yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping, so let's give them a 1000us pulse while calibrating the gyro.
    PORTB |= B00001111;                                                   //Set digital poort 8, 9, 10 and 11 high.
    delayMicroseconds(1000);                                              //Wait 1000us.
    PORTB &= B11110000;                                                   //Set digital poort 8, 9, 10 and 11 low.
    delay(3);                                                             //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to divide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;                                               //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;                                               //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000;                                               //Divide the yaw total by 2000.

  //Print out the cal data.
//  Serial.print("Gyro Pitch Cal: ");
//  Serial.println(gyro_axis_cal[1]);  
//  Serial.print("Gyro Roll Cal: ");
//  Serial.println(gyro_axis_cal[2]);
//  Serial.print("Gyro Yaw Cal: ");
//  Serial.println(gyro_axis_cal[3]);
}
