///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading and converting the barometer data
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void get_barometer_data(void) {
  bar_count ++;

  //Reading the altitude directly takes too much.  It causes the program loop to go over the 4ms timer.
  //To solve this issue we need to read the temp and pressure separately.
  //We will read the temp every 20 cycles and read the pressure the other cycles.

  if (bar_count == 1) {
    //Read temp data from the bmp280
    if (temp_count == 0) {
      //Read the temperature data that was requested
      Wire.beginTransmission(bmp_address);                                    //Start conection to the barometer
      Wire.write(0xFA);                                                       //We want to read from the temp data at this register
      Wire.endTransmission();                                                 //End the transmission to the barometer
      Wire.requestFrom(bmp_address, 3);                                       //Request 3 bytes of data from the register
      raw_temperature_reading[1] = Wire.read();
      raw_temperature_reading[2] = Wire.read();
      raw_temperature_reading[3] = Wire.read();
      
    } else {
      //Read the pressure data that was requested
      Wire.beginTransmission(bmp_address);                                    //Start conection to the barometer
      Wire.write(0xF7);                                                       //We want to read from the pressure data at this register
      Wire.endTransmission();                                                 //End the transmission to the barometer
      Wire.requestFrom(bmp_address, 3);                                       //Request 3 bytes of data from the register

      raw_pressure_reading[1] = Wire.read();
      raw_pressure_reading[2] = Wire.read();
      raw_pressure_reading[3] = Wire.read();
    }
  }
    
  if (bar_count == 2) {  
    temp_count ++;
    if (temp_count == 20) {
      temp_count = 0; //Reset the temp counter back to 0
      //Request the temperature data
      
      //In forced mode we need to send a command every time we want to take a measurement.  
      Wire.beginTransmission(bmp_address);          //Start conection to the barometer
      Wire.write(0xF4);                             //Write to the ctrl_meas register
      Wire.write(0x22);                             //Send register bits 00100010 (1 temp measurement, no pressure measurement, forced mode)
      Wire.endTransmission();                       //End the transmission to the barometer

      //Calculate the average temp here to save time.  bar_count = 1 has the slowest cycle.      
      //Store the temperature in a rolling average to prevent spikes
      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      raw_temperature_rotating_memory[average_temperature_mem_location] = raw_temperature_reading[1] << 12 | raw_temperature_reading[2] << 4 | raw_temperature_reading[3] >> 4;
      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;                      //Calculate the average temperature of the last 5 measurements.
    } else {
      //Request the pressure data

      //In forced mode we need to send a command every time we want to take a measurement.  
      Wire.beginTransmission(bmp_address);          //Start conection to the barometer
      Wire.write(0xF4);                             //Write to the ctrl_meas register
      Wire.write(0x06);                             //Send register bits 00000110 (no temp measurement, 1 pressure measurement, forced mode)
      Wire.endTransmission();                       //End the transmission to the barometer

      //Calculate the average temp here to save time.  bar_count = 1 has the slowest cycle.  
      raw_pressure = raw_pressure_reading[1] << 12 | raw_pressure_reading[2] << 4 | raw_pressure_reading[3] >> 4;   //Shift the bytes into the correct positions to read
    }
  }

  if (bar_count == 3) {
    //Calculate the temp & pressure

    //Get temp data here
    float T = bmp.calculateTemperature(raw_temperature);    //Get the temperature and update the t_fine variable to calculate pressure
//    Serial.println(T);
    float P = bmp.calculatePressure(raw_pressure);          //Get the temperature in pascals

    //To get a smoother pressure value we will use a 20 location rotating memory.
    pressure_total_average -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                                //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_average += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term average value.
    pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_average / 20.0;                                              //Calculate the average pressure of the last 20 pressure readings.

    //To get better results, we will use a complementary filter that can be adjusted by the fast average.
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow average value.
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
    //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;                                                                   //The actual_pressure is used in the program for altitude calculations.
  }

  if (bar_count == 4) {
    bar_count = 0;

    //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term average value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.

    //Calculate the PID altitude output
    //Need to get set the pid_altitude_setpoint when we enable altitude hold mode 
    //Check the status register of the chip to see whether the measurements are ready to be taken
    if (flight_status >= 2 && current_height > auto_height_min){
      // If we are at least 2.5m off the ground and the sticks are level;
      if (pid_altitude_setpoint == 0)pid_altitude_setpoint = actual_pressure;         //If not yet set, set the PID altitude setpoint.
      //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
      //will indicate if the altitude of the quadcopter is changed by the pilot.
      manual_altitude_change = 0;                                                     //Preset the manual_altitude_change variable to 0.
      manual_throttle = 0;                                                            //Set the manual_throttle variable to 0.
      if (receiver_input_channel_3 > 1000 + throttle_signal_center + 75) {            //If the throtttle is increased above 1425us (60%).
        manual_altitude_change = 1;                                                   //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                      //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (receiver_input_channel_3 - \
                           1000 + throttle_signal_center + 75) / 3;                   //To prevent very fast changes in height limit the function of the throttle.
      }
      if (receiver_input_channel_3 < 1000 + throttle_signal_center - 75) {            //If the throtttle is lowered below 1275us (40%).
        manual_altitude_change = 1;                                                   //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                      //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (receiver_input_channel_3 - 
                           1000 + throttle_signal_center - 75) / 5;                   //To prevent very fast changes in height limit the function of the throttle.
      }
      
      pid_altitude_input = actual_pressure;                                           //Set the setpoint (pid_altitude_input) of the PID-controller.
      pid_error_temp_altitude = pid_altitude_input - pid_altitude_setpoint;           //Calculate the difference of actual altitude and desired altitude
  
      //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
      //The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
      pid_error_gain_altitude = 0;                                                    //Set pid_error_gain_altitude to 0
      if (pid_error_temp_altitude > 10 || pid_error_temp_altitude < -10) {            //Check for a difference in altitude greater than 10
        pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;                  //Calculate the pid_error_gain_altitude value
        if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;                  //Set a limit to the p-gains at 3
      }
  
      //Calculate the I-output of the PID controller
      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp_altitude;
      if (pid_i_mem_altitude > pid_max_altitude)pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
      //In the following line the PID-output is calculated.
      //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
      //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
      //D = pid_d_gain_altitude * parachute_throttle.
      pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp_altitude + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
      //To prevent extreme PID-output the output must be limited.
      if (pid_output_altitude > pid_max_altitude)pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;
    }
  
    //We will need to reset some vaiables to 0 when not in auto hold mode
    else if (flight_status == 2 && pid_altitude_setpoint != 0) {                      //If the altitude hold mode is not set and the PID altitude setpoint is still set.
      pid_altitude_setpoint = 0;                                                      //Reset the PID altitude setpoint.
      pid_output_altitude = 0;                                                        //Reset the output of the PID controller.
      pid_i_mem_altitude = 0;                                                         //Reset the I-controller.
      manual_throttle = 0;                                                            //Set the manual_throttle variable to 0 .
      manual_altitude_change = 1;                                                     //Set the manual_altitude_change to 1.
    }

  //Lets calculate the current height in this loop where there is extra time.
  current_height = calculate_altitude(altitude_cal);
  }
}
