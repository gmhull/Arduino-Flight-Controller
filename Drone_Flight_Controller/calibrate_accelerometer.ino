///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for getting the accelerometer pitch and roll offsets
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_accelerometer() {
  //Calculate the pitch and roll calibration values when resting on a flat surface.
  //Or you can find these by running the ESC_Calibrate file and enter manually.
  bool manual_entry = false;
  int acc_cal_counter = 100;

  if (!manual_entry) {
    for (int i=0; i < acc_cal_counter; i++) {
      angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
      angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.
    
      //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
      angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
      angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.
    
      //Accelerometer angle calculations
      acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector.
    
      if (abs(acc_y) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
        angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;       //Calculate the pitch angle.
      }
      if (abs(acc_x) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
        angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;       //Calculate the roll angle.
      }
    
      angle_pitch_acc_offset += angle_pitch_acc;                                //Accelerometer calibration value for pitch.
      angle_roll_acc_offset += angle_roll_acc;                                  //Accelerometer calibration value for roll.
    
      angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
      angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.
    }
    angle_pitch_acc_offset /= acc_cal_counter;                                               //Divide the sum by the amount of loops run
    angle_roll_acc_offset /= acc_cal_counter;                                                //Divide the sum by the amount of loops run
  } 
  if (manual_entry) {
    angle_pitch_acc_offset = 1.94;
    angle_roll_acc_offset = 0.85;
  }
  
 Serial.print("Angle Pitch Acc Offset: ");
 Serial.println(angle_pitch_acc_offset);
 Serial.print("Angle Roll Acc Offset: ");
 Serial.println(angle_roll_acc_offset);

}
