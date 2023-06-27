///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating drone height based on the pressure read from the barometer
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float calculate_altitude(float seaLevelhPa) {
  float altitude;

  //Convert the units from pascals to hectopascals
  float pressure = actual_pressure/100;

  //Calculate the altitude based on the pressure at sea level
  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  //This prevents the height var from spiking at the start of the program.
  if (!barometer_cal_complete) {
    if (altitude < 1) {
      Serial.print("Barometer Cal Done: ");
      Serial.println(barometer_cal_complete);
      barometer_cal_complete = true;
    }
    return 1.1;
  }

  return altitude;
}
