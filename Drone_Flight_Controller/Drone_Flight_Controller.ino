///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM.
#include <bmp_garth.h>                     //Include the Adafruit_BMP280 library so that we can get pressure data.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.15;              //Gain setting for the roll P-controller (0.7)
float pid_i_gain_roll = 0.02;              //Gain setting for the roll I-controller (0.025)
float pid_d_gain_roll = 4.0;               //Gain setting for the roll D-controller (4.0)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 3.0;                //Gain setting for the pitch P-controller. //3.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

float pid_p_gain_altitude = 1.4;           //Gain setting for the altitude p controller (1.2)
float pid_i_gain_altitude = 0.2;           //Gain setting for the altitude i controller (0.002)
float pid_d_gain_altitude = 0.75;           //Gain setting for the altitude d controller (1.5)
int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, manual_throttle, battery_voltage;
int cal_int, start, flight_status, gyro_address, bmp_address;
int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;
double gyro_axis_cal[4], acc_axis_cal[4];
long signal_center = 500;
long throttle_signal_center = 350;
int LED_PIN;


long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_pitch_cal, gyro_roll_cal, gyro_yaw_cal;

//Auto-Level PID Settings
bool auto_height = true;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float angle_pitch_acc_offset, angle_roll_acc_offset;

//Altitude PID Gain Settings
float pid_error_gain_altitude, pid_error_temp_altitude, pid_throttle_gain_altitude, pid_i_mem_altitude, pid_last_d_error_altitude;
float pid_altitude_setpoint, pid_output_altitude, pid_altitude_input;
float altitude_cal; 
float current_height = 1.5;
uint8_t altitude_counter, bar_count, temp_count, manual_altitude_change;
float auto_height_min = 2.5;
bool barometer_cal_complete = false;

//Temperature
uint32_t raw_temperature, raw_average_temperature_total, raw_temperature_rotating_memory[6], raw_temperature_reading[6];
uint8_t average_temperature_mem_location;
//Pressure
uint32_t raw_pressure, raw_pressure_reading[6];
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_average;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;

unsigned long test_timer;

//Variable used to determine which board we are using. 0 = Arduino Uno, 1 = Arduino Nano
int board_type = 1;

Adafruit_BMP280 bmp;        // I2C initialization

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(57600);
  Serial.println("Starting up.");
  //Copy the EEPROM data into var for fast access.
  for (start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
  start = 0;                                                                //Set start to zero.
  gyro_address = eeprom_data[32];                                           //Store the gyro address in the variable.
  bmp_address = 0x77;                                                       //Store the barometer address as a variable

  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRB |= B00111111;                                                        //Configure digital port 8, 9, 10, 11, 12, and 13 as output.

  //Use the led on the Arduino for startup indication.
  if (board_type == 0) {
    LED_PIN = 12;                                                           //Default LED pin for Arduino Uno
  } else if (board_type == 1) {
    LED_PIN = 13;                                                           //Default LED pin for Arduino Nano
  }
  digitalWrite(LED_PIN, HIGH);                                              //Turn on the warning led.

  //Check the EEPROM signature to make sure that the setup program is executed.
  while (eeprom_data[33] != 'G' || eeprom_data[34] != 'H' || eeprom_data[35] != 'M')delay(10);

  //The flight controller needs the MPU-6050 with gyro and accelerometer
  //If setup is completed without MPU-6050 stop the flight controller program
  if (eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

  //Use the Adafruit BMP library to initialize the BMP280 chip
  calibrate_barometer();

  //Wait 5 seconds before continuing.
  for (cal_int = 0; cal_int < 1250 ; cal_int ++) {
    PORTB |= B00001111;                                                     //Set digital poort 8, 9, 10 and 11 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTB &= B11110000;                                                     //Set digital poort 8, 9, 10 and 11 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
  }

  Wire.begin();                                                             //Start the I2C as master.

  Wire.setClock(400000);                                                    //Set the I2C clock speed to 400kHz.  This makes the barometer fast enough to fit in our 4ms loop time.

  set_gyro_registers();                                                     //Set the specific gyro registers.

  calibrate_gyro();                                                         //Get calibration offsets for the MPU6050
  calibrate_accelerometer();
  Serial.println("MPU Calibrated");

  PCICR |= (1 << PCIE2);                                                    //Set PCIE0 to enable PCMSK0 scan.
  if (board_type == 0) {
    PCMSK2 |= (1 << PCINT20);                                               //Set PCINT20 (digital input 4) to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT21);                                               //Set PCINT21 (digital input 5) to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT22);                                               //Set PCINT22 (digital input 6) to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT23);                                               //Set PCINT23 (digital input 7) to trigger an interrupt on state change.
  } else if (board_type == 1) {
    PCMSK2 |= (1 << PCINT18);                                               //Set PCINT18 (digital input 2) to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT19);                                               //Set PCINT19 (digital input 3) to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT20);                                               //Set PCINT20 (digital input 4) to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT21);                                               //Set PCINT21 (digital input 5) to trigger an interrupt on state change.
  }

  //Wait until the receiver is active and the throttle is set to the lower position.
  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400) {
    receiver_input_channel_3 = convert_throttle_receiver_channel(3);        //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                               //While waiting, increment start whith every loop.
    //Give the ESC's a 1000us pulse while waiting for the receiver inputs to prevent them from beeping.
    PORTB |= B00001111;                                                     //Set digital port 8, 9, 10 and 11 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTB &= B11110000;                                                     //Set digital port 8, 9, 10 and 11 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    if (start == 125) {                                                     //Every 125 loops (500ms).
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));                         //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }
  // Flight Status: 0 = Flight Disabled, 1 = Ready to Fly, 2 = Normal Flying Mode, 3 = Auto Height Mode
  flight_status = 0;                                                        //Set the current flight status to 0 to prevent motors from starting

  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  loop_timer = micros();                                                    //Set the timer for the next loop.

  //When everything is done, turn off the led.
  digitalWrite(LED_PIN, LOW);                                               //Turn off the warning led.

  Serial.println("Ready to fly");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //Turn off the LED light
  digitalWrite(LED_PIN, LOW);

  //    if(loop_counter == 0)Serial.print("Angle Pitch: ");
  //    if(loop_counter == 1)Serial.print(angle_pitch);
  //    if(loop_counter == 2)Serial.print(" Angle Roll: ");
  //    if(loop_counter == 3)Serial.println(angle_roll);
  if (loop_counter == 4)Serial.print("Current Height: ");
  if (loop_counter == 5)Serial.println(current_height);
//  if (loop_counter == 6)Serial.print(" Actual Pressure: ");
//  if (loop_counter == 7)Serial.println(actual_pressure);
  //
  loop_counter ++;
  if (loop_counter == 60)loop_counter = 0;


  get_barometer_data();                                                     //Read the barometer data and calculate the altitude PID outputs

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
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

  angle_pitch_acc -= angle_pitch_acc_offset;                                //Accelerometer calibration value for pitch.
  angle_roll_acc -= angle_roll_acc_offset;                                  //Accelerometer calibration value for roll.

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction


  //For starting the motors: throttle low and yaw left (step 1).
  if (receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)flight_status = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if (flight_status == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450) {
    flight_status = 2;                                                      //Prepare the quad for flight

    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    //Reset the PID controllers at start of flight
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  
  //Start and stop auto altitude depending on the height and stick placement
  //Enter auto height mode if throttle is in center of the range and barometer reads above minimum height.
  if (flight_status == 2 && auto_height) {
    if (receiver_input_channel_3 > (1000 + throttle_signal_center - 50) && \
        receiver_input_channel_3 < (1000 + throttle_signal_center + 50) && \
        current_height > auto_height_min) {
      flight_status = 3;
      pid_altitude_setpoint = current_height;                                      //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
    }
  }
  //Exit auto height mode if the throttle is in the center of the range.  Reset altitude PID values.
  if (flight_status == 3 && \
      (receiver_input_channel_3 > (1000 + throttle_signal_center - 50) && \
       receiver_input_channel_3 < (1000 + throttle_signal_center + 50) && \
       receiver_input_channel_4 > 1950)) {
    flight_status = 2;
    pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
    pid_output_altitude = 0;                                                       //Reset the output of the PID controller.
    pid_i_mem_altitude = 0;                                                        //Reset the I-controller.
  }

  //Stopping the motors: throttle low and yaw right.
  if (flight_status >= 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)flight_status = 0;

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of dividing by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if (receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of dividing by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if (receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of dividing by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050) { //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
    else if (receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
  }

  //PID inputs are known. So we can calculate the pid output.
  calculate_pid();

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if (battery_voltage < 1000 && battery_voltage > 600)digitalWrite(LED_PIN, HIGH);

  if (flight_status >= 2) {                                                 //Active when the motors are started.
    //The throttle variable is calculated in the following part. It forms the base throttle for every motor.
    throttle = receiver_input_channel_3;                                    //The base throttle is the receiver throttle channel + the detected take-off throttle.
    if (flight_status == 3) {                                               //If altitude hold is active.
      throttle = 1000 + throttle_signal_center + \
                 pid_output_altitude + manual_throttle;                     //The base throttle is the receiver throttle channel + the detected take-off throttle + the PID controller output.
    }

    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800) {                  //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-4 pulse for voltage drop.
    }

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if (esc_1 > 1950)esc_1 = 1950;                                          //Limit the esc-1 pulse to 1950us.
    if (esc_2 > 1950)esc_2 = 1950;                                          //Limit the esc-2 pulse to 1950us.
    if (esc_3 > 1950)esc_3 = 1950;                                          //Limit the esc-3 pulse to 1950us.
    if (esc_4 > 1950)esc_4 = 1950;                                          //Limit the esc-4 pulse to 1950us.
  }

  else {
    esc_1 = 1000;                                                           //If flight_status is not 2 or 3 keep a 1000us pulse for esc-1.
    esc_2 = 1000;                                                           //If flight_status is not 2 or 3 keep a 1000us pulse for esc-2.
    esc_3 = 1000;                                                           //If flight_status is not 2 or 3 keep a 1000us pulse for esc-3.
    esc_4 = 1000;                                                           //If flight_status is not 2 or 3 keep a 1000us pulse for esc-4.
  }

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is
  //longer or shorter than 4000us, the angle calculation is off. If you modify the code make sure
  //that the loop time is still 4000us and no longer.
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

  if (micros() - loop_timer > 4050)digitalWrite(LED_PIN, HIGH);             //Turn on the LED if the loop time exceeds 4050us.
  //  Serial.println(micros() - loop_timer);

  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while (micros() - loop_timer < 4000);                                     //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTB |= B00001111;                                                       //Set digital outputs 8, 9, 10 and 11 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the falling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the falling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the falling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the falling edge of the esc-4 pulse.

  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
  gyro_signalen();

  while ((PORTB & B00001111) != 0) {                                        //Stay in this loop until output 8,9,10 and 11 are low.
    esc_loop_timer = micros();
    if (timer_channel_1 <= esc_loop_timer)PORTB &= B11111110;               //Set digital output 8 to low if the time is expired.
    if (timer_channel_2 <= esc_loop_timer)PORTB &= B11111101;               //Set digital output 9 to low if the time is expired.
    if (timer_channel_3 <= esc_loop_timer)PORTB &= B11111011;               //Set digital output 10 to low if the time is expired.
    if (timer_channel_4 <= esc_loop_timer)PORTB &= B11110111;               //Set digital output 11 to low if the time is expired.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Timer used to read receiver signals
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT2_vect) {
  current_time = micros();
  if (board_type == 0) {
    //Channel 1=========================================
    if (PIND & B00010000) {                                                   //Is input 4 high?
      if (last_channel_1 == 0) {                                              //Input 4 changed from 0 to 1.
        last_channel_1 = 1;                                                   //Remember current input state.
        timer_1 = current_time;                                               //Set timer_1 to current_time.
      }
    }
    else if (last_channel_1 == 1) {                                           //Input 4 is not high and changed from 1 to 0.
      last_channel_1 = 0;                                                     //Remember current input state.
      receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
    }
    //Channel 2=========================================
    if (PIND & B00100000 ) {                                                  //Is input 5 high?
      if (last_channel_2 == 0) {                                              //Input 5 changed from 0 to 1.
        last_channel_2 = 1;                                                   //Remember current input state.
        timer_2 = current_time;                                               //Set timer_2 to current_time.
      }
    }
    else if (last_channel_2 == 1) {                                           //Input 5 is not high and changed from 1 to 0.
      last_channel_2 = 0;                                                     //Remember current input state.
      receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
    }
    //Channel 3=========================================
    if (PIND & B01000000 ) {                                                  //Is input 6 high?
      if (last_channel_3 == 0) {                                              //Input 6 changed from 0 to 1.
        last_channel_3 = 1;                                                   //Remember current input state.
        timer_3 = current_time;                                               //Set timer_3 to current_time.
      }
    }
    else if (last_channel_3 == 1) {                                           //Input 6 is not high and changed from 1 to 0.
      last_channel_3 = 0;                                                     //Remember current input state.
      receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.
    }
    //Channel 4=========================================
    if (PIND & B10000000 ) {                                                  //Is input 7 high?
      if (last_channel_4 == 0) {                                              //Input 7 changed from 0 to 1.
        last_channel_4 = 1;                                                   //Remember current input state.
        timer_4 = current_time;                                               //Set timer_4 to current_time.
      }
    }
    else if (last_channel_4 == 1) {                                           //Input 7 is not high and changed from 1 to 0.
      last_channel_4 = 0;                                                     //Remember current input state.
      receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
    }

  } else if (board_type == 1) {

    //Channel 1=========================================
    if (PIND & B00000100) {                                                   //Is input 2 high?
      if (last_channel_1 == 0) {                                              //Input 2 changed from 0 to 1
        last_channel_1 = 1;                                                   //Remember current input state
        timer_1 = current_time;                                               //Set timer_1 to current_time
      }
    }
    else if (last_channel_1 == 1) {                                           //Input 2 is not high and changed from 1 to 0
      last_channel_1 = 0;                                                     //Remember current input state
      receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1
    }
    //Channel 2=========================================
    if (PIND & B00001000 ) {                                                  //Is input 3 high?
      if (last_channel_2 == 0) {                                              //Input 3 changed from 0 to 1
        last_channel_2 = 1;                                                   //Remember current input state
        timer_2 = current_time;                                               //Set timer_2 to current_time
      }
    }
    else if (last_channel_2 == 1) {                                           //Input 3 is not high and changed from 1 to 0
      last_channel_2 = 0;                                                     //Remember current input state
      receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2
    }
    //Channel 3=========================================
    if (PIND & B00010000 ) {                                                  //Is input 4 high?
      if (last_channel_3 == 0) {                                              //Input 4 changed from 0 to 1
        last_channel_3 = 1;                                                   //Remember current input state
        timer_3 = current_time;                                               //Set timer_3 to current_time
      }
    }
    else if (last_channel_3 == 1) {                                           //Input 4 is not high and changed from 1 to 0
      last_channel_3 = 0;                                                     //Remember current input state
      receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3

    }
    //Channel 4=========================================
    if (PIND & B00100000 ) {                                                  //Is input 5 high?
      if (last_channel_4 == 0) {                                              //Input 5 changed from 0 to 1
        last_channel_4 = 1;                                                   //Remember current input state
        timer_4 = current_time;                                               //Set timer_4 to current_time
      }
    }
    else if (last_channel_4 == 1) {                                           //Input 5 is not high and changed from 1 to 0
      last_channel_4 = 0;                                                     //Remember current input state
      receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4
    }
  }
}
