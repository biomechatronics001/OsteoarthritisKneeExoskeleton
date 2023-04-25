#include <SPI.h>
#include "Motor_Control_Tmotor.h"
#include <FlexCAN_T4.h>
#include "WL_IMU.h"
#include "CIC.h"
#include <Arduino.h>
#include <stdio.h>
//#include <SD.h>

// Settings
int assist_mode = 12;
int stopFlag = 0;
int saveDataFlag = 1;
int numOfInitialSteps = 1;
int enableExtensionStop = 0;
uint32_t Motor_ID1 = 1; // Motor Can Bus ID, left leg, loadcell: port 1, positive current = extension // updated on 2023-02-11 2
uint32_t Motor_ID2 = 2; // Motor Can Bus ID, right leg, loadcell: port 2, positive current = flexion // updated on 2023-02-11 3
int CAN_ID = 3;         // CAN port from Teensy
double Gear_ratio = 9;  //The actuator gear ratio, will enfluence actuator angle and angular velocity
double torque_constant_before_gear = 0.091; // before gear at 24 V. Ref: https://store.tmotor.com/goods.php?id=982

// *** for RingBuf *** //
#include "SdFat.h"
#include "RingBuf.h"

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Interval between points for 25 ksps.
#define LOG_INTERVAL_USEC 2000 // 500 Hz

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
// #define LOG_FILE_SIZE 10*25000*600  // Size to log 10 byte lines at 25 kHz for more than ten minutes = 150,000,000 bytes.
// #define LOG_FILE_SIZE 100 * 500 * 80 // Size to log 10 byte lines at 500 Hz for more than 80 seconds =  bytes.
#define LOG_FILE_SIZE 220 * 500 * 600 // Size to log 10 byte lines at 500 Hz for more than 80 seconds = 66,000,000 bytes.

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// #define RING_BUF_CAPACITY 400*512 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// #define RING_BUF_CAPACITY 400 * 512 * 10 / 50 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 250 * 500 * 1 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.

// #define LOG_FILENAME "Walking.csv"
//#define LOG_FILENAME "0407_Powered_Jennifer_Walking_bd0.04_0.00_kd_1_0.6_0_0_0_0_newtau.csv"
#define LOG_FILENAME "2022-05-24-Weibo-Walking_Powered_06.csv"

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;


// *** End for RingBuf *** //

/*Canbus Setup*/
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;
/*Canbus Setup*/

// Tmotor related code //
float p_des = 0;
float v_des = 0;
float kp = 0;
float kd = 0;
float t_ff_L = 0;
float t_ff_R = 0;

double LGPPrevious = 0;
double LGPCurrent = 0;
double dataCounter = 0;
int initialStepCounter = 0;

double weight = 52; // [kg] weight of the subject
uint32_t ID_offset = 0x140;

int Stop_button = 0;    // Stop function
String mode = "start";
double milli_time_current_double = 0;

Motor_Control_Tmotor m1(Motor_ID1, CAN_ID);
Motor_Control_Tmotor m2(Motor_ID2, CAN_ID);
IMU imu;                                                      //Create IMU object see WL_IMU.h
CIC cic;
double Pgain = 7.5;    //P gain of torque control
double Igain = 0.7;    //I gain of torque control
double Dgain = 0;      //D gain of torque control
double MaxPIDout = 10; //Max torque control PID output (Unit Current A, inner loop is current controller)

double Fsample = 500;        // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
double Fsample_ble = 100;    // [Hz] Bluetooth sending data frequency
unsigned long current_time = 0;
unsigned long previous_time = 0;                                                        // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                                    // used to control the Bluetooth communication frequency
unsigned long Tinterval_microsecond = (unsigned long)(1000000 / Fsample);               // used to control the teensy controller frequency
unsigned long Tinterval_ble_microsecond = (unsigned long)(1000000 / Fsample_ble);       // used to control the Bluetooth communication frequency

double Cur_command_L = 0;
double Cur_command_R = 0;
int current_limitation = 10; //(unit Amp)//double Tor_command_L = 0;
// Trigger
int triggerOn = 0;
int triggerPin = A9; //reading pin
int triggerVal = 0;  // analog trigger pin value
// Data logging
int isLogging = 0;
int taskIdx = 1;
int conditionIdx = 1; // baseline=1, sham=2, powered=3
int trialIdx = 1;
//

//***Data sent via bluetooth
char datalength_ble = 32;      // Bluetooth Data Length (32)
char data_ble[60] = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};  // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy
int LK_ble = 0;                //left knee angle
int RK_ble = 0;                //right knee angle
int current_command_L_ble = 0; //current reference(A) for inner loop current control
int current_command_R_ble = 0;
int current_actual_L_ble = 0;
int current_actual_R_ble = 0;
int torque_command_L_ble = 0; //total torque reference(N-m)
int torque_command_R_ble = 0;
int torque_command_imp_L_ble = 0; // impedance torque
int torque_command_imp_R_ble = 0; // impedance torque
int torque_command_ff_L_ble = 0;  // feedforward torque
int torque_command_ff_R_ble = 0;  // feedforward torque
int torque_L_ble = 0;             //actual torque(N-m)(measured by torque sensor)
int torque_R_ble = 0;
int motor_speed_L_ble = 0;
int motor_speed_R_ble = 0;
double torque_command_L = 0;
double torque_command_R = 0;

int gait_percentage_L_ble = 0;
int insole_torque_command_L_ble = 0;
int insole_torque_command_R_ble = 0;
int insole_gait_percent_L_ble = 0;
int insole_gait_percent_R_ble = 0;
int imu_gait_percent_L_ble = 0;
int imu_gait_percent_R_ble = 0;
int imu_gait_percent_imp_L_ble = 0; // impedance torque
int imu_gait_percent_imp_R_ble = 0; // mpedance torque
int imu_gait_percent_ff_L_ble = 0;  // eedforward biological torque
int imu_gait_percent_ff_R_ble = 0;  // feedforward biological torque
double Insole_gain = 5;

//int percentage_ble = 0;  //gait percentage(%)
//int torque_commandGP_ble = 0;     //torque reference(N-m) generated by gait percentage VS torque reference table
//int current_ble = 0;              //actual current (A)

double relTime = 0.0;
int SDcounter = 0;
int SDtotalCounter = 0;

int squat_bio[140] = { -16,  -16,  -16,  -16,  -15,  -14,  -12,  -11,  -10,  -9, -8, -6, -4, -2, -1, 1,  3,  4,  5,  8,  9,  10, 11, 13, 14, 16, 17, 18, 20, 22, 23, 25, 26, 28, 29, 30, 31, 32, 33, 36, 37, 38, 39, 41, 42, 43, 44, 46, 47, 48, 48, 50, 51, 51, 52, 54, 54, 56, 56, 58, 58, 59, 60, 61, 62, 63, 64, 65, 66, 66, 67, 68, 68, 69, 69, 70, 71, 72, 72, 73, 74, 75, 76, 77, 78, 79, 79, 80, 81, 82, 82, 83, 84, 84, 85, 85, 86, 87, 88, 88, 89, 89, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 91, 92, 92, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93};
int squat_bio_angle = 0;


void setup()
{
  // put your setup code here, to run once:
  delay(1000);
  Serial.begin(115200);  //used for communication with computer.
  Serial5.begin(115200); //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  initial_CAN2();
  delay(500);
  IMUSetup();
  // m1.init_motor(); // Strat the CAN bus communication & Motor Control
  // delay(100);
  // m2.init_motor(); // Strat the CAN bus communication & Motor Control
  // delay(100);
  reset_motor_angle();
  CurrentControlSetup();
  pinMode(A8, INPUT); // init trigger pin
  
  // cic.calculateSigmoid();
  cic.calculateSigmoid(cic.kdMax, cic.kdMin, cic.kd_arr_Junxi);
  cic.calculateSigmoid(cic.bdMax, cic.bdMin, cic.bd_arr_Junxi);
  cic.calculateSigmoid(1, 0, cic.sigmoid);
}

void IMUSetup()
{
  imu.Gain_E = 0;
  imu.Gain_F = 0;
  imu.STS_Gain = 0;
  imu.delaypoint = 0;
  imu.alpha = 5;
  imu.beta = 2;
  imu.angleRelativeThreshold = 20;
}

void loop()
{
  while (stopFlag)
  {
  };
  
  // imu.READ();  
  CurrentControl();
}

void SDCardSetup()
{
  sd.remove(LOG_FILENAME);

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG))
  {
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC))
  {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE))
  {
    Serial.println("preAllocate failed\n");
    file.close();
    return;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  Serial.println("Data logging initialized.");
}

void SDCardSetup(const char* logFileName)
{
  sd.remove(logFileName);

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG))
  {
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(logFileName, O_RDWR | O_CREAT | O_TRUNC))
  {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE))
  {
    Serial.println("preAllocate failed\n");
    file.close();
    return;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  Serial.print("Data logging initialized. File name = ");
  Serial.println(logFileName);
}

void CurrentControlSetup()
{
  imu.INIT(); //Initialize IMU;
  delay(500);
  imu.INIT_MEAN();
  current_time = micros();
  previous_time = current_time;
  previous_time_ble = current_time;
  // previous_time_insole = current_time;
}

void CurrentControl()
{
  imu.READ();                          //Check if IMU data available and read it. the sample rate is 100 hz
  current_time = micros();             //query current time (microsencond)
  //********* use to control the teensy controller frequency **********//
  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    imu.READ();
    if (Stop_button) //stop
    {
      p_des=0;//dont change this
      v_des=0;//dont change this
      kp=0;//dont change this
      kd=0;//dont change this
      torque_command_L = 0; // torque command
      torque_command_R = 0; // torque command
    }
    else
    {
      Compute_Tor_Commands(); // TODO
    }

    if (assist_mode == 10)
    {
      // enforceInitialSteps();
    }
    // Serial.println(torque_command_L);
    m1.send_cmd(p_des, v_des, kp, kd, torque_command_L);//torque_command_L);
    // delay(1);

    for(int  qwe=0; qwe<100;qwe++)
    {
      
    }
    receive_CAN_data();
    // delay(1);
        for(int  qwe=0; qwe<100;qwe++)
    {
      
    }
    m2.send_cmd(p_des, v_des, kp, kd, torque_command_R);//torque_command_R);
    // delay(1);
        for(int  qwe=0; qwe<100;qwe++)
    {
      
    }
    receive_CAN_data();
        for(int  qwe=0; qwe<100;qwe++)
    {
      
    }
    // delay(1);

    // Read trigger signal
    triggerVal = analogRead(triggerPin);
    digitalWrite(A8, LOW); //pin converted into GND
    triggerVal = analogRead(A8); // temp change on 01/17/2023 for the Sinai visit
    if (triggerVal < 400)
    {
      triggerOn = 0;
    }
    else
    {
      triggerOn = 1;
    }
    
    if (isLogging)
    {
      logData3();
    }

    previous_time = current_time; //reset previous control loop time
    relTime += Tinterval_microsecond / 1000;
  }

  //********* use to control the Bluetooth communication frequency **********//
  if (current_time - previous_time_ble > Tinterval_ble_microsecond)
  {
    receive_ble_Data();
    send_ble_Data(); // send the BLE data
    previous_time_ble = current_time;
    
    plot_cic_data();
    
  }
  if (Serial.available())
  {
    char cmd = (char)Serial.read();
    if (cmd == 's')
    {
      stopFlag = 1;
      Serial.println("Stopped");
      if (saveDataFlag)
      {
        SDCardSaveToFile();
      }
    }
  }
}

void Compute_Tor_Commands()
{
  if (assist_mode == 1) //IMU walking
  {
    mode = "Walking (IMU)";
  }
  else if (assist_mode == 4) //sine wave
  {
    mode = "Sine Wave";
    p_des=0;//dont change this
    v_des=0;//dont change this
    kp=0;//dont change this
    kd=0;//dont change this
    torque_command_L = 2 * sin(2 * PI * current_time / 1000000);
    torque_command_R = -2 * sin(2 * PI * current_time / 1000000);
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
  }
  else if (assist_mode == 8)
  {
    mode = "Sit-to-Stand (IMU)";
    torque_command_L = +imu.STSTorque;
    torque_command_R = -imu.STSTorque;
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
  }
  else if (assist_mode == 12)
  {
    // TODO
    // Serial.println("Entered mode 12");
    cic.computeGaitPhase(imu.LTAVx - imu.RTAVx, -m1.pos, m2.pos, -m1.spe, m2.spe);
    cic.computeSineAssistanceTorque();
    p_des=0;//dont change this
    v_des=0;//dont change this
    kp=0;//dont change this
    kd=0;//dont change this
    torque_command_L = cic.sineAssistanceTotalTorqueLeft;
    torque_command_R = -cic.sineAssistanceTotalTorqueRight; 
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
  }

 else if (assist_mode == 51)
  {
    mode = "Squatting (gravity)";
    p_des=0;//dont change this
    v_des=0;//dont change this
    kp=0;//dont change this
    kd=0;//dont change this
    torque_command_L = -0.3 * 48 * sin((imu.LTx+imu.RTx) * 1.5 /180 * 3.14 / 2 / 2);
    torque_command_R = 0.3 * 48 * sin((imu.LTx+imu.RTx) * 1.5 /180 * 3.14 / 2 / 2);
    Serial.print(imu.LTx);
    Serial.print(" ");
    Serial.print(imu.RTx);
    Serial.print(" ");
    Serial.print(torque_command_L);
    Serial.print(" ");
    Serial.println(torque_command_R);
    torque_command_L = 0;
    torque_command_R = 0;
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
  }
  else if (assist_mode == 52)
  {
    mode = "Squatting (biological)";
    
    p_des=0;//dont change this
    v_des=0;//dont change this
    kp=0;//dont change this
    kd=0;//dont change this
    
    squat_bio_angle = int(-(imu.LTx+imu.RTx) * 1.5 / 2);
    
    if (squat_bio_angle < 0)
    {
      squat_bio_angle = 0;
    }
    if (squat_bio_angle > 139)
    {
      squat_bio_angle = 139;
    }

    torque_command_L = -0.4 * 0.3 * squat_bio[squat_bio_angle]*(-1);
    torque_command_R = 0.4 * 0.3 * squat_bio[squat_bio_angle]*(-1);
    
    Serial.print(imu.LTx);
    Serial.print(" ");
    Serial.print(imu.RTx);
    Serial.print(" ");
    Serial.print(torque_command_L);
    Serial.print(" ");
    Serial.println(torque_command_R);
    
    torque_command_L = 0;
    torque_command_R = 0;
    
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
  }
  else if (assist_mode == 53)
  {
    mode = "Squatting (quasi-static Yu RAL)";
    
    p_des=0;//dont change this
    v_des=0;//dont change this
    kp=0;//dont change this
    kd=0;//dont change this
    
    torque_command_L = -0.4 * 0.25 * imu.Gain_E * imu.SquatTorque * (-1);
    torque_command_R = 0.4 * 0.25 * imu.Gain_E * imu.SquatTorque * (-1);
    
    Serial.print(imu.LTx);
    Serial.print(" ");
    Serial.print(imu.RTx);
    Serial.print(" ");
    Serial.print(torque_command_L);
    Serial.print(" ");
    Serial.println(torque_command_R);
    
    torque_command_L = 0;
    torque_command_R = 0;
    
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
  }
    else if (assist_mode == 54)
  {
    mode = "Squatting (quasi-static Yu RAL)*0.5";
    
    p_des=0;//dont change this
    v_des=0;//dont change this
    kp=0;//dont change this
    kd=0;//dont change this
    
    torque_command_L = -0.4 * 0.25 * imu.Gain_E * imu.SquatTorque * (-1);
    torque_command_R = 0.4 * 0.25 * imu.Gain_E * imu.SquatTorque * (-1);
     
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;

      if ((((imu.RTAVx + imu.LTAVx) / 2) < -20) && (((imu.RTx + imu.LTx) / 2) > -105))
    {
      torque_command_L = 0.5 * torque_command_L;
      torque_command_R = 0.5 * torque_command_R;
    }
    else
    {
    }
//    Serial.print(imu.LTx);
//    Serial.print(" ");
//    Serial.print(imu.RTx);
//    Serial.print(" ");
//    Serial.print(torque_command_L);
//    Serial.print(" ");
    Serial.println(torque_command_R);
     
    if (torque_command_L < 0)
    {
      torque_command_L = 0;
    }

    if (torque_command_R > 0)
    {
      torque_command_R = 0;
    }
    
    torque_command_L = 0;
    torque_command_R = 0;
}    
    else if (assist_mode == 55)
  {
    mode = "Squatting (quasi-static Yu RAL)*0.5";
    
    p_des=0;//dont change this
    v_des=0;//dont change this
    kp=0;//dont change this
    kd=0;//dont change this
    
    torque_command_L = -0.4 * 0.25 * imu.Gain_E * imu.SquatTorque * (-1);
    torque_command_R = 0.4 * 0.25 * imu.Gain_E * imu.SquatTorque * (-1);
     
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;

      if ((((imu.RTAVx + imu.LTAVx) / 2) < -20) && (((imu.RTx + imu.LTx) / 2) > -105))
    {
      torque_command_L = 0 * torque_command_L;
      torque_command_R = 0 * torque_command_R;
    }
    else
    {
    }
//    Serial.print(imu.LTx);
//    Serial.print(" ");
//    Serial.print(imu.RTx);
//    Serial.print(" ");
//    Serial.print(torque_command_L);
//    Serial.print(" ");
    Serial.println(torque_command_R);
    
    if (torque_command_L < 0)
    {
      torque_command_L = 0;
    }

    if (torque_command_R > 0)
    {
      torque_command_R = 0;
    }

    torque_command_L = 0;
    torque_command_R = 0;
    
  }
  
  else if (assist_mode == 100)
  {
    mode = "Stop";
    p_des=0;//dont change this
    v_des=0;//dont change this
    kp=0;//dont change this
    kd=0;//dont change this
    torque_command_L = 0;
    torque_command_R = 0;
    Cur_command_L = torque_command_L / torque_constant_before_gear / Gear_ratio;
    Cur_command_R = torque_command_R / torque_constant_before_gear / Gear_ratio;
  }
}



void SDCardSaveToFile()
{
  rb.sync();
  file.truncate();
  file.rewind();
  file.close();
}



// void Cur_limitation()
// {
//   //************* Current limitation *************//

//   Cur_command_L = min(current_limitation, Cur_command_L);
//   Cur_command_L = max(-current_limitation, Cur_command_L);
//   Cur_command_R = min(current_limitation, Cur_command_R);
//   Cur_command_R = max(-current_limitation, Cur_command_R);

//   torque_command_L = Cur_command_L * 0.232 * 9;//2.2;
//   torque_command_R = Cur_command_R * 0.232 * 9;//2.2;
// }

void enforceInitialSteps()
{
  if (dataCounter == 1)
  {
    LGPCurrent = cic.GP_IMU_L;
  }
  else
  {
    LGPPrevious = LGPCurrent;
    LGPCurrent = cic.GP_IMU_L;
  }
  dataCounter += 1;

  if ((LGPPrevious > 95) && (LGPCurrent < 1))
  {
    cic.initialStepCounter += 1;
  }

  if (cic.initialStepCounter <= numOfInitialSteps)
  {
    Cur_command_L = Cur_command_L * (cic.initialStepCounter - 0) / numOfInitialSteps;
    Cur_command_R = Cur_command_R * (cic.initialStepCounter - 0) / numOfInitialSteps;
  }
}

void receive_ble_Data()
{
  if (Serial5.available() >= 20)
  {
    // Serial.println("-------------New data received-------------------");
    data_rs232_rx[0] = Serial5.read();
    if (data_rs232_rx[0] == 165)
    {
      data_rs232_rx[1] = Serial5.read();
      if (data_rs232_rx[1] == 90)
      {
        data_rs232_rx[2] = Serial5.read();
        if (data_rs232_rx[2] == 20)
        {
          Serial5.readBytes(&data_rs232_rx[3], 17);
          if (data_rs232_rx[3] == 0)
          {
            Stop_button = int(data_rs232_rx[4]);
            if (Stop_button)
            {
              Serial.println("STOP button pressed");
            }
            else
            {
              Serial.println("START button pressed");
            }
          }
          else if (data_rs232_rx[3] == 1)
          {
            assist_mode = int(data_rs232_rx[4]);
            Serial.print("Mode: ");
            Serial.println(assist_mode);
            //Serial.print("    ");
            //Serial.println(mode);
          }
          else if (data_rs232_rx[3] == 2)
          {
            float Gain_E = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            if (assist_mode == 1)
            {
              imu.Gain_E = Gain_E;
            }
            
            else if (assist_mode == 8)
            {
              imu.Gain_E = Gain_E;
            }
            else if (assist_mode == 9)
            {
              imu.Gain_E = Gain_E;
            }

            Serial.print("Extension gain from Matlab: ");
            Serial.println(Gain_E);
          }
          else if (data_rs232_rx[3] == 3)
          {
            float Gain_F = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            if (assist_mode == 1)
            {
              imu.Gain_F = Gain_F;
            }
            
            else if (assist_mode == 8)
            {
              imu.beta = Gain_F;
            }
            else if (assist_mode == 9)
            {
              imu.Gain_F = Gain_F;
            }

            Serial.print("Flexion gain from matlab: ");
            Serial.println(Gain_F);
          }
          else if (data_rs232_rx[3] == 4)
          { // 10 ms per timepoint, delaypoint needs to be less than 100
            // 1 delaypoint in matlab interface equals 5 timepoints here, that is 50 ms
            int delaypoint = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000;
            if (assist_mode == 1)
            {
              imu.delaypoint = delaypoint;
            }
            
            else if (assist_mode == 8)
            {
              imu.angleRelativeThreshold = delaypoint; // initial angle is 80 degrees
            }
            else if (assist_mode == 9)
            {
              imu.delaypoint = delaypoint;
            }

            Serial.print("Delay [ms]: ");
            Serial.println(delaypoint * 10);
            // delay = delaypoint*sample time
          }
          else if (data_rs232_rx[3] == 5)
          {
            weight = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            Serial.print("Weight [kg]: ");
            Serial.println(weight);
          }
          else if (data_rs232_rx[3] == 6)
          {
            Insole_gain = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            Serial.print("Insole gain: ");
            Serial.println(Insole_gain);
          }
          else if (data_rs232_rx[3] == 7)
          {
            reset_motor_angle();
            Serial.println("The angle of motor has been reset");
            imu.INIT_MEAN();
            Serial.println("The angle of IMUs has been reset");
          }

          else if (data_rs232_rx[3] == 11)
          {
            float Gain_Sw = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            cic.CICGain_L = Gain_Sw;
            cic.CICGain_R = Gain_Sw;
            Serial.print("Swing gain from matlab: ");
            Serial.println(Gain_Sw);
          }
          else if (data_rs232_rx[3] == 12)
          {
            float Gain_St = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            cic.FeedForwardGain_L = Gain_St;
            cic.FeedForwardGain_R = Gain_St;
            Serial.print("Stance gain from matlab: ");
            Serial.println(Gain_St);
          }
          else if (data_rs232_rx[3] == 13)
          {
            float Timing_Sw = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            cic.P_ahead_imp = Timing_Sw;
            Serial.print("Swing timing from matlab: ");
            Serial.println(Timing_Sw);
          }
          else if (data_rs232_rx[3] == 14)
          {
            float Timing_St = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            cic.P_ahead_ff = Timing_St;
            Serial.print("Stance timing from matlab: ");
            Serial.println(Timing_St);
          }
          else if (data_rs232_rx[3] == 15)
          {
            double STSMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            imu.STS_Gain = STSMagnitude;
            Serial.print("Sit-to-stand gain from matlab: ");
            Serial.println(STSMagnitude);
          }
          else if (data_rs232_rx[3] == 16)
          {
            double STSSensitivity = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            imu.STSSlopeThreshold = STSSensitivity;
            Serial.print("Sit-to-stand slope threshold from matlab: ");
            Serial.println(STSSensitivity);
          }
          else if (data_rs232_rx[3] == 17)
          {
            double STSAlpha = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            imu.alpha = STSAlpha;
            Serial.print("Sit-to-stand alpha from matlab: ");
            Serial.println(STSAlpha);
          }
          else if (data_rs232_rx[3] == 18)
          {
            double STSBeta = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            imu.beta = STSBeta;
            Serial.print("Sit-to-stand beta from matlab: ");
            Serial.println(STSBeta);
          }
          else if (data_rs232_rx[3] == 20)
          {
            isLogging = int(data_rs232_rx[7]);
            if (isLogging == 1)
            {
              taskIdx = int(data_rs232_rx[4]);
              conditionIdx = int(data_rs232_rx[5]);
              trialIdx = int(data_rs232_rx[6]);
              String taskName;
              if (taskIdx == 1)
              {
                taskName = "Walking";
              }
              else if (taskIdx == 2)
              {
                taskName = "STS";
              }
              String stringOne = taskName + '-';

              String conditionName;
              if (conditionIdx == 1)
              {
                conditionName = "Baseline";
              }
              else if (conditionIdx == 2)
              {
                conditionName = "Sham";
              }
              else if (conditionIdx == 3)
              {
                conditionName = "Powered";
              }
              else
              {
                conditionName = "Others";
              }
              String stringTwo = stringOne + conditionName + "-Trial";
              String logFileName = stringTwo + String(trialIdx) + ".csv";
              SDCardSetup(logFileName.c_str());
              relTime = 0.0;
              Serial.println("Data logging started......");
            }
            else if (isLogging == 0)
            {
              SDCardSaveToFile();
              Serial.println("Data logging stopped......");
            }
          }
          else if (data_rs232_rx[3] == 25)
          {
            int sineAssistanceTypeIdx = int(data_rs232_rx[4]);
            String sineAssistanceTypeName;
            if (sineAssistanceTypeIdx == 1)
            {
              sineAssistanceTypeName = "Flexion";
            }
            else if (sineAssistanceTypeIdx == 2)
            {
              sineAssistanceTypeName = "Extension";
            }
            double sineAssistanceMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[5]) | ((uint16_t)data_rs232_rx[6] << 8))) / 100.0;
            int sineAssistanceShift = int(data_rs232_rx[7]);
            int sineAssistanceDuration = int(data_rs232_rx[8]);
            double sineAssistanceSaturation = ((int16_t)(((uint16_t)data_rs232_rx[9]) | ((uint16_t)data_rs232_rx[10] << 8))) / 100.0;
            cic.addSineAssistanceProfile(sineAssistanceTypeIdx,sineAssistanceMagnitude,sineAssistanceShift,sineAssistanceDuration,sineAssistanceSaturation);
            Serial.print("Added ");
            Serial.print(sineAssistanceTypeName);
            Serial.print(" assistance. Magnitude = ");
            Serial.print(sineAssistanceMagnitude);
            Serial.print(", shift = ");
            Serial.print(sineAssistanceShift);
            Serial.print(", duration = ");
            Serial.print(sineAssistanceDuration);
            Serial.print(", saturation = ");
            Serial.print(sineAssistanceSaturation);
            Serial.print(". Number of flexion profile = ");
            Serial.print(cic.sineAssistanceFlexionProfileCounter);
            Serial.print(". Number of extension profile = ");
            Serial.println(cic.sineAssistanceExtensionProfileCounter);
          }
          else if (data_rs232_rx[3] == 26)
          {
            int sineAssistanceTypeIdx = int(data_rs232_rx[4]);
            String sineAssistanceTypeName;
            if (sineAssistanceTypeIdx == 1)
            {
              sineAssistanceTypeName = "Flexion";
            }
            else if (sineAssistanceTypeIdx == 2)
            {
              sineAssistanceTypeName = "Extension";
            }
            int sineAssistanceProfileIdx = int(data_rs232_rx[5]);
            // for (int i = 0; i < 3; i++)
            // {
            //   Serial.print(cic.sineAssistanceExtensionParameterList[0][i]);
            //   Serial.print(" ");
            // }
            // Serial.println(" ");
            cic.removeSineAssistanceProfile(sineAssistanceTypeIdx,sineAssistanceProfileIdx);
            Serial.print("Removed ");
            Serial.print(sineAssistanceTypeName);
            Serial.print(" assistance. ID = ");
            Serial.println(sineAssistanceProfileIdx);

            // for (int i = 0; i < 3; i++)
            // {
            //   Serial.print(cic.sineAssistanceExtensionParameterList[0][i]);
            //   Serial.print(" ");
            // }
            // Serial.println(" ");
          }
          else if (data_rs232_rx[3] == 27)
          {
            int sineAssistanceTypeIdx = int(data_rs232_rx[4]);
            String sineAssistanceTypeName;
            if (sineAssistanceTypeIdx == 1)
            {
              sineAssistanceTypeName = "Flexion";
            }
            else if (sineAssistanceTypeIdx == 2)
            {
              sineAssistanceTypeName = "Extension";
            }
            int sineAssistanceProfileIdx = int(data_rs232_rx[5]);
            double sineAssistanceMagnitude = ((int16_t)(((uint16_t)data_rs232_rx[6]) | ((uint16_t)data_rs232_rx[7] << 8))) / 100.0;
            int sineAssistanceShift = int(data_rs232_rx[8]);
            int sineAssistanceDuration = int(data_rs232_rx[9]);
            double sineAssistanceSaturation = ((int16_t)(((uint16_t)data_rs232_rx[10]) | ((uint16_t)data_rs232_rx[11] << 8))) / 100.0;
            cic.editSineAssistanceProfile(sineAssistanceTypeIdx,sineAssistanceProfileIdx,sineAssistanceMagnitude,sineAssistanceShift,sineAssistanceDuration,sineAssistanceSaturation);
            Serial.print("Edit ");
            Serial.print(sineAssistanceTypeName);
            Serial.print(" assistance. ID = ");
            Serial.print(sineAssistanceProfileIdx);
            Serial.print(". Magnitude = ");
            Serial.print(sineAssistanceMagnitude);
            Serial.print(", shift = ");
            Serial.print(sineAssistanceShift);
            Serial.print(", duration = ");
            Serial.print(sineAssistanceDuration);
            Serial.print(", saturation = ");
            Serial.println(sineAssistanceSaturation);
          }
        }
      }
    }
  }
}

void send_ble_Data()
{
  LK_ble = imu.LKx * 100;
  RK_ble = imu.RKx * 100;

  current_command_L_ble = -Cur_command_L * 100; // Gui flexion is negative
  current_command_R_ble = Cur_command_R * 100;  // Gui flexion is negative
  // current_command_L_ble = -m1.iq_A * 100;
  // current_command_R_ble = m2.iq_A * 100;

  torque_command_L_ble = torque_command_L * 100; // Gui flexion is negative
  torque_L_ble = m1.torque * 100;  // motor torque constant = 0.232 Nm/A, gear ratio = 9;

  torque_command_R_ble = -torque_command_R * 100; // Gui flexion is negative
  torque_R_ble = -m2.torque * 100; 

  //  gait_percentage_L_ble = imu.gait_percentage_L * 100;
  gait_percentage_L_ble = 0;

  // insole_torque_command_L_ble = insole.normalized_torque_command_L * 100;
  // insole_torque_command_R_ble = insole.normalized_torque_command_R * 100;
  // insole_gait_percent_L_ble = insole.gait_percent_L * 100;
  // insole_gait_percent_R_ble = insole.gait_percent_R * 100;
  imu_gait_percent_L_ble = cic.GP_IMU_L_ahead * 100;
  imu_gait_percent_R_ble = cic.GP_IMU_R_ahead * 100;
  //
  imu_gait_percent_imp_L_ble = cic.GP_IMU_L_ahead_imp * 100;
  imu_gait_percent_imp_R_ble = cic.GP_IMU_R_ahead_imp * 100;
  imu_gait_percent_ff_L_ble = cic.GP_IMU_L_ahead_ff * 100;
  imu_gait_percent_ff_R_ble = cic.GP_IMU_R_ahead_ff * 100;
  //
  torque_command_imp_L_ble = -cic.impedanceTorque_L * 100;
  torque_command_imp_R_ble = -cic.impedanceTorque_R * 100;
  torque_command_ff_L_ble = cic.feedforwardTorque_L * 100;
  torque_command_ff_R_ble = cic.feedforwardTorque_R * 100;
  //
  // motor_speed_L_ble = m1.speed_value / 180 * 3.1415 * 100; // radian
  // motor_speed_R_ble = m2.speed_value / 180 * 3.1415 * 100; // radian

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0] = 165;
  data_ble[1] = 90;
  data_ble[2] = datalength_ble;
  data_ble[3] = LK_ble;
  data_ble[4] = LK_ble >> 8;
  data_ble[5] = RK_ble;
  data_ble[6] = RK_ble >> 8;
  data_ble[7] = current_command_L_ble;
  data_ble[8] = current_command_L_ble >> 8;
  data_ble[9] = current_command_R_ble;
  data_ble[10] = current_command_R_ble >> 8;
  data_ble[11] = torque_command_L_ble;
  data_ble[12] = torque_command_L_ble >> 8;
  data_ble[13] = torque_command_R_ble;
  data_ble[14] = torque_command_R_ble >> 8;
  data_ble[15] = torque_L_ble;
  data_ble[16] = torque_L_ble >> 8;
  data_ble[17] = torque_R_ble;
  data_ble[18] = torque_R_ble >> 8;
  data_ble[19] = triggerOn;
  data_ble[20] = triggerVal;
  data_ble[21] = imu_gait_percent_L_ble;
  data_ble[22] = imu_gait_percent_L_ble >> 8;
  data_ble[23] = imu_gait_percent_R_ble;
  data_ble[24] = imu_gait_percent_R_ble >> 8;
  data_ble[25] = motor_speed_L_ble;
  data_ble[26] = motor_speed_L_ble >> 8;
  data_ble[27] = motor_speed_R_ble;
  data_ble[28] = motor_speed_R_ble >> 8;
  //
  
  Serial5.write(data_ble, datalength_ble);
}

//*** Ringbuf ***//


void logData3()
{
  // Max RingBuf used bytes. Useful to understand RingBuf overrun.
  size_t maxUsed = 0;

  // Min spare micros in loop.
  int32_t minSpareMicros = INT32_MAX;

  // Start time.
  uint32_t logTime = micros();
  // Log data until Serial input or file full.
  //  while (!Serial.available()) {
  // Amount of data in ringBuf.
  size_t n = rb.bytesUsed();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20))
  {
    Serial.println("File full - quitting.");
    return; // break;
  }
  if (n > maxUsed)
  {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy())
  {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512))
    {
      Serial.println("writeOut failed");
      return; //break;
    }
  }
  // Time for next point.
  //  logTime += LOG_INTERVAL_USEC;
  //  int32_t spareMicros = logTime - micros();
  //  if (spareMicros < minSpareMicros) {
  //    minSpareMicros = spareMicros;
  //  }
  //  if (spareMicros <= 0) {
  //    Serial.print("Rate too fast ");
  //    Serial.println(spareMicros);
  //    break;
  //  }
  // Wait until time to log data.
  //  while (micros() < logTime) {}

  // Read ADC0 - about 17 usec on Teensy 4, Teensy 3.6 is faster.
  //  uint16_t adc = analogRead(0);
  // Print spareMicros into the RingBuf as test data.
     rb.print(relTime);
     rb.write(" ");
     rb.print(imu.TKx);
     rb.write(" ");
     rb.print(imu.TKy);
     rb.write(" ");
     rb.print(imu.TKz);
     rb.write(" ");
     rb.print(imu.LTx);
     rb.write(" ");
     rb.print(imu.LTy);
     rb.write(" ");
     rb.print(imu.LTz);
     rb.write(" ");
     rb.print(imu.RTx);
     rb.write(" ");
     rb.print(imu.RTy);
     rb.write(" ");
     rb.print(imu.RTz);
     rb.write(" ");
     rb.print(imu.LSx);
     rb.write(" ");
     rb.print(imu.LSy);
     rb.write(" ");
     rb.print(imu.LSz);
     rb.write(" ");
     rb.print(imu.RSx);
     rb.write(" ");
     rb.print(imu.RSy);
     rb.write(" ");
     rb.print(imu.RSz);
     rb.write(" ");
     rb.print(imu.TKAVx);
     rb.write(" ");
     rb.print(imu.TKAVy);
     rb.write(" ");
     rb.print(imu.TKAVz);
     rb.write(" ");
     rb.print(imu.LTAVx);
     rb.write(" ");
     rb.print(imu.LTAVy);
     rb.write(" ");
     rb.print(imu.LTAVz);
     rb.write(" ");
     rb.print(imu.RTAVx);
     rb.write(" ");
     rb.print(imu.RTAVy);
     rb.write(" ");
     rb.print(imu.RTAVz);
     rb.write(" ");
     rb.print(imu.LSAVx);
     rb.write(" ");
     rb.print(imu.LSAVy);
     rb.write(" ");
     rb.print(imu.LSAVz);
     rb.write(" ");
     rb.print(imu.RSAVx);
     rb.write(" ");
     rb.print(imu.RSAVy);
     rb.write(" ");
     rb.print(imu.RSAVz);
     rb.write(" ");
     rb.print(cic.GP_IMU_L);
     rb.write(" ");
     rb.print(cic.GP_IMU_R);
     rb.write(" ");
     rb.print(Cur_command_L);
     rb.write(" ");
     rb.print(Cur_command_R);
     rb.write(" ");
     rb.print(m1.torque / torque_constant_before_gear / Gear_ratio);
     rb.write(" ");
     rb.print(m1.torque / torque_constant_before_gear / Gear_ratio);
     rb.write(" ");
     rb.print(m1.pos);
     rb.write(" ");
     rb.print(m2.pos);
     rb.write(" ");
     rb.print(m1.spe);
     rb.write(" ");
     rb.print(m2.spe);
     rb.write(" ");
     rb.print(triggerOn);
     rb.write(" ");
     rb.println(triggerVal);
    //  rb.print("\n");
  // Print adc into RingBuf.
  //  rb.println(adc);
  if (rb.getWriteError())
  {
    // Error caused by too few free bytes in RingBuf.
    Serial.println("WriteError");
    return; //break;
  }
  
}


//**************Plot Data*****************//

void plot_cic_data()
{
  // double tempVal = analogRead(A8);
  Serial.print(torque_command_L);
  Serial.print(" ");
  Serial.print(torque_command_R);
  Serial.print(" ");
  Serial.print(m1.torque);
  Serial.print(" ");
  Serial.println(m2.torque);
  // Serial.println(imu.LTx);
//    Serial.print(m1.pos/3.14*180);
//    Serial.print(" ");
//    Serial.print(m2.pos/3.14*180);
//    Serial.print(" ");
//    Serial.println(imu.LTx);
  // for (int i = 0; i < 3; i++)
  // {
  //   Serial.print(cic.sineAssistanceExtensionParameterList[0][i]);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");
  
  // Serial.print(Cur_command_L);
  // Serial.print(" ");
  // Serial.print(cic.T_command_L/0.232);
  // Serial.print(" ");
  // Serial.println(m1.iq_A);
  // Serial.println(triggerVal);

}


void initial_CAN2()
{
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(100);
  Serial.println("Can bus setup done...");

  // motor 1
  m1.initial_CAN();
    delay(100);
  m1.exit_control_mode();
  delay(200);
  m1.exit_control_mode();
  delay(200);
  m1.enter_control_mode();
  delay(200);
  receive_CAN_data();
  delay(200);
  m1.set_origin();
  delay(200);
  receive_CAN_data();
  delay(200);
  // motor 2
  m2.initial_CAN();
    delay(100);
  m2.exit_control_mode();
  delay(200);
  m2.exit_control_mode();
  delay(200);
  m2.enter_control_mode();
  delay(200);
  receive_CAN_data();
  delay(200);
  m2.set_origin();
  delay(200);
  receive_CAN_data();
  delay(200);
}
void receive_CAN_data()
{
  if (Can3.read(msgR))
  {
    Can3.read(msgR);
    int id = msgR.buf[0];
    // Serial.print(msgR.id, HEX );
    if (id == Motor_ID1)
    {
      m1.unpack_reply(msgR);
    }
    if (id == Motor_ID2)
    {
      m2.unpack_reply(msgR);
    }
  }
}

void reset_motor_angle()
{
  // TODO: check if this is correct!
  m1.set_origin();
  delay(200);
  receive_CAN_data();
  delay(2);
  m2.set_origin();
  delay(200);
  receive_CAN_data();
  delay(2);
  
}
