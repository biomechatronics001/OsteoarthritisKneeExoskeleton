//for right knee exo, positive current assist extension, and torque sensor measures positive torque
// #include "Insole_mod.h"
#include "Motor_Control_Pediatric_V2.h"
#include <SPI.h>
#include <FlexCAN.h>
// #include "ads1292r.h"
// #include "Torque_Control.h"
#include "WL_IMU.h"
#include "WL_Encoder.h"
#include "CIC.h"
#include <Arduino.h>
#include <stdio.h>
//#include <SD.h>

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

int stopFlag = 0;
// *** End for RingBuf *** //

CAN_message_t msgR;
struct CAN_filter_t defaultMask;

int assist_mode = 12;
int saveDataFlag = 1;
int numOfInitialSteps = 1;
int enableExtensionStop = 0;
double LGPPrevious = 0;
double LGPCurrent = 0;
double dataCounter = 0;
int initialStepCounter = 0;

double weight = 52; // [kg] weight of the subject
uint32_t ID_offset = 0x140;
uint32_t Motor_ID1 = 2; // Motor Can Bus ID, left leg, loadcell: port 1, positive current = extension // updated on 2022-04-01 2
uint32_t Motor_ID2 = 3; // Motor Can Bus ID, right leg, loadcell: port 2, positive current = flexion // updated on 2022-04-01 3
int CAN_ID = 0;         // CAN port from Teensy
double Gear_ratio = 9;  //The actuator gear ratio, will enfluence actuator angle and angular velocity
int Stop_button = 0;    // Stop function
String mode = "start";
double milli_time_current_double = 0;
Motor_Control_Pediatric_V2 m1(Motor_ID1, CAN_ID, Gear_ratio); //Create motor object see Motor_Control_Pediatric_V2.h
Motor_Control_Pediatric_V2 m2(Motor_ID2, CAN_ID, Gear_ratio); //Create motor object see Motor_Control_Pediatric_V2.h
// ads1292r torque_sensor1;                                      //Create torque sensor object see ads1292r.h
// Torque_Control Tor_control;                                   //Creare torque control object see Torque_control.h
IMU imu;                                                      //Create IMU object see WL_IMU.h
// Insole insole;                                                // Create insole object
EncoderCl encoder;
CIC cic;
double Pgain = 7.5;    //P gain of torque control
double Igain = 0.7;    //I gain of torque control
double Dgain = 0;      //D gain of torque control
double MaxPIDout = 10; //Max torque control PID output (Unit Current A, inner loop is current controller)

double Fsample = 500;        // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
double Fsample_ble = 100;    // [Hz] Bluetooth sending data frequency
// double Fsample_insole = 200; // [Hz] insole reading data frequency
unsigned long current_time = 0;
unsigned long previous_time = 0;                                                        // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                                    // used to control the Bluetooth communication frequency
// unsigned long previous_time_insole = 0;                                                 // used to control the insole reading data frequency
unsigned long Tinterval_microsecond = (unsigned long)(1000000 / Fsample);               // used to control the teensy controller frequency
unsigned long Tinterval_ble_microsecond = (unsigned long)(1000000 / Fsample_ble);       // used to control the Bluetooth communication frequency
// unsigned long Tinterval_insole_microsecond = (unsigned long)(1000000 / Fsample_insole); // used to control the insole reading data frequency
//double Tsample = 1 / Fsample;           // sample period (second)

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
//double Tor_command_R = 0;
//double Pos_command_L = 0;
//double Pos_command_R = 0;
//double milli_time_current_double = 0;

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

//***********High Level Communication*********//
int LK_highlevel = 0;
int RK_highlevel = 0;

int TK_highlevel = 0;
int LT_highlevel = 0;
int RT_highlevel = 0;
int LS_highlevel = 0;
int RS_highlevel = 0;

int Left_Knee_Torque = 0;
int Right_Knee_Torque = 0;
char data_serial_highlevel[101] = {0};
char data_highlevel_rx[7] = {0}; //data high level communication
char Highlevel_Data_Length_Send = 101;
double Left_Knee_Torque_Command;
double Right_Knee_Torque_Command;
double relTime = 0.0;
int SDcounter = 0;
int SDtotalCounter = 0;
String dataBuffer = "";
#define charBufferSize 100
char charBuffer[charBufferSize];
long charBufferPosition = 0;
//File dataFile;

//File dataFile = SD.open("walking.txt", FILE_WRITE);

void setup()
{
  // put your setup code here, to run once:
  delay(3000);
  Serial.begin(115200);  //used for communication with computer.
  Serial4.begin(115200); //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  initial_CAN();
  // torque_sensor1.Torque_sensor_initial();                                           //initial the torque sensor see ads1292r.cpp.
  // torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) * 2, 0.0003446 * (-1) * 2.35); //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.
  // torque_sensor1.Torque_sensor_offset_calibration();                                //Auto offset the torque sensor to zero. see ads1292r.cpp.
  delay(500);
  imu.Gain_E = 1;         //Extension gain for delay output feedback control
  imu.Gain_F = 1;         //Flexion gain for delay output feedback control  DOFC.Gain_E = 10;            //Extension gain for delay output feedback control
  imu.STS_Gain = 0;
  imu.delaypoint = 0;     //realative to delay time (delaypoint*sampletime=delaytime) for delay output feedback control
  encoder.Gain_E = 1;     //Extension gain for delay output feedback control
  encoder.Gain_F = 1;     //Flexion gain for delay output feedback control
  encoder.delaypoint = 0; //realative to delay time (delaypoint*sampletime=delaytime) for delay output feedback control
  imu.alpha = 5;
  imu.beta = 2;
  imu.angleRelativeThreshold = 20;
  m1.init_motor(); // Strat the CAN bus communication & Motor Control
  delay(100);
  m2.init_motor(); // Strat the CAN bus communication & Motor Control
  delay(100);
  reset_motor_angle();
  CurrentControlSetup();
  // initialize sd card
  // Serial.print("Initializing SD card...");
  //   see if the card is present and can be initialized:
  // if (!SD.begin(chipSelect)) {
  //   Serial.println("Card failed, or not present");
  //   // don’t do anything more:
  //   return; 
  // }
  // SD.remove("walking.txt");
  // Serial.println("card initialized.");

  //TorqueControlSetup();    //initialize torque control (if you want to use torque sensor as a feedback you also need to use this function)
  //CurrentControlSetup();
  //PositionControlSetup();
  // insole.INIT();
  // init trigger
  // pinMode(triggerPin, INPUT);
  // pinMode(A8, OUTPUT);
  pinMode(A8, INPUT);
  
  // cic.calculateSigmoid();
  cic.calculateSigmoid(cic.kdMax, cic.kdMin, cic.kd_arr_Junxi);
  cic.calculateSigmoid(cic.bdMax, cic.bdMin, cic.bd_arr_Junxi);
  cic.calculateSigmoid(1, 0, cic.sigmoid);
  //  dataFile = SD.open("walking.txt", O_WRITE|O_CREAT);
  // if (saveDataFlag)
  // {
  //   SDCardSetup();
  // }
  
}

void loop()
{

  // digitalWrite(A8, LOW); //pin converted into GND
  //  File dataFile = SD.open("walking.txt", FILE_WRITE);
  // clearSerialInput();
  while (stopFlag)
  {
  };
  // logData();
  // clearSerialInput();
  CurrentControl();
  //  dataFile.close();
  //TorqueControlExample();    //if you want to try torque control, please uncommment TorqueControlExample(); and use this function.
  //CurrentControlExample();   //if you want to try current control, please uncommment CurrentControlExample(); and use this function.
  //PositionControlExample();  //if you want to try position control, please uncommment PositionControlExample(); and use this function.
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
  ////******IMU+Current Control Example Torque Constant 0.6 Nm/A**********////////
  imu.READ();                          //Check if IMU data available and read it. the sample rate is 100 hz
  // torque_sensor1.Torque_sensor_read(); //Check if torque sensor1 is available // Vahid
  current_time = micros();             //query current time (microsencond)

  //********* use to control the insole reading data frequency **********//
  // if (current_time - previous_time_insole > Tinterval_insole_microsecond)
  // {
  //   insole.READ();
  //   previous_time_insole = current_time;
  //   // plot_cic_IMU_data();
  // }

  //********* use to control the teensy controller frequency **********//
  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    if (Stop_button) //stop
    {
      Cur_command_L = 0;
      Cur_command_R = 0;
    }
    else
    {
      Compute_Cur_Commands(); // Vahid
    }
    Cur_limitation();
    if (assist_mode == 10)
    {
      // enforceInitialSteps();
    }

    //Cur_command_L = 2;//positive=flex
    //Cur_command_R = -2;//positive=exten
    m1.send_current_command(Cur_command_L);
    receive_CAN_data();
    m2.send_current_command(Cur_command_R);
    receive_CAN_data();
    /////*********Print motor Position - Use below code to read Position 2021-08-17 by Howard*********/////
    m1.read_multi_turns_angle(); //read angle and angle velocity
    receive_CAN_data();
    m2.read_multi_turns_angle(); //read angle and angle velocity
    receive_CAN_data();
    encoder.UpdateData(m1.motorAngle, m2.motorAngle);
    encoder.DelayedOutputTorqueControl(); // calculate the encoder-based torque using Samsung algorithm
    // Read trigger signal
    // triggerVal = analogRead(triggerPin);
    // digitalWrite(A8, LOW); //pin converted into GND
    triggerVal = analogRead(A8); // temp change on 01/17/2023 for the Sinai visit
    if (triggerVal < 400)
    {
      triggerOn = 0;
    }
    else
    {
      triggerOn = 1;
    }
    //Serial.println(triggerVal);
    // Serial.print(" ");
    // Serial.println(triggerOn);

    //    save_debug_data_SDCard();
    // if (saveDataFlag){
    //   logData3();
    // }
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

void SDCardSaveToFile()
{
  rb.sync();
  file.truncate();
  file.rewind();
  file.close();
}

void Compute_Cur_Commands()
{
  if (assist_mode == 1) //IMU walking
  {
    mode = "Walking (IMU)";
    Cur_command_L = -imu.DOTC[0] / 2.2; //this is for bilateral walking assistance_left leg
    Cur_command_R = +imu.DOTC[1] / 2.2; //this is for bilateral walking assistance_right leg
    // Serial.print(Cur_command_L);
    // Serial.print(" ");
    // Serial.println(Cur_command_R);
    // Serial.print("imu.gainE=");
    // Serial.println(imu.Gain_E);
  }
  else if (assist_mode == 2)
  {
    mode = "Walking (Insoles)";
    // Cur_command_L = Insole_gain * insole.normalized_torque_command_L;
    // Cur_command_R = Insole_gain * insole.normalized_torque_command_R;
  }
  else if (assist_mode == 3) //Encode Walking
  {
    mode = "Walking (Encoder)";
    Cur_command_L = encoder.DOFC[0] / 2.2;
    Cur_command_R = -encoder.DOFC[1] / 2.2;

    //    Serial.print("left encoder torque: ");
    //    Serial.println(encoder.DOFC[0]);
    //    Serial.print("right encoder torque: ");
    //    Serial.println(encoder.DOFC[1]);
  }
  else if (assist_mode == 4) //sine wave
  {
    mode = "Sine Wave";
    //      Cur_command_L =  imu.Gain_E * sin(2 * PI * current_time / 1000000)/2.2;    //(unit Amp);
    //      Cur_command_R = -imu.Gain_F * sin(2 * PI * current_time / 1000000)/2.2;    //(unit Amp);
    Cur_command_L = 1 * sin(2 * PI * current_time / 1000000) / 2.2; //(unit Amp);
    Cur_command_R = 1 * sin(2 * PI * current_time / 1000000) / 2.2; //(unit Amp);
    milli_time_current_double = millis();
    milli_time_current_double = milli_time_current_double / 1000.0;
  }
  else if (assist_mode == 5)
  {

    mode = "High Level Control";

    send_serial_Data_Highlevel(); //Serial communication with High level system
    receive_serial_Data_Highlevel();
    Cur_command_L = Left_Knee_Torque_Command / 2.2;
    Cur_command_R = Right_Knee_Torque_Command / 2.2;
  }
  else if (assist_mode == 6)
  {
    mode = "Constant Signal";
    Cur_command_L = 1; //* imu.Gain_E;
    Cur_command_R = -1; //* imu.Gain_F;
  }
  else if (assist_mode == 7)
  {
    mode = "Squatting (IMU)";

    Cur_command_L = 10 * 0.02 * imu.Gain_E * imu.SquatTorque / 2.2;
    // Serial.print(imu.Gain_E);
    // Serial.print(" ");
    // Serial.println(imu.Gain_F);
    Cur_command_R = -10 * 0.02 * imu.Gain_E * imu.SquatTorque / 2.2;
  }
  else if (assist_mode == 8)
  {
    mode = "Sit-to-Stand (IMU)";
    Cur_command_L = +imu.STSTorque / 2.2;
    Cur_command_R = -imu.STSTorque / 2.2;
  }
  else if (assist_mode == 9)
  {
    mode = "Stair Ascending (IMU)";
    Cur_command_L = +imu.DOTC_ascending[0] / 2.2;
    Cur_command_R = -imu.DOTC_ascending[1] / 2.2;
  }
  else if (assist_mode == 10)
  {
    mode = "Collocated Impedance Control (IMU)";
    cic.CIC_update(imu.LTAVx - imu.RTAVx, -m1.motorAngle * Gear_ratio, m2.motorAngle * Gear_ratio, -m1.speed_value * Gear_ratio, m2.speed_value * Gear_ratio);
    Cur_command_L = -cic.Cur_command_L; //flexion is + in the CIC class and extension is -.
    Cur_command_R = cic.Cur_command_R;  //flexion is + in the CIC class and extension is -.
    //Cur_command_L=0; //current + is extension
    //Cur_command_R=0; //current + is flexion
    //plot_cic_IMU_data();
  }
  else if (assist_mode == 11)
  {
    mode = "Sit-to-Stand (IMU)";
    Cur_command_L = +imu.STSTorque / 0.232;
    Cur_command_R = -imu.STSTorque / 0.232;
  }
  else if (assist_mode == 12)
  {
    cic.computeGaitPhase(imu.LTAVx - imu.RTAVx, -m1.motorAngle * Gear_ratio, m2.motorAngle * Gear_ratio, -m1.speed_value * Gear_ratio, m2.speed_value * Gear_ratio);
    cic.computeSineAssistanceTorque();
    Cur_command_L = cic.Cur_command_L;
    Cur_command_R = -cic.Cur_command_R;
  }
  else if (assist_mode == 100)
  {
    mode = "Stop";
    Cur_command_L = 0;
    Cur_command_R = 0;
  }
}

void Cur_limitation()
{
  //************* Current limitation *************//

  Cur_command_L = min(current_limitation, Cur_command_L);
  Cur_command_L = max(-current_limitation, Cur_command_L);
  Cur_command_R = min(current_limitation, Cur_command_R);
  Cur_command_R = max(-current_limitation, Cur_command_R);

  torque_command_L = Cur_command_L * 0.232 * 9;//2.2;
  torque_command_R = Cur_command_R * 0.232 * 9;//2.2;
}

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
  if (Serial4.available() >= 20)
  {
    // Serial.println("-------------New data received-------------------");
    data_rs232_rx[0] = Serial4.read();
    if (data_rs232_rx[0] == 165)
    {
      data_rs232_rx[1] = Serial4.read();
      if (data_rs232_rx[1] == 90)
      {
        data_rs232_rx[2] = Serial4.read();
        if (data_rs232_rx[2] == 20)
        {
          Serial4.readBytes(&data_rs232_rx[3], 17);
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
            else if (assist_mode == 3)
            {
              encoder.Gain_E = Gain_E;
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
            else if (assist_mode == 3)
            {
              encoder.Gain_F = Gain_F;
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
            else if (assist_mode == 3)
            {
              encoder.delaypoint = delaypoint;
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
  if (assist_mode == 3)
  {
    LK_ble = encoder.KneeAngle[0] * 100;
    RK_ble = encoder.KneeAngle[1] * 100;
  }
  else
  {
    LK_ble = imu.LKx * 100;
    RK_ble = imu.RKx * 100;
  }

  current_command_L_ble = -Cur_command_L * 100; // Gui flexion is negative
  current_command_R_ble = Cur_command_R * 100;  // Gui flexion is negative
  // current_command_L_ble = -m1.iq_A * 100;
  // current_command_R_ble = m2.iq_A * 100;

  torque_command_L_ble = torque_command_L * 100; // Gui flexion is negative
  torque_L_ble = m1.iq_A * 0.232 * 9 * 100;  // motor torque constant = 0.232 Nm/A, gear ratio = 9;

  torque_command_R_ble = -torque_command_R * 100; // Gui flexion is negative
  torque_R_ble = -m2.iq_A * 0.232 * 9 * 100; 

  //  gait_percentage_L_ble = imu.gait_percentage_L * 100;
  gait_percentage_L_ble = 30 * 100;

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
  motor_speed_L_ble = m1.speed_value / 180 * 3.1415 * 100; // radian
  motor_speed_R_ble = m2.speed_value / 180 * 3.1415 * 100; // radian

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
  // data_ble[15] = torque_command_L_ble;
  // data_ble[16] = torque_command_L_ble >> 8;
  // data_ble[17] = torque_command_R_ble;
  // data_ble[18] = torque_command_R_ble >> 8;
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
  // data_ble[25] = current_actual_L_ble;
  // data_ble[26] = current_actual_L_ble >> 8;
  // data_ble[27] = current_actual_R_ble;
  // data_ble[28] = current_actual_R_ble >> 8;
  //
  // data_ble[25] = imu_gait_percent_imp_L_ble;
  // data_ble[26] = imu_gait_percent_imp_L_ble >> 8;
  // data_ble[27] = imu_gait_percent_imp_R_ble;
  // data_ble[28] = imu_gait_percent_imp_R_ble >> 8;
  // data_ble[29] = imu_gait_percent_ff_L_ble;
  // data_ble[30] = imu_gait_percent_ff_L_ble >> 8;
  // data_ble[31] = imu_gait_percent_ff_R_ble;
  // data_ble[32] = imu_gait_percent_ff_R_ble >> 8;
  // //
  // data_ble[33] = torque_command_imp_L_ble;
  // data_ble[34] = torque_command_imp_L_ble >> 8;
  // data_ble[35] = torque_command_imp_R_ble;
  // data_ble[36] = torque_command_imp_R_ble >> 8;
  // data_ble[37] = torque_command_ff_L_ble;
  // data_ble[38] = torque_command_ff_L_ble >> 8;
  // data_ble[39] = torque_command_ff_R_ble;
  // data_ble[40] = torque_command_ff_R_ble >> 8;
  // //
  // data_ble[41] = insole_gait_percent_L_ble;
  // data_ble[42] = insole_gait_percent_L_ble >> 8;
  // data_ble[43] = insole_gait_percent_R_ble;
  // data_ble[44] = insole_gait_percent_R_ble >> 8;
  // data_ble[19] = gait_percentage_L_ble;
  // data_ble[20] = gait_percentage_L_ble >> 8;
  // data_ble[21] = insole_torque_command_L_ble;
  // data_ble[22] = insole_torque_command_L_ble >> 8;
  // data_ble[23] = insole_torque_command_R_ble;
  // data_ble[24] = insole_torque_command_R_ble >> 8;
  // data_ble[25] = insole_gait_percent_L_ble;
  // data_ble[26] = insole_gait_percent_L_ble >> 8;
  // data_ble[27] = insole_gait_percent_R_ble;
  // data_ble[28] = insole_gait_percent_R_ble >> 8;

  //  data_ble[29] = torque_commandGP_ble;
  //  data_ble[30] = torque_commandGP_ble>>8;
  //  data_ble[31] = imu.isStand;
  //Serial.println("in");
  Serial4.write(data_ble, datalength_ble);
}

//******************Receive high level controller Command****************//
void receive_serial_Data_Highlevel()
{
  if (Serial.available() >= 7)
  {
    //Serial.println("receive");
    data_highlevel_rx[0] = Serial.read();
    if (data_highlevel_rx[0] == 165)
    {
      data_highlevel_rx[1] = Serial.read();
      if (data_highlevel_rx[1] == 90)
      {
        Serial.readBytes(&data_highlevel_rx[2], 5); //int data_rs232_rx[7]
        //Highlevel_Data_Length_Receive = int(data_highlevel_rx[2]); // read data length
        Left_Knee_Torque_Command = ((int16_t)(((int16_t)data_highlevel_rx[3]) | ((int16_t)data_highlevel_rx[4] << 8)));
        Right_Knee_Torque_Command = ((int16_t)(((int16_t)data_highlevel_rx[5]) | ((int16_t)data_highlevel_rx[6] << 8)));
        Left_Knee_Torque_Command = Left_Knee_Torque_Command / 100;
        Right_Knee_Torque_Command = Right_Knee_Torque_Command / 100;
      }
    }
  }
}
//******************Send high level controller Command****************//
void send_serial_Data_Highlevel()
{
  LK_highlevel = imu.LKx * 100;
  RK_highlevel = imu.RKx * 100;

  TK_highlevel = imu.TKx * 100;
  LT_highlevel = imu.LTx * 100;
  RT_highlevel = imu.RTx * 100;
  LS_highlevel = imu.LSx * 100;
  RS_highlevel = imu.RSx * 100;
  // Left_Knee_Torque = torque_sensor1.torque[0] * 100;
  // Right_Knee_Torque = torque_sensor1.torque[1] * 100;

  data_serial_highlevel[0] = 165;
  data_serial_highlevel[1] = 90;
  data_serial_highlevel[2] = Highlevel_Data_Length_Send;
  data_serial_highlevel[3] = Left_Knee_Torque >> 8;
  data_serial_highlevel[4] = Left_Knee_Torque;
  data_serial_highlevel[5] = Right_Knee_Torque >> 8;
  data_serial_highlevel[6] = Right_Knee_Torque;
  data_serial_highlevel[7] = LK_highlevel >> 8;
  data_serial_highlevel[8] = LK_highlevel;
  data_serial_highlevel[9] = RK_highlevel >> 8;
  data_serial_highlevel[10] = RK_highlevel;

  data_serial_highlevel[11] = LT_highlevel >> 8;
  data_serial_highlevel[12] = LT_highlevel;
  data_serial_highlevel[29] = RT_highlevel >> 8;
  data_serial_highlevel[30] = RT_highlevel;
  data_serial_highlevel[47] = LS_highlevel >> 8;
  data_serial_highlevel[48] = LS_highlevel;
  data_serial_highlevel[65] = RS_highlevel >> 8;
  data_serial_highlevel[66] = RS_highlevel;
  data_serial_highlevel[83] = TK_highlevel >> 8;
  data_serial_highlevel[84] = TK_highlevel;

  Serial.write(data_serial_highlevel, Highlevel_Data_Length_Send);
}
void initial_CAN()
{
  //initial CAN Bus
  Can0.begin(1000000, defaultMask, 1, 1);
  delay(3000);
  pinMode(28, OUTPUT);
  digitalWrite(28, LOW);
  Serial.println("Can bus setup done...");
}
void receive_CAN_data()
{
  while (Can0.available() > 0)
  {
    Can0.read(msgR);
    if (msgR.id == (ID_offset + Motor_ID1))
    {
      m1.DataExplanation(msgR);
    }
    else if (msgR.id == (ID_offset + Motor_ID2))
    {
      m2.DataExplanation(msgR);
    }
  }
}

//*** Ringbuf ***//
void logData()
{
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
  Serial.println("Type any character to stop");

  // Max RingBuf used bytes. Useful to understand RingBuf overrun.
  size_t maxUsed = 0;

  // Min spare micros in loop.
  int32_t minSpareMicros = INT32_MAX;

  // Start time.
  uint32_t logTime = micros();
  // Log data until Serial input or file full.
  while (!Serial.available())
  {
    // Amount of data in ringBuf.
    size_t n = rb.bytesUsed();
    if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20))
    {
      Serial.println("File full - quitting.");
      break;
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
        break;
      }
    }
    // Time for next point.
    logTime += LOG_INTERVAL_USEC;
    int32_t spareMicros = logTime - micros();
    if (spareMicros < minSpareMicros)
    {
      minSpareMicros = spareMicros;
    }
    if (spareMicros <= 0)
    {
      Serial.print("Rate too fast ");
      Serial.println(spareMicros);
      break;
    }
    // Wait until time to log data.
    while (micros() < logTime)
    {
    }

    // Read ADC0 - about 17 usec on Teensy 4, Teensy 3.6 is faster.
    uint16_t adc = analogRead(0);
    // Print spareMicros into the RingBuf as test data.
    rb.print(spareMicros);
    rb.write(',');
    // Print adc into RingBuf.
    rb.println(adc);
    if (rb.getWriteError())
    {
      // Error caused by too few free bytes in RingBuf.
      Serial.println("WriteError");
      break;
    }
  }
  // Write any RingBuf data to file.
  rb.sync();
  file.truncate();
  file.rewind();
  // Print first twenty lines of file.
  Serial.println("spareMicros,ADC0");
  for (uint8_t n = 0; n < 20 && file.available();)
  {
    int c = file.read();
    if (c < 0)
    {
      break;
    }
    Serial.write(c);
    if (c == '\n')
      n++;
  }
  Serial.print("fileSize: ");
  Serial.println((uint32_t)file.fileSize());
  Serial.print("maxBytesUsed: ");
  Serial.println(maxUsed);
  Serial.print("minSpareMicros: ");
  Serial.println(minSpareMicros);
  file.close();
}

void logData2()
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
  
     rb.print(relTime);
     rb.write(" ");
     rb.print(imu.LTAVx);
     rb.write(" ");
     rb.print(imu.RTAVx);
     rb.write(" ");
     rb.print(imu.LTx);
     rb.write(" ");
     rb.print(cic.TA_filter);
     rb.write(" ");
     rb.print(cic.qTd_L);
     rb.write(" ");
     rb.print(cic.qTd_L_mean);
     rb.write(" ");
     rb.print(cic.qTd_L_std);
     rb.write(" ");
     rb.print(cic.dqTd_L);
     rb.write(" ");
     rb.print(cic.dqTd_L_std);
     rb.write(" ");
     rb.print(cic.w);
     rb.write(" ");
     rb.print(cic.dqTd_L_nor);
     rb.write(" ");
     rb.print(cic.qTd_L_nor);
     rb.write(" ");
     rb.print(cic.GP_IMU_L);
     rb.write(" ");
     rb.print(cic.GP_IMU_L_Pre);
     rb.write(" ");
     rb.print(cic.delta_GP_IMU_L);
     rb.write(" ");
     rb.print(cic.mag);
     rb.write(" ");
     rb.println(cic.state);
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
     rb.print(m1.iq_A);
     rb.write(" ");
     rb.print(m2.iq_A);
     rb.write(" ");
     rb.print(m1.motorAngle);
     rb.write(" ");
     rb.print(m2.motorAngle);
     rb.write(" ");
     rb.print(m1.speed_value);
     rb.write(" ");
     rb.print(m2.speed_value);
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

void clearSerialInput()
{
  for (uint32_t m = micros(); micros() - m < 10000;)
  {
    if (Serial.read() >= 0)
    {
      m = micros();
    }
  }
}

//**************Plot Data*****************//

void plot_cic_data()
{
  double tempVal = analogRead(A8);
  Serial.println(tempVal);
  // Serial.println(cic.GP_IMU_L);
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

void plot_debug_data()
{
  char str8[8];
  // sprintf(str8,"%03.2f",imu.LTAVx);
  dtostrf(imu.LTAVx, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(imu.RTAVx, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(imu.LTAVx - imu.RTAVx, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.TA_filter, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.qTd_L, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.qTd_L_mean, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.qTd_L_std, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.dqTd_L, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.dqTd_L_std, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.w, 5, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(-cic.dqTd_L_nor, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.qTd_L_nor, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.GP_IMU_L, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.GP_IMU_L_Pre, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.delta_GP_IMU_L, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.mag, 7, 2, str8);
  Serial.print(str8);
  Serial.print("  ");
  dtostrf(cic.state, 7, 2, str8);
  Serial.println(str8);
}



void insertBuffer(double inputD)
{
  String input = String(inputD, 2);
  long inputSize = input.length() + 1;
  char tempArr[inputSize];
  input.toCharArray(tempArr, inputSize);
  strcpy(&charBuffer[charBufferPosition], &tempArr[0]);
  charBufferPosition += inputSize + 1;
}

void insertBuffer(String input)
{
  long inputSize = input.length() + 1;
  char tempArr[inputSize];
  input.toCharArray(tempArr, inputSize);
  strcpy(&charBuffer[charBufferPosition], &tempArr[0]);
  charBufferPosition += inputSize + 1;
}


void reset_motor_angle()
{
  for (int i = 0; i < 20; i++)
  {
    m1.read_multi_turns_angle();
    delay(10);
    receive_CAN_data();
    m1.motorAngle_offset = m1.motorAngle_raw;
    m2.read_multi_turns_angle();
    delay(10);
    receive_CAN_data();
    m2.motorAngle_offset = m2.motorAngle_raw;
  }
}
