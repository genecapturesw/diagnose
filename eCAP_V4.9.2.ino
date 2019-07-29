/*
   [eCAP Controller]
   
   User inputs settings for heater and/or electric field by using pre-set options specific to that as described in protocol "eCAP HYB Protocol V3".
      -Control of electric field, along with heating and cooling is done through relays which are connected to the pump module holder.
      -This is where the PCB boards with electrodes and resistors are along with a TEC, fan, heat sink, and 3 of the 4 thermocouples,
      -The last of which is to be placed in the target well.
      -This version of code requires pump/volume and time/pump to be adjusted programatically
     
VERSIONS:
eCAP 1.0 - 1st version of code for simplified user input
eCAP TEST - Testing Version of Code
eCAP 2.0 - Functioning Code for simplified user input
eCAP 2.1 - Updated PV regulation
eCAP 2.2 - added LCD to steps 3-5
eCAP 2.3 - Additional comments for readability & removal of "Step 6" (option for user to perform Step 1 with no electric field)
           - User will manually disable electric field if necessary
eCAP 2.4 - Added volumes to load fluid screen and added time and temp on running displays
eCAP 2.5 - Only displays time every couple of seconds
eCAP 2.7 - Updated values for Pressure regulation
eCAP 2.8 - Better fail-safe for pressure and vacuum pump, changed logic
eCAP 2.9 - Implemented changed logic for PV control to all steps
eCAP 3.0 - Changes to pneumatic pressure regulation
eCAP 3.3 - Additional comments added
eCAP 3.4 - Adjusted times for Step 0 and Step 3. Changed volume siplays for steps 0,2,3,4 and 5.
eCAP 3.7 - Fixed a bug where Step 0 would run longer than needed
eCAP 3.8 - Added communication for voltage/current reader
eCAP 3.9 - Adjusted Vol/Pump for Pump Module #4 and adjusted version number display
eCAP 4.0 - Protocol for V8
eCAP 4.1 - Adjusted PID loop for cooling
eCAP 4.2 - Changes made to Step 0 & 2
eCAP 4.3 - Fixed a bug where Step 0 would finish before pumping and edited Step 2 pump times
eCAP 4.3.2 - Version for pump module #2
eCAP 4.4.2 - Doubled Step 0 Time (Module #2)
eCAP 4.5.2 - Removed extra time from Step 0 and added 2 pumps to step 4 (Module #2)
eCAP 4.9.2 - Modified Step 1 Protocol to only do concentration for a longer period of time 

STATES
***throughout the steps, the return option returns you to setup or the running screen, depending on what you were on previously***
00-Boot up 
0-Splash Screen 
    -User Presses any Button to Begin
1-Setup
    -User chooses whether to adjust step number or start program
    -User can press start here to run chosen step
    -After making changes to settings, display always returns here unless the program is already running
2-Step Adjustment
    -User chooses what step to perform (0-5)
3-Confirmation of Settings
    -Settings chosen will display on LCD
    -User has the option to conifrm settings or go back 
4-Load Pump Module
    -LCD prompts user to load solution appropriate to the step chosen
    -User has the option to go back or press finished when solution has been loaded
5-Confirmation to Start
    -Program prompts user to start w/ selected settings or to return to setup screen
6-Confirm with User to turn on Electric Field
    -If user selected Step 1, LCD asks user if electric field should be applied
7-Step 0
    -Program initiates Step 0 in accordance to current protocol
    -User has the option to stop program or change steps
    -Refer to Section titled "States" for more details
8-Step 1
    -Program initiates Step 1 in accordance to current protocol
    -User has the option to stop program or change steps
    -Refer to Section titled "States" for more details
9-Step 2
    -Program initiates Step 2 in accordance to current protocol
    -User has the option to stop program or change steps
    -Refer to Section titled "States" for more details
10-Step 3
    -Program initiates Step 3 in accordance to current protocol
    -User has the option to stop program or change steps
    -Refer to Section titled "States" for more details
11-Step 4
    -Program initiates Step 4 in accordance to current protocol
    -User has the option to stop program or change steps
    -Refer to Section titled "States" for more details
12-Step 5
    -Program initiates Step 5 in accordance to current protocol
    -User has the option to stop program or change steps
    -Refer to Section titled "States" for more details
14-Change Steps while Running
    -User has the option of changing steps (0-5) while a current step is running
    -This pauses all functions with the exception of the heating cycle
    -After selecting new step, the program will return to state 4 and prompt user to load before requiring another input to start
15-Change Steps after completing a Step
    -User has the option of changing steps (0-5) after a step has been completed
    -This pauses all functions with the exception of the heating cycle
    -After selecting new step, the program will return to state 4 and prompt user to load before requiring another input to start

HEATING STATES
***heating states are implemented seperately so that the user can cycle through certain LCD states while remaining active***
1-Heating Active
    -PID heat control loop based on input temp, the temp on the slide, and the temp on the well
    -Utilizes Fan, TEC, and resistors
    -Limits for overall system heat based on temperature on the TEC and the resistors
2-Cooling Active
    -PID cool control loop based on input temp and the temp on the slide
    -Utilizes Fan and TEC

STEPS
***steps are utilized to follow standard hybridization protocol implemented in lab. Please refer to "eCAP Protocol" for more information***
0-Priming: User wets the entire pump module until fluid exits at the waste well. User then begins heating. 5+ min. Room Temp - 54 C.
1-Target: Target is washed over flowcell in steps, with a pause between pumps for mixing using electric field states. 30 min. 54 C.
2-Rinse 1: 2:05 | Rinse is washed over flowcell at temp for 25 s. Then temp is ramped to 18 C while pumping. 5-10 min. 54 - 18 C.
3-Detector: TE-Norm2 13D-1 | Detector is washed over flowcell. 18 C.
4-Rinse 2: 2:05 | Another rinse is washed over flowcell. 18 C.
5-Rinse 3: HAsc | HAsc is washed over for the same time as Rinse 2. 18 C.
*/



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////DEFINE VARIABLES//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//User Defined Settings
float tp = 2500; //time per pump
float vp = 6.0; //volume per pump
float flowcell_vol = 7.4; //sets expected flow cell volume (uL)
float reach_vol = 28; //sets expected vol to reach flow cell (uL)
#define version_num 4.9

////////////////////////////////////////////////////Include libraries for LCD Display and Thermocouple Interface Circuit//////////////////////////////////////////////////
//Define pins for Thermocouple Interface Circuit
#define DBIT A8 //TC Serial Output
#define CS A9 //TC Chip Select
#define CLK A10 //TC Serial Clock
#define T0 A11 //Output for changing TC read
#define T1 A12 //Output for changing TC read
#define T2 A13 //Output for changing TC read

//Include libraries for Thermocouple interface
#include <SPI.h>
#include "Adafruit_MAX31855.h"
Adafruit_MAX31855 thermocouple(CLK, CS, DBIT);

//Include libraries for LCD display
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

//////////////////////////////////////////////////////////define arduino input/output ports (relays, buttons, pneumatics)////////////////////////////////////////////////////////////
//Define ports for buttons (switches)
#define B5 45 //reset button
#define B4 47 //start button
#define B3 49 //right button
#define B2 51 //center button
#define B1 53 //left button

//Define ports for relays controlling electrodes
#define E1_V 2 //switches Electrode 1 between voltage and floating
#define E1_G 3 //switches Electrode 1 between floating and ground
#define E2_V 4 //switches Electrode 2 between voltage and floating
#define E2_G 5 //switches Electrode 2 between floating and ground
#define E3_V 6 //switches Electrode 3 between voltage and floating
#define E3_G 7 //switches Electrode 3 between floating and ground
#define E4 8 //switches Electrode 4 between voltage and ground
#define E5 9 //switches Electrode 5 between voltage and ground

//Define ports for relays controlling heating system
#define fan_relay 31 //controls the fan
#define res_relay 30 //controls the resistors
#define cool_relay 32 //controls the TEC for cooling
#define heat_relay 33 //controls the TEC for heat

//define pins on the arduino to be connected to pneumatic control
#define pressure_power 22
//to turn on the pressure pump
#define vacuum_power 23 //to turn on the vacuum pump
#define pressure_relief 24 //to relieve the pressure above a certain PSI
#define vacuum_relief 25 //to relieve the vacuum above a certain PSI
#define PV_valve1 27 //top valve for connection to flowcell holder
#define PV_pump 28 //middle pump for connection to flowcell holder
#define PV_valve2 29 //bottom valve for connection to flowcell holder
#define PV_extra 26 //extra valve for applying pressure and or vacuum as needed
#define pressure_read A0 //pin to read pressure
#define vacuum_read A1 //pin to read vacuum

//Define pins for Voltage communication
#define C_ON 10
#define ML_ON 11
#define MR_ON 12

/////////////////////////////////////////////////////////////////////////state and choice tracker defaults////////////////////////////////////////////////////////////////
int state = 0; //intialize state
int prev_state = 0; //keep track of previous states
int active_heat = -1; //don't run heat
int prev_heat = 0; //keeps track of what heating state user was in previously
int steps = -1; //saves user choice and tells running state what to do

///////////////////////////////////////////////////////////////////////////////initialize ON/OFF/////////////////////////////////////////////////////////////////////////
#define ON 0 //on constant for buttons
#define OFF 1 //off constant for buttons

///////////////////////////////////////////////////////////////////////Initialize variables for temp reading///////////////////////////////////////////////////////////
float input_temp = 23.0; //sets default temp to room temp
float temp1,temp2,temp3,temp4 = 0; //allows for avg of temp on slide
float temp_res1,temp_res2,temp_res3,temp_res4 = 0; //allows for avg of temp on res
float temp_TEC1,temp_TEC2,temp_TEC3,temp_TEC4 = 0; //allows for avg of temp on TEC
float temp_well1,temp_well2,temp_well3,temp_well4 = 0; //allows for avg of temp in well
int temp_switch = 1; //allows the temp to go back and forth between reading ports 4-7
float temp, temp_res, temp_TEC, temp_well; //sets the readout of the thermocouple

///////////////////////////////////////////////////////////saves previous temp value in case of error (NAN)////////////////////////////////////////////////////////////////
//Values hold current temp reading so that the system can determine whether to use this value or last recorded value to avoid any errors
float value = 0; 
float value2 = 0;
float value3 =0;
float value4 = 0;

///////////////////////////////////////////////////////////Initialize Pre-Heating Variables/////////////////////////////////////////////////////////////////////////////
int start_timer0 = 0; //

///////////////////////////////////////////////////////////program defined settings for pumps and electric field/////////////////////////////////////////////////////////
//Calculated variables for pumps
float time_conc = 0; //sets time wanted to conc sample in ms
float fill_pump = 0; //pumps it takes to fill a flowcell of defined size
float target_pump = 0; //pumps to empty target well
float reach_pump = 0; //pumps to reach flow cell
float reach_time = 0; //time it takes to reach flow cell
int target_steps = 0; //number of steps needed (step 1)
float steps_time = 0; //time per step
float fill_time = 0; //time needed to fill flow cell
float field_time = 0; //time needed to perform electric field functions
float conc_target = 0;  //time reserved for concentration
float detector_time = 0; //time reserved for step 3
float rinse_time = 0; //time reserved for steps 4 and 5
float rinse_vol = 0; //volume necessary for steps 4 and 5
float step2_vol = 0; //volume necessary for step 2
float detector_vol = 0; //volume necessary for step 3

//Defined variables for pumps
float flowcell_height = 150; //sets the flow cell height (um)
float flowcell_length = 5000; //sets the flow cell length (um)
float target_vol = 1000; //sets expected target volume (uL)
float totalHYB_time = 1800000; //sets expected hybridization time (ms)


///////////////////////////////////////////////////////////////////////////////////Timers///////////////////////////////////////////////////////////////////////////////
//Field Timers
float time_elapsed = 0;
float start_time = 0;

//Display Timers
float time_display = 0; //Timer for Settings Display
float start_display = 0; //allows the program to know the start time of settings display
int rotate_display = 0; //variable for rotating LCD Display on running screen

//Timing for temp readings
float temp_read = 0; //timer for displaying temp
float temp_read2 = 0; //timer for reading temp
float temp_interval = 3200; //sets how often the temp is displayed
float temp_interval2 = 200; //sets how often the temp is read8
float start_temp = 0; //sets the start time for displaying temp
float start_temp2 = 0; //sets the start time for reading temp

//Timing for PV readings
int PV_count = 0; //counter for how often the pressure and vacuum are displayed
int PV_count2 = 0; //counter for how often the pressure and vacuum are read
int PV_interval = 8; //sets how often the pressure and vacuum are displayed
int PV_interval2 = 1; //sets how often the pressure and vacuuma are read

//timer for total run time
float total = 0; //total runtime of the eCAP
float start_total = 0; //lets the program know when the eCAP has been powered on

//timer for each loop
float looptimer = 0; //keeps track of loop time
float start_loop = 0; //lets the program know the starting time of the loop

//PID timer (heat/cool)
float PID_elapsed = 0; //time tracker for PID loop
float PID_start = 0; //previous PID time
float PID_time= 0; //current PID time
int PID_count = 0; //counter for PID loops for relay control

//PID timer (pressure/vacuum)
int PID_countPV = 0; //counter for PID loops for PV control
float PID_startPV = 0; //previous PID time (PV)
float PID_timePV = 0; //current PID time (PV)
float PID_elapsedPV = 0; //time tracker for PID loop

//Step Timers
float time_step0 = 0; //time tracker for step 0
float start_step0 = 0; //lets the program know when step 0 has begun
float time_step1 = 0; //time tracker for step 1 (iterative)
float start_step = 0; //lets the program know when the step 1 has begun (iterative)
float overall_step1 = 0; //time tracker for step 1 (overall)
float start_overall1 = 0; //lets the program know when step 1 has begun (overall)
float start_step2 = 0;//lets the program know when step 2 has begun
float time_step2 = 0; //time tracker for step 2
float start_step3 = 0;//lets the program know when step 3 has begun
float time_step3 = 0; //time tracker for step 3
float start_step4 = 0;//lets the program know when step 4 has begun
float time_step4 = 0;//time tracker for step 4
float start_step5 = 0; //lets the program know when step 5 has begun
float time_step5 = 0;//time tracker for step 5

//Pump Timers
int pump_count =0; //keeps track of pumping timer when actually pumping fluid
int power_count = 0; //keeps track of how long the pressure pump should be on
int relief_count = 0; //keeps track of how long the pressure relief valve will be on
int power_count2 = 0; //keeps track of how long the vacuum pump should be on
int relief_count2 = 0; //keeps track of how long the vacuum relief valve will be on

/////////////////////////////////////////////////////////////////////////Variables for buttons/////////////////////////////////////////////////////////////////////////
//Left Button Defaults
int LB = OFF; //reading for Button State
int LB_prev = OFF; //Stores previous Button State
int LB_push = OFF; //tells the program that the button has been pressed

//Center Button Defaults
int CB = OFF;//reading for Button State
int CB_prev = OFF;//Stores previous Button State
int CB_push = OFF;//tells the program that the button has been pressed

//Right Button Defaults
int RB = OFF;//reading for Button State
int RB_prev = OFF;//Stores previous Button State
int RB_push = OFF;//tells the program that the button has been pressed

//Start Button Defaults
int StartB = OFF;//reading for Button State
int StartB_prev = OFF;//Stores previous Button State
int StartB_push = OFF;//tells the program that the button has been pressed

//reset Button Defaults
int ResetB = OFF;//reading for Button State
int resetB_prev = OFF;//Stores previous Button State
int resetB_push = OFF;//tells the program that the button has been pressed

/////////////////////////////////////////////////////////// //PID variables for the slide (heating)///////////////////////////////////////////////////////////////////////////
float error = 0; //difference between input and actual
float prev_error = 0; //stores previous error value
float PID = 0; //total PID value
float heat_on = 0; //gives the amount of time that the TEC will be on
int count_heat = 0; //gives the amount of loops the TEC will be on
float P = 0; //proportional adjustment based on temp on slide
float I = 0; //integral adjustment based on temp on slide
float D = 0; //derivative adjustment based on temp on slide
float kp = 61.9; //proportional term for PID (slide)
float ki = 0.8; //integral term for PID (slide)
float kd = 0.9; //derivative term for PID (slide)

///////////////////////////////////////////////////////////////PID variables for the well (heating)/////////////////////////////////////////////////////////////////////////////////
float errorW = 0; //difference between input and actual
float prev_errorW = 0;//stores previous error value
float PIDW = 0; //total PID value
float heat_onW = 0; //gives the amount of time the resistors will be on
int count_heatW = 0; //gives the amount of loops the resistors will be on
float PW = 0; //proportional adjustment based on temp in well
float IW = 0; //integral adjustment based on temp in well
float DW = 0; //derivative adjustment based on temp in well
float kpW =60; //proportional term for PID (well)
float kiW = 0.19; //integral term for PID (well)
float kdW = 0.9; //derivative term for PID (well)

////////////////////////////////////////////////////////////////PID variables for cooling////////////////////////////////////////////////////////////////////////////////
float errorC = 0; //difference between input and actual
float prev_errorC = 0; //stores previous error value
float PIDC = 0; //total PID value
float heat_onC = 0; //gives the amount of time the TEC will be on for cooling
int count_heatC = 0; //gives the amount of loops the TEC will be on for cooling
float PC = 0; //proportional adjustment based on cooling temp on slide
float IC = 0; //integral adjustment based on cooling temp on slide
float DC = 0; //derivative adjustment based on cooling temp on slide
float kpC =755; //proportional term for PID (cooling)
float kiC = 0.69; //proportional term for PID (cooling)
float kdC = 0.2; //proprtional term for PID (coolling)

////////////////////////////////////////////////////////////////PID variables for Pressure//////////////////////////////////////////////////////////////////////////////// 
float errorP = 0; //difference between input and actual
float prev_errorP = 0; //stores previous error value
float PIDP = 0; //total PID value
float pres_on = 0; //gives the amount of time the pressure pump will be on
int count_pres = 0; //gives the amount of loops the pressure pump will be on
float PP = 0; //proportional adjustment for pressure
float IP = 0; //integral adjustment for pressure
float DP = 0; //derivative adjustment for pressure
float kpP = 49; //proportional term for PID (pressure)
float kiP = 0.2; //integral term for PID (pressure)
float kdP = 15; //derivative term for PID (pressure)
float input_pressure = 6.0; //input PSI for pressure

////////////////////////////////////////////////////////////////PID variables for Vacuum////////////////////////////////////////////////////////////////////////////////
float errorV = 0; //difference between input and actual
float prev_errorV = 0; //stores previous error value
float PIDV = 0; //total PID value
float vac_on = 0; //gives the amount of time the vacuum pump will be on
int count_vac = 0; //gives the amount of loops the vacuum pump will be on
float PV = 0; //proportional adjustment for vacuum
float IV = 0; //integral adjustment for vacuum
float DV = 0; // derivative adjustment for vacuu,
float kpV = 49; //proportional term for PID (vacuum)
float kiV = 1; //integral term for PID (vacuum)
float kdV = 15; //derivative term for PID (vacuum)
float input_vacuum = 6.0; //input PSI for vacuum

////////////////////////////////////////////////////////////////Pneumatic Variables////////////////////////////////////////////////////////////////////////////////
//Initialize variables for PV reading
float PV_vac1,PV_vac2,PV_vac3,PV_vac4 = 0; //allows for avg of vacuum PSI readings
float PV_pres1,PV_pres2,PV_pres3,PV_pres4 = 0; //allows for avg of pressure PSI readings

//reading variables for PV values and conversion
float p_voltage = 0; //stores raw voltage from pressure sensor
float pressure = 0; //pressure value in PSI
float v_voltage = 0; //stores raw voltage from vacuum sensor
float vacuum = 0; //vacuum value in PSI
float p_conv = 0; //pressure conversion
float v_conv = 0; //vacuum conversion

//setting limits for PV and PV relief valves
float V_LO = 5.5; //lower limit for vacuum PSI
float V_HI = 7.5; //upper limit for vacuum PSI
float P_LO = 7.0; //lower limit for pressure PSI
float P_HI = 7.75; //upper limit for pressure PSI


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////SETUP/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

//set up serial communications
Serial.begin(9600); 

///////////////////////////////////////////////////////////////////////Pneumatic Control/////////////////////////////////////////////////////////////////////////////
//Set pin types for Pneumatics
pinMode(pressure_power, OUTPUT);
pinMode(vacuum_power, OUTPUT);
pinMode(pressure_relief, OUTPUT);
pinMode(vacuum_relief, OUTPUT);
pinMode(PV_valve1, OUTPUT);
pinMode(PV_pump, OUTPUT);
pinMode(PV_valve2, OUTPUT);
pinMode(PV_extra, OUTPUT);
pinMode(pressure_read, INPUT);
pinMode(vacuum_read, INPUT);

//set defaults for pneumatics
digitalWrite(pressure_power, HIGH); //defaults pressure to off
digitalWrite(vacuum_power, HIGH); //defaults vacuum to off
digitalWrite(pressure_relief, LOW); //defaults pressure relief valve to closed
digitalWrite(vacuum_relief, LOW); //defaults vacuum relief valve to open
digitalWrite(PV_valve1, HIGH); //defaults valve 1 on pump module to open
digitalWrite(PV_pump, HIGH); //defaults pump on pump module to open
digitalWrite(PV_valve2, HIGH); //defaults valve 2 on pump module to open
digitalWrite(PV_extra, HIGH); //defualts extra valve to open

///////////////////////////////////////////////////////////////ThermoCouple Interface Control/////////////////////////////////////////////////////////////////////////////
//Set pin types for thermocouple interface
pinMode(CLK, OUTPUT);
pinMode(DBIT, INPUT);
pinMode(CS, OUTPUT);
pinMode(T0, OUTPUT);
pinMode(T1, OUTPUT);
pinMode(T2, OUTPUT);

//Set defaults for thermocouple interface
digitalWrite(CS, HIGH);
digitalWrite(CLK, LOW);

//////////////////////////////////////////////////////////////////////////Voltage Communication Control///////////////////////////////////////////////////////////////////////////////
pinMode(C_ON,OUTPUT);
pinMode(ML_ON,OUTPUT);
pinMode(MR_ON,OUTPUT);
digitalWrite(C_ON,LOW);
digitalWrite(ML_ON,LOW);
digitalWrite(MR_ON,LOW);

//////////////////////////////////////////////////////////////////////////Button Control///////////////////////////////////////////////////////////////////////////////
//Set pin types for buttons
pinMode(B1, INPUT_PULLUP);//sets Left Button 
pinMode(B2, INPUT_PULLUP);//sets Center Button 
pinMode(B3, INPUT_PULLUP);//sets Right Button 
pinMode(B4, INPUT_PULLUP);//sets Start Button 
pinMode(B5, INPUT_PULLUP);//sets RESET Button 

//////////////////////////////////////////////////////////////////////////E field Control///////////////////////////////////////////////////////////////////////////////
//Set pin types and defaults for electrodes
digitalWrite(E1_V,HIGH); //defaults E1 relay 1 to nothing
pinMode(E1_V,OUTPUT); //sets control for E1 relay 1 between voltage and nothing
digitalWrite(E1_G,HIGH); //defaults E1 relay 2 to nothing
pinMode(E1_G,OUTPUT); //sets control for E1 relay 2 between ground and nothing
digitalWrite(E2_V,HIGH); //defaults E2 relay 1 to nothing
pinMode(E2_V,OUTPUT); //sets control for E2 relay 1 between voltage and nothing
digitalWrite(E2_G,HIGH); //defaults E2 relay 2 to nothing
pinMode(E2_G,OUTPUT); //sets control for E2 relay 2 between ground and nothing
digitalWrite(E3_V,HIGH);//defaults E3 relay 1 to nothing
pinMode(E3_V,OUTPUT); //sets control for E3 relay 1 between voltage and nothing
digitalWrite(E3_G,HIGH);//defaults E3 relay 2 to nothing
pinMode(E3_G,OUTPUT); //sets control for E3 relay 2 between ground and nothing
digitalWrite(E4,HIGH);//defaults E4 relay to ground
pinMode(E4,OUTPUT); //sets control for E4 between ground and voltage
digitalWrite(E5,LOW);//defaults E5 relay to ground
pinMode(E5,OUTPUT); //sets control for E5 between ground and voltage

////////////////////////////////////////////////////////////////////////////Heat Control/////////////////////////////////////////////////////////////////////////////////////
//Set pin types and defaults for relays associated with heat loops
digitalWrite(heat_relay,HIGH); //Initialize TEC Heat OFF 
pinMode(heat_relay,OUTPUT); // To control TEC Heat between voltage and ground
digitalWrite(cool_relay,HIGH); // Initialize TEC Cool OFF
pinMode(cool_relay,OUTPUT); // To control TEC Cool between voltage and ground
digitalWrite(fan_relay,LOW); //Initialize fan OFF
pinMode(fan_relay,OUTPUT); // To control fan
digitalWrite(res_relay,HIGH); //Initialize resistors OFF
pinMode(res_relay,OUTPUT); // To control resistors

//initialize LCD
lcd.init(); //set up LCD
lcd.backlight(); //turn on LCD backlight
lcd.clear(); //clear LCD screen
lcd.setCursor(0,0); //set cursor to beginning of line 1
lcd.print("Welcome -  eCAP "); //print statement
lcd.setCursor(0,1); //set cursor to beginning of line 2
lcd.print("Setting Up..."); //print statement
lcd.print(version_num,1);

state = 0; //set default state to 0

//initialize thermocouple interface to read 1st value
digitalWrite(T0,HIGH); 
digitalWrite(T1,LOW);
digitalWrite(T2,HIGH);


//Print statements to Serial monitor for organization of values
Serial.print("Time (s)");
Serial.print("\t");
Serial.print("Temp");
Serial.print("\t");
Serial.print("Temp Well");
Serial.print("\t");
Serial.print("Setpoint");
Serial.print("\t");
Serial.print("Pressure");
Serial.print("\t");
Serial.print("Vacuum");
Serial.print("\n");

//allow setup to finish
delay(5000); 

//initialize timers
start_temp = millis();
start_temp2 = millis();
start_loop = millis();
start_total = millis();
}

//reset function for reset button
void(*reset)(void) = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////MAIN CODE/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

//initialize fan ON at all times
digitalWrite(fan_relay, LOW);

//keep track of total time and loop time
total = millis() - start_total; //time for total 
looptimer = millis() - start_loop; //time for loop

/////////////////////////////////////////////////////////////////////////////////Read PV//////////////////////////////////////////////////////////////////////////////////////
//take 4 readings of PSI on vacuum for averaging
PV_vac4 = PV_vac3; //stores each as the previous read vacuum value
PV_vac3 = PV_vac2; //stores each as the previous read vacuum value
PV_vac2 = PV_vac1; //stores each as the previous read vacuum value
PV_vac1 = analogRead(vacuum_read);//read vacuum value from the arduino

//take 4 readings of PSI on pressure for averaging
PV_pres4 = PV_pres3; //stores each as the previous read pressure value
PV_pres3 = PV_pres2; //stores each as the previous read vacuum value
PV_pres2 = PV_pres1; //stores each as the previous read vacuum value
PV_pres1 = analogRead(pressure_read);//read pressure value from the arduino

//Use averaged pressure and vacuum readings
v_voltage = (PV_vac1 + PV_vac2 + PV_vac3 + PV_vac4)/4; //averages the most recent 4 vacuum values each loop
p_voltage = (PV_pres1 + PV_pres2 + PV_pres3 + PV_pres4)/4; //averages the most recent 4 pressure values each loop
p_conv = (p_voltage/1023)*5; //converts pressure to accurate voltage value
v_conv = (v_voltage/1023)*5; //converts vacuum to accurate voltage value
pressure = (11.125 * p_conv) - 19.76; //uses defined trend line to convert pressure voltage to PSI
vacuum = (11.125 * v_conv) - 19.56; //uses defined trend line to convert vacuum voltage to PSI
vacuum = abs(vacuum); //takes the absolute value of the vacuum PSI for simplified comparisons

/////////////////////////////////////////////////////////////////////////////////Read Temp//////////////////////////////////////////////////////////////////////////////////////
//Calculates the times for temperature readings and temperature displays
temp_read = millis() - start_temp; //time for temp display
temp_read2 = millis() - start_temp2; //time for temp read

//Each loop takes a temp reading at a different TC, then averages the last 4 before displaying
if (temp_read2>=temp_interval2){
  //Program alternates between temp_switch 1-4 to change to different Thermocouples on the Interface Circuit
  if (temp_switch == 1){ //default state reads temp at slide
    //take 4 readings of temp on slide before averaging
    temp4 = temp3; //stores each as the previous read temp value
    temp3 = temp2; //stores each as the previous read temp value
    temp2 = temp1; //stores each as the previous read temp value
    value3 = thermocouple.readCelsius(); //reads value of thermocouple on slide
    
    //if value is NAN then ignore that value and use the previous temp reading
    if(isnan(value3)) {
      temp1 = temp1; //stores previous value
    }
    else{
      temp1 = value3;//uses actual value
    }
    
    //change thermocouple reading to resistors for next iteration
    digitalWrite(T0,LOW);
    digitalWrite(T1,HIGH);
    digitalWrite(T2,HIGH);
    temp_switch = 2;
  }
  else if (temp_switch == 2){ //reads temp at resistors
    //take 4 readings of temp on resistors before averaging
    temp_res4 = temp_res3; //stores each as the previous read temp value
    temp_res3 = temp_res2; //stores each as the previous read temp value
    temp_res2 = temp_res1; //stores each as the previous read temp value
    value = thermocouple.readCelsius(); //reads value of thermocouple on resistors
    
    //if value is NAN then ignore that value and use the previous temp reading
    if(isnan(value)) {
      temp_res1 = temp_res1; //stores previous value
    }
    else{
      temp_res1 = value; //uses actual value
    }

    //change thermocouple reading to TEC for next iteration
    digitalWrite(T0,LOW);
    digitalWrite(T1,LOW);
    digitalWrite(T2,HIGH);
    temp_switch = 3;
  }
  else if (temp_switch == 3){ //reads temp at TEC
    //take 4 readings of temp at TEC before averaging
    temp_TEC4 = temp_TEC3; //stores each as the previous read temp value
    temp_TEC3 = temp_TEC2; //stores each as the previous read temp value
    temp_TEC2 = temp_TEC1; //stores each as the previous read temp value
    value2 = thermocouple.readCelsius(); //reads value of thermocouple on TEC
    
    //if value is NAN then ignore that value and use the previous temp reading
    if(isnan(value2)) {
      temp_TEC1 = temp_TEC1; //stores previous value
    }
    else{
      temp_TEC1 = value2; //uses actual value
    }

    //change thermocouple reading to entrance well for next iteration
    digitalWrite(T0,HIGH);
    digitalWrite(T1,HIGH);
    digitalWrite(T2,HIGH);
    temp_switch = 4;
  }
  else if (temp_switch == 4){ //reads temp at entrance well
    //take 4 readings of temp on entrance well before averaging
    temp_well4 = temp_well3; //stores each as the previous read temp value
    temp_well3 = temp_well2; //stores each as the previous read temp value
    temp_well2 = temp_well1; //stores each as the previous read temp value
    value4 = thermocouple.readCelsius(); //reads value of thermocouple on entrance well
    
    //if value is NAN then ignore that value and use the previous temp reading
    if(isnan(value4)) {
      temp_well1 = temp_well1; //stores previous value
    }
    else{
      temp_well1 = value4; //uses actual value
    }

    //change thermocouple reading to slide for next iteration
    digitalWrite(T0,HIGH);
    digitalWrite(T1,LOW);
    digitalWrite(T2,HIGH);
    temp_switch = 1;
  }
  //restart timer for temp readings
  start_temp2 = millis();
}

//Keep track of temp and data on serial monitor, only read every temp_interval as defined above
if (temp_read >= temp_interval){
  //average temp values
  temp = (temp1 + temp2 + temp3 + temp4)/4; //averages the most recent 4 slide values each loop
  temp_res = (temp_res1 + temp_res2 + temp_res3 + temp_res4)/4; //averages the most recent 4 resistors values each loop
  temp_TEC = (temp_TEC1 + temp_TEC2 + temp_TEC3 + temp_TEC4)/4; //averages the most recent 4 TEC values each loop
  temp_well = (temp_well1 + temp_well2 + temp_well3 + temp_well4)/4; //averages the most recent 4 well values each loop

  /*
  //print out values to Serial
  Serial.print(total/1000,0);
  Serial.print("\t");
  Serial.print(temp,1);
  Serial.print("\t");
  Serial.print(temp_well,1);
  Serial.print("\t");
  Serial.print(input_temp,1);
  Serial.print("\t");
  Serial.print(pressure,1);
  Serial.print("\t");
  Serial.print(vacuum,1);
  Serial.print("\n");
*/
  //print times to LCD when running a step
  if (state == 7){
    lcd.setCursor(11,0);
    lcd.print(time_step0/1000,0);
  }
  if (state == 8){
    lcd.setCursor(11,0);
    lcd.print(overall_step1/1000,0);
  }
  if (state == 9){
    lcd.setCursor(11,0);
    lcd.print(time_step2/1000,0);
  }
  if (state == 10){
    lcd.setCursor(11,0);
    lcd.print(time_step3/1000,0);
  }
  if (state == 11){
    lcd.setCursor(11,0);
    lcd.print(time_step4/1000,0);
  }
  if (state == 12){
    lcd.setCursor(11,0);
    lcd.print(time_step5/1000,0);
  }
  
  //restart timer for temp display/averaging
  start_temp = millis();
}

///////////////////////////////////////////////////////////////////////////Flow Cell Calculations////////////////////////////////////////////////////////////////////////
float target_pump = target_vol/vp; //pumps to empty target well
float fill_pump = flowcell_vol/vp; //pumps to fill flow cell
float reach_pump = reach_vol/vp; //pumps to reach flow cell
float reach_time = reach_pump * tp; //time it takes to reach flow cell
target_steps = (target_pump - reach_pump)/fill_pump; //number of steps needed (step 1)
steps_time = totalHYB_time/target_steps; //time per step
fill_time = fill_pump*tp; //time needed to fill flow cell
field_time = steps_time - fill_time; //time needed to perform electric field functions
conc_target = 3000;  //time reserved for concentration
detector_time = reach_time + 200000; //time required for step 3
rinse_time = ((8*fill_pump) + reach_pump)*tp; //time required for steps 4 and 5
detector_vol = (detector_time/tp)*vp; //volume needed for step 3
rinse_vol = ((8*fill_pump) + reach_pump)*vp + 200; //volume needed for steps 4 and 5
step2_vol = ((15000/tp)*vp) + ((270000/30000)*vp) + 200; //volume needed for step 2

    Serial.print("Step 2 Vol:");
    Serial.println(step2_vol,0);
    Serial.print("Step 3 Vol:");
    Serial.println(detector_vol,0);
    Serial.print("Step 4 Vol:");
    Serial.println(rinse_vol,0);
    Serial.print("Step 5 Vol:");
    Serial.println(rinse_vol,0);
    Serial.print("Step4 and 5 time: ");
    Serial.println(rinse_time,0);
    
///////////////////////////////////////////////////////////////////////////Read buttons//////////////////////////////////////////////////////////////////////////////////
//read all five buttons
LB = digitalRead(B1); //reads switch state of the left button
CB = digitalRead(B2); //reads switch state of the center button
RB = digitalRead(B3); //reads switch state of the right button
StartB = digitalRead(B4); //reads switch state of the start button
ResetB = digitalRead(B5); //reads switch state of the reset button

//Make reset button functional
if(resetB_push == ON){ //if reset button is pushed perform all the following actions to reset system and variables
  resetB_push = OFF; //reset the switch to default state
  
  //turn everything off and reset active states/settings to defaults
  digitalWrite(E1_V,HIGH);
  digitalWrite(E1_G,HIGH);
  digitalWrite(E2_V,HIGH);
  digitalWrite(E2_G,HIGH);
  digitalWrite(E3_V,HIGH);
  digitalWrite(E3_G,HIGH);
  digitalWrite(E4,HIGH);
  digitalWrite(E5,LOW);
  digitalWrite(fan_relay,LOW);
  digitalWrite(res_relay,HIGH);
  digitalWrite(heat_relay,HIGH);
  digitalWrite(cool_relay,HIGH);
  digitalWrite(pressure_power, HIGH);
  digitalWrite(vacuum_power, HIGH);
  digitalWrite(pressure_relief, HIGH);
  digitalWrite(vacuum_relief, LOW);
  digitalWrite(PV_valve1, HIGH);
  digitalWrite(PV_pump, HIGH);
  digitalWrite(PV_valve2, HIGH);
  digitalWrite(PV_extra, HIGH);
  PID_count = 0;
  count_heat = 0;
  count_heatW = 0;
  count_heatC = 0;
  active_heat = 0;
  PID_countPV = 0;
  count_pres = 0;
  count_vac = 0;

  //go into the reset loop to reset the arduino
  reset();
}

  //set buttons up to reset after each step so that user does not accidentally run through many states at once
  if ((LB==ON) and (LB_prev == OFF)) { //Notifies system when button was pressed and was previously unpressed
    LB_prev = ON; //sets previous button state to ON
    LB_push = ON; //Saves current button state as pressed
  }
  if (LB == OFF){ //Button is OFF or not being pressed
    LB_prev = OFF; //sets previous button state to OFF
  }
  if ((CB==ON) and (CB_prev == OFF)) {
    CB_prev = ON; //sets previous button state to ON
    CB_push = ON; //Saves current button state as pressed
  }
  if (CB == OFF){ //Button is OFF or not being pressed
    CB_prev = OFF; //sets previous button state to OFF
  }
  if ((RB==ON) and (RB_prev == OFF)) {
    RB_prev = ON; //sets previous button state to ON
    RB_push = ON; //Saves current button state as pressed
  }
  if (RB == OFF){ //Button is OFF or not being pressed
    RB_prev = OFF; //sets previous button state to OFF
  }
  if ((StartB==ON) and (StartB_prev == OFF)) {
    StartB_prev = ON; //sets previous button state to ON
    StartB_push = ON; //Saves current button state as pressed
  }
  if (StartB == OFF){ //Button is OFF or not being pressed
    StartB_prev = OFF; //sets previous button state to OFF
  }
  if ((ResetB==ON) and (resetB_prev == OFF)) {
    resetB_prev = ON; //sets previous button state to ON
    resetB_push = ON; //Saves current button state as pressed
  }
  if (ResetB == OFF){ //Button is OFF or not being pressed
    resetB_prev = OFF; //sets previous button state to OFF
  }


///////////////////////////////////////////////////////////////////////////STATE 0////////////////////////////////////////////////////////////////////////////////////
if (state == 0){ //power up
  //turn everything off and reset active states/settings
  digitalWrite(E1_V,HIGH);
  digitalWrite(E1_G,HIGH);
  digitalWrite(E2_V,HIGH);
  digitalWrite(E2_G,HIGH);
  digitalWrite(E3_V,HIGH);
  digitalWrite(E3_G,HIGH);
  digitalWrite(E4,HIGH);
  digitalWrite(E5,LOW);
  digitalWrite(fan_relay,LOW);
  digitalWrite(res_relay,HIGH);
  digitalWrite(heat_relay,HIGH);
  digitalWrite(cool_relay,HIGH);
  digitalWrite(pressure_power, HIGH);
  digitalWrite(vacuum_power, HIGH);
  digitalWrite(pressure_relief, HIGH);
  digitalWrite(vacuum_relief, LOW);
  digitalWrite(PV_valve1, HIGH);
  digitalWrite(PV_pump, HIGH);
  digitalWrite(PV_valve2, HIGH);
  digitalWrite(PV_extra, HIGH);
  PID_count = 0;
  count_heat = 0;
  count_heatW = 0;
  count_heatC = 0;
  active_heat = 0;
  PID_countPV = 0;
  count_pres = 0;
  count_vac = 0;
  
  //state 0 display
  lcd.setCursor(0,0);
  lcd.print("Welcome -  eCAP ");
  lcd.setCursor(0,1);
  lcd.print("Press Any Button");

  //Left Button Actions for State 0
  if(LB_push == ON){
    LB_push = OFF; //reset button press
    lcd.clear(); //clear LCD
    state = 1; //go to state 1
  }

  //Center Button Actions for State 0
  if(CB_push == ON){
    CB_push = OFF; //reset button press
    lcd.clear(); //clear LCD
    state = 1; //go to state 1
  }

  //Right Button Actions for State 0
  if(RB_push == ON){
    RB_push = OFF; //reset button press
    lcd.clear(); //clear LCD
    state = 1; //go to state 1
  }

  //Start Button Actions for State 0 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push = OFF; //reset button press
  }
}

///////////////////////////////////////////////////////////////////////////STATE 1////////////////////////////////////////////////////////////////////////////////////
else if (state == 1){ //Setup Options
    //turn everything off and reset active states/settings
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(fan_relay,LOW);
    digitalWrite(res_relay,HIGH);
    digitalWrite(heat_relay,HIGH);
    digitalWrite(cool_relay,HIGH);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, HIGH);
    digitalWrite(vacuum_relief, LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    PID_count = 0;
    count_heat = 0;
    count_heatW = 0;
    count_heatC = 0;
    PID_countPV = 0;
    count_pres = 0;
    count_vac = 0;
    active_heat = 0;
  
  //state 1 display
  lcd.setCursor(0,0);
  lcd.print("Press Start   OR");
  lcd.setCursor(0,1);
  lcd.print("Select Step HERE");

  //Left Button Actions for State 1 (DO NOTHING)
  if (LB_push == ON){
    LB_push = OFF; //reset button press
  }

  //Center Button Actions for State 1 (DO NOTHING)
  if (CB_push == ON){
    CB_push = OFF; //reset button press
  }

  //Right Button Actions for State 1 (SELECT STEP)
  if (RB_push ==ON){
    RB_push = OFF; //reset button press
    lcd.clear(); //clear LCD
    state = 2; //go to state 2
  }

  //start button functionality (will only work when in state 1)
  if (StartB_push == ON){
    StartB_push = OFF; //reset button press
    start_display = millis(); //to help scroll through settings on state 3
    lcd.clear(); //clear LCD
    state = 3; //go to state 3
  }
}

///////////////////////////////////////////////////////////////////////////STATE 2////////////////////////////////////////////////////////////////////////////////////
else if (state == 2){ //Choose Step
  //state 2 display
  lcd.setCursor(0,0);
  lcd.print("Set Step:  ");

  //Display Steps on LCD
  lcd.setCursor(11,0);
  if (steps == -1){
    //Step number -1 corresponds to the system being OFF
    lcd.print("OFF");
  }
  else{
    //Print the step chosen
    lcd.print(steps);
    lcd.setCursor(12,0);
    lcd.print("    ");
  }
  lcd.setCursor(0,1);
  lcd.print("SAVE   UP   DOWN");

  //Left Button Actions for State 2 (SAVE)
  if (LB_push == ON){ //SAVE and calculate appropriate pump values
    LB_push = OFF; //reset button press
    lcd.clear(); //clear LCD
    state = 1; //go to state 1
  }

  //Center Button Actions for State 2 (UP)
  if (CB_push == ON){
    CB_push = OFF; //reset button press
    steps = steps + 1; //increase step number
    if (steps >5){ //do not allow step numbers past 5
      steps = 5;
    }
  }

  //Right Button Actions for State 2 (DOWN)
  if (RB_push == ON){
    RB_push = OFF; //reset button press
    steps = steps - 1; //decrease step number
    if (steps<0){ //any number below 0 will be given a value to -1 or OFF
      steps = -1;
    }
  }

  //Start Button Actions for State 2 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push = OFF; //reset button press
  }
}

///////////////////////////////////////////////////////////////////////////STATE 3////////////////////////////////////////////////////////////////////////////////////
else if (state == 3){ //Confirm Settings
  //state 3 display
  time_display = millis() - start_display;
  //show all settings on display in rotation
  if (time_display <=3000){ //for three seconds display the following
      lcd.setCursor(0,0);
      lcd.print("Confirm Settings:"); 
    }
  if (time_display >3000){ //for four seconds display the following
    lcd.setCursor(0,0);
    lcd.print("Step =           ");
    lcd.setCursor(8,0);
    lcd.print(steps);
  }
  if (time_display > 7000){ //after 7 seconds total cycle back to original display and repeat
    start_display = millis();       
  }
  lcd.setCursor(0,1);
  lcd.print("Return   Confirm");

  //Left Button Actions for State 3 (RETURN)
  if (LB_push == ON){
    LB_push = OFF; //reset button press
    lcd.clear(); //clear LCD
    state = 1; //go to state 1
  }

  //Center Button Actions for State 3 (DO NOTHING)
  if (CB_push == ON){
    CB_push = OFF; //reset button press
  }

  //Right Button Actions for State 3 (CONFIRM)
  if (RB_push ==ON){
    RB_push = OFF; //reset button press
    lcd.clear(); //clear LCD
    state = 4; //go to state 4
  } 

  //Start Button Actions for State 3 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push == OFF; //reset button press
  }
  
  prev_state = 3; //save previous state
}
///////////////////////////////////////////////////////////////////////////STATE 4////////////////////////////////////////////////////////////////////////////////////
else if (state == 4){
  //state 4 display
  lcd.setCursor(0,0);
  lcd.print("Load");
  //print specified volume for the appropriate step chosen
  if (steps == 0){
    lcd.setCursor(5,0);
    lcd.print("1000");
  }
  if (steps == 1){
    lcd.setCursor(5,0);
    lcd.print("1100");
  }
  if (steps == 2){
    lcd.setCursor(5,0);
    lcd.print(step2_vol,0);
  }
  if (steps == 3){
    lcd.setCursor(5,0);
    lcd.print(detector_vol,0);
  }
  if ((steps == 4) or (steps == 5)){
    lcd.setCursor(5,0);
    lcd.print(rinse_vol,0);
  }
  
  lcd.setCursor(9,0);
  lcd.print("uL");
  
  lcd.setCursor(13,0);
  lcd.print("(");
  lcd.setCursor(14,0);
  lcd.print(steps);
  lcd.setCursor(15,0);
  lcd.print(")");
  
  lcd.setCursor(0,1);
  lcd.print("Return  Finished");

  //Left Button Actions for State 4 (RETURN)
  if (LB_push == ON){
    LB_push = OFF; //reset button press
    lcd.clear(); //clear LCD
    //go to saved previous state:
    if (prev_state == 3){
      state = 3;
    }
    if (prev_state == 14){
      state = 14;
    }
    if(prev_state == 15){
      state = 15;
    }
  }

  //Center Button Actions for State 4 (DO NOTHING)
  if (CB_push == ON){
    CB_push = OFF; //reset button press
  }

  //Right Button Actions for State 4 (FINISHED)
  if (RB_push ==ON){
    RB_push = OFF; //reset btton press
    lcd.clear(); //clear LCD
    state = 5; //go to state 5
  }

  //Start Button Actions for State 4 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push == OFF; //reset button press
  }
}
///////////////////////////////////////////////////////////////////////////STATE 5////////////////////////////////////////////////////////////////////////////////////
else if (state == 5){//Start Program?
  //state 5 display
  lcd.setCursor(0,0);
  lcd.print("Start Step");
  lcd.setCursor(11,0);
  lcd.print(steps);
  lcd.setCursor(13,0);
  lcd.print("?");
  lcd.setCursor(0,1);
  lcd.print("NO           YES");

  //Left Button Actions for State 5 (NO)
  if (LB_push == ON){
    LB_push = OFF; //reset button press
    lcd.clear(); //clear LCD
    state = 1; //go to state 1
    //Reset Values Completely:
    PID_count = 0;
    count_heat = 0;
    count_heatW = 0;
    count_heatC = 0;
    PID_countPV = 0;
    count_pres = 0;
    count_vac = 0;
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, HIGH);
    digitalWrite(vacuum_relief, LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(res_relay, HIGH);
    digitalWrite(heat_relay, HIGH);
    digitalWrite(cool_relay, HIGH);
  }
  
  //Center Button Actions for State 5 (DO NOTHING)
  if (CB_push == ON){
    CB_push = OFF; //reset button press
  }

  //Right Button Actions for State 5 (YES) ***STARTS RUNNING SELECTED STEP***
  if (RB_push ==ON){
    RB_push = OFF;//reset button press
    lcd.clear(); //clear LCD
    //start all timers necessary for step control
    start_step0 = millis();
    start_step = millis();
    start_overall1 = millis();
    start_step2 = millis();
    start_step3 = millis();
    start_step4 = millis();
    start_step5 = millis();
    start_timer0 = 0;
    state = steps + 7; //go to appropriate state based on selected step
    /*
     * Step 0 ---> State 7
     * Step 1 ---> State 8
     * Step 2 ---> State 9
     * Step 3 ---> State 10
     * Step 4 ---> State 11
     * Step 5 ---> State 12
     */
  }
  
  //Start Button Actions for State 5 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push == OFF; //reset button press
  }
}

///////////////////////////////////////////////////////////////////////////STATE 14////////////////////////////////////////////////////////////////////////////////////
else if (state == 14){ //Change settings when user selected steps
  //state 14 display
  lcd.setCursor(0,0);
  lcd.print("Change Step:");
  //Display Step
  lcd.setCursor(13,0);
  if (steps == -1){
    lcd.print("OFF"); //print OFF for a step defined as -1
  }
  else{
    lcd.print(steps); //print step number
  }
  lcd.setCursor(0,1);
  lcd.print("SAVE   UP   DOWN");

  //Left Button Actions for State 14 (SAVE)
  if (LB_push == ON){
    LB_push = OFF; //reset button press
    if (steps == -1){ //reset values if user chooses OFF
      //Reset Values Completely
      PID_count = 0;
      count_heat = 0;
      count_heatW = 0;
      count_heatC = 0;
      PID_countPV = 0;
      count_pres = 0;
      count_vac = 0;
      digitalWrite(E1_V,HIGH);
      digitalWrite(E1_G,HIGH);
      digitalWrite(E2_V,HIGH);
      digitalWrite(E2_G,HIGH);
      digitalWrite(E3_V,HIGH);
      digitalWrite(E3_G,HIGH);
      digitalWrite(E4,HIGH);
      digitalWrite(E5,LOW);
      digitalWrite(pressure_power, HIGH);
      digitalWrite(vacuum_power, HIGH);
      digitalWrite(pressure_relief, HIGH);
      digitalWrite(vacuum_relief, LOW);
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, HIGH);
      digitalWrite(PV_valve2, HIGH);
      digitalWrite(PV_extra, HIGH);
      digitalWrite(res_relay, HIGH);
      digitalWrite(heat_relay, HIGH);
      digitalWrite(cool_relay, HIGH);
      lcd.clear(); //clear LCD
      state = 1; //go to state 1
    }
    else {
      lcd.clear(); //clear LCD
      state = 4; //go to state 4
    }
    //Start step timers again
    start_step = millis();
    start_overall1 = millis();
    start_step2 = millis();
    start_step3 = millis();
    start_step4 = millis();
    start_step5 = millis();
  }

  //Center Button Actions for State 14 (UP)
  if (CB_push == ON){
    CB_push = OFF; //reset button press
    steps = steps + 1; //increase step number
    if (steps >5){ //don't allow values over 5
      steps = 5;
    }
  }

  //Right Button Actions for State 14 (DOWN)
  if (RB_push ==ON){ 
    RB_push = OFF; //reset button press
    steps = steps - 1; //decrease step number
    if (steps<0){ //Any number below 0 will be given a value of -1 or OFF
      steps = -1;
    }
  }

  //Start Button Actions for State 14 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push = OFF; //reset button press
  }
  
  prev_state = 14;  //save previous state
}
///////////////////////////////////////////////////////////////////////////STATE 15////////////////////////////////////////////////////////////////////////////////////
else if (state == 15){ //Change settings after a step has been completed
  //reset values
  digitalWrite(E1_V,HIGH);
  digitalWrite(E1_G,HIGH);
  digitalWrite(E2_V,HIGH);
  digitalWrite(E2_G,HIGH);
  digitalWrite(E3_V,HIGH);
  digitalWrite(E3_G,HIGH);
  digitalWrite(E4,HIGH);
  digitalWrite(E5,LOW);
  digitalWrite(pressure_power, HIGH);
  digitalWrite(vacuum_power, HIGH);
  digitalWrite(pressure_relief, HIGH);
  digitalWrite(vacuum_relief, LOW);;
  
  //state 15 display
  lcd.setCursor(0,0);
  lcd.print("Next Step:");
  lcd.setCursor(13,0);
  if (steps == -1){
    lcd.print("OFF");
  }
  else{
    lcd.print(steps);
  }
  lcd.setCursor(0,1);
  lcd.print("SAVE   UP   DOWN");

  //Left Button Actions for State 15 (SAVE)
  if (LB_push == ON){
    LB_push = OFF; //reset button press
    if (steps == -1){ //reset values if user chooses OFF
      //Reset Values Completely
      PID_count = 0;
      count_heat = 0;
      count_heatW = 0;
      count_heatC = 0;
      PID_countPV = 0;
      count_pres = 0;
      count_vac = 0;
      digitalWrite(E1_V,HIGH);
      digitalWrite(E1_G,HIGH);
      digitalWrite(E2_V,HIGH);
      digitalWrite(E2_G,HIGH);
      digitalWrite(E3_V,HIGH);
      digitalWrite(E3_G,HIGH);
      digitalWrite(E4,HIGH);
      digitalWrite(E5,LOW);
      digitalWrite(pressure_power, HIGH);
      digitalWrite(vacuum_power, HIGH);
      digitalWrite(pressure_relief, HIGH);
      digitalWrite(vacuum_relief, LOW);
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, HIGH);
      digitalWrite(PV_valve2, HIGH);
      digitalWrite(PV_extra, HIGH);
      digitalWrite(res_relay, HIGH);
      digitalWrite(heat_relay, HIGH);
      digitalWrite(cool_relay, HIGH);
      lcd.clear(); //clear LCD
      state = 1; //go to state 1
    }
    else {
      lcd.clear(); //clear LCD
      state = 4; //go to state 4
    }
    //Reset Step Timers
    start_step = millis();
    start_overall1 = millis();
    start_step2 = millis();
    start_step3 = millis();
    start_step4 = millis();
    start_step5 = millis();
  }

  //Center Button Actions for State 15 (UP)
  if (CB_push == ON){
    CB_push = OFF; //reset button press
    steps = steps + 1; //increase step number
    if (steps >5){ //do not let user input steps over 5
      steps = 5;
    }
  }

  //Right Button Actions for State 15 (DOWN)
  if (RB_push ==ON){
    RB_push = OFF; //reset button press
    steps = steps - 1; //decrease step number
    if (steps<0){ //give any step under 0 a value of -1 or OFF
      steps = -1;
    }
  }

  //Start Button Actions for State 15 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push = OFF; //reset button press
  }
  prev_state = 15; //save previous state
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////STEPS 0-5///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////Step 0////////////////////////////////////////////////////////////////////////////////////////
if (state == 7){
  //state 7 display
  lcd.setCursor(0,0);
  lcd.print("Step 0");
  lcd.setCursor(7,0);
  lcd.print(temp,0); //actively displays current temp
  lcd.setCursor(9,0);
  lcd.print("C");
  lcd.setCursor(15,0);
  lcd.print("s");
  lcd.setCursor(0,1);
  lcd.print("Change      STOP");

  //Left Button Actions for Step 0 (CHANGE STEP)
  if (LB_push == ON){
    LB_push = OFF; //reset button press
    
    //reset pump and EFIELD values and clear LCD
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, HIGH);
    digitalWrite(vacuum_relief, LOW);
    lcd.clear(); //clears display
    state = 14; //go to state 14
  }

  //Center Button Actions for Step 0 (DO NOTHING)
  if (CB_push == ON){
    CB_push = OFF; //reset button press
    //DO NOTHING
  }

  //Right Button Actions for Step 0 (STOP PROGRAM)
  if (RB_push ==ON){
    RB_push = OFF; //reset button press
    lcd.clear(); //clear display
    
    //Reset Values Completely
    active_heat = 0;
    PID_count = 0;
    count_heat = 0;
    count_heatW = 0;
    count_heatC = 0;
    PID_countPV = 0;
    count_pres = 0;
    count_vac = 0;
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, HIGH);
    digitalWrite(vacuum_relief, LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(res_relay, HIGH);
    digitalWrite(heat_relay, HIGH);
    digitalWrite(cool_relay, HIGH);
    state = 1; //go to state 1
  }

  //Start Button Actions for Step 0 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push = OFF; //reset button press
  }
  
  //SET DEFINED TEMP AND TURN ON HEATER TO PRE-HEAT
  input_temp = 54; //sets the temperature to 54
  active_heat = 1; //turns on the heater


  /////////////////////////////////////////////////////////////////////Pump Cycle after Pre-Heating
  if ((temp >= (input_temp - 1)) and (time_step0 < ((600*tp)/vp))){ //once target reaches temp begin pumping until fluid reaches waste
    if (start_timer0 == 0){ //starts the timer for pumping
      start_timer0 = 1; //lets the program know that the system has been pre-heated
      start_step0 = millis(); //reset timer for step 0
    }
    time_step0 = millis() - start_step0; //timer for step 0
    
    //Pump Cycle
    if(pump_count < 1){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, HIGH);
      digitalWrite(PV_valve2, LOW);
    }
    if ((pump_count >= 1) and (pump_count < 9)){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, LOW);
      digitalWrite(PV_valve2, LOW);
    }
    if ((pump_count >= 9) and (pump_count < 10)){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, LOW);
      digitalWrite(PV_valve2, HIGH);
    }
    if ((pump_count >= 10) and (pump_count < 11)){
      digitalWrite(PV_valve1, LOW);
      digitalWrite(PV_pump, LOW);
      digitalWrite(PV_valve2, HIGH);
    }
    if ((pump_count >= 11) and (pump_count < 19)){
      digitalWrite(PV_valve1, LOW);
      digitalWrite(PV_pump, HIGH);
      digitalWrite(PV_valve2, HIGH);
    }
    if ((pump_count >= 19) and (pump_count < 20)){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, HIGH);
      digitalWrite(PV_valve2, HIGH);
    }
    pump_count = pump_count + 1; //go through pump cycle by increasing count
    if (pump_count >= 20){ //once reaching the end of pump cycle, restart cycle
      pump_count = 0;
    }

    //Heat Cycle Variables
    input_temp  = 54; //sets the temperature to 54
    active_heat = 1; //turns on heater
  }

  ////////////////////////////////////////////////////////////Finish Step 0
  if (time_step0 >= ((600*tp)/vp)){ //once fluid has reached the waste well, stop pumping and notify user
    active_heat = 1; //continue heating
    start_step0 = millis(); //reset timer
    start_timer0 = 0; //reset timer start to 0
    
    //Reset Values (not heat)
    PID_countPV = 0;
    count_pres = 0;
    count_vac = 0;
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, HIGH);
    digitalWrite(vacuum_relief, LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    Serial.println("Step 0 Vol: 1000");
    Serial.println("Step 1 Vol: 1100");
    Serial.println("Step 2 Vol:");
    Serial.print(step2_vol,0);
    Serial.println("Step 3 Vol:");
    Serial.print(detector_vol,0);
    Serial.println("Step 4 Vol:");
    Serial.print(rinse_vol,0);
    Serial.println("Step 5 Vol:");
    Serial.print(rinse_vol,0);
    lcd.clear(); //clear display
    state = 15; //go to state 15
  }
 
  //////////////////////////////////////////////////////////////PV regulation for Step 0
  //Pressure Pump Regulation
  if (pressure <= P_LO){ //when pressure is below the limit perform the following...
    if (power_count == 0){ //at the beginning of the pressure regulation if the pump needs to be turned on also turn on the valve because the chamber fills quickly
      digitalWrite(pressure_power, LOW); //turn pressure pump on
      digitalWrite(pressure_relief, HIGH); //open pressure relief valve
      delay(3); //delay 3 ms
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if ((power_count > 0) and (power_count < 24)){ //leave the pressure on for up to 3 s
      digitalWrite(pressure_power, LOW);  //turn the pressure pump on
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if (power_count >= 24){ //after 3s turn the pressure pump off for at least 2s to prevent stalling
      digitalWrite(pressure_power, HIGH); //turn pressure pump off
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if (power_count >= 40){//allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }

  if ((pressure > P_LO) and (pressure < P_HI)){ //when the pressure is between limits perform the following...
    digitalWrite(pressure_power, HIGH); //turn pressure pump on
    digitalWrite(pressure_relief, LOW); //close pressure relief valve
    if (power_count >= 40){ //allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }
  
  if (pressure >= P_HI){ //when the pressure is above the limit perform the following...
    digitalWrite(pressure_power, HIGH); //turn pressure pump off
    digitalWrite(pressure_relief, HIGH); //open pressure relief valve
    delay(2); //delay 3 ms 
    digitalWrite(pressure_relief, LOW); //close pressure relief valve
    delay(2); //delay 2 ms
    if (power_count >= 40){//allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }
  power_count = power_count + 1; //timer for PV fuzzy logic regulator

  //vacuum regulation
  if (vacuum <= V_LO){ //when vacuum is below the limit perform the following...
    if (power_count2 == 0){ //at the beginning of the vacuum regulation if the pump needs to be turned on also turn on the valve because the chamber fills quickly
      digitalWrite(vacuum_power, LOW); //turn vacuum pump on
      digitalWrite(vacuum_relief, LOW); //open vacuum relief valve
      delay(3); //delay 3 ms
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if ((power_count2 > 0) and (power_count2 < 24)){ //leave the vacuum on for up to 3 s
      digitalWrite(vacuum_power, LOW); //turn vacuum pump on
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if (power_count2 >= 24){ //after 3s turn the vacuu, pump off for at least 2s to prevent stalling
      digitalWrite(vacuum_power, HIGH); //turn off vacuum pump
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if (power_count2 >= 40){//allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }

  if ((vacuum > V_LO) and (vacuum < V_HI)){ //if vacuum is between limits perform the following...
    digitalWrite(vacuum_power, HIGH); //turn vacuum pump off
    digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    if (power_count2 >= 40){ //allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }
  
  if (vacuum >= V_HI){ //if vacuum is above limit perform the following...
    digitalWrite(vacuum_power, HIGH); //turn vacuum pump off
    digitalWrite(vacuum_relief, LOW); //open vacuum relief valve
    delay(3); //delay 3 ms
    digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    delay(3); //delay 3 ms
    if (power_count2 >= 40){//allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }
  power_count2 = power_count2 + 1; //limit timer for how long the pump can be on
 
}

//////////////////////////////////////////////////////////////////////////Step 1////////////////////////////////////////////////////////////////////////////////////////
if (state == 8){
  lcd.setCursor(0,0);
  lcd.print("Step 1");
  lcd.setCursor(7,0);
  lcd.print(temp,0); //display current temp on moitor
  lcd.setCursor(9,0);
  lcd.print("C");
  lcd.setCursor(15,0);
  lcd.print("s");
  lcd.setCursor(0,1);
  lcd.print("Change      STOP");

    //Left Button Actions for Step 1 (CHANGE)
    if (LB_push == ON){
    LB_push = OFF; //reset button press

    //reset values (except heat)
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    
    //Tell Voltage Reader that EFIELD is OFF
    digitalWrite(C_ON,LOW);
    digitalWrite(ML_ON,LOW);
    digitalWrite(MR_ON,LOW);
    
    lcd.clear(); //clear LCD
    state = 14; //go to state 14
  }

  //Center Button Actions for Step 1 (DO NOTHING)
  if (CB_push == ON){
    CB_push = OFF; //reset button press
    //DO NOTHING
  }

  //Right Button Actions for Step 1 (STOP)
  if (RB_push ==ON){
    RB_push = OFF; //reset button press
    lcd.clear(); //clear LCD
    
    //Reset Values Completely
    active_heat = 0;
    PID_count = 0;
    count_heat = 0;
    count_heatW = 0;
    count_heatC = 0;
    PID_countPV = 0;
    count_pres = 0;
    count_vac = 0;
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(res_relay, HIGH);
    digitalWrite(heat_relay, HIGH);
    digitalWrite(cool_relay, HIGH);
    
    //Tell Voltage Reader that EFIELD is OFF
    digitalWrite(C_ON,LOW);
    digitalWrite(ML_ON,LOW);
    digitalWrite(MR_ON,LOW);
    
    lcd.clear(); //clear LCD
    state = 1; //go to state 1
  }

  //Start Button Actions for Step 1 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push = OFF; //reset button press
  }
  
  input_temp = 54; //set the temperature to 54C
  active_heat = 1; //turn on the heater
  overall_step1 = millis() - start_overall1; //begin the timer for step 1
  
  //Pump until fluid reaches flowcell
  if (overall_step1 < reach_time){ 
    start_time = millis();//reset EFIELD loop timer
    start_step = millis();// reset step timer
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    //Tell Voltage Reader that EFIELD is OFF
    digitalWrite(C_ON,LOW);
    digitalWrite(ML_ON,LOW);
    digitalWrite(MR_ON,LOW);
    
    //pump Cycle
    if(pump_count < 1){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, HIGH);
      digitalWrite(PV_valve2, LOW);
    }
    if ((pump_count >= 1) and (pump_count < 9)){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, LOW);
      digitalWrite(PV_valve2, LOW);
    }
    if ((pump_count >= 9) and (pump_count < 10)){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, LOW);
      digitalWrite(PV_valve2, HIGH);
    }
    if ((pump_count >= 10) and (pump_count < 11)){
      digitalWrite(PV_valve1, LOW);
      digitalWrite(PV_pump, LOW);
      digitalWrite(PV_valve2, HIGH);
    }
    if ((pump_count >= 11) and (pump_count < 19)){
      digitalWrite(PV_valve1, LOW);
      digitalWrite(PV_pump, HIGH);
      digitalWrite(PV_valve2, HIGH);
    }
    if ((pump_count >= 19) and (pump_count < 20)){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, HIGH);
      digitalWrite(PV_valve2, HIGH);
    }
    pump_count = pump_count + 1; //pump cycles through counts
    if (pump_count >= 20){ //if count is greater than 20 reset pump cycle
      pump_count = 0; //reset pump count to go through another pump cycle
    }
  }

  //AFTER fluid reaches flowcell, begin STEPS
  if ((overall_step1 >= reach_time) and (overall_step1 <= totalHYB_time)){
    time_step1 = millis() - start_step; //start step timer
    
    //pump until fluid fills flowcell
    if (time_step1 <= fill_time){ 
      start_time = millis(); //reset EFIELD timer
      digitalWrite(E1_V,HIGH);
      digitalWrite(E1_G,HIGH);
      digitalWrite(E2_V,HIGH);
      digitalWrite(E2_G,HIGH);
      digitalWrite(E3_V,HIGH);
      digitalWrite(E3_G,HIGH);
      digitalWrite(E4,HIGH);
      digitalWrite(E5,LOW);
      //Tell Voltage Reader that EFIELD is OFF
      digitalWrite(C_ON,LOW);
      digitalWrite(ML_ON,LOW);
      digitalWrite(MR_ON,LOW);
      
      //pump Cycle
      if(pump_count < 1){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, LOW);
      }
      if ((pump_count >= 1) and (pump_count < 9)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, LOW);
      }
      if ((pump_count >= 9) and (pump_count < 10)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 10) and (pump_count < 11)){
        digitalWrite(PV_valve1, LOW);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 11) and (pump_count < 19)){
        digitalWrite(PV_valve1, LOW);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 19) and (pump_count < 20)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, HIGH);
      }
      pump_count = pump_count + 1; //go through counts for pump cycle
      if (pump_count >= 20){ //if counter is greater than 20, reset the pump cycle
        pump_count = 0; //reset pump cycle counter
      }
    }

    //Start the Electric Field Portion of the STEP
    else if ((time_step1 > fill_time) and (time_step1 <= steps_time)){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, HIGH);
      digitalWrite(PV_valve2, HIGH);
      digitalWrite(PV_extra, HIGH); 
      time_conc = conc_target; //set the appropriate concentration time
      time_elapsed = millis() - start_time; //start the electric field timer
      
      //turn on EFIELD to concentrate and mix Target
      if (time_elapsed <= time_conc){
        //Turn on Field 3s
        digitalWrite(E1_V,HIGH);
        digitalWrite(E1_G,LOW);
        digitalWrite(E2_V,HIGH);
        digitalWrite(E2_G,LOW);
        digitalWrite(E3_V,HIGH);
        digitalWrite(E3_G,LOW);
        digitalWrite(E4,LOW);
        digitalWrite(E5,HIGH);
        //Tell the voltage reader that the device is concentrating
        digitalWrite(C_ON,HIGH);
        digitalWrite(ML_ON,LOW);
        digitalWrite(MR_ON,LOW);
      }
      if (time_elapsed > time_conc and time_elapsed <= (time_conc*2)){
        //Turn off Field 3s
        digitalWrite(E1_V,HIGH);
        digitalWrite(E1_G,HIGH);
        digitalWrite(E2_V,HIGH);
        digitalWrite(E2_G,HIGH);
        digitalWrite(E3_V,HIGH);
        digitalWrite(E3_G,HIGH);
        digitalWrite(E4,HIGH);
        digitalWrite(E5,LOW);
        //Tell voltage reader that the device is mixing to the LEFT
        digitalWrite(C_ON,LOW);
        digitalWrite(ML_ON,LOW);
        digitalWrite(MR_ON,LOW);
      }
      if (time_elapsed > (time_conc*2) and time_elapsed <= (time_conc*2 + 500)){
        //Turn on Field 0.5s
        digitalWrite(E1_V,HIGH);
        digitalWrite(E1_G,LOW);
        digitalWrite(E2_V,HIGH);
        digitalWrite(E2_G,LOW);
        digitalWrite(E3_V,HIGH);
        digitalWrite(E3_G,LOW);
        digitalWrite(E4,LOW);
        digitalWrite(E5,HIGH);
        //Tell voltage reader that the device is mixing to the right
        digitalWrite(C_ON,HIGH);
        digitalWrite(ML_ON,LOW);
        digitalWrite(MR_ON,LOW);
      }
      if (time_elapsed > (time_conc*2 + 500)){
        //Turn off Field
        digitalWrite(E1_V,HIGH);
        digitalWrite(E1_G,HIGH);
        digitalWrite(E2_V,HIGH);
        digitalWrite(E2_G,HIGH);
        digitalWrite(E3_V,HIGH);
        digitalWrite(E3_G,HIGH);
        digitalWrite(E4,HIGH);
        digitalWrite(E5,LOW);
        //Tell voltage reader that the device is mixing to the LEFT
        digitalWrite(C_ON,LOW);
        digitalWrite(ML_ON,LOW);
        digitalWrite(MR_ON,LOW);
      }
    }
    
    //After the step time has been reached, reset EFIELD and step timer to perform another step
    if (time_step1 > steps_time){
      digitalWrite(E1_V,HIGH);
      digitalWrite(E1_G,HIGH);
      digitalWrite(E2_V,HIGH);
      digitalWrite(E2_G,HIGH);
      digitalWrite(E3_V,HIGH);
      digitalWrite(E3_G,HIGH);
      digitalWrite(E4,HIGH);
      digitalWrite(E5,LOW);
      start_step = millis();
      //Tell Voltage Reader that EFIELD is OFF
      digitalWrite(C_ON,LOW);
      digitalWrite(ML_ON,LOW);
      digitalWrite(MR_ON,LOW);
    }
  }
  
  //Reset All Values after total hybridization time has been reached
  if (overall_step1 > totalHYB_time){
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    //Tell Voltage Reader that EFIELD is OFF
    digitalWrite(C_ON,LOW);
    digitalWrite(ML_ON,LOW);
    digitalWrite(MR_ON,LOW);
    lcd.clear(); //clear LCD
    state = 15; //go to state 15
  }
  
  //////////////////////////////////////////////////////////////PV regulation for Step 1
  //Pressure Pump Regulation
  if (pressure <= P_LO){ //when pressure is below the limit perform the following...
    if (power_count == 0){ //at the beginning of the pressure regulation if the pump needs to be turned on also turn on the valve because the chamber fills quickly
      digitalWrite(pressure_power, LOW); //turn pressure pump on
      digitalWrite(pressure_relief, HIGH); //open pressure relief valve
      delay(3); //delay 3 ms
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if ((power_count > 0) and (power_count < 24)){ //leave the pressure on for up to 3 s
      digitalWrite(pressure_power, LOW);  //turn the pressure pump on
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if (power_count >= 24){ //after 3s turn the pressure pump off for at least 2s to prevent stalling
      digitalWrite(pressure_power, HIGH); //turn pressure pump off
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if (power_count >= 40){//allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }

  if ((pressure > P_LO) and (pressure < P_HI)){ //when the pressure is between limits perform the following...
    digitalWrite(pressure_power, HIGH); //turn pressure pump on
    digitalWrite(pressure_relief, LOW); //close pressure relief valve
    if (power_count >= 40){ //allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }
  
  if (pressure >= P_HI){ //when the pressure is above the limit perform the following...
    digitalWrite(pressure_power, HIGH); //turn pressure pump off
    digitalWrite(pressure_relief, HIGH); //open pressure relief valve
    delay(2); //delay 3 ms 
    digitalWrite(pressure_relief, LOW); //close pressure relief valve
    delay(2); //delay 2 ms
    if (power_count >= 40){//allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }
  power_count = power_count + 1; //timer for PV fuzzy logic regulator

  //vacuum regulation
  if (vacuum <= V_LO){ //when vacuum is below the limit perform the following...
    if (power_count2 == 0){ //at the beginning of the vacuum regulation if the pump needs to be turned on also turn on the valve because the chamber fills quickly
      digitalWrite(vacuum_power, LOW); //turn vacuum pump on
      digitalWrite(vacuum_relief, LOW); //open vacuum relief valve
      delay(3); //delay 3 ms
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if ((power_count2 > 0) and (power_count2 < 24)){ //leave the vacuum on for up to 3 s
      digitalWrite(vacuum_power, LOW); //turn vacuum pump on
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if (power_count2 >= 24){ //after 3s turn the vacuu, pump off for at least 2s to prevent stalling
      digitalWrite(vacuum_power, HIGH); //turn off vacuum pump
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if (power_count2 >= 40){//allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }

  if ((vacuum > V_LO) and (vacuum < V_HI)){ //if vacuum is between limits perform the following...
    digitalWrite(vacuum_power, HIGH); //turn vacuum pump off
    digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    if (power_count2 >= 40){ //allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }
  
  if (vacuum >= V_HI){ //if vacuum is above limit perform the following...
    digitalWrite(vacuum_power, HIGH); //turn vacuum pump off
    digitalWrite(vacuum_relief, LOW); //open vacuum relief valve
    delay(3); //delay 3 ms
    digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    delay(3); //delay 3 ms
    if (power_count2 >= 40){//allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }
  power_count2 = power_count2 + 1; //limit timer for how long the pump can be on
  

}

////////////////////////////////////////////////////////////////////////STEP 2//////////////////////////////////////////////////////////////////////////////////////////
if (state == 9){
  lcd.setCursor(0,0);
  lcd.print("Step 2");
  lcd.setCursor(7,0);
  lcd.print(temp,0); //display current temp on LCD
  lcd.setCursor(9,0);
  lcd.print("C");
  lcd.setCursor(15,0);
  lcd.print("s");
  lcd.setCursor(0,1);
  lcd.print("Change      STOP");

  //Left Button Actions for Step 2 (CHANGE)
  if (LB_push == ON){
    LB_push = OFF; //reset button press
    
    //Reset values, DO NOT reset heating/cooling
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    lcd.clear(); //clear LCD
    state = 14; //go to state 14
  }

  //Center Button Actions for Step 2 (DO NOTHING)
  if (CB_push == ON){
    CB_push = OFF;//reset button press
    //DO NOTHING
  }

  //Right Button Actions for Step 2 (STOP)
  if (RB_push ==ON){
    RB_push = OFF; //reset button press
    
    //Reset Values Completely
    active_heat = 0;
    PID_count = 0;
    count_heat = 0;
    count_heatW = 0;
    count_heatC = 0;
    PID_countPV = 0;
    count_pres = 0;
    count_vac = 0;
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(res_relay, HIGH);
    digitalWrite(heat_relay, HIGH);
    digitalWrite(cool_relay, HIGH);
    lcd.clear(); //clear LCD
    state = 1; //go to state 1
  }

  //Start Button Actions for Step 2 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push = OFF; //reset button press
  }

  //turn off all EFIELD functions
  digitalWrite(E1_V,HIGH);
  digitalWrite(E1_G,HIGH);
  digitalWrite(E2_V,HIGH);
  digitalWrite(E2_G,HIGH);
  digitalWrite(E3_V,HIGH);
  digitalWrite(E3_G,HIGH);
  digitalWrite(E4,HIGH);
  digitalWrite(E5,LOW);
  
  time_step2 = millis() - start_step2; //start timer for step 2

  //For the first 15 s continue pumping at a normal rate while maintaining temp at 54 C
  if (time_step2 < 15000){
      input_temp = 54; //set current temp to 54 C
      active_heat = 1; //turn on heater
      
      //pump Cycle
      if(pump_count < 1){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, LOW);
      }
      if ((pump_count >= 1) and (pump_count < 9)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, LOW);
      }
      if ((pump_count >= 9) and (pump_count < 10)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 10) and (pump_count < 11)){
        digitalWrite(PV_valve1, LOW);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 11) and (pump_count < 19)){
        digitalWrite(PV_valve1, LOW);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 19) and (pump_count < 20)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, HIGH);
      }
      pump_count = pump_count + 1;
      if (pump_count >= 20){
        pump_count = 0;
      }
  }

  //After 15s turn on TEC COOL and continue pumping at 30s/pump until reaches 18 C
  if ((time_step2 >= 15000) and (temp > 18 + 0.2)) {
    input_temp = 18; //sets current temp to 18 C
    active_heat = 2; //turn on cooler
    
    digitalWrite(heat_relay, HIGH); //keep TEC heater OFF
    digitalWrite(res_relay, HIGH); //keep resistors OFF
    
    //pump Cycle
    if(pump_count < 1){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, HIGH);
      digitalWrite(PV_valve2, LOW);
    }
    if ((pump_count >= 1) and (pump_count < 9)){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, LOW);
      digitalWrite(PV_valve2, LOW);
    }
    if ((pump_count >= 9) and (pump_count < 10)){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, LOW);
      digitalWrite(PV_valve2, HIGH);
    }
    if ((pump_count >= 10) and (pump_count < 11)){
      digitalWrite(PV_valve1, LOW);
      digitalWrite(PV_pump, LOW);
      digitalWrite(PV_valve2, HIGH);
    }
    if ((pump_count >= 11) and (pump_count < 19)){
      digitalWrite(PV_valve1, LOW);
      digitalWrite(PV_pump, HIGH);
      digitalWrite(PV_valve2, HIGH);
    }
    if ((pump_count >= 19) and (pump_count < 240)){
      digitalWrite(PV_valve1, HIGH);
      digitalWrite(PV_pump, HIGH);
      digitalWrite(PV_valve2, HIGH);
    }
    pump_count = pump_count + 1;
    if (pump_count >= 240){
      pump_count = 0;
    }
  }

  //Exit step 2 once temp has been reached
  if (temp <= 18.2){
    //reset all values except cooling
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    lcd.clear();//clear LCD
    state = 15; //go to state 15
  }
  
 //////////////////////////////////////////////////////////////PV regulation for Step 2
  //Pressure Pump Regulation
  if (pressure <= P_LO){ //when pressure is below the limit perform the following...
    if (power_count == 0){ //at the beginning of the pressure regulation if the pump needs to be turned on also turn on the valve because the chamber fills quickly
      digitalWrite(pressure_power, LOW); //turn pressure pump on
      digitalWrite(pressure_relief, HIGH); //open pressure relief valve
      delay(3); //delay 3 ms
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if ((power_count > 0) and (power_count < 24)){ //leave the pressure on for up to 3 s
      digitalWrite(pressure_power, LOW);  //turn the pressure pump on
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if (power_count >= 24){ //after 3s turn the pressure pump off for at least 2s to prevent stalling
      digitalWrite(pressure_power, HIGH); //turn pressure pump off
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if (power_count >= 40){//allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }

  if ((pressure > P_LO) and (pressure < P_HI)){ //when the pressure is between limits perform the following...
    digitalWrite(pressure_power, HIGH); //turn pressure pump on
    digitalWrite(pressure_relief, LOW); //close pressure relief valve
    if (power_count >= 40){ //allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }
  
  if (pressure >= P_HI){ //when the pressure is above the limit perform the following...
    digitalWrite(pressure_power, HIGH); //turn pressure pump off
    digitalWrite(pressure_relief, HIGH); //open pressure relief valve
    delay(2); //delay 3 ms 
    digitalWrite(pressure_relief, LOW); //close pressure relief valve
    delay(2); //delay 2 ms
    if (power_count >= 40){//allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }
  power_count = power_count + 1; //timer for PV fuzzy logic regulator

  //vacuum regulation
  if (vacuum <= V_LO){ //when vacuum is below the limit perform the following...
    if (power_count2 == 0){ //at the beginning of the vacuum regulation if the pump needs to be turned on also turn on the valve because the chamber fills quickly
      digitalWrite(vacuum_power, LOW); //turn vacuum pump on
      digitalWrite(vacuum_relief, LOW); //open vacuum relief valve
      delay(3); //delay 3 ms
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if ((power_count2 > 0) and (power_count2 < 24)){ //leave the vacuum on for up to 3 s
      digitalWrite(vacuum_power, LOW); //turn vacuum pump on
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if (power_count2 >= 24){ //after 3s turn the vacuu, pump off for at least 2s to prevent stalling
      digitalWrite(vacuum_power, HIGH); //turn off vacuum pump
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if (power_count2 >= 40){//allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }

  if ((vacuum > V_LO) and (vacuum < V_HI)){ //if vacuum is between limits perform the following...
    digitalWrite(vacuum_power, HIGH); //turn vacuum pump off
    digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    if (power_count2 >= 40){ //allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }
  
  if (vacuum >= V_HI){ //if vacuum is above limit perform the following...
    digitalWrite(vacuum_power, HIGH); //turn vacuum pump off
    digitalWrite(vacuum_relief, LOW); //open vacuum relief valve
    delay(3); //delay 3 ms
    digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    delay(3); //delay 3 ms
    if (power_count2 >= 40){//allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }
  power_count2 = power_count2 + 1; //limit timer for how long the pump can be on


}

////////////////////////////////////////////////////////////////////////STEP 3//////////////////////////////////////////////////////////////////////////////////////////
if (state == 10){
  lcd.setCursor(0,0);
  lcd.print("Step 3");
  lcd.setCursor(7,0);
  lcd.print(temp,0); //display current temp on LCD
  lcd.setCursor(9,0);
  lcd.print("C");
  lcd.setCursor(15,0);
  lcd.print("s");
  lcd.setCursor(0,1);
  lcd.print("Change      STOP");

  //Left Button Actions for Step 3 (CHANGE)
  if (LB_push == ON){
    LB_push = OFF; //reset button press
    
    //reset values (NOT HEAT)
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    lcd.clear(); //clear LCD
    state = 14; //go to state 14
  }

  //Center Button Actions for Step 3 (DO NOTHING)
  if (CB_push == ON){
    CB_push = OFF; //reset button press
    //DO NOTHING
  }

  //Right Button Actions for Step 4 (STOP)
  if (RB_push ==ON){
    RB_push = OFF; //reset button press
    lcd.clear(); //clear LCD
    
    //Reset Values Completely
    active_heat = 0;
    PID_count = 0;
    count_heat = 0;
    count_heatW = 0;
    count_heatC = 0;
    PID_countPV = 0;
    count_pres = 0;
    count_vac = 0;
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(res_relay, HIGH);
    digitalWrite(heat_relay, HIGH);
    digitalWrite(cool_relay, HIGH);
    lcd.clear();//clear LCD
    state = 1; //go to state 1
  }
  
  //Start Button Actions for Step 3 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push = OFF; //reset button press
  }
  
  input_temp = 18; //set current temp to 18
  active_heat = 2; //turn on cooler
  time_step3 = millis() - start_step3; //start step 3 timer

  //Detector will pass over array for a total of 200s
  if (time_step3 <= detector_time){
      //pump Cycle
      if(pump_count < 1){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, LOW);
      }
      if ((pump_count >= 1) and (pump_count < 9)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, LOW);
      }
      if ((pump_count >= 9) and (pump_count < 10)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 10) and (pump_count < 11)){
        digitalWrite(PV_valve1, LOW);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 11) and (pump_count < 19)){
        digitalWrite(PV_valve1, LOW);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 19) and (pump_count < 20)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, HIGH);
      }
      pump_count = pump_count + 1;
      if (pump_count >= 20){
        pump_count = 0;
      }
  }

  //Reset Values once Step 3 has been completed
  if (time_step3 > detector_time){
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    lcd.clear(); //clear LCD
    state = 15; //go to state 15
  }

   //////////////////////////////////////////////////////////////PV regulation for Step 3
  //Pressure Pump Regulation
  if (pressure <= P_LO){ //when pressure is below the limit perform the following...
    if (power_count == 0){ //at the beginning of the pressure regulation if the pump needs to be turned on also turn on the valve because the chamber fills quickly
      digitalWrite(pressure_power, LOW); //turn pressure pump on
      digitalWrite(pressure_relief, HIGH); //open pressure relief valve
      delay(3); //delay 3 ms
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if ((power_count > 0) and (power_count < 24)){ //leave the pressure on for up to 3 s
      digitalWrite(pressure_power, LOW);  //turn the pressure pump on
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if (power_count >= 24){ //after 3s turn the pressure pump off for at least 2s to prevent stalling
      digitalWrite(pressure_power, HIGH); //turn pressure pump off
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if (power_count >= 40){//allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }

  if ((pressure > P_LO) and (pressure < P_HI)){ //when the pressure is between limits perform the following...
    digitalWrite(pressure_power, HIGH); //turn pressure pump on
    digitalWrite(pressure_relief, LOW); //close pressure relief valve
    if (power_count >= 40){ //allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }
  
  if (pressure >= P_HI){ //when the pressure is above the limit perform the following...
    digitalWrite(pressure_power, HIGH); //turn pressure pump off
    digitalWrite(pressure_relief, HIGH); //open pressure relief valve
    delay(2); //delay 3 ms 
    digitalWrite(pressure_relief, LOW); //close pressure relief valve
    delay(2); //delay 2 ms
    if (power_count >= 40){//allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }
  power_count = power_count + 1; //timer for PV fuzzy logic regulator

  //vacuum regulation
  if (vacuum <= V_LO){ //when vacuum is below the limit perform the following...
    if (power_count2 == 0){ //at the beginning of the vacuum regulation if the pump needs to be turned on also turn on the valve because the chamber fills quickly
      digitalWrite(vacuum_power, LOW); //turn vacuum pump on
      digitalWrite(vacuum_relief, LOW); //open vacuum relief valve
      delay(3); //delay 3 ms
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if ((power_count2 > 0) and (power_count2 < 24)){ //leave the vacuum on for up to 3 s
      digitalWrite(vacuum_power, LOW); //turn vacuum pump on
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if (power_count2 >= 24){ //after 3s turn the vacuu, pump off for at least 2s to prevent stalling
      digitalWrite(vacuum_power, HIGH); //turn off vacuum pump
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if (power_count2 >= 40){//allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }

  if ((vacuum > V_LO) and (vacuum < V_HI)){ //if vacuum is between limits perform the following...
    digitalWrite(vacuum_power, HIGH); //turn vacuum pump off
    digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    if (power_count2 >= 40){ //allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }
  
  if (vacuum >= V_HI){ //if vacuum is above limit perform the following...
    digitalWrite(vacuum_power, HIGH); //turn vacuum pump off
    digitalWrite(vacuum_relief, LOW); //open vacuum relief valve
    delay(3); //delay 3 ms
    digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    delay(3); //delay 3 ms
    if (power_count2 >= 40){//allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }
  power_count2 = power_count2 + 1; //limit timer for how long the pump can be on
  

}

////////////////////////////////////////////////////////////////////////STEP 4//////////////////////////////////////////////////////////////////////////////////////////
if (state == 11){
  lcd.setCursor(0,0);
  lcd.print("Step 4");
  lcd.setCursor(7,0);
  lcd.print(temp,0); //dispay temp on LCD
  lcd.setCursor(9,0);
  lcd.print("C");
  lcd.setCursor(15,0);
  lcd.print("s");
  lcd.setCursor(0,1);
  lcd.print("Change      STOP");

  //Left Button Actions for Step 4 (CHANGE)
  if (LB_push == ON){
    LB_push = OFF; //reset button press
    
    //reset values except heating
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    lcd.clear(); //clear LCD
    state = 14; //go to state 14
  }

  //Center Button Actions for Step 4 (DO NOTHING)
  if (CB_push == ON){
    CB_push = OFF; //reset button press
    //DO NOTHING
  }

  //Right Button Actions for Step 4 (STOP)
  if (RB_push ==ON){
    RB_push = OFF; //reset button press
    lcd.clear();
    
    //Reset Values Completely
    active_heat = 0;
    PID_count = 0;
    count_heat = 0;
    count_heatW = 0;
    count_heatC = 0;
    PID_countPV = 0;
    count_pres = 0;
    count_vac = 0;
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(res_relay, HIGH);
    digitalWrite(heat_relay, HIGH);
    digitalWrite(cool_relay, HIGH);
    lcd.clear(); //clear LCD
    state = 1; //go to state 1
  }

  //Start Button Actions for Step 4 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push = OFF; //reset button press
  }
  
  input_temp = 18; //set current temp to 18 C
  active_heat = 2; //turn on cooler
  time_step4 = millis() - start_step4; //start step 4 timer

  //Pump until reaches Rinse Time
  if (time_step4 <= rinse_time){
      //pump Cycle
      if(pump_count < 1){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, LOW);
      }
      if ((pump_count >= 1) and (pump_count < 9)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, LOW);
      }
      if ((pump_count >= 9) and (pump_count < 10)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 10) and (pump_count < 11)){
        digitalWrite(PV_valve1, LOW);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 11) and (pump_count < 19)){
        digitalWrite(PV_valve1, LOW);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 19) and (pump_count < 20)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, HIGH);
      }
      pump_count = pump_count + 1;
      if (pump_count >= 20){
        pump_count = 0;
      }
  }

  //Once Step 4 is complete, reset values
  if (time_step4 > rinse_time){
    //rest values
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    lcd.clear(); //clear LCD
    state = 15; //go to state 15
  }
  
  //////////////////////////////////////////////////////////////PV regulation for Step 4
  //Pressure Pump Regulation
  if (pressure <= P_LO){ //when pressure is below the limit perform the following...
    if (power_count == 0){ //at the beginning of the pressure regulation if the pump needs to be turned on also turn on the valve because the chamber fills quickly
      digitalWrite(pressure_power, LOW); //turn pressure pump on
      digitalWrite(pressure_relief, HIGH); //open pressure relief valve
      delay(3); //delay 3 ms
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if ((power_count > 0) and (power_count < 24)){ //leave the pressure on for up to 3 s
      digitalWrite(pressure_power, LOW);  //turn the pressure pump on
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if (power_count >= 24){ //after 3s turn the pressure pump off for at least 2s to prevent stalling
      digitalWrite(pressure_power, HIGH); //turn pressure pump off
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if (power_count >= 40){//allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }

  if ((pressure > P_LO) and (pressure < P_HI)){ //when the pressure is between limits perform the following...
    digitalWrite(pressure_power, HIGH); //turn pressure pump on
    digitalWrite(pressure_relief, LOW); //close pressure relief valve
    if (power_count >= 40){ //allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }
  
  if (pressure >= P_HI){ //when the pressure is above the limit perform the following...
    digitalWrite(pressure_power, HIGH); //turn pressure pump off
    digitalWrite(pressure_relief, HIGH); //open pressure relief valve
    delay(2); //delay 3 ms 
    digitalWrite(pressure_relief, LOW); //close pressure relief valve
    delay(2); //delay 2 ms
    if (power_count >= 40){//allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }
  power_count = power_count + 1; //timer for PV fuzzy logic regulator

  //vacuum regulation
  if (vacuum <= V_LO){ //when vacuum is below the limit perform the following...
    if (power_count2 == 0){ //at the beginning of the vacuum regulation if the pump needs to be turned on also turn on the valve because the chamber fills quickly
      digitalWrite(vacuum_power, LOW); //turn vacuum pump on
      digitalWrite(vacuum_relief, LOW); //open vacuum relief valve
      delay(3); //delay 3 ms
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if ((power_count2 > 0) and (power_count2 < 24)){ //leave the vacuum on for up to 3 s
      digitalWrite(vacuum_power, LOW); //turn vacuum pump on
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if (power_count2 >= 24){ //after 3s turn the vacuu, pump off for at least 2s to prevent stalling
      digitalWrite(vacuum_power, HIGH); //turn off vacuum pump
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if (power_count2 >= 40){//allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }

  if ((vacuum > V_LO) and (vacuum < V_HI)){ //if vacuum is between limits perform the following...
    digitalWrite(vacuum_power, HIGH); //turn vacuum pump off
    digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    if (power_count2 >= 40){ //allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }
  
  if (vacuum >= V_HI){ //if vacuum is above limit perform the following...
    digitalWrite(vacuum_power, HIGH); //turn vacuum pump off
    digitalWrite(vacuum_relief, LOW); //open vacuum relief valve
    delay(3); //delay 3 ms
    digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    delay(3); //delay 3 ms
    if (power_count2 >= 40){//allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }
  power_count2 = power_count2 + 1; //limit timer for how long the pump can be on
  

}

////////////////////////////////////////////////////////////////////////STEP 5//////////////////////////////////////////////////////////////////////////////////////////
if (state == 12){
  lcd.setCursor(0,0);
  lcd.print("Step 5");
  lcd.setCursor(7,0);
  lcd.print(temp,0); //display temp on monitor
  lcd.setCursor(9,0);
  lcd.print("C");
  lcd.setCursor(15,0);
  lcd.print("s");
  lcd.setCursor(0,1);
  lcd.print("Change      STOP");

  //Left Button Actions for Step 5 (CHANGE)
  if (LB_push == ON){
    LB_push = OFF; //reset button press

    //reset values
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    lcd.clear(); //clear LCD
    state = 14; //go to state 15
  }

  //Center Button Actions for Step 5 (DO NOTHING)
  if (CB_push == ON){
    CB_push = OFF; //reset button press
    //DO NOTHING
  }

  //Right Button Actions for Step 5 (STOP)
  if (RB_push ==ON){
    RB_push = OFF; //reset button press
 
    //Reset Values Completely
    active_heat = 0;
    PID_count = 0;
    count_heat = 0;
    count_heatW = 0;
    count_heatC = 0;
    PID_countPV = 0;
    count_pres = 0;
    count_vac = 0;
    digitalWrite(E1_V,HIGH);
    digitalWrite(E1_G,HIGH);
    digitalWrite(E2_V,HIGH);
    digitalWrite(E2_G,HIGH);
    digitalWrite(E3_V,HIGH);
    digitalWrite(E3_G,HIGH);
    digitalWrite(E4,HIGH);
    digitalWrite(E5,LOW);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(res_relay, HIGH);
    digitalWrite(heat_relay, HIGH);
    digitalWrite(cool_relay, HIGH);
    lcd.clear(); //clear LCD
    state = 1; //go to state 1
  }

  //Start Button Actions for Step 5 (DO NOTHING)
  if (StartB_push == ON){
    StartB_push = OFF; //reset button press
  }

  //Turn on Cooling State
  input_temp = 18; //set the current temp to 18
  active_heat = 2; //turn on cooling

  time_step5 = millis() - start_step5; //start step 5 timer

  //Pump until timer reaches rinse time
    if (time_step5 <= rinse_time){
      //pump Cycle
      if(pump_count < 1){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, LOW);
      }
      if ((pump_count >= 1) and (pump_count < 9)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, LOW);
      }
      if ((pump_count >= 9) and (pump_count < 10)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 10) and (pump_count < 11)){
        digitalWrite(PV_valve1, LOW);
        digitalWrite(PV_pump, LOW);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 11) and (pump_count < 19)){
        digitalWrite(PV_valve1, LOW);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, HIGH);
      }
      if ((pump_count >= 19) and (pump_count < 20)){
        digitalWrite(PV_valve1, HIGH);
        digitalWrite(PV_pump, HIGH);
        digitalWrite(PV_valve2, HIGH);
      }
      pump_count = pump_count + 1;
      if (pump_count >= 20){
        pump_count = 0;
      }
  }

//Reset values after reaching rinse time
if (time_step5 > rinse_time){
    //reset values
    digitalWrite(PV_valve1, HIGH);
    digitalWrite(PV_pump, HIGH);
    digitalWrite(PV_valve2, HIGH);
    digitalWrite(PV_extra, HIGH);
    digitalWrite(pressure_power, HIGH);
    digitalWrite(vacuum_power, HIGH);
    digitalWrite(pressure_relief, LOW);
    digitalWrite(vacuum_relief, LOW);
    lcd.clear(); //clear LCD
    state = 15; //go to state 15
  }

  //////////////////////////////////////////////////////////////PV regulation for Step 5
  //Pressure Pump Regulation
  if (pressure <= P_LO){ //when pressure is below the limit perform the following...
    if (power_count == 0){ //at the beginning of the pressure regulation if the pump needs to be turned on also turn on the valve because the chamber fills quickly
      digitalWrite(pressure_power, LOW); //turn pressure pump on
      digitalWrite(pressure_relief, HIGH); //open pressure relief valve
      delay(3); //delay 3 ms
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if ((power_count > 0) and (power_count < 24)){ //leave the pressure on for up to 3 s
      digitalWrite(pressure_power, LOW);  //turn the pressure pump on
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if (power_count >= 24){ //after 3s turn the pressure pump off for at least 2s to prevent stalling
      digitalWrite(pressure_power, HIGH); //turn pressure pump off
      digitalWrite(pressure_relief, LOW); //close pressure relief valve
    }
    if (power_count >= 40){//allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }

  if ((pressure > P_LO) and (pressure < P_HI)){ //when the pressure is between limits perform the following...
    digitalWrite(pressure_power, HIGH); //turn pressure pump on
    digitalWrite(pressure_relief, LOW); //close pressure relief valve
    if (power_count >= 40){ //allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }
  
  if (pressure >= P_HI){ //when the pressure is above the limit perform the following...
    digitalWrite(pressure_power, HIGH); //turn pressure pump off
    digitalWrite(pressure_relief, HIGH); //open pressure relief valve
    delay(2); //delay 3 ms 
    digitalWrite(pressure_relief, LOW); //close pressure relief valve
    delay(2); //delay 2 ms
    if (power_count >= 40){//allows us to always wait 2s between the pump being on and off
      power_count = 0; //reset power counter
    }
  }
  power_count = power_count + 1; //timer for PV fuzzy logic regulator

  //vacuum regulation
  if (vacuum <= V_LO){ //when vacuum is below the limit perform the following...
    if (power_count2 == 0){ //at the beginning of the vacuum regulation if the pump needs to be turned on also turn on the valve because the chamber fills quickly
      digitalWrite(vacuum_power, LOW); //turn vacuum pump on
      digitalWrite(vacuum_relief, LOW); //open vacuum relief valve
      delay(3); //delay 3 ms
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if ((power_count2 > 0) and (power_count2 < 24)){ //leave the vacuum on for up to 3 s
      digitalWrite(vacuum_power, LOW); //turn vacuum pump on
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if (power_count2 >= 24){ //after 3s turn the vacuu, pump off for at least 2s to prevent stalling
      digitalWrite(vacuum_power, HIGH); //turn off vacuum pump
      digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    }
    if (power_count2 >= 40){//allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }

  if ((vacuum > V_LO) and (vacuum < V_HI)){ //if vacuum is between limits perform the following...
    digitalWrite(vacuum_power, HIGH); //turn vacuum pump off
    digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    if (power_count2 >= 40){ //allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }
  
  if (vacuum >= V_HI){ //if vacuum is above limit perform the following...
    digitalWrite(vacuum_power, HIGH); //turn vacuum pump off
    digitalWrite(vacuum_relief, LOW); //open vacuum relief valve
    delay(3); //delay 3 ms
    digitalWrite(vacuum_relief, HIGH); //close vacuum relief valve
    delay(3); //delay 3 ms
    if (power_count2 >= 40){//allows us to always wait 2s between the pump being on and off
      power_count2 = 0; //reset power counter 2
    }
  }
  power_count2 = power_count2 + 1; //limit timer for how long the pump can be on
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////HEATING STATES//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////HEATING//////////////////////////////////////////////////////////////////////////////////////
if (active_heat == 1){
  if (PID_count <=20){ //PID loop/Duty Cycle based on 20 timed loops
      //calculate PID values when PID loop/Duty Cycle 1st begins
      if (PID_count == 0){ 
        error = input_temp - temp; //error on slide
        errorW = input_temp - temp_well; //error on sample well
        P = kp*error; //P Value for slide
        PW = kpW*errorW; //P Value for sample well
        I = I + (ki*error); //I Value for slide
        IW = IW + (kiW*errorW); //I Value for sample well
        PID_start = PID_time; //start time for derivative value assigned as last time value
        PID_time = millis(); //current time value for derivative
        PID_elapsed = (PID_time - PID_start) / 1000; //time elapsed for derivative calculation
        D = kd*((error - prev_error)/PID_elapsed); //D value for slide
        DW = kdW*((errorW - prev_errorW)/PID_elapsed); //D value for sample well
        PID = P + I + D; //PID value for slide
        PIDW = PW + IW + DW; //PID value for sample well
        
        //round off min/max PID values for simplified duty cycle output
        if(PID <= 100){
          PID = 0;    
        }
        if((PID>100) and (PID<200)){
          PID = 200;    
        }
        if((PID>=200) and (PID<=1800)){
          PID = PID;    
        }
        if((PID>1800) and (PID < 1900)){
          PID = 1800;    
        }
        if(PID >= 1900){
          PID = 2000;  
        }
        if(PIDW <= 100){
          PIDW = 0;    
        }
        if((PIDW>100) and (PIDW<200)){
          PIDW = 200;    
        }
        if((PIDW>=200) and (PIDW<=1800)){
          PIDW = PIDW;    
        }
        if((PIDW>1800) and (PIDW < 1900)){
          PIDW = 1800;    
        }
        if(PIDW >= 1900){
          PIDW = 2000;  
        }
        
        prev_error = error; //save previous error from slide
        prev_errorW = errorW; //save previous error from waste well
        heat_on = 2000-PID; //calculates how long slide heating will be per cycle
        heat_onW = 2000-PIDW; //calculates how long well heating will be per cycle
        count_heat = heat_on/100; //gives number of loops the heat will remain on slide
        count_heatW = heat_onW/100; //gives number of loops the heat will remain on sample well
      }
      
        //DUTY CYCLE for HEATING SLIDE
        if (PID_count == count_heat){
          if (PID_count == 0){
            digitalWrite(heat_relay, LOW);
            digitalWrite(cool_relay, HIGH);
          }
          else if (PID_count == 20){
            digitalWrite(heat_relay, HIGH);
            digitalWrite(cool_relay, HIGH);
          }
          else{
            digitalWrite(heat_relay, LOW);
            digitalWrite(cool_relay, HIGH);
          }
        }
        else if (PID_count < count_heat){
          digitalWrite(heat_relay, HIGH);
          digitalWrite(cool_relay, HIGH);
        }
        else if (PID_count > count_heat){
          digitalWrite(heat_relay, LOW);
          digitalWrite(cool_relay, HIGH);
        }

        //DUTY CYCLE for HEATING SAMPLE WELL
        if (PID_count == count_heatW){
          if (PID_count == 0){
            digitalWrite(res_relay, LOW);
          }
          else if (PID_count == 20){
            digitalWrite(res_relay, HIGH);
          }
          else{
            digitalWrite(res_relay, LOW);
          }
        }
        else if (PID_count < count_heatW){
          digitalWrite(res_relay, HIGH);
        }
        else if (PID_count > count_heatW){
          digitalWrite(res_relay, LOW);
        }
        
      //progress through duty cycle
      PID_count = PID_count + 1;
    }

    //Set limits for TEC temp
    if(temp_TEC >55){ //if TEC temp is above 55C turn TEC off to cool
      digitalWrite(heat_relay, HIGH);
      digitalWrite(cool_relay, HIGH);
    }

    //Set limits for resistor temp
    if(temp_res >60){ //if resistors temp is above 60C turn resistors off to cool
      digitalWrite(res_relay, HIGH);
    }
}

//////////////////////////////////////////////////////////////////////////////////COOLING////////////////////////////////////////////////////////////////////////////////
if (active_heat == 2){
  if (PID_count <=20){ //PID loop/Duty Cycle based on 20 timed loops
    //calculate PID values when PID loop/Duty Cycle 1st begins
    if (PID_count == 0){ 
      errorC = temp - input_temp; //error on slide (cooling)
      PC = kpC*errorC; //P Value for slide (cooling)
      IC = IC + (kiC*errorC); //I value for slide (cooling)
      PID_start = PID_time; //start time for derivative value assigned as last time value 
      PID_time = millis(); //current time value for derivative
      PID_elapsed = (PID_time - PID_start) / 1000; //time elapsed for derivative calculation
      DC = kdC*((errorC - prev_errorC)/PID_elapsed); //D value for slide (cooling)
      PIDC = PC + IC + DC; //PID value for slide (cooling)

      //round off min/max PID values for simplified duty cycle output
      if(PIDC <= 100){
        PIDC = 0;    
      }
      if((PIDC>100) and (PIDC<200)){
        PIDC = 200;    
      }
      if((PIDC>=200) and (PIDC<=1800)){
        PIDC = PIDC;    
      }
      if((PIDC>1800) and (PIDC < 1900)){
        PIDC = 1800;    
      }
      if(PIDC >= 1900){
        PIDC = 2000;  
      }
      prev_errorC = errorC; //save previous error for cooling
      heat_onC = 2000-PIDC; //calculates how long slide cooling will be per cycle
      count_heatC = heat_onC/100; //gives number of loops cooling will remain on slide
    }

    //DUTY CYCLE for COOLING SLIDE
    if (PID_count == count_heatC){
      if (PID_count == 0){
        digitalWrite(cool_relay, LOW);
        digitalWrite(heat_relay, HIGH);
        digitalWrite(res_relay, HIGH);
      }
      else if (PID_count == 20){
        digitalWrite(cool_relay, HIGH);
        digitalWrite(heat_relay, HIGH);
        digitalWrite(res_relay, HIGH);
      }
      else{
        digitalWrite(cool_relay, LOW);
        digitalWrite(heat_relay, HIGH);
        digitalWrite(res_relay, HIGH);
      }
    }
    else if (PID_count < count_heatC){
      digitalWrite(cool_relay, HIGH);
      digitalWrite(heat_relay, HIGH);
      digitalWrite(res_relay, HIGH);
    }
    else if (PID_count > count_heatC){
      digitalWrite(cool_relay, LOW);
      digitalWrite(heat_relay, HIGH);
      digitalWrite(res_relay, HIGH);
    }
   
    PID_count = PID_count + 1; //step through duty cycle
  }
}

  //PID Duty Cycle Limits for Heating/ Cooling
  if (PID_count >20){ //duty cycle is out of 20 steps...if duty cycle reaches 20 steps reset to begin the PID loop again
    PID_count = 0; //resets PID duty cycle
  }
  
  while (looptimer < 125){ //go into this loop if the loop timer is ever under 125 ms
    looptimer = millis()-start_loop; //keeps track of loop time until it reaches 125 ms
  }
  start_loop = millis(); //start loop timer over after reaching 125 ms
}
