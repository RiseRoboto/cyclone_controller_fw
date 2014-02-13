//DV, Cyclone code 2/14/14
//tested motor shield/position loop on cyclone
//WIP for hakathon on 2/15
//remove: filter current signal scratch
//add limit switch functionality
//sensible comands, hand typable
//gui readbacks disablable
//analog feedback mode
//pwm input mode
//manual/auto mode
//current limits
#include <FreeRTOS_AVR.h>
#include <SerialCommand.h>
#include <MegaEncoderCounter.h>
#include <Motor.h>
#include <Servo.h>
#include <PID_v1.h>
#include <PID_micro.h>
#include <firFilter.h>
//--------------------------------------------
//Config, uncomment based on driver used
//TalonSR pwm driver
//#define USE_TALON
//Arduino 2A motor shield (L298), pwm
#define USE_MOTORSHIELD

//--------------------------------------------
//globals and constants
const uint8_t TEST_PIN = 40;  //debugging/timinin
SerialCommand SCmd;  //serial terminal

//motor control and current measurements
#ifdef USE_MOTORSHIELD
//motor shield radioshack L298, arduino official v3.0
Motor motorA = Motor(12, 3);  //chA dir pin, pwm pin
Motor motorB = Motor(13, 11);
#endif

#define PWM_MIN -255
#define PWM_MAX 255
#define PWM_IDLE 10
#ifdef USE_TALON
//talonSR driver, pwm in
#define PWM_MICROS_MINSET 1000
#define PWM_MICROS_MAXSET 2000
Servo talonServoA, talonServoB;
const uint8_t TALON_A_OUT_PIN = 5;
const uint8_t TALON_B_OUT_PIN = 6;
#endif

#define PIN_AMPS_CH_A 0
#define PIN_AMPS_CH_B 1
double ChA_Current = 0.0, ChB_Current = 0.0;
firFilter FilterA, FilterB;

//PID position controller, links and initial tuning parameters
MegaEncoderCounter encoder(4); // Initializes the Mega Encoder Counter in the 4X Count Mode
long enc_pos_A = 0, enc_pos_B = 0;

#define Pinit 0.1
#define Iinit 0.0
#define Dinit 0.0
#define CMD_MIN -255
#define CMD_MAX 255
#define POS_MAX 4500.0
#define POS_MIN 0.0

double analog_spA = 0.0, analog_spB = 0.0;
firFilter SP_FilterA;
double Apos_pid_sp = 0.0, Apos_pid_in = 0.0, Apos_pid_out = 0.0;
double Bpos_pid_sp = 0.0, Bpos_pid_in = 0.0, Bpos_pid_out = 0.0;
int motorA_cmd = 0, motorB_cmd = 0;
int motorA_cmd_prev = 0, motorB_cmd_prev = 0;
PID pos_controllerA(&Apos_pid_in, &Apos_pid_out, &Apos_pid_sp,Pinit,Iinit,Dinit, DIRECT);
PID pos_controllerB(&Bpos_pid_in, &Bpos_pid_out, &Bpos_pid_sp,Pinit,Iinit,Dinit, DIRECT);

//Analog IN
#define PIN_ANA_IN_A 3
#define PIN_ANA_IN_B 4

//FreeRTOS periods
#define SAMPLE_PERIOD 25L//50L // 2L min measured at 600us 10L=8.73m  5L=3.6ms
#define PUBLISH_PERIOD 250L //1000L
#define SAFETY_PERIOD 50L
#define POSITION_PERIOD 100L
#define CURRENT_PERIOD 5L//2L
//--------------------------------------------
//Arduino functions
void setup() {
  
  #ifdef USE_TALON
  Serial.println("servo init");
  talonServoA.attach(3);//TALON_A_OUT_PIN); 
  talonServoA.writeMicroseconds(map(0,-255,255,1000,2000)); 
  talonServoB.attach(TALON_B_OUT_PIN); 
  talonServoB.writeMicroseconds(map(0,-255,255,1000,2000)); 
  #endif  

  #ifdef USE_MOTORSHIELD //<-fast pwm setting
  TCCR3B = TCCR3B & 0b11111000 | 0x01;  //31kHz p0m on pin3
  #endif
  //current filters
  FilterA.begin();
  FilterB.begin();
  SP_FilterA.begin();
  encoder.switchCountMode(1);
  encoder.XAxisReset( );
  encoder.YAxisReset( );
  
  pos_controllerA.SetSampleTime(POSITION_PERIOD);
  pos_controllerA.SetControllerDirection(DIRECT);//(DIRECT);//(REVERSE);
  pos_controllerA.SetOutputLimits(CMD_MIN,CMD_MAX); //(-255, 255);
  pos_controllerA.SetMode(AUTOMATIC);   //turn the PID on

  pos_controllerB.SetSampleTime(POSITION_PERIOD);
  pos_controllerB.SetControllerDirection(DIRECT);//(DIRECT);//(REVERSE);
  pos_controllerB.SetOutputLimits(CMD_MIN,CMD_MAX); //(-255, 255);
  pos_controllerB.SetMode(MANUAL);

  Serial.begin(115200);        //repmotlace with serial setup
  while(!Serial) {}
  SCmd.addCommand("h", motor_halt);   
  SCmd.addCommand("gui", parse_gui_input);
  SCmd.addCommand("pidset", parse_pid_set);
  SCmd.setDefaultHandler(unrecognized);    // Handler for command that isn't matched  (says "What?") 

  xTaskCreate(vMotorTask,
    (signed portCHAR *)"motor_task",
    configMINIMAL_STACK_SIZE + 50,
    NULL,
    tskIDLE_PRIORITY + 4,
    NULL);
    
  xTaskCreate(vPositionLoopTask,
    (signed portCHAR *)"position_task",
    configMINIMAL_STACK_SIZE + 50,
    NULL,
    tskIDLE_PRIORITY + 2,
    NULL);

  // create print task
  xTaskCreate(vPrintTask,
    (signed portCHAR *)"print_task",
    configMINIMAL_STACK_SIZE + 100,
    NULL,
    tskIDLE_PRIORITY + 1,
    NULL);

  // start FreeRTOS
  vTaskStartScheduler();

  // should never return
  Serial.println(F("Die"));
  while(1);
}

void loop() {
  //freeRTOS tasks start in setup(), loop never reached
}


//--------------------------------------------
//FreeRtos tasks


//Print Task
static void vPrintTask(void *pvParameters) {
  while (true) {
    SCmd.readSerial();     // process serial commands    
    pid_gui_response();
    vTaskDelay((PUBLISH_PERIOD * configTICK_RATE_HZ) / 1000L);
  }
}

static void vPositionLoopTask(void *pvParameters) {
  while(true) {
    //get setpoint from analog in
    analog_spA = (double)analogRead(PIN_ANA_IN_A);
    analog_spA = constrain(analog_spA, 512, 1024);
    Apos_pid_sp = map(analog_spA,512,1024, POS_MIN, POS_MAX);    
    //analog_spB = (double)analogRead(PIN_ANA_IN_B);
    //analog_spB = constrain(analog_sp, 512, 1024);
//    analog_spB = SP_FilterA.run(analog_spA);
    Bpos_pid_sp = map(analog_spA,512,1024, POS_MIN, POS_MAX);  
    
    enc_pos_A = encoder.XAxisGetCount();
    enc_pos_B = encoder.YAxisGetCount();
    Apos_pid_in = double(enc_pos_A);
    Bpos_pid_in = double(enc_pos_B);
    pos_controllerA.Compute();
    pos_controllerB.Compute();

    vTaskDelay((POSITION_PERIOD * configTICK_RATE_HZ) / 1000L);
  }
}

//reads currents,checks faults,controls driver command
static void vMotorTask(void *pvParameters) {
  while(true) {    
    if(pos_controllerA.GetMode() != MANUAL)  motorA_cmd = map(int(Apos_pid_out),-255.0,255.0,-250.0,250.0); //CMD_MIN
    else  motorA_cmd = 0;
    if(pos_controllerB.GetMode() != MANUAL)  motorB_cmd = int(Bpos_pid_out);
    else  motorB_cmd = 0;
    read_current();
    motorA_cmd = constrain(motorA_cmd, PWM_MIN, PWM_MAX);
    motorB_cmd = constrain(motorB_cmd, PWM_MIN, PWM_MAX);

    set_motor();
    vTaskDelay((CURRENT_PERIOD * configTICK_RATE_HZ) / 1000L);   
  }
}

void set_motor() 
{
    #ifdef USE_MOTORSHIELD
    if(motorA_cmd !=  motorA_cmd_prev) { motorA.go(motorA_cmd); motorA_cmd_prev = motorA_cmd; }
    if(motorB_cmd !=  motorB_cmd_prev) { motorB.go(motorB_cmd); motorB_cmd_prev = motorB_cmd; } 

    if((motorA_cmd > -PWM_IDLE) && (motorA_cmd < PWM_IDLE)) { motorA_cmd = 0;  motorA.stop(); }
    if((motorB_cmd > -PWM_IDLE) && (motorB_cmd < PWM_IDLE)) { motorB_cmd = 0;  motorB.stop(); }
    #endif
    #ifdef USE_TALON
    //-255,255,1000,2000
      talonServoA.writeMicroseconds(map(motorA_cmd/2,CMD_MIN,CMD_MAX,PWM_MICROS_MINSET,PWM_MICROS_MAXSET)); 
    #endif  
}

//--------------------------------------------
//current conversion
//ACS712 20A current sensor, ADC in, 2.5V zero offset, 10A/V
//Imeas = (Vmeas - Vzero) * ISCALE = (Vmeas - 2.5)*10 = 10 * Vmeas - 25.0
//Imeas = ICONVERT * Vmeas - IOFFSET
//1024*0.100/5=20.48=20 counts/A
#define VREF 5.0
#define MAX_ADC_COUNT 1024
#define ISCALE20A 10000.0//10.0  //ACS712 20A sensor 0.1V/A or 10A/V
#define ISCALE5A 5405.0//5.405  //ACS712 5A sensor 0.185V/A or 5.405A/V
#define ISCALE2A 606.0  //L298 Arduino motorshiel 1.65V/A or 0.606A/V
#define ICONV20A ISCALE20A*VREF/MAX_ADC_COUNT
#define ICONV5A ISCALE5A*VREF/MAX_ADC_COUNT
#define ICONV2A ISCALE2A*VREF/MAX_ADC_COUNT
#define IOFFSET20A ISCALE20A*2.5
#define IOFFSET5A ISCALE5A*2.5

void read_current() {
    ChA_Current = (double)analogRead(PIN_AMPS_CH_A);
    ChB_Current = (double)analogRead(PIN_AMPS_CH_B);
    #ifdef USE_TALON
    ChA_Current = convert_ADC2AMPS_5A(ChA_Current);
    ChB_Current = convert_ADC2AMPS_5A(ChB_Current);
    #endif
    #ifdef USE_MOTORSHIELD
    ChA_Current = convert_ADC2AMPS_2A(ChA_Current);
    ChB_Current = convert_ADC2AMPS_2A(ChB_Current);
    #endif
    
    ChB_Current = FilterA.run(ChA_Current);
    //ChA_Current = FilterA.run(ChA_Current);
    //ChB_Current = FilterB.run(ChA_Current);
}

double convert_ADC2AMPS_5A(double count) {
    double amps;
    return amps = count * ICONV5A - IOFFSET5A;
}
double convert_ADC2AMPS_20A(double count) {
    double amps;
    return amps = count * ICONV20A - IOFFSET20A;
}
double convert_ADC2AMPS_2A(double count) {
    double amps;
    return amps = count * ICONV2A; // - IOFFSET20A;
}
//Serial callbacks -------------------------------------
void unrecognized(const char *command)
{
  Serial.println("What?"); 
}

void motor_halt()
{
  pos_controllerA.SetMode(MANUAL);
  pos_controllerB.SetMode(MANUAL);
  motorA_cmd = 0;
  motorB_cmd = 0;
  set_motor();
}

void pid_gui_response()
{
  Serial.print("PID ");
//1
  Serial.print(enc_pos_A);  
  Serial.print(" ");
//2  
  Serial.print(ChA_Current);
  Serial.print(" ");
  //3
  Serial.print(ChB_Current);
  Serial.print(" ");
  //4
  Serial.print(pos_controllerA.GetKp());   
  Serial.print(" ");
  //5
  Serial.print(pos_controllerA.GetKi());   
  Serial.print(" ");
  //6
  Serial.print(pos_controllerA.GetKd());   
  Serial.print(" ");
  //7
  if(pos_controllerA.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  //8
  if(pos_controllerA.GetDirection()==DIRECT) Serial.print("Direct");
  else Serial.print("Reverse");
  Serial.print(" ");
  //9
  Serial.print(Apos_pid_sp);
  Serial.print(" ");
  //10
  Serial.println(motorA_cmd);
  //Serial.println(Apos_pid_out);

}

void parse_gui_input()
{
  int aut_man, dir_rev;
  double p, i, d;
  char *arg; 
  arg = SCmd.next();
  if (arg != NULL)
  {
    aut_man = atoi(arg);
    arg = SCmd.next();
    if (arg != NULL)
    {
      dir_rev = atoi(arg);
      arg = SCmd.next();
      if (arg != NULL)
      {
        //pos_pid_sp = map(atof(arg),-7500,7500,-3000,3000);
        //current_pid_sp = map(atof(arg),-7500,7500,-2000,2000);
        arg = SCmd.next();
        if (arg != NULL)
        {
          //pos_pid_in = atof(arg);    //whay?
          arg = SCmd.next();
          if (arg != NULL)
          {
            //pos_pid_out = atof(arg); //whay?
            arg = SCmd.next();
            if (arg != NULL)
            {
              p = atof(arg);
              arg = SCmd.next();
              if (arg != NULL)
              {
                i = atof(arg);
                arg = SCmd.next();
                if (arg != NULL)
                {
                  d = atof(arg);
                  pos_controllerA.SetTunings(p, i, d); 
                  pos_controllerB.SetTunings(p, i, d); 
                  if(aut_man==0) { pos_controllerA.SetMode(MANUAL);// * set the controller mode
                                   pos_controllerB.SetMode(MANUAL); }
                  else { pos_controllerA.SetMode(AUTOMATIC);
                         pos_controllerB.SetMode(AUTOMATIC); }
                  if(dir_rev==0) { pos_controllerA.SetControllerDirection(DIRECT);// * set the controller Direction
                                   pos_controllerB.SetControllerDirection(DIRECT); }
                  else { pos_controllerA.SetControllerDirection(REVERSE);
                         pos_controllerA.SetControllerDirection(REVERSE);  }
                  //Serial.println("setted?");
                  pid_gui_response();
                } //else Serial.println("er d");
              } //else Serial.println("er i");
            } //else Serial.println("er p");
          } //else Serial.println("er out");
        } //else Serial.println("er in");
      } //else Serial.println("er sp");
    } //else Serial.println("er dir");
  } //else Serial.println("er aut");
}

void parse_pid_set() {
  double p, i, d;
  char *arg; 
  arg = SCmd.next();
  if (arg != NULL)
  {
    p = atof(arg);
    arg = SCmd.next();
    if (arg != NULL)
    {
      i = atof(arg);
      arg = SCmd.next();
      if (arg != NULL)
      {
        d = atof(arg);
        pos_controllerA.SetTunings(p, i, d); 
        pos_controllerB.SetTunings(p, i, d); 
        pid_gui_response();
      } //else Serial.println("er d");
    } //else Serial.println("er i");
  } //else Serial.println("er p");
}

