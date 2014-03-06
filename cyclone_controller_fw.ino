//DV, Cyclone code 2/15/15 modified during hackahon. 
//put in homing, analog in, serial setpoint cmd "sp xxxx" 
//tested motor shield/position loop on cyclone, 10A cytrone motorshield
//robogaia+USdigital incr enc
//WIP for hakathon on 2/15
//remove: filter current signal scratch. DV 2/15 done
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
#include <MedianFilter.h>
#include <TimerThree.h>
#include <TimerFour.h>
//--------------------------------------------
//Config, uncomment based on driver used

//TalonSR pwm driver
// #define USE_TALON

//Arduino 4A motor shield (L298), pwm
//#define USE_MOTORSHIELD_4A

//10 amp driver
#define USE_MOTORSHIELD_10A

//#define USE_ANALOG_INPUT
//#define USE_SERIAL_INPUT

#define USE_PWM_INPUT
//--------------------------------------------
//globals and constants
const uint8_t TEST_PIN = 14;  //debugging/timinin
SerialCommand SCmd;  //serial terminal

//motor control and current measurements
#ifdef USE_MOTORSHIELD_4A
#define MOTOR_POLARITY 1
#define CHA_DIR 12
#define CHA_PWM 3
#define CHB_DIR 13
#define CHB_PWM 11
//motor shield radioshack L298, arduino official v3.0
Motor motorA = Motor(CHA_DIR, CHA_PWM);  //chA dir pin, pwm pin
Motor motorB = Motor(CHB_DIR, CHB_PWM);
#endif


#ifdef USE_MOTORSHIELD_10A
#define MOTOR_POLARITY -1
#define CHA_DIR 12//12
#define CHA_PWM 3//11//3
#define CHB_DIR 13
#define CHB_PWM 11
//Cytron MD10 10A motor shield 
Motor motorA = Motor(CHA_DIR, CHA_PWM);
Motor motorB = Motor(CHB_DIR, CHB_PWM);
#endif


#define PWM_MIN -255
#define PWM_MAX 255
#define PWM_IDLE 10
#define PWM_MICROS_MINSET 1000
#define PWM_MICROS_MAXSET 2000
#ifdef USE_TALON
//talonSR driver, pwm in
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
#define ENCODER_POLARITY -1

#define PIN_PWM_IN_A 2
#define PIN_ANA_IN_A 8

//End Condition Switches
#define PIN_HOME_SW 7
#define AT_HOME 1 //1 for active hi sw, 0 for active lo
int home_sw_state = 0;
int homing_blank_delay = 0;
//motor states
#define INIT_STATE 0
#define HALT_STATE 1
#define HOME_STATE 2
#define RUN_STATE 3
int State_Motor_A = INIT_STATE, State_Motor_B = INIT_STATE;
#define HOME_SPD_CMD 25


#define Pinit 0.2//FASTNSHAKY 0.3 //STABLER 0.1
#define Iinit 0.1//FASTNSHAKY 0.1 //STABLER 0.05
#define Dinit 0.0
#define CMD_MIN -64 //-127 //-255
#define CMD_MAX 64 //127 //255
#define POS_MAX 0.0
#define POS_MIN -1350//4xmode-4500// 1xmode -1350.0
//#define POS_MAX 4500.0
//#define POS_MIN 0.0


double analog_spA = 0.0, analog_spB = 0.0;
double serial_spA = 0.0, serial_spB = 0.0;
firFilter SP_FilterA;
MedianFilter SP_FilterB;
double Apos_pid_sp = 0.0, Apos_pid_in = 0.0, Apos_pid_out = 0.0;
double Bpos_pid_sp = 0.0, Bpos_pid_in = 0.0, Bpos_pid_out = 0.0;
int motorA_cmd = 0, motorB_cmd = 0;
int motorA_cmd_prev = 0, motorB_cmd_prev = 0;
PID pos_controllerA(&Apos_pid_in, &Apos_pid_out, &Apos_pid_sp,Pinit,Iinit,Dinit, DIRECT);
PID pos_controllerB(&Bpos_pid_in, &Bpos_pid_out, &Bpos_pid_sp,Pinit,Iinit,Dinit, DIRECT);

//Analog IN
//#ifdef USE_MOTORSHIELD_4A
//#define PIN_ANA_IN_A 3
//#define PIN_ANA_IN_B 4
//#endif

//#ifdef USE_MOTORSHIELD_10A
#define PIN_ANA_IN_A 8
#define PIN_ANA_IN_B 9
//#endif

//FreeRTOS periods
#define SAMPLE_PERIOD 5L//25L//5L//50L // 2L min measured at 600us 10L=8.73m  5L=3.6ms
#define PUBLISH_PERIOD 25L//250L//25L//50L //1000L
#define SAFETY_PERIOD 50L//50LN
#define POSITION_PERIOD 5L//100L//5L
#define CURRENT_PERIOD 5L//10L//5L//2L
//--------------------------------------------
//Arduino functions
void setup() {
  pinMode(TEST_PIN, OUTPUT);
  digitalWrite(TEST_PIN, LOW);
  pinMode(PIN_HOME_SW, INPUT);
  pinMode(PIN_PWM_IN_A, INPUT);
  attachInterrupt(0, pwm_input_ISR, CHANGE);
  
  #ifdef USE_TALON
  talonServoA.attach(3);//TALON_A_OUT_PIN); 
  talonServoA.writeMicroseconds(map(0,-255,255,1000,2000)); 
  talonServoB.attach(TALON_B_OUT_PIN); 
  talonServoB.writeMicroseconds(map(0,-255,255,1000,2000)); 
  #endif  

  #ifdef USE_MOTORSHIELD_4A //<-fast pwm setting
  pinMode(CHA_DIR, OUTPUT);   pinMode(CHA_PWM, OUTPUT);
  pinMode(CHB_DIR, OUTPUT);   pinMode(CHB_PWM, OUTPUT);
  TCCR3B = TCCR3B & 0b11111000 | 0x001;  //0x001;no prescale, 31kHz pwm on pin3
  #endif
  #ifdef USE_MOTORSHIELD_10A
  pinMode(CHA_DIR, OUTPUT);   pinMode(CHA_PWM, OUTPUT);
  pinMode(CHB_DIR, OUTPUT);   pinMode(CHB_PWM, OUTPUT);
  TCCR3B = TCCR3B & 0b11111000 | 0x001;    //prescale:8, 4kHz pwm on pin3, driver input max 10kHz
  #endif
  //current filters
  FilterA.begin();
  FilterB.begin();
  SP_FilterA.begin();
  encoder.switchCountMode(1);
  encoder.XAxisReset( );
  encoder.YAxisReset( );
  
  #ifdef USE_PWM_INPUT
    timer_four_setup();
  #endif
  
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
  SCmd.addCommand("sp", parse_sp);
  SCmd.setDefaultHandler(unrecognized);    // Handler for command that isn't matched  (says "What?") 

//  xTaskCreate(vPWMinTask,
//    (signed portCHAR *)"pwm_in_task",
//    configMINIMAL_STACK_SIZE + 50,
//    NULL,
//    tskIDLE_PRIORITY + 1,
//    NULL);
    
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
    tskIDLE_PRIORITY + 3,
    NULL);

  // create print task
  xTaskCreate(vPrintTask,
    (signed portCHAR *)"print_task",
    configMINIMAL_STACK_SIZE + 100,
    NULL,
    tskIDLE_PRIORITY + 5, //2,
    NULL);

//  Serial.println("starting rtos");

  // start FreeRTOS
  vTaskStartScheduler();

  // should never return
  Serial.println(F("Die"));
  while(1);
}

void loop() {
  //freeRTOS tasks start in setup(), loop never reached
}

void timer_four_setup()
{
  //TCCR3B = TCCR3B & 0b11111000 | 0x01;  //31kHz p0m on pin3
  //attachInterrupt(3, pwmISR, CHANGE);
  //interrupts();
  Timer4.initialize(20000);  //microseconds
  Timer4.start();
  
  //pwm setup using Timer 3
  //  Timer3.pwm(3, 250); //,9000); //5, 255); 
  //  motor1.go(100);
  //Timer3.attachInterrupt(PWMcallback);
  //  Timer3.setPwmDuty(3, 50);  

}

//Pulse input ISR, tried Pulse in reading in isr
volatile int time_rise_edge = 0, time_pulse = 0;
void pwm_input_ISR() {
//  unsigned long timestamp;
 int inputPin, pinstate = 0;
//  digitalWrite(TEST_PIN, HIGH);
//  timestamp = micros();//TCNT1;
  inputPin = digitalRead(PIN_PWM_IN_A);
  //noInterrupts();   
  if(inputPin == HIGH)
  {
    //digitalWrite(TEST_PIN, HIGH);
//    pinstate = 1;
//    time_rise_edge = timestamp;
    Timer4.restart();
  }  
  if(inputPin == LOW)
  {
//    pinstate = 0; 
    //time_pulse = time_rise_edge - timestamp;
    //time_pulse = timestamp;
    time_pulse = TCNT4;
    //digitalWrite(TEST_PIN, LOW);
  }
  //interrupts();   
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

int time_pwm_input = 0;
static void vPWMinTask(void *pvParameters) {
    while (true) {
//      digitalWrite(TEST_PIN, HIGH);
//      if(digitalRead(PIN_PWM_IN_A) == 0 ) 
//      {
//        time_pwm_input = pulseIn(PIN_PWM_IN_A, HIGH,10000);
//      }
//      digitalWrite(TEST_PIN, LOW);
      vTaskDelay((SAMPLE_PERIOD * configTICK_RATE_HZ) / 1000L);
  }
}

static void vPositionLoopTask(void *pvParameters) {
  while(true) {
    #ifdef USE_ANALOG_INPUT
    //get setpoint from analog in
    analog_spA = (double)analogRead(PIN_ANA_IN_A);
    analog_spA = constrain(analog_spA, 512, 1024);
//    Apos_pid_sp =  map(analog_spA,512,1024, POS_MIN, POS_MAX);    
    Apos_pid_sp = -1 * map(analog_spA,0,512, POS_MIN, POS_MAX);  
    //analog_spB = (double)analogRead(PIN_ANA_IN_B);
    //analog_spB = constrain(analog_sp, 512, 1024);
//    analog_spB = SP_FilterA.run(analog_spA);
    Bpos_pid_sp = map(analog_spA,512,1024, POS_MIN, POS_MAX);  
    #endif

    #ifdef USE_SERIAL_INPUT
    //Apos_pid_sp = map(serial_spA,-4000,0, POS_MIN, POS_MAX);  
    Apos_pid_sp = constrain(serial_spA, POS_MIN, POS_MAX);
    #endif
    
    #ifdef USE_PWM_INPUT
//    if((time_pwm_input < PWM_MICROS_MINSET) || (time_pwm_input > PWM_MICROS_MAXSET))
//    {
//      //zero?
//    }
//    else 
//    {
//    if(time_pwm_input != 0)
//    {
      Apos_pid_sp = map(time_pulse, 5000, 3000, -200, -1200);
      //Apos_pid_sp = map(time_pwm_input, PWM_MICROS_MINSET, PWM_MICROS_MAXSET, -200, -1200);
      Bpos_pid_sp = SP_FilterB.run(Apos_pid_sp);
//      //Apos_pid_sp = Bpos_pid_sp;
//    }
    #endif
    
    enc_pos_A = ENCODER_POLARITY*encoder.XAxisGetCount();
    enc_pos_B = ENCODER_POLARITY*encoder.YAxisGetCount();
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
    digitalWrite(TEST_PIN, HIGH);
    state_machine_advance();
    switch ( State_Motor_A )
    {
      case INIT_STATE:
        motorA_cmd = 0;
        motorB_cmd = 0; 
        break;
      case HOME_STATE:
        if(homing_blank_delay <= 0)
        { 
          if(pos_controllerA.GetMode() != MANUAL)  motorA_cmd = HOME_SPD_CMD;
          else  motorA_cmd = 0;
          if(pos_controllerB.GetMode() != MANUAL)  motorB_cmd = HOME_SPD_CMD;
          else  motorB_cmd = 0;
          read_current();
          motorA_cmd = constrain(motorA_cmd, PWM_MIN, PWM_MAX);
          motorB_cmd = constrain(motorB_cmd, PWM_MIN, PWM_MAX);
        }
        break;
      case RUN_STATE:
        if(pos_controllerA.GetMode() != MANUAL)  motorA_cmd = map(int(Apos_pid_out),-255.0,255.0,-250.0,250.0); //CMD_MIN
        else  motorA_cmd = 0;
        if(pos_controllerB.GetMode() != MANUAL)  motorB_cmd = int(Bpos_pid_out);
        else  motorB_cmd = 0;
        read_current();
        motorA_cmd = constrain(motorA_cmd, PWM_MIN, PWM_MAX);
        motorB_cmd = constrain(motorB_cmd, PWM_MIN, PWM_MAX);

        break;
      default:
      case HALT_STATE:
        motorA_cmd = 0;
        motorB_cmd = 0; 
        break;
    }
    set_motor();
//    digitalWrite(TEST_PIN, LOW);
    vTaskDelay((CURRENT_PERIOD * configTICK_RATE_HZ) / 1000L);   
  }
}

void state_machine_advance() {
    home_sw_state = digitalRead(PIN_HOME_SW);
    switch ( State_Motor_A )
    {
      case INIT_STATE: 
        State_Motor_A = HOME_STATE;
        homing_blank_delay = 0;
        break;
      case HOME_STATE:
        if (home_sw_state == AT_HOME) 
        {
            encoder.XAxisReset( );
            encoder.YAxisReset( );
            motorA_cmd = 0;
            motorB_cmd = 0;
            Apos_pid_sp = 0;
            Apos_pid_out = 0;
            pos_controllerA.SetMode(MANUAL); 
            set_motor(); 
            pos_controllerA.Compute();
            pos_controllerB.Compute();

            //delay(100);
            if(homing_blank_delay < 500)  homing_blank_delay++;
            else State_Motor_A = RUN_STATE;
            pos_controllerA.SetMode(AUTOMATIC); 
        }
        break;
      case RUN_STATE:
        if (detect_faults()) State_Motor_A = HALT_STATE;
        break;
      default: 
      case HALT_STATE:
        break;
    } 

}

int home_limit_triggered = 0;
int detect_faults()
{
  if( (State_Motor_A != HOME_STATE) && (home_sw_state == AT_HOME)  && (home_limit_triggered))
  {
    return 1;
  }
  else return 0;
}

void set_motor() 
{
    #ifdef USE_MOTORSHIELD_4A
    if(motorA_cmd !=  motorA_cmd_prev) { motorA.go(MOTOR_POLARITY*motorA_cmd); motorA_cmd_prev = motorA_cmd; }
    if(motorB_cmd !=  motorB_cmd_prev) { motorB.go(MOTOR_POLARITY*motorB_cmd); motorB_cmd_prev = motorB_cmd; } 

    if((motorA_cmd > -PWM_IDLE) && (motorA_cmd < PWM_IDLE)) { motorA_cmd = 0;  motorA.stop(); }
    if((motorB_cmd > -PWM_IDLE) && (motorB_cmd < PWM_IDLE)) { motorB_cmd = 0;  motorB.stop(); }
    #endif

    #ifdef USE_MOTORSHIELD_10A
    if(motorA_cmd !=  motorA_cmd_prev) { motorA.go(MOTOR_POLARITY*motorA_cmd); motorA_cmd_prev = motorA_cmd; }
    if(motorB_cmd !=  motorB_cmd_prev) { motorB.go(MOTOR_POLARITY*motorB_cmd); motorB_cmd_prev = motorB_cmd; } 

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
    #ifdef USE_MOTORSHIELD_4A
    ChA_Current = convert_ADC2AMPS_2A(ChA_Current);
    ChB_Current = convert_ADC2AMPS_2A(ChB_Current);
    #endif
    #ifdef USE_MOTORSHIELD_10A  //no cur sense support
    ChA_Current = 0; 
    ChB_Current = 0;
    #endif

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
  Serial.print(Apos_pid_sp);
//  Serial.print(ChA_Current);
//  Serial.print(Bpos_pid_sp);
//  Serial.print(time_rise_edge);
  Serial.print(" ");
  //3
  //Serial.print(ChB_Current);
//  Serial.print(Apos_pid_out);
  //Serial.print(Bpos_pid_sp);
  //Serial.print(time_pwm_input);//
  //Serial.print(time_rise_edge); //
  Serial.print(time_pulse);
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
  //Serial.print(State_Motor_A); Serial.print(" ");
  //Serial.println(homing_blank_delay);
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
        #ifdef USE_SERIAL_INPUT
        //serial_spA = map(atof(arg),-7500,7500,0,-3000);
        //serial_spA = map(atof(arg),-4000,0,-4000,0);
        serial_spA = atof(arg);
        #endif
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

void parse_sp()
{
  double sp;
  char *arg; 
  arg = SCmd.next();
  if (arg != NULL)
  {
    serial_spA = map(atof(arg),-4000,0,-4000,0);
  } //else Serial.println("er p");  
}
