/*MTRAS_LF_V1.c
Date: 03/16/2019
Author: Steve Ghertner

This program is based on pololu line following adapted
for MTRAS kit.  The motor routine was replaced to drive two
continuous rotation servo motors.  The speeds are modified to
go from 0-100 (negative is in reverse).  The PID calculations are
modified to scale them appropriately for 0-100 speed range and
the Kp and Kd have been tuned for these motors. 

NOTE:  The MTRAS kit runs best on advanced course with
    scoop off! The scoop drags on the ground and slows the robots
    ability to pivot quick enough on turns.

Pin outs are as follows:
IR Sensor  ->  Nano Pin
  GND       GND
  VCC       2
  ODD       5VDC
  1       3
  3       4
  5       5
  7       6
  9       7
  11        8 

  Servo ->    Nano Pin
  Left      
  orange      10
  red       V (5VDC)
  Brown     G (GND)

  Servo ->    Nano Pin
  Right

  orange      11
  red       V (5VDC)
  Brown       G (GND)

Library versions:
I found that I had to manually delete the current library from the Arduino/libraries/ folder then install 
the new version.  You may need to quit arduino IDE and then restart it to load the new library modules.

02-20-2022:  QTRSensors 4.0.0


Changelog:
02/26/2022: lowered speed from 0.85 of max to 0.75
02/20/2022:  QTRSensors library didn't work anymore.  I am using V4.0 of QTRSensors now
            I suspect we were using an older version. Modified syntax for the newer version
            of the library
            This seems to be written for 5 sensor line sensor whereas we are using 6 sensor.
04/19/2019:  got PID working for servo motors, quick tuned to Kp=1.3, Kd=0.2
    these values worked for my small test track.  I'm not sure how these 
    will work on a longer straight section.
    also seemed to work using (1.2,0.2) at 0.85 of max speed
    look at determining straight section and setting to full speed

TO DO:
  - trouble with tight turns on advanced course.  runs best on advanced course with
    scoop off.  Try adjusting PID and speed without scoop or with 3D print "caster" that
    lifts scoop off the ground

*/

#include <ZumoBuzzer.h>
//#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
//#include <ZumoReflectanceSensorArray.h>
#include <Servo.h>


//Defines 
#define LED_PIN 13
const int PB_PIN = A0; 
const int PIEZO = A1;
#define LT_SERVO_PIN 10
#define RT_SERVO_PIN 11
 
#define NUM_SENSORS   6     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2


// these might need to be tuned for different motor types
#define REVERSE_SPEED     100 // 0 is stopped, 400 is full speed
#define TURN_SPEED        75
#define FORWARD_SPEED     100
#define FORWARD_SPEED_SLOW  FORWARD_SPEED*0.5
//#define RIGHT_ADJUST      50 //slow right track to compensate for motors
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms
#define MAX_SPEED 100

//for random turn routine
#define RIGHT 1
#define LEFT -1
 
ZumoBuzzer buzzer;
//ZumoMotors motors;
Pushbutton button(PB_PIN,PULL_UP_ENABLED); // pushbutton on pin 9
//instantiate the servos
Servo rtServo; 
Servo ltServo;
QTRSensors qtrrc;

 
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
unsigned int sensors[NUM_SENSORS];
int lastError = 0;

//analog i/o
//int  leftEyePin=4; //left sharp IR sensor pin A0
//int  rightEyePin=5;//right sharp IR sensor pin A3
//int  batLevelPin=1;//battery level A1

//timing variables
unsigned long last_turn_time; //for randomizing turn time
//constants for servo motors
const int RT_MOTOR_CALIB=1;
const int LT_MOTOR_CALIB=0;
const boolean RT_INVERT=false;
const boolean LT_INVERT=true;
const float LINE_FOLLOW_SPD_RED = 0.75; //slow the line follow speed, 0.75 for basic course
//const float LINE_FOLLOW_SPD_RED = 1.0;  // for advanced course with tight turns

//global variables for motor
int rtMotorSpeed=180;
int ltMotorSpeed=180;


//sensors 0 through 5 are connected to digital pins 2 through 8, respectively
unsigned int sensorValues[NUM_SENSORS];


void waitForButtonAndCountDown()
{
  digitalWrite(LED_PIN, HIGH);
  button.waitForButton();
  digitalWrite(LED_PIN, LOW);
  /* 
  // play audible countdown
  for (int i = 0; i < 4; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);  
  */
  delay(1000);
}

// execute turn 
// direction:  RIGHT or LEFT
// randomize: to improve searching
void turn(char direction, bool randomize){
#ifdef LOG_SERIAL
  Serial.print("turning ...");
  Serial.println();
#endif

  // assume contact lost
  //on_contact_lost();
  
  static unsigned int duration_increment = TURN_DURATION / 4;
  
  // motors.setSpeeds(0,0);
  // delay(STOP_DURATION);
  RunMotors(-REVERSE_SPEED, -REVERSE_SPEED);
  delay(REVERSE_DURATION);
  RunMotors(TURN_SPEED * direction, -TURN_SPEED * direction);
  delay(randomize ? TURN_DURATION + (random(8) - 2) * duration_increment : TURN_DURATION);
  //int speed = getForwardSpeed();
  int speed=FORWARD_SPEED*0.8;
  RunMotors(speed, speed);
  last_turn_time = millis();
}

int mapMotorValue(int mValue){
  //mValue -100 to 100 
  //mapped 180-0
  return map(mValue,-100,100,180,0);
}

void RunMotors(int ltS, int rtS){
  //speed sent 0-100 stop-> full speed
  //and negative value reverse, positive value forward

  //map 0-100 values to servo values
  int rtSpeed = map(rtS,-100,100,180,0);
  int ltSpeed = map(ltS,-100,100,180,0);

  Serial.print(rtSpeed);
  Serial.print(",");
  Serial.println(ltSpeed);
  delay(20);
  //update motor speeds with calibration value
  rtSpeed+=RT_MOTOR_CALIB;
  ltSpeed+=LT_MOTOR_CALIB;
  
  //invert one of the motors so they turn the same direction
  if(RT_INVERT){
    rtSpeed=180-rtSpeed;
  }
  if(LT_INVERT){
    ltSpeed=180-ltSpeed;
  }
  
  //constrain motor speeds between 0 and 180
  rtSpeed=constrain(rtSpeed,0,180);
  ltSpeed=constrain(ltSpeed,0,180);

  //run the motors
  ltServo.write(ltSpeed);
  rtServo.write(rtSpeed);
    
}

void setup(){
  //set up serial
  Serial.println("Beginning setup routine ...");
  
  Serial.println("setting up serial...");
  Serial.begin(38400); //setup serial 

  //set up IR line sensor
  Serial.println("Setting up IR line sensor...");
  qtrrc.setTypeRC(); //setup qt line sensor type, pin#'s
  qtrrc.setSensorPins((const uint8_t[]) {3, 4, 5, 6, 7,8},
  NUM_SENSORS);
  qtrrc.setEmitterPin(EMITTER_PIN);
  delay(500);
  
  
  //Play a little welcome song
  Serial.println("Playing short song...");
  buzzer.play(">g32>>c32");
  
  //setup servos
  Serial.println("Setting up servos...");
  ltServo.attach(LT_SERVO_PIN);
  rtServo.attach(RT_SERVO_PIN);
  RunMotors(0,0); //initialize motors to stop
 
  // Initialize the reflectance sensors module
  //reflectanceSensors.init();

  // Wait for the user button to be pressed and released
  Serial.println("waiting for button push...");
  button.waitForButton();


  // Turn on LED to indicate we are in calibration mode
  Serial.println("entering calibration mode...");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  int i;
  int turnsCountLength = 80; //number times to run calibration motion loop
  for(i = 0; i < 90; i++){
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
      RunMotors(-50, 50);
    else
      RunMotors(50, -50);
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)

    // Since our counter runs to 80, the total delay will be
    // 80*delaytimeMs = 80*40 =3,200 ms.
    delay(40);
  }
  RunMotors(0,0);

  
  // Turn off LED to indicate we are through with calibration
  digitalWrite(LED_PIN, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  //buzzer.play(">g32>>c32"); //play tune when finished

  
  // print the calibration minimum values measured when emitters were on
  Serial.println("calibration minium values: ");
  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(qtrrc.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  Serial.println("calibration maximum values: ");
  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(qtrrc.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  // Wait for the user button to be pressed and released
  button.waitForButton();

  // Play music and wait for it to finish before we start driving.
  //buzzer.play("L16 cdegreg4");
  //while(buzzer.isPlaying());  //locks up if wait for buzzer to end
  Serial.println("Finished Calibration routine...");
  Serial.println("finished setup routine...");
}

void loop(){
  //int integral;
  //int last_proportional;
  //works OK except tight circle (1.5,0)
  //works 4/19/2019 (Kp,Kd)=(1.3,0.2) at 0.75 of max speed, assumes center 2500
  //works 4/19/2019 (1.2,0.2) at 0.85 of max speed, assumes center 2500
  float Kp=1.2;  //proportional term (2 too high
  float Kd=0.2;  //derivative term  orig=6
  
  //if (button.isPressed()){
   
   // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int position = qtrrc.readLineBlack(sensorValues);
   // Our "error" is how far we are away from the center of the line, which
  // corresponds to position 2500.
  int error = position - 2500;  //get position error
  error*=0.04;  //normalize 2500 center scale to speed max of 100
  // Get motor speed difference using proportional and derivative PID terms
  // (the integral term is generally not very useful for line following).
  // Here we are using a proportional constant of 1/4 and a derivative
  // constant of 6, which should work decently for many Zumo motor choices.
  // You probably want to use trial and error to tune these constants for
  // your particular Zumo and line course.
  int speedDifference = error*Kp + Kd * (error - lastError);
  Serial.print("error, speedDiff: ");Serial.print(error);Serial.print(",");Serial.println(speedDifference);
  lastError = error;

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int m1Speed = MAX_SPEED/LINE_FOLLOW_SPD_RED + speedDifference;
  int m2Speed = MAX_SPEED/LINE_FOLLOW_SPD_RED - speedDifference;

  //constrain speeds
  constrain(m1Speed,-100,100);
  constrain(m2Speed,-100,100);
  
  RunMotors(m1Speed, m2Speed);
  //}
 

delay(100);  //delay at end of each loop
}
