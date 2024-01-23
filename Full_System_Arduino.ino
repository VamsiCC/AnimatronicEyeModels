#include <AccelStepper.h> //accelstepper library
#include <Servo.h>
AccelStepper stepper(1, 9, 10); // direction Digital 9 (CCW), pulses Digital 8 (CLK)
//Servo objects created to control the servos
Servo myServo1;
Servo myServo2;


//Pins  
const byte Analog_X_pin = A0; //x-axis readings
const byte Analog_R_pin = A2; //r-axis readings
const byte LED_pin = 3; //PWM output for 

int servo1 = 5; //Digital PWM pin used by the servo 1
int servo2 = 6; //Digital PWM pin used by the servo 2   //Analog pin to which the joystick (X) is connected
int joyY = 1;   //Analog pin to which the joystick (Y) is connected
int angle=0;


//Variables
int Analog_X = 0; //x-axis value
int Analog_Y = 0; //y-axis value
int Analog_R = 0; //r-axis value

int Analog_X_AVG = 0; //x-axis value average
int Analog_R_AVG = 0; //r-axis value average

int Analog_R_Value = 0; //this is used for the PWM value
//public int tempp=0;


void setup()
{

  //SERIAL
  Serial.begin(9600);
  //----------------------------------------------------------------------------    
  //PINS
  pinMode(Analog_X_pin, INPUT);
  pinMode(Analog_R_pin, INPUT); 
  pinMode(LED_pin, OUTPUT);
  
  //----------------------------------------------------------------------------  
  InitialValues(); //averaging the values of the 3 analog pins (values from potmeters)
  //----------------------------------------------------------------------------  
  //Stepper parameters
  //setting up some default values for maximum speed and maximum acceleration
  stepper.setMaxSpeed(40000); //SPEED = Steps / second  
  stepper.setAcceleration(10000); //ACCELERATION = Steps /(second)^2    
  //stepper.setSpeed(2500);
  delay(50);
  //----------------------------------------------------------------------------
  /*stepper2.setMaxSpeed(5000); //SPEED = Steps / second  
  stepper2.setAcceleration(1000); //ACCELERATION = Steps /(second)^2    
  stepper2.setSpeed(500);
  delay(500);
  */  
  
  myServo1.attach(servo1);
  myServo2.attach(servo2);
  myServo1.write(90);   
  myServo2.write(90);

}

void loop()
{
  ReadAnalog();  
  
  if(abs(map(joyY, 0, 1023, -90, 90))>90)
  {
    stepper.setSpeed(0);
    }
  else{
    stepper.runSpeed(); //step the motor (this will step the motor by 1 step at each loop indefinitely)
  }
  
 int valX = analogRead(joyY); //Read the joystick X value (value between 0 and 1023)
  int valY = (analogRead(joyY)); //Read the joystick Y value (value between 0 and 1023)

 valX = map(valX, 0, 1023, 0, 180); //Scale the joystick X value to use it with the servo
 valY = map(valY, 0, 1023, 180, 0);
 myServo1.write(valX);   
  myServo2.write(valY);

 // delay(5);

}

void ReadAnalog()
{
  //Reading the 3 potentiometers in the joystick: x, y and r.
  Analog_X = analogRead(Analog_X_pin);
  Analog_R = analogRead(Analog_R_pin);    

 // int angle = map(joyY, 0, 1023, -90, 90);

  //if the value is 25 "value away" from the average (midpoint), we allow the update of the speed
  //This is a sort of a filter for the inaccuracy of the reading
  if(abs(Analog_X-Analog_X_AVG)>25) 
  {
  stepper.setSpeed(500*(Analog_X-Analog_X_AVG));    
  }
  else
  {
    stepper.setSpeed(0);
  }
  //----------------------------------------------------------------------------  
  if(abs(Analog_R-Analog_R_AVG)>25) 
  {
    Analog_R_Value = map(Analog_R, 0, 1023, 0, 255); //10 bit ADC (0-1023) and 8 bit PWM (0-255)
    analogWrite(LED_pin, Analog_R_Value); //modify the PWM value     
  }
 
}

void InitialValues()
{
  //Set the values to zero before averaging
  float tempX = 0;
  float tempR = 0;
  //----------------------------------------------------------------------------  
  //read the analog 50x, then calculate an average. 
  //they will be the reference values
  for(int i = 0; i<50; i++)
  {
     tempX += analogRead(Analog_X_pin);  
     delay(10); //allowing a little time between two readings
     tempR += analogRead(Analog_R_pin);
     delay(10);
     
  }
  //----------------------------------------------------------------------------  
  Analog_X_AVG = tempX/50; 
  Analog_R_AVG = tempR/50; 
  //----------------------------------------------------------------------------  
  Serial.print("AVG_X: ");
  Serial.println(Analog_X_AVG);

  Serial.print("AVG_R: ");
  Serial.println(Analog_R_AVG);
  Serial.println("Calibration finished");  
  
}
