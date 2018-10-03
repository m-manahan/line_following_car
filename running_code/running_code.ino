#include <arduino2.h>
//#include <ArduinoRobotMotorBoard.h>
//#include <EasyTransfer2.h>
//#include <LineFollow.h>
//#include <Multiplexer.h>
#include <Servo.h>
#include <Wire.h>
Servo carServo;
//int angle;
byte cLine;          //Value of CenterLine in bytes
//byte pins[] = {0, 1, 2, 3, 4, 5, 6, 7};
///int pin0 = 0;
//int pin1 = 1;
  
//Assign variables
  int pin2 = 2;
  int pin3 = 3;
  int pin4 = 4;
  int pin5 = 5;
  int pin6 = 6;
  int pin7 = 7;
  int pin8 = 8;
  int pin12 = 12;
  int syncpin = 13;    //B of Camera
  int clockpin = 9;  //PWM pin for C of camera
  int servoPin = 10;  //Servo input PIN PWM
  int datapin = A0; // A of Camera
  int lightval[128];
  int lightvalder[127];
  int totalBrightness=0;
  int lightRatio=0;
  int lightCounter;
  int endPixel;
//Variables for Servo
  int center_pixl = 64;
  int centerLine=0;
  int prevcenterLine = 0;
  
  int servoVal;
  int newServoVal=0;

int line_err;

double Kp = 2;  //10 //1 //Guess: 2 , 20
double Kd = 0.041;  //1.5 //1 //Guess: 1 , 3.5
double Ki = 0; //20    //0 leave 0 for servo don't need I for servo! it has its own I

double derivative;
double integral = 0;
double output;
int error;
int setpoint = 64;
int prev_error = 0;
double dt=.006868; //calculated 6868 microseconds for time
unsigned long t_ime;

int valueBinary;  //For debugging port manipulation
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 
//PID servoPID(&desired_Val, &Servo_control, &Setpoint, Kp,Ki,Kd, DIRECT) //Don't necessarily need PID for the Servo. Only if it is really woobly when you are running it.
void setup ()
{ 
  carServo.attach(servoPin); //Servo attached to ~11
  //Wire.begin();
  pinMode (clockpin, OUTPUT); 
  pinMode (syncpin, OUTPUT);
  pinMode (datapin, INPUT);
  pinMode(servoPin, OUTPUT );
  pinMode2(pin2, OUTPUT);
  pinMode2(pin3, OUTPUT);
  pinMode2(pin4, OUTPUT);
  pinMode2(pin5, OUTPUT);
  pinMode2(pin6, OUTPUT);
  pinMode2(pin7, OUTPUT);
  pinMode2(pin8, OUTPUT);
  pinMode2(pin12, OUTPUT);
  Serial.begin(57600);
  cbi(ADCSRA,ADPS2);
  sbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
}

void loop()
{
 // t_ime = micros();
 // Wire.beginTransmission(9); // transmit to device #9
  getCamera2();
  //I need to filter the camera
  //If the pixel count is greater than a certain value, then ignore it
  
    //Serial.println(endPixel);
  getPID();
  getServo();
//  Serial.print(servoVal);
 // Wire.endTransmission();
  //delay for transmission of data
 // delay(450);
  //portManip();
//  t_ime = micros() - t_ime;
  //dt = currentTime - startTime;
 // Serial.println(t_ime);
}

void getPID()
{
  
  error = setpoint - centerLine;
  integral = integral + error*dt;  //integral + error*dt
  derivative = (error - prev_error)/dt;
  output = (Kp*error + Ki*integral + Kd*derivative);
  prev_error = error;
  delayMicroseconds(6868);
  
}


void getServo()
{
 // Serial.print(centerLine);
  // servoVal = centerLine; //servoVal = centerLine; works!!!!
  servoVal = output; ////For PID control of servo
  // Serial.print("In get servo function, centerLine is:");
  // Serial.println(centerLine);
  //testing output
   servoVal = map(servoVal, 0, 127, 1250, 1750);
//servoVal = map(servoVal, 0, 127, 1750, 1250);//1750 to 1250 is a good range for hi tec servo but wrong direction //this is correct working one
 newServoVal = (.75*newServoVal) + (.25*servoVal);//part of the code!!!
  //turn motor towards 0 degrees
  // servoVal = map(servoVal, 0, 127, 0, 180);//Using different values because of millis
  //   Serial.println(servoVal)
     carServo.writeMicroseconds(servoVal);
     //Serial.print(servoVal);
     servoVal = newServoVal;  // need this line!!
    // delay(15); 
    //Serial.print("Now center line is: ");
    //Serial.print(centerLine);
    
}
void getCamera2()
{
  //delay (2000);
  //digitalWrite (syncpin, LOW);
  totalBrightness = 0;
  //lightCounter = 0;
  for (int j = 0; j < 128; j++)
  {
    if (j==0){
      lightCounter = 0;
      digitalWrite (syncpin, HIGH);

    }
    digitalWrite(clockpin, HIGH); 
    // delay (10);
    if (j==0){
      digitalWrite (syncpin, LOW) ;
    }
    digitalWrite(clockpin, LOW);
    //delay(10);
    lightval[j] = analogRead(datapin);
    //lightvalder[j] = lightval[j+1] - lightval[j];
    //totalBrightness = abs(lightvalder[j]) + totalBrightness;
    //lightRatio = totalBrightness / j;
    //lightRatio = lightRatio * 1.5;
    totalBrightness = lightval[j] + totalBrightness; 
    lightRatio = totalBrightness / j;
    lightRatio = lightRatio *1.6;  //then 1.51.8//2 worked last time
    if ((lightval[j] > lightRatio))//(abs(lightvalder[j]) > lightRatio)
    {
      if(lightCounter < 10)
      {
      //Serial.print("1");
      lightCounter++;
      endPixel = j;
      centerLine = endPixel - (lightCounter/2);
      prevcenterLine = centerLine;
      }
     // prevcenterLine = centerLine;
      //Send end pixel to other arduino
  //    Wire.write(endPixel);
    }
    else
    {
     // Serial.print("0");
      centerLine = prevcenterLine;
    }
    
    // //Serial.print(abs(lightvalder[j]));
    if (j == 127)
    {
      
    //  Serial.print(lightCounter);
     // Serial.print(prevcenterLine);
   //   Serial.print("and the center pixel of the line is ");
   //   Serial.print("  ");
    //  Serial.print(centerLine);
   //   Serial.println ();
    }
  }
}
//////Old camera code
void getCamera()
{
  //delay (2000);
  //digitalWrite (syncpin, LOW);
  //totalBrightness = 0;
  //lightCounter = 0;
  for (int j = 0; j < 128; j++)
  {
    if (j==0){
      lightCounter = 0;
      digitalWrite (syncpin, HIGH);
    }
    digitalWrite(clockpin, HIGH); 
    // delay (10);
    if (j==0){
      digitalWrite (syncpin, LOW) ;
    }
    digitalWrite(clockpin, LOW);
    lightval[j] = analogRead(datapin);
    totalBrightness = abs(lightvalder[j]) + totalBrightness;
   
   lightRatio = totalBrightness / j;
    lightRatio = lightRatio * (2); //Good value was times 2 at school  //A good value before was 1.2 *50
    if (lightval[j] > lightRatio)//(abs(lightvalder[j]) > lightRatio)
    {
      Serial.print("1");
      lightCounter++;
      endPixel = j;
       centerLine = endPixel - (lightCounter/2)-10;//I do not think this changes it to center...
      //Send end pixel to other arduino
  //    Wire.write(endPixel);
    }
    else
    {
      Serial.print("0");    
    }  
   // Serial.println(lightval[0]);
    if (j == 127)
    {
   // Serial.print(lightCounter);
  //   Serial.print("and the last pixel of the line is ");
  // Serial.print(endPixel);
  //  Serial.print("centerLine is");
  //    Serial.print(centerLine);
      Serial.println ();
    } 
     totalBrightness = lightval[j] + totalBrightness;    
  } 
}





