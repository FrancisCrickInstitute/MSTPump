#include "TeensyStep.h"
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bounce.h>

Encoder knob(10, 11);
const int encoder_SW = 12;
Bounce pushbutton = Bounce(encoder_SW, 10);  // 10 ms debounce
bool state = 0;

Stepper pump(7, 6);       // STEP pin: 7, DIR pin: 6  // The stepper class encapsulates the physical properties of a stepper motor like pin numbers of the STEP and DIR signals, speed and acceleration of the motor.
//StepControl step_controller;    // The StepControl class is used to synchronously move up to 10 motors to their target positions.
RotateControl rotate_controller; // The RotateControl class is used to synchronously rotate up to 10 motors.
int speed_ = 0;

//Fluid global constants
//const int lead_size = 8; // lead size (8mm)
const int lead_size = 1; // lead size (1mm)
const float steps_rev = 3200; // Stepper resolution at 1/16 steps per revolution
const float syringe_area = 560;// Syringe cross section area(560 mm^2 Hardvare apparatus).(549.88 Datasheet)
float linear_resolution = (lead_size/steps_rev)*syringe_area; //uL/step or mm^3/step

float Volume;
float flow_rate;
int steps;
int steps_s;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MS1 = 3;
const int MS2 = 4;
const int MS3 = 5;
const int Enable = 2;

// limit switches
const int LSW1 = 15;
const int LSW2 = 14;

Bounce limitswitch_back = Bounce(LSW2,10);
Bounce limitswitch_front = Bounce(LSW1,10);

//Direction
bool dir = true;

void setup()
{
  pinMode(MS1, OUTPUT);    // set the MS1, MS2, MS3 and Enable as an outputs
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(Enable, OUTPUT);

  digitalWrite(MS1, HIGH); // sets the digital pin MS1, MS2, MS3 to HIGH
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);
  digitalWrite(Enable, HIGH); // set the enable to HIGH
  
  // Set the motor max acceleration
  pump.setAcceleration(500)   // stp/s^2
      .setInverseRotation(true);
  
  pinMode(LSW1, INPUT_PULLUP);
  pinMode(LSW2, INPUT_PULLUP);
  pinMode(encoder_SW, INPUT_PULLUP);
  
  Serial.begin(115200);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(1000); 
  display.clearDisplay();

}

void loop() 
{ 
  
  read_pushbutton();
  
  OLED_display();
  
  flow_rate =(knob.read()/2)*linear_resolution*60;
      
  steps_s = get_speed(flow_rate);

  digitalWrite(Enable, HIGH);
  
  rotate_controller.stop();
  
  while (digitalRead(LSW2)==HIGH && digitalRead(LSW1)==HIGH && state == HIGH)
  {
      digitalWrite(Enable, LOW);

      read_pushbutton();    
      
      flow_rate =(knob.read()/2)*linear_resolution*60;
 
      steps_s = get_speed(flow_rate);
      Volume = get_volume(pump.getPosition());
  
      pump.setMaxSpeed(steps_s)
      .setInverseRotation(dir);
      rotate_controller.rotateAsync(pump);
  
      OLED_display();


  }
  

}

int get_speed(float flow_rate)
{
  float uL_s = flow_rate/60;
  int steps_s;
  steps_s = uL_s*(1/linear_resolution);
  return steps_s;
}

int get_steps(int Volume)
{
  int linear_displacement = Volume/linear_resolution;
  return linear_displacement;
}

float get_volume(int steps)
{
  int Volume = steps*linear_resolution;
  return Volume;
}

void OLED_display()
{   
    display.clearDisplay();
    display.setTextColor(WHITE);        // Draw white text
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.print(F("MSTPump"));
    display.setTextSize(1);
    display.println();
    display.println();
    display.print(F("Flow rate selected:"));
    display.println();
    display.setTextSize(1);
    display.print((knob.read()/2)*linear_resolution*60);
    display.setTextSize(1);
    display.println  (F(" uL/min"));
    display.print(F("Flow rate dispensed:"));
    display.println();
    display.print(rotate_controller.getCurrentSpeed()*linear_resolution*60); 
    display.println  (F(" uL/min"));
    display.setTextSize(1);
    display.print(F("Volume:"));
    display.print(int(Volume));
    display.setTextSize(1);
    display.println(F(" uL"));
    display.display(); 
}

void read_pushbutton()
{
    if (pushbutton.update()) 
    {
      if (pushbutton.fallingEdge()) 
      {
        state = !state;
      }
    } 
}
