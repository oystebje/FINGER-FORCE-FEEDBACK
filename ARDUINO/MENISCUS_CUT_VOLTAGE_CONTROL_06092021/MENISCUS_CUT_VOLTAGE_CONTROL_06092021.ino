/**
 * 
 * Torque control of partial mensicetomy using voltage control loop.
 * By Øystein Bjelland, CPS Lab/AABL, Dept. of ICT and Natural Sciences, NTNU

 */
#include <SimpleFOC.h>
#include <math.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// encoder instance
Encoder encoder = Encoder(3, 2, 2048);


// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


float target_voltage = 0; // voltage set point variable

double xh = 0; //handle position [rad]
double vh = 0; //Handle velocity [rad/s]

double xOpen = 0; // Variable for storing the opening position [rad]. Used for computing hysteresis. 
double xIntercept = 4.8*(M_PI/180);   // position of interception point [rad]
double xBottomOut = 11*(M_PI/180);   // Position of bottom out point [rad]


double xOpening = 11*(M_PI/180);      // Position of opening point [rad]
double fOpening = 0;                  // Opening force [N]
double kOpening = 16.4;               // Stiffness of opening region [N/rad]

double force = 0;   // Force in [N]
double penetration = 0; // penetration depth [deg]

bool collisionDetected = false;   // Boolean variable for detecting collision.

//polynomial coefficients for nonlinear force-deflection curve
double a = 0.0812;  //0.074;
double b = -1.0304;  //-1.8568;
double c = 3.8805;  //15.103;
double d = 0;   //-35.591;

unsigned long currentMillis = 0;



void setup() { 
  
  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB); 
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);


  // aligning voltage
  motor.voltage_sensor_align = 5;
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  Serial.println(F("Motor ready."));
  _delay(1000);
}

void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  motor.loopFOC();

  xh = encoder.getAngle();
  double xhDeg = (xh/M_PI)*180;
  vh = encoder.getVelocity();


  //*******************************************
  // Detect collision between tool and tissue 

  if(xh > xIntercept){
      collisionDetected = true;
      }else{
        collisionDetected = false;
  }


  // Render force to handle

  if((vh >= 0) && (collisionDetected = true)){
    
    penetration = xhDeg - xIntercept;
    force = a*pow(penetration, 3) + b*pow(penetration,2) + c*penetration + d;
    fOpening = force;
    xOpen = xhDeg;
  
    
  } else if((vh < 0) && (collisionDetected = true)){
    force = fOpening + kOpening*(xhDeg - xOpen);
  
    
  } else{
    force = 0;
    
  }



  // Mapping force to voltage
    //  target_voltage = map(force, 0, 8, -12, 0);
   target_voltage = -force*1.5;

  // Finally a capping to prevent overloading the motor
  if(target_voltage < -12){
      target_voltage = -12;
  } else if (target_voltage > 0){
    target_voltage = 0;
  }

 //***********************************
    
//  Serial.print(target_voltage);
//  Serial.println("\t");
  Serial.print(force);
  Serial.println("\t");
//  Serial.print(xhDeg);
//  Serial.print("\t");
//  Serial.print(vh);
//  Serial.print("\t");
//  currentMillis = millis();
//  Serial.print(currentMillis);
//  Serial.println("\t");

  
  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_voltage);

}
