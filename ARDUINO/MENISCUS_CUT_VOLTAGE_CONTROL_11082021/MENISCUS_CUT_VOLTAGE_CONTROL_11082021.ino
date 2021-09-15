/**
 * Virtual wall rendering with:
 * Torque control example using voltage control loop.
 * 

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

// voltage set point variable
float target_voltage = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }


// Variables for cutting curve and the four regions
double xh = 0; //handle position [rad]
double vh = 0; //Handle velocity [rad/s]

double xOpen = 0; // Variable for storing the opening position. Used for computing hysteresis.

double xIntercept = 4.8*(M_PI/180);   // position of interception point [rad]
//double kElastic = 100;                  // stiffness of elastic region [V/rad]

//double xYield = 7*(M_PI/180);         // Position of yield point [rad]
//double fYield = 0;                    // Yield Force [V]
//double kPlastic = 30;                  // Stiffness of plastic region [V/rad]

double xBottomOut = 11*(M_PI/180);   // Position of bottom out point [rad]
double fBottomOut = 0;                // Bottom out force [N]
double kBottomOut = 20;               // Stiffness of bottom out region [N/rad]

double xOpening = 11*(M_PI/180);       // Position of opening point [rad]
double fOpening = 0;                  // Opening force [N]
double kOpening = 16.4;                  // Stiffness of opening region [N/rad]

double force = 0;   // Force in [N]

//polynomial coefficients
double a = 0.074;
double b = -1.8568;
double c = 15.103;
double d = -35.591;


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

  // add target command T
  //command.add('T', doTarget, "target voltage");

  Serial.println(F("Motor ready."));
  //Serial.println(F("Set the target voltage using serial terminal:"));
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
  // Implementing the four different regions 

  //Linear-elastic and plastic regions
  if((xh > xIntercept) && (xh < xBottomOut)) {
    
    if(vh > 0){
        force = a*pow(xh, 3) + b*pow(xh,2) + c*xh + d;
        fOpening = force;
        fBottomOut = force;
        xOpen = xh;
    }else{
        force = fOpening - kOpening*(xOpen - xh);
    }

   // Bottom-out region (this should be stiffer)
   }else if((xh >= xBottomOut)){

     if(vh > 0){
        force = fBottomOut - kBottomOut*(xBottomOut - xh);
        fOpening = force;
        xOpen = xh;
     }else{
        force = fOpening - kOpening*(xOpen - xh);
     }
   
   }else{
    force = 0;
   }


  // Mapping force to voltage
  target_voltage = map(force, 0, 8, 0, 12);

  // Finally a capping to prevent overloading the motor
  if(target_voltage > 12){
      target_voltage = 12;
  } 

 //***********************************
    
  Serial.print(target_voltage);
  Serial.println("\t");
//  Serial.print(xhDeg);
//  Serial.print("\t");
//  Serial.print(vh);
//  Serial.println("\t");

  
  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_voltage);

}
