/**
 *
 * Torque control using voltage control loop.
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
bool checkpoint = false;

double xIntercept = 4.8*(M_PI/180);   // position of interception point [rad]
double kElastic = 250;                  // stiffness of elastic region [V/rad]

double xYield = 7*(M_PI/180);         // Position of yield point [rad]
double fYield = 0;                    // Yield Force [V]
double kPlastic = 30;                  // Stiffness of plastic region [V/rad]

double xBottomOut = 11*(M_PI/180);   // Position of bottom out point [rad]
double fBottomOut = 0;                // Bottom out force [V]
double kBottomOut = 300;               // Stiffness of bottom out region [V/rad]

double xOpening = 11*(M_PI/180);       // Position of opening point [rad]
double fOpening = 0;                  // Opening force [V]
double kOpening = 100;                  // Stiffness of opening region [V/rad]

double a = 0.1644;
double b = -2.227;
double c = 9.1432;
double d = -0;
//double d = -35.525;

double currentMillis = 0;

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


    // Implementing the four different regions 

    //Linear-elastic region
    if((xh > xIntercept) && (xh < xBottomOut) && (checkpoint == false)) {
    
        target_voltage = -10*(a*pow((xh-xIntercept),3) + b*pow((xh-xIntercept),2) + c*(xh-xIntercept) + d);     
        fBottomOut = target_voltage;
        xOpen = xh;

//    // Plastic/cutting region
//    }else if((xh >= xYield) && (xh < xBottomOut) && (checkpoint == false)) {
//
//        target_voltage = fYield + kPlastic*(xh-xYield);
//        fBottomOut = target_voltage;
//        xOpen = xh;

    // Bottom-out region (this should be stiffer)
    }else if((xh >= xBottomOut) && (checkpoint == false)){

        target_voltage = fBottomOut - kBottomOut*(xh-xBottomOut);
        fOpening = target_voltage;
        xOpen = xh;
        checkpoint = true;

    }else if((checkpoint == true)){

        target_voltage = fBottomOut - kBottomOut*(xh-xBottomOut);
   
    }else{
      target_voltage = 0;
    }

  if((xh < xIntercept) && (checkpoint == true)){
    checkpoint = false;
  }

  // Finally a capping to prevent overloading the motor
  if(target_voltage < -12){
      target_voltage = -12;
  }else if(target_voltage > 0){
    target_voltage = 0;
  } 


 //******************
    
  Serial.print(target_voltage);
  Serial.print("\t");
  Serial.print(xhDeg);
  Serial.print("\t");
  Serial.print(vh);
  Serial.print("\t");
  currentMillis = millis();
  Serial.print(currentMillis);
  Serial.println("\t");
  
  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_voltage);

}
