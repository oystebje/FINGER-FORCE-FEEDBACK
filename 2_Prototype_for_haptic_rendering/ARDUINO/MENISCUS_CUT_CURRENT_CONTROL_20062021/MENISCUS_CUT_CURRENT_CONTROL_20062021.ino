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

// current sensor
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, A0, A2);

// current set point variable
float target_current = 0.0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_current, cmd); }


// Variables for cutting curve and the four regions
double xh = 0; //handle position [rad]
double vh = 0; //Handle velocity [rad/s]

double xOpen = 0; // Variable for storing the opening position. Used for computing hysteresis.
bool checkpoint = false;
double force = 0; // Target force [N]

double xIntercept = 4.8*(M_PI/180);   // position of interception point [rad]
double kElastic = 182;                  // stiffness of elastic region [N/rad] (3.17 N/deg)

double xYield = 7*(M_PI/180);         // Position of yield point [rad]
double fYield = 0;                    // Yield Force [N]
double kPlastic = 22;                  // Stiffness of plastic region [N/rad]

double xBottomOut = 11*(M_PI/180);   // Position of bottom out point [rad]
double fBottomOut = 0;                // Bottom out force [N]
double kBottomOut = 300;               // Stiffness of bottom out region [N/rad]

double xOpening = 11*(M_PI/180);       // Position of opening point [rad]
double fOpening = 0;                  // Opening force [N]
double kOpening = 100;                  // Stiffness of opening region [N/rad]

// For sampling time stamps
double currentMillis = 0;

//  Constant for converting from force to current
double forceC = 0.01; // I.e. 10N becomes 0.1A


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


  // current sense init hardware
  current_sense.init();
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

  // set torque mode:
  // TorqueControlType::dc_current 
  // TorqueControlType::voltage
  // TorqueControlType::foc_current
  motor.torque_controller = TorqueControlType::dc_current; 
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // foc current control parameters (Arduino UNO/Mega)
  motor.PID_current_q.P = 5;
  motor.PID_current_q.I= 300;
  motor.LPF_current_q.Tf = 0.01; 

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


    // Implementing the four different regions 

    //Linear-elastic region
    if((xh > xIntercept) && (xh < xYield) && (checkpoint == false)) {
    
        force = -kElastic*(xh-xIntercept);     
        fYield = force;
        xOpen = xh;

    // Plastic/cutting region
    }else if((xh >= xYield) && (xh < xBottomOut) && (checkpoint == false)) {

        force = fYield + kPlastic*(xh-xYield);
        fBottomOut = force;
        xOpen = xh;

    // Bottom-out region (this should be stiffer)
    }else if((xh >= xBottomOut) && (checkpoint == false)){

        force = fBottomOut - kBottomOut*(xh-xBottomOut);
        fOpening = force;
        xOpen = xh;
        checkpoint = true;

    }else if((checkpoint == true)){

        force = fBottomOut - kBottomOut*(xh-xBottomOut);
   
    }else{
      force = 0;
    }

  if((xh < xIntercept) && (checkpoint == true)){
    checkpoint = false;
  }


  target_current = force*forceC;

  // Finally a capping to prevent overloading the motor
  if(target_current < -0.3){
      target_current = -0.3;
  }else if(target_current > 0.3){
    target_current = 0.3;
  } 


 //******************
    
  Serial.print(target_current);
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
  motor.move(target_current);

}
