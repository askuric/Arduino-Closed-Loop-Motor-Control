#include "DCMotor.h"
#include "Encoder.h"

// Only pins 2 and 3 are supported
#define arduinoInt1 2             // Arduino UNO interrupt 0
#define arduinoInt2 3             // Arduino UNO interrupt 1

// DCMotor(int _pwm_pin, int _direction_pin, int en)
//   - _pwm_pin         - pwm pin
//   - _direction_pin   - direction pin
//   - enable pin       - (optional input) enable pin
DCMotor motor = DCMotor(5, 6, 7);
//  Encoder(int encA, int encB , int cpr, int index)
//  - encA, encB    - encoder A and B pins
//  - ppr           - impulses per rotation  (cpr=ppr*4)
//  - index pin     - (optional input) 
Encoder encoder = Encoder(arduinoInt1, arduinoInt2, 128, 21);

// Interrupt rutine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
// index calback interrupt code 
// either provide it to the encoder.init()
// or in the ISR function
void doIndex(){encoder.handleIndex();}

// please set the right PCINT(0,1,2)_vect parameter
//  PCINT0_vect - index pin in between D8 and D13
//  PCINT1_vect - index pin in between A0 and A5 (recommended)
//  PCINT2_vect - index pin in between D0 and D7
// ISR (PCINT1_vect) {  doIndex(); }


void setup() { 
  // debugging port
  Serial.begin(115200);

  // check if you need internal pullups
  //  Quadrature::ENABLE - CPR = 4xPPR  - default
  //  Quadrature::DISABLE - CPR = PPR
  encoder.quadrature = Quadrature::ENABLE;

  // check if you need internal pullups
  // Pullup::EXTERN - external pullup added - dafault
  // Pullup::INTERN - needs internal arduino pullup
  encoder.pullup = Pullup::EXTERN;
  
  // initialise encoder hardware
  encoder.init(doA, doB, doIndex);
  //encoder.init(doA, doB);

  // power supply voltage
  // default 12V
  motor.voltage_power_supply = 12;

  // index search velocity - default 1rad/s
  motor.index_search_velocity = 1;
  // index search PI contoller parameters
  // default K=0.5 Ti = 0.01
  motor.PI_velocity_index_search.K = 0.1;
  motor.PI_velocity_index_search.Ti = 0.01;
  //motor.PI_velocity_index_search.voltage_limit = 3;
  // jerk control using voltage voltage ramp
  // default value is 100
  motor.PI_velocity_index_search.voltage_ramp = 100;
  
  // set control loop type to be used
  // ControlType::voltage
  // ControlType::velocity
  // ControlType::velocity_ultra_slow
  // ControlType::angle
  motor.controller = ControlType::velocity;

  // contoller configuration based on the controll type 
  // velocity PI controller parameters
  // default K=1.0 Ti = 0.003
  motor.PI_velocity.K = 0.3;
  motor.PI_velocity.Ti = 0.003;
  //defualt voltage_power_supply/2
  motor.PI_velocity.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PI_velocity.voltage_ramp = 300;


  // link the motor to the sensor
  motor.linkEncoder(&encoder);

  // use debugging with serial for motor init
  // comment out if not needed
  motor.useDebugging(Serial);

  // intialise motor
  motor.init();

  Serial.println("Motor ready.");
  Serial.println("Input the new target velocity:");
  delay(1000);

}

// target velocity variable
float target_velocity = 0;

void loop() {
  
  // iterative function setting the outter loop target
  // velocity, position or voltage
  // this funciton should be run at the highers frequency possible
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor_monitor();
}

// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
void motor_monitor() {
  switch (motor.controller) {
    case ControlType::velocity:
      Serial.print(motor.voltage);
      Serial.print("\t");
      Serial.print(motor.shaft_velocity_sp);
      Serial.print("\t");
      Serial.print(motor.shaft_velocity);
      Serial.print("\t");
      Serial.println(motor.shaft_angle);
      break;
    case ControlType::angle:
      Serial.print(motor.voltage);
      Serial.print("\t");
      Serial.print(motor.shaft_angle_sp);
      Serial.print("\t");
      Serial.println(motor.shaft_angle);
      break;
    case ControlType::voltage:
      Serial.print(motor.voltage);
      Serial.print("\t");
      Serial.print(motor.shaft_angle);
      Serial.print("\t");
      Serial.println(motor.shaft_velocity);
      break;
  }
}

// Serial communication callback
void serialEvent() {
  // a string to hold incoming data
  static String inputString;
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline
    // end of input
    if (inChar == '\n') {
      target_velocity = inputString.toFloat();
      Serial.print("Tagret Velocity: ");
      Serial.println(target_velocity);
      inputString = "";
    }
  }
}


