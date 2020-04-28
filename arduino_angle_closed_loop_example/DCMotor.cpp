#include "DCMotor.h"


/*
  DCMotor(int _pwm_pin, int _direction_pin, int en)
  - _pwm_pin         - pwm pin
  - _direction_pin   - direction pin
  - enable pin       - (optional input) enable pin
*/
DCMotor::DCMotor(int _pwm_pin, int _direction_pin, int en)
{
  // Pin intialization
  pwm_pin = _pwm_pin;
  direction_pin = _direction_pin;
  // enable_pin pin
  enable_pin = en;

  // Power supply woltage
  voltage_power_supply = DEF_POWER_SUPPLY;

  // Velocity loop config
  // PI contoroller constant
  PI_velocity.K = DEF_PI_VEL_K;
  PI_velocity.Ti = DEF_PI_VEL_TI;
  PI_velocity.timestamp = micros();
  PI_velocity.voltage_limit = voltage_power_supply/2;
  PI_velocity.voltage_ramp = DEF_PI_VEL_U_RAMP;
  PI_velocity.voltage_prev = 0;
  PI_velocity.tracking_error_prev = 0;

  // position loop config
  // P controller constant
  P_angle.K = DEF_P_ANGLE_K;
  // maximum angular velocity to be used for positioning 
  P_angle.velocity_limit = DEF_P_ANGLE_VEL_LIM;

  // index search PI controller
  PI_velocity_index_search.K = DEF_PI_VEL_INDEX_K;
  PI_velocity_index_search.Ti = DEF_PI_VEL_INDEX_TI;
  PI_velocity_index_search.voltage_limit = voltage_power_supply/2;
  PI_velocity_index_search.voltage_ramp = DEF_PI_VEL_INDEX_U_RAMP;
  PI_velocity_index_search.timestamp = micros();
  PI_velocity_index_search.voltage_prev = 0;
  PI_velocity_index_search.tracking_error_prev = 0;

  // index search velocity
  index_search_velocity = DEF_INDEX_SEARCH_TARGET_VELOCITY;

  //debugger 
  debugger = nullptr;
}

// init hardware pins   
void DCMotor::init() {
  if(debugger) debugger->println("DEBUG: Initilaise the motor pins.");
  // PWM pins
  pinMode(pwm_pin, OUTPUT);
  pinMode(direction_pin, OUTPUT);
  if (enable_pin) pinMode(enable_pin, OUTPUT);

  if(debugger) debugger->println("DEBUG: Set high frequency PWM.");

  // sanity check for the voltage limit configuration
  if(PI_velocity.voltage_limit > voltage_power_supply) PI_velocity.voltage_limit = PI_velocity.voltage_limit > voltage_power_supply;
  if(PI_velocity_index_search.voltage_limit > voltage_power_supply) PI_velocity_index_search.voltage_limit = voltage_power_supply;

  delay(500);
  // enable motor
  if(debugger) debugger->println("DEBUG: Enabling motor.");
  enable();
  delay(500);
  
  alignEncoder();
  delay(500);
}


/*
	disable motor
*/
void DCMotor::disable()
{
  // disable the driver - if enable_pin pin available
  if (enable_pin) digitalWrite(enable_pin, LOW);
  // set zero to PWM
  setPwm(0);
}
/*
  disable motor
*/
void DCMotor::enable()
{
  // set zero to PWM
  setPwm(0);
  // enable_pin the driver - if enable_pin pin available
  if (enable_pin) digitalWrite(enable_pin, HIGH);

}

void DCMotor::linkEncoder(Encoder* enc) {
  encoder = enc;
}


/*
	Encoder alignment to electrical 0 angle
*/
int DCMotor::alignEncoder() {
  if(debugger) debugger->println("DEBUG: Align the encoder and motor electrical 0 angle.");
  // set encoder to zero
  encoder->setCounterZero();
  delay(500);

  // find the index if available
  int exit_flag = indexSearch();
  delay(500);
  if(debugger){
    if(exit_flag< 0 ) debugger->println("DEBUG: Error: Index not found!");
    if(exit_flag> 0 ) debugger->println("DEBUG: Success: Index found!");
    else debugger->println("DEBUG: Index not available!");
  }
  return exit_flag;
}


/*
	Encoder alignment to electrical 0 angle
*/
int DCMotor::indexSearch() {
  // if no index return
  if(!encoder->hasIndex()) return 0;
  
  if(debugger) debugger->println("DEBUG: Search for the encoder index.");

  // search the index with small speed
  while(!encoder->indexFound() && shaft_angle < _2PI){
    voltage = velocityIndexSearchPI(index_search_velocity - shaftVelocity());
    setPwm(voltage);
  }
  voltage = 0;
  setPwm(0);

  // set index to zero if it has been found
  if(encoder->indexFound()){
    encoder->setIndexZero();  
  }
  // return bool is index found
  return encoder->indexFound() ? 1 : -1;
}

/**
	State calculation methods
*/
// shaft angle calculation
float DCMotor::shaftAngle() {
  return encoder->getAngle();
}
// shaft velocity calculation
float DCMotor::shaftVelocity() {
  return encoder->getVelocity();
}

/*
  Iterative funciton running outer loop of the FOC algorithm
  Bahvior of this funciton is determined by the motor.controller variable
  It runs either angle, veloctiy, velocity ultra slow or voltage loop
  - needs to be called iteratively it is asynchronious function
*/
void DCMotor::move(float target) {
  // get angular velocity
  shaft_velocity = shaftVelocity();
  // get angle
  shaft_angle = shaftAngle();
  switch (controller) {
    case ControlType::voltage:
      voltage = target;
      break;
    case ControlType::angle:
      // angle set point
      // include angle loop
      shaft_angle_sp = target;
      shaft_velocity_sp = positionP( shaft_angle_sp - shaft_angle );
      voltage = velocityPI(shaft_velocity_sp - shaft_velocity);
      break;
    case ControlType::velocity:
      // velocity set point
      // inlcude velocity loop
      shaft_velocity_sp = target;
      voltage = velocityPI(shaft_velocity_sp - shaft_velocity);
      break;
  }
  // set the voltage to the motor
  setPwm(voltage);
}

/*
	Set voltage to the pwm pin
  - function a bit optimised to get better performance
*/
void DCMotor::setPwm(float U) {
  // max value
  float U_max = voltage_power_supply;
  
  // sets the voltage [-U_max,U_max] to pwm [0,255]
  int U_pwm = 255.0 * (U/U_max);

  // limit the values between 0 and 255
  U_pwm = (U_pwm < -255) ? -255 : (U_pwm >= 255) ? 255 : U_pwm;
  // write the direction
  digitalWrite(direction_pin, U_pwm > 0 ? 1 : 0);
  // write the pwm
  analogWrite(pwm_pin, abs(U_pwm));
}




/**
	Motor control functions
*/
// PI controller function
float DCMotor::controllerPI(float tracking_error, PI_s& cont){
  float Ts = (micros() - cont.timestamp) * 1e-6;
  if(Ts > 0.5) Ts = 1e-3;

  // u(s) = Kr(1 + 1/(Ti.s))
  // tustin discretisation of the PI controller ( a bit optimised )
  // uk = uk_1  +  K*(Ts/(2*Ti) + 1)*ek + K*(Ts/(2*Ti)-1)*ek_1
  float tmp = 0.5 * Ts / cont.Ti;
  float voltage = cont.voltage_prev + cont.K * ((tmp + 1.0) * tracking_error + (tmp - 1.0) * cont.tracking_error_prev);

  // antiwindup - limit the output voltage
  if (abs(voltage) > cont.voltage_limit) voltage = voltage > 0 ? cont.voltage_limit : -cont.voltage_limit;
  // limit the acceleration by ramping the the voltage
  float d_voltage = voltage - cont.voltage_prev;
  if (abs(d_voltage)/Ts > cont.voltage_ramp) voltage = d_voltage > 0 ? cont.voltage_prev + cont.voltage_ramp*Ts : cont.voltage_prev - cont.voltage_ramp*Ts;

  cont.voltage_prev = voltage;
  cont.tracking_error_prev = tracking_error;
  cont.timestamp = micros();
  return voltage;
}
// velocity control loop PI controller
float DCMotor::velocityPI(float tracking_error) {
  return controllerPI(tracking_error, PI_velocity);
}
// index search PI contoller
float DCMotor::velocityIndexSearchPI(float tracking_error) {
  return controllerPI(tracking_error, PI_velocity_index_search);
}

// P controller for position control loop
float DCMotor::positionP(float ek) {
  float velk = P_angle.K * ek;
  if (abs(velk) > P_angle.velocity_limit) velk = velk > 0 ? P_angle.velocity_limit : -P_angle.velocity_limit;
  return velk;
}

/**
 *  Debugger functions
 */
// funciton implementing the debugger setter
void DCMotor::useDebugging(Print &print){
  debugger = &print; //operate on the adress of print
  if(debugger )debugger->println("DEBUG: Serial debugger enabled!");
}