#ifndef ENCODER_LIB_H
#define ENCODER_LIB_H

#include "Arduino.h"
#include "utils.h"


// Pullup configuation structure
enum Pullup{
    INTERN,
    EXTERN
};

enum Quadrature{
  ENABLE, // CPR = 4xPPR
  DISABLE // CPR = PPR
};

class Encoder{
 public:
    /*
    Encoder(int encA, int encB , int cpr, int index)
    - encA, encB    - encoder A and B pins
    - ppr           - impulses per rotation  (cpr=ppr*4)
    - index pin     - (optional input)
    */
    Encoder(int encA, int encB , float ppr, int index = 0);

    // encoder initialise pins
    void init(void (*doA)() = nullptr, void(*doB)() = nullptr, void(*doIndex)() = nullptr);

    //  Encoder interrupt callback functions
    //  enabling CPR=4xPPR behaviour
    // A channel
    void handleA();
    // B channel
    void handleB();
    // index handle
    void handleIndex();
    
    // encoder getters
    // shaft velocity getter
    float getVelocity();
    float getAngle();
    // getter for index pin
    int indexFound();
    int hasIndex();
    float getIndexAngle();

    // setter for counter to zero
    void setCounterZero();
    void setIndexZero();
    
    // pins A and B
    int pinA, pinB;           // encoder hardware pins
    // index pin
    int index_pin;
    // encoder pullup type
    Pullup pullup;
    // use 4xppr or not
    Quadrature quadrature;


  private:
    volatile long pulse_counter;        // current pulse counter
    volatile long pulse_timestamp;      // last impulse timestamp in us
    float cpr;                 // impulse cpr
    volatile int A_active, B_active;    // current active states of A and B line
    volatile int I_active;              // index active
    volatile long index_pulse_counter;  // pulse counter of the index

    // velocity calculation varibles
    float prev_Th, pulse_per_second;
    volatile long prev_pulse_counter, prev_timestamp_us;

};


#endif
