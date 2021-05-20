#ifndef TOM_INCLUDED
#define TOM_INCLUDED

#include <Arduino.h> // Required for digitalWrites, etc.


struct Data {
    
    int yaw;
    int throt;
    int roll;
    int pitch;
    //Buttons
    int knobVal;
    //states
    int arm;
    //iden.
    int ID = 666;
    
    Data():yaw(0),throt(0),roll(0),pitch(0),knobVal(0),arm(0),ID(666){
        //empty constructor
    }
    
};


void tfFlush();
uint8_t tfBegin(uint8_t channel);

#endif
