//DO NOT DELTE 
#ifndef RADIO_INCLUDED
#define RADIO_INCLUDED

//#include "radio.cpp"

/* RadioFunctions.h
  A handful of sending and receiving functions for the 
  ATmega128RFA1.
  by: Jim Lindblom
      SparkFun Electronics
  date: July 8, 2013
  license: Beerware. Use, distribut, and modify this code freely
  however you please. If you find it useful, you can buy me a beer
  some day.
  
  Functions in here:
    rfBegin(uint8_t channel) - Initializes the radio on a channel
                                between 11 and 26.
    rfWrite(uint8_t b) - Sends a byte over the radio.
    rfPrint(String toPrint) - Sends a string over the radio.
    int rfAvailable() - Returns number of characters currently
                        in receive buffer, ready to be read.
    char rfRead() - Reads a character out of the buffer.
    
  Interrupt Sub-Routines (ISRs) in here:
    TRX24_TX_END_vect - End of radio transfer.
    TRX24_RX_START_vect - Beginning of radio receive.
    TRX24_RX_END_vect - End of radio receive. Characters are 
                        collected into a buffer here.
*/
  
#include <Arduino.h> // Required for digitalWrites, etc.


struct Coefficients {
    
    float proportional;
    float integral;
    float derivative;
    
    Coefficients(): proportional(0.0), integral(1.0), derivative(2.0){
        //empty constructor.
    }
};



struct Data {
 
    int yaw;
    int throt;
    int roll;
    int pitch;
    
	//PID values
    Coefficients coefficientsContainer[3];
 
    //states
    int arm;
	int pitchTrimSign;
	int pitchTrimMagnitude;
	int rollTrimSign;
	int rollTrimMagnitude;
	//Radio data checksum. Do not put any data after this.
    int ID; // index 56 + 57 of the buffer
 
    Data():yaw(0),throt(0),roll(0),pitch(0),  coefficientsContainer(),arm(0),pitchTrimSign(0),pitchTrimMagnitude(0),rollTrimSign(0),rollTrimMagnitude(0),ID(666){
        //empty constructor
    }
};



void rfFlush();
uint8_t rfBegin(uint8_t channel);
void rfPrint(String toPrint);
void rfWrite(uint8_t b);
void rfWrite(uint8_t * b, uint8_t length);
unsigned int rfAvailable();
char rfRead();
char rfRead(uint8_t * buf, uint8_t len);

/*
template<class T>
bool rfReceive(T & g)
{
	T t;
	if (rfAvailable()) {
		int l = rfRead((uint8_t *)&t, sizeof(T));
		if (l == sizeof(T)) {
		
            if (  t.validate()    ) {
				g = t;
				Serial.println();
				return true;
			} else {
				Serial.print("Bad packet: ");
				Serial.print(t.get_name());
				Serial.println();
			}
		} else {
			Serial.print("Short packet: ");
			Serial.print(t.get_name());
			Serial.println();
		}
	}  
	return false;
}*/



bool rfReceive(Data & g)
{
    
    Data t;
    if (rfAvailable()) {
        
        int l = rfRead((uint8_t *)&t, sizeof(Data));
        
         Serial.print(l);
         Serial.print(sizeof(Data));
         Serial.println();
        
        if (l == sizeof(Data)) {
            
             Serial.println( t.ID );
             Serial.println();
            
            if( t.ID == 666){
                g = t;
                Serial.println("Recieved data");
                return true;
            }
            else{
                Serial.print("Did not receive ");
                Serial.println();
            }
          
        }else {
            Serial.print("Short packet: ");
            Serial.println();
        }
    }
    return false;
}


/*template<class T>
bool rfSend(T & g)
{
	g.bless();
	rfWrite((uint8_t *)&g, sizeof(T));
}*/

bool rfSend(Data & g){
    rfWrite((uint8_t *)&g, sizeof(Data)); // sends 
}




#endif 
