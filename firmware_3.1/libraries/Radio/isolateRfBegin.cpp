#ifndef TOM_INCLUDED
#define TOM_INCLUDED

#include <Arduino.h> // Required for digitalWrites, etc.

// Board pin definitions.
const int RX_LED = LED1;  // B6 - RF RX LED
const int TX_LED = LED2;  // B7 - RF TX LED

static uint8_t rssiRaw; // Global variable shared between RX ISRs

// A buffer to maintain data being received by radio.
static const int RF_BUFFER_SIZE = 127;

struct ringBuffer
{
    unsigned char buffer[RF_BUFFER_SIZE];
    volatile unsigned int head;
    volatile unsigned int tail;
} tadioRXBuffer;


void tfFlush() {
    for (int i = 0; i < RF_BUFFER_SIZE; i++)
    {
        tadioRXBuffer.buffer[i] = 0;
    }
    tadioRXBuffer.head = tadioRXBuffer.tail = 0;
}


// Initialize the RFA1's low-power 2.4GHz transciever.
// Sets up the state machine, and gets the radio into
// the RX_ON state. Interrupts are enabled for RX
// begin and end, as well as TX end.
uint8_t tfBegin(uint8_t channel)
{
    tfFlush();
    
    // Setup RX/TX LEDs: These are pins B6/34 (RX) and B7/35 (TX).
    pinMode(RX_LED, OUTPUT);
    digitalWrite(RX_LED, LOW);
    pinMode(TX_LED, OUTPUT);
    digitalWrite(TX_LED, LOW);
    
    // Transceiver Pin Register -- TRXPR.
    // This register can be used to reset the transceiver, without
    // resetting the MCU.
    TRXPR |= (1<<TRXRST);   // TRXRST = 1 (Reset state, resets all registers)
    
    // Transceiver Interrupt Enable Mask - IRQ_MASK
    // This register disables/enables individual radio interrupts.
    // First, we'll disable IRQ and clear any pending IRQ's
    IRQ_MASK = 0;  // Disable all IRQs
    
    // Transceiver State Control Register -- TRX_STATE
    // This regiseter controls the states of the radio.
    // First, we'll set it to the TRX_OFF state.
    TRX_STATE = (TRX_STATE & 0xE0) | TRX_OFF;  // Set to TRX_OFF state
    delay(1);
    
    // Transceiver Status Register -- TRX_STATUS
    // This read-only register contains the present state of the radio transceiver.
    // After telling it to go to the TRX_OFF state, we'll make sure it's actually
    // there.
    if ((TRX_STATUS & 0x1F) != TRX_OFF) // Check to make sure state is correct
    return 0;    // Error, TRX isn't off
    
    // Transceiver Control Register 1 - TRX_CTRL_1
    // We'll use this register to turn on automatic CRC calculations.
    TRX_CTRL_1 |= (1<<TX_AUTO_CRC_ON);  // Enable automatic CRC calc.
    
    // Enable RX start/end and TX end interrupts
    IRQ_MASK = (1<<RX_START_EN) | (1<<RX_END_EN) | (1<<TX_END_EN);
    
    // Transceiver Clear Channel Assessment (CCA) -- PHY_CC_CCA
    // This register is used to set the channel. CCA_MODE should default
    // to Energy Above Threshold Mode.
    // Channel should be between 11 and 26 (2405 MHz to 2480 MHz)
    if ((channel < 11) || (channel > 26)) channel = 11;
    PHY_CC_CCA = (PHY_CC_CCA & 0xE0) | channel; // Set the channel
    
    // Finally, we'll enter into the RX_ON state. Now waiting for radio RX's, unless
    // we go into a transmitting state.
    TRX_STATE = (TRX_STATE & 0xE0) | RX_ON; // Default to receiver
    
    return 1;
}

#endif
