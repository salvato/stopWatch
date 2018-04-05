/* Basket Chrono:
 *  
    Ogni partita dura 40 minuti suddivisi in 4 periodi di 10 minuti di gioco effettivo ciascuno 
    (12 minuti nella NBA per un totale di 48 minuti), suddivisione che precedentemente era 
    di 2 periodi da 20 minuti. 
    Tra il 2º ed il 3º periodo viene effettuato un intervallo di 10 o 20 minuti 
    (in Italia quello più lungo si applica nei campionati nazionali e quello più breve in tutti 
    gli altri campionati). 
    Alla ripresa del gioco le squadre devono scambiarsi il campo, mentre negli altri casi l'intervallo
    è solamente di 2 minuti (senza cambio campo).
    
    Ogni squadra per completare un attacco ha 24 secondi di tempo, se non ci riesce il possesso passa
    alla squadra avversaria. I 24 secondi vengono ripristinati ogni qualvolta la palla cambia di possesso.


    ==================================================================================================
    =====================>> Ma chi controlla e quale pulsante controlla i 14 secondi ? <<=============
    ==================================================================================================
    Se invece la squadra che attacca è nuovamente con la palla in mano dopo che essa ha toccato l'anello
    di ferro, il cronometro si riporta a 14 secondi. 
    Nel caso ci sia un fallo o un'infrazione di piede, se sono rimasti 13 secondi o meno il cronometro 
    viene riportato a 14 secondi. 
    Se invece il il tempo di gioco è maggiore di 14, si prosegue il conteggio. 
    ==================================================================================================
    ==================================================================================================

    La squadra vincente della partita è quella che ha realizzato il maggior numero di punti alla fine del 
    4° periodo di gioco. 
    In caso di parità la gara si prolunga di altri 5 minuti (tempo supplementare) e in caso di ulteriore 
    parità si procederà ad oltranza con altri tempi.

    The Timer/Counter can be clocked by an internal or an external clock source. 
    The clock source is selected by the Clock Select logic which is controlled by the 
    Clock Select (CS12:0) bits located in the Timer/Counter control Register B (TCCR1B).

    ============>>>>  An external clock source can not be prescaled. <<<<============

    You can use the T1 pin with Atmega328 based boards as a hardware counter,
    which is labeled "D5" (PD5) on an Arduino NANO.
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// Pinout  nRF24L01:
// 1  GND
// 2  3V3
// 3  CE
// 4  CSN
// 5  SCK
// 6  MOSI
// 7  MISO
// 8  IRQ


// Pinout Arduino Nano
//  9 CE
// 10 CSN
// 11 MOSI
// 12 MISO
// 13 SCK


#define BASKET      1
#define HANDBALL    2

//////////////////////////////////////////////////////////
// Fuction prototypes
//////////////////////////////////////////////////////////

void serialEvent();
bool executeCommand();
void check_radio(void);
void decodeCommand(byte *inputString, byte *decodedString);
void errorSignal();
void sendTime();
void sendPossess();


enum commands {
    AreYouThere    = 0xAA,
    Stop           = 0x01,
    Start          = 0x02,
    Start14        = 0x04,
    NewGame        = 0x11,
    RadioInfo      = 0x21,
    Configure      = 0x31,
    Time           = 0x41,
    Possess        = 0x42,
    StartSending   = 0x81,
    StopSending    = 0x82
};



byte          command;
byte          gameType = BASKET;// Default speciality

byte          ACK  = byte(0xFF);
byte          EOS  = byte(127);
byte          ack  = ACK;

byte          startMarker    = byte(0xFF);
byte          endMarker      = byte(0xFE);
byte          specialByte    = byte(0xFD);

int           maxMessageLen  = 64;
byte          inputString[64];
byte          outputString[64];
byte          decodedString[64];
int           decodedLength;
int           nReceivedChar;
int           commandLength;

boolean       inProgress     = false;
boolean       startFound     = false;
boolean       allReceived    = false;

long          baudRate       = 115200;

volatile bool bSendTime = false;

long          msec;
long          elapsed;
long          iDiv= 100;// dieci letture/s

volatile bool bPlaying;          
long          possessTime24;
long          possessTime14;

volatile long playTime;
volatile long possessTime;

int           endPossessPin = 3;
int           endGamePin    = 4;
int           endSigTonePin = 6;
unsigned      endPosToneFrq = 220;
unsigned long endPosSigDur  = 1000UL;
unsigned      endGamToneFrq = 440;
unsigned long endGamSigDur  = 3000UL;


// Radio Hardware configuration
RF24 radio(9, 10); // Set up nRF24L01 radio on SPI bus plus pins 9(CE) & 10(CSN)                                   
const byte interruptPin = 2;
byte address[][5] = { 0x01,0x23,0x45,0x67,0x89, 
                      0x01,0x23,0x45,0x67,0x89
                    };

/////////////////////////////////////////////////////////////////////
// Arduino Pins 2, 3, 4, 6, 9, 10, 11, 12, 13 Are already occupied //
/////////////////////////////////////////////////////////////////////

void 
setup() {
    Serial.begin(baudRate, SERIAL_8N1); // initialize serial: 8 data bits, no parity, one stop bit.
    
    bSendTime = false;
    bPlaying = false;
    
    pinMode(endPossessPin, OUTPUT);
    pinMode(endGamePin,    OUTPUT);

    digitalWrite(endPossessPin, LOW);
    digitalWrite(endGamePin, LOW);
    
    possessTime14 = 14L * 100L;
    possessTime24 = 24L * 100L;

    playTime      = 10L * 60L * 100L;
    possessTime   = possessTime24;
    
    // Set Timer1 on CTC mode: Clear Timer on Compare match.
    //
    // When the timer counter reaches the compare match register(OCR1A),
    // the timer will be cleared.
    noInterrupts(); // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    // We would have 100 interrupts per second
    OCR1A = 625;              // compare match register 16MHz/256/100Hz
    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B &= ~(1 << CS12);   // stop the counter (i.e. the 256 prescaler) 
    // When the output compare interrupt enable bit OCIExy in the interrupt mask register TIMSKx is set,
    // the output compare match interrupt service ISR(TIMERx_COMPy_vect) routine will be called.
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    interrupts();             // enable all interrupts

    // Setup and configure rf radio
    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);

    radio.enableDynamicPayloads();// Enable dynamically-sized payloads
    radio.setAutoAck(false);      // Disable Auto Ack   
                                                      
    radio.openWritingPipe(address[1]);                // Open pipes to other node for communication
    radio.openReadingPipe(1, address[0]);
    radio.startListening();
    
    // Attach interrupt handler to interrupt #0
    // (using pin D2) on BOTH the sender and receiver
    attachInterrupt(digitalPinToInterrupt(interruptPin), check_radio, LOW);
    msec = millis();
}


// Timer1 compare interrupt service routine
ISR(TIMER1_COMPA_vect) {
  if(bPlaying) {
      possessTime--;
      if(possessTime <= 0) {
          digitalWrite(endPossessPin, HIGH);
          noTone(endSigTonePin);
          tone(endSigTonePin, endPosToneFrq, endPosSigDur);
          bPlaying = false;
      }
      playTime--;
      if(playTime <= 0) {
          digitalWrite(endGamePin, HIGH);
          noTone(endSigTonePin);
          tone(endSigTonePin, endGamToneFrq, endGamSigDur);
          bPlaying = false;
      }
  }
}


// Radio Interrupt 
void
check_radio(void) {
    bool tx, fail, rx;
    radio.whatHappened(tx, fail, rx);// What happened?
      
    if(tx) {// Have godd transmision
//        for(int i=0; i<2; i++) {
            digitalWrite(endPossessPin, digitalRead(endPossessPin) ^ 1);
            delay(500);
        }
    }

    if(fail) { // Have we failed to transmit?
        errorSignal();
    }

    if(rx || radio.available()) { // Did we receive a message?   
        static char got_cmd; // Get this payload and dump it
        radio.read(&got_cmd, sizeof(got_cmd));
        if(got_cmd == Stop) {
            noInterrupts();
            bPlaying = false;
            TCCR1B &= ~(1 << CS12); // stop the counter (i.e. the 256 prescaler)
            interrupts(); 
        }
        else if((got_cmd == Start) && !bPlaying && (playTime > 0)) {
            bSendTime = true;
            digitalWrite(endPossessPin, LOW);
            if(playTime > possessTime24)
                possessTime = possessTime24;
            else
                possessTime = playTime;
            noInterrupts();
            bPlaying = true;
            TCCR1B |= (1 << CS12); // start the counter (i.e. the 256 prescaler) 
            interrupts(); 
        }
        else if((got_cmd == Start14) && !bPlaying && (playTime > 0)) {
            bSendTime = true;
            digitalWrite(endPossessPin, LOW);
            if(playTime > possessTime14)
                possessTime = possessTime14;
            else
                possessTime = playTime;
            noInterrupts();
            bPlaying = true;
            TCCR1B |= (1 << CS12); // start the counter (i.e. the 256 prescaler) 
            interrupts(); 
        }
        radio.writeAckPayload(1, &ack, sizeof(ack));      // Add an ack packet for the next time around.
    }
}


void 
loop() {
    serialEvent();
    if(allReceived) {
        executeCommand();
        allReceived = false;
    }
    elapsed = millis();
    if (bSendTime && (elapsed-msec >= iDiv)) {
        sendTime();
        if(gameType == BASKET) {
            sendPossess();
        }
        msec += iDiv;
    }
}


void
sendTime() {
    byte currentByte;
    // Prepare header and command
    outputString[0] = startMarker;
    outputString[2] = Time;
    
    long playTimeValue = playTime;
    int nBytes = 3;
    for(int i=0; i<4; i++) {
        currentByte = byte(playTimeValue & 0xFF);
        if(currentByte >= specialByte) {
            outputString[nBytes++] = specialByte;
            outputString[nBytes++] = byte(currentByte-specialByte);
        }
        else {
            outputString[nBytes++] = currentByte;
        }
        playTimeValue = playTimeValue >> 8;
    }
    outputString[nBytes++] = endMarker;
    outputString[1] = nBytes;
    for(int i=0; i<outputString[1]; i++) {
        Serial.write(byte(outputString[i]));
    }
}


void
sendPossess() {
    byte currentByte;
    long PossessValue = 0;
    int nBytes = 3;
    outputString[2] = Possess;
    PossessValue = possessTime;
    for(int i=0; i<4; i++) {
        currentByte = byte(PossessValue & 0xFF);
        if(currentByte >= specialByte) {
            outputString[nBytes++] = specialByte;
            outputString[nBytes++] = byte(currentByte-specialByte);
        }
        else {
            outputString[nBytes++] = currentByte;
        }
        PossessValue = PossessValue >> 8;
    }
    outputString[nBytes++] = endMarker;
    outputString[1] = nBytes;
    for(int i=0; i<outputString[1]; i++) {
        Serial.write(byte(outputString[i]));
    }
}


// SerialEvent occurs whenever a new data comes in the
// hardware serial RX.
// This routine is run between each time loop() runs, 
// so using delay inside loop can delay response.
// Multiple bytes of data may be available.
void 
serialEvent() {
    while(Serial.available()) {
        byte inChar = (byte)Serial.read();
        if(inChar == startMarker) {
            nReceivedChar = 0;
            inProgress = true;
        }
        if(inProgress) {
            inputString[nReceivedChar] = inChar;
            nReceivedChar++;
        }
        if(nReceivedChar == maxMessageLen) {
            errorSignal();
        }
        if(inChar == endMarker) {
            allReceived = true;
            commandLength = nReceivedChar;
            inProgress = false;
            return;
        }
    }
}


void 
decodeCommand(byte *decoded) {
    decodedLength = 0;
    for(int i=1; i<commandLength-1; i++) {
        if(inputString[i] == specialByte) {
            i++;
            decoded[decodedLength] = byte(specialByte + inputString[i]); 
        }
        else {
            decoded[decodedLength] = byte(inputString[i]);
        }
        decodedLength++;
    }
}


bool
executeCommand() {
    decodeCommand(decodedString);
    command = decodedString[1];
    
    if(command == byte(AreYouThere)) {// Are you there ? 
        outputString[0] = startMarker;
        outputString[1] = byte(4);// Total Command length
        outputString[2] = byte(AreYouThere);// The command
        outputString[3] = endMarker;
        for(int i=0; i<outputString[1]; i++) {
            Serial.write(byte(outputString[i]));
        }
        return true;
    }

    if(command == byte(Configure)) {
        if(decodedLength < 2) {// Bad command received: discard and go further
            errorSignal();
            return false;
        }
        gameType = decodedString[2];
        if(gameType == byte(BASKET)) {
            if(decodedLength < 9) {// Bad command received: discard and go further
                errorSignal();
                return false;
            }
            playTime       = long(decodedString[3]) + (long(decodedString[4]) << 8);// Get play time in seconds
            playTime      *=  100L;                                                // and transforms in cents of seconds 
            possessTime24  = long(decodedString[5]) + (long(decodedString[6]) << 8);// Get possess time (24) in seconds
            possessTime24 *=  100L;                                                // and transforms in cents of seconds 
            possessTime14  = long(decodedString[7]) + (long(decodedString[8]) << 8);// Get possess time (14) in seconds
            possessTime14 *=  100L;                                                // and transforms in cents of seconds 
        }
        else if(gameType == byte(HANDBALL)) {
            if(decodedLength < 5) {// Bad command received: discard and go further
                errorSignal();
                return false;
            }
            playTime       = long(decodedString[3]) + (long(decodedString[4]) << 8);// Get play time in seconds
            playTime      *=  100L;                                                // and transforms in cents of seconds 
            possessTime24  = playTime; // There is no possess time in Handball so making it equal to the play Time
            possessTime14  = playTime; // we ensure that the end of the possess will be at end of the game
        }
        possessTime = possessTime24;
        digitalWrite(endPossessPin, LOW);
        sendTime();
        sendPossess();
        return true;
    }

    else if(command == byte(StopSending)) {
        bSendTime = false;
        return true;
    }

    else if(command == byte(StartSending)) {
        bSendTime = true;
        return true;
    }

    else if(command == byte(NewGame)) {
        digitalWrite(endGamePin, LOW);
        return true;
    }
    
    else {
        errorSignal();
    }
    return false;
}


void
errorSignal() {
    for(int i=0; i<25; i++) {
        digitalWrite(endPossessPin, digitalRead(endPossessPin) ^ 1);
        delay(40);
    }
    digitalWrite(endPossessPin, LOW);
}

