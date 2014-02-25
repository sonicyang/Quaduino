#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

// to the pins used:
const int pAnalogInPin = A5;  // Analog input pin that the pitch potentiometer is attached to
const int rAnalogInPin = A4;  // Analog input pin that the roll potentiometer is attached to
const int tAnalogInPin = A0; // Analog input pin that the Throttle potentiometer is attached to
const int switchPin = 2;  // On/Off switch
const int LED = 3;  // On/Off switch

const int Arcomode = 4;  // On/Off switch
const int AngleMode = 5;  // On/Off switch
const int ctly = 6;  // On/Off switch
const int ctlyp = 7;  // On/Off switch

const int pPin = A3; 
const int iPin = A2; 
const int dPin = A1; 

int pSensorValue = 0;        // value read from the Pitch Ctl pot
int rSensorValue = 0;        // value read from the Roll Ctl pot
int tSensorValue = 0;        // value read from the Throttle Ctl pot
int pV = 0; 
int iV = 0; 
int dV = 0; 

int onoff = 0;
int ctlbit = 2;

RF24 radio(9,10);
const uint64_t pipes[2] = { 
0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

void setup() {
    // initialize serial communications at 57600 bps:
    Serial.begin(57600); 
    printf_begin();
    radio.begin();
    radio.setRetries(15,15);
    //radio.setPayloadSize(3);
    radio.openWritingPipe(pipes[0]);
    radio.printDetails();

    pinMode(switchPin, INPUT);
    digitalWrite(switchPin, HIGH); 
    pinMode(LED, OUTPUT);
    pinMode(Arcomode, INPUT);
    digitalWrite(Arcomode, HIGH); 
    pinMode(AngleMode, INPUT);
    digitalWrite(AngleMode, HIGH); 
    pinMode(ctly, INPUT);
    digitalWrite(ctly, HIGH); 
    pinMode(ctlyp, INPUT);
    digitalWrite(ctlyp, HIGH); 
}

void loop() {
    // read the analog in value:
    pSensorValue = analogRead(pAnalogInPin);            
    rSensorValue = analogRead(rAnalogInPin);         
    tSensorValue = analogRead(tAnalogInPin); 
    pV = analogRead(pPin); 
    iV = analogRead(iPin); 
    dV = analogRead(dPin); 

    if(digitalRead(switchPin) == LOW)
        onoff = 1;
    if(digitalRead(switchPin) == HIGH)
        onoff = 0;
    if(onoff)
        digitalWrite(LED, HIGH); 
    else
        digitalWrite(LED, LOW); 
    float outValue[7] = {0};
    
    if(digitalRead(Arcomode) == LOW)
        ctlbit = 2;
    else if(digitalRead(AngleMode) == LOW)
        ctlbit = 4;
    
    // map it to the range of the angle:
    if(ctlbit == 2){
        outValue[0] = (int)(((float)map(pSensorValue, 0, 1023, 0, 1000)) / 5) / 10.0 - 10;
        outValue[1] = (int)(((float)map(rSensorValue, 0, 1023, 0, 1000)) / 5) / 10.0 - 10;
    } else if(ctlbit == 4){
        outValue[0] = (int)(((float)map(pSensorValue, 0, 1023, 0, 1000)) / 16.6 * 10) / 10.0 - 30.1;
        outValue[1] = (int)(((float)map(rSensorValue, 0, 1023, 0, 1000)) / 16.6 * 10)/10.0 - 30.3;
    }
    outValue[2] = onoff + ctlbit;  
    outValue[3] = map(tSensorValue, 0, 1023, 80, 160);  
    outValue[4] = (((float)map(pV, 0, 1023, 0, 1000))/1000.0)
    ;
    outValue[5] = (((float)map(iV, 0, 1023, 0, 1000))/1000.0);
    outValue[6] = (-1)*(((float)map(dV, 0, 1023, 0, 1000))/4000.0);
    

    // print the results to the serial monitor:
    Serial.print("Pitch: " );                       
    Serial.print(outValue[0]);      
    Serial.print("\tRoll: ");      
    Serial.print(outValue[1]);
    Serial.print("\tThrottle: ");      
    Serial.print(outValue[3]);
    Serial.print("\tCtl: ");      
    Serial.print(onoff);
    Serial.print(",");    
    Serial.print(ctlbit);
    Serial.print("\tP: ");      
    Serial.print(outValue[4],4);  
    Serial.print("\tI: ");      
    Serial.print(outValue[5],4);  
    Serial.print("\tD: ");      
    Serial.print(outValue[6],4);  
    bool ok = radio.write( outValue, sizeof(float)*7 );
    if (ok)
        printf("\tok...\n\r");
    else
        printf("\tfailed.\n\r");
    // wait 2  before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    delay(50);                     
}

