//  Initialize and Scan Sensor Data for Serial Transmission to JMRI Sensor Table
//  Author: Geoff Bunza 2018
//   Version 1.2
//  Transmission starts with a synchronizing character, here the character "A" followed by 
//  1 byte with  bit 7 Sensor ON/OFF bit 6-0 sensor # 1-127
//  Transmission is received in JMRI via SerialSensorMux.py Python Script
//  It is assumed that the Arduino sensor mux starts up before the Python Script which will transmit "!!!\n"
//  To indicate the script is running and ready for reception
//  The Arduino will then poll all (up to 70) sensors and transmit thir states for initialization in the JMRI sensor table
//  The serial port assigned to this Arduino must correspond to the Serial Port in the corresponding Sensor Script
//  The Arduino will update JMRI ONLY upon detecting a sensor change minimizing overhead transmission to JMRI

// Sampling Parameters
const unsigned long sampleTime = 100000UL; // sample over 100ms
const unsigned long numSamples = 250UL; // the number of samples divides sampleTime exactly, 
                                        // but low enough for the ADC to keep up
const unsigned long sampleInterval = sampleTime/numSamples;  // the sampling interval
                                         //  must be longer than then ADC conversion time
#define SENSITIVITY 185  // from ACS712 data sheet for 5A version, in mV/A
int adc_zero[16];  // variable to hold calibrated sensor quiescent voltage
boolean occupied = false;
float occupancy_threshold = .0259;
float current = 0;
float prev_current;



#define Digital_Sensor_Pin_Max   54  // Max sensor pin NUMBER (plus one) Mega=70,UNO,Pro Mini,Nano=20
#define Analogue_Sensor_Pin_Max   16  // Max sensor pin NUMBER (plus one) Mega=70,UNO,Pro Mini,Nano=20
#define Digital_Sensor_Pin_Start  2  // Starting Sensor Pin number (usually 2 as 0/1 are TX/RX
#define Analogue_Sensor_Pin_Start  0  // Starting Sensor Pin number (usually 2 as 0/1 are TX/RX
#define Sensor_Offset     0  // This Offset will be ADDED to the value of each Sensor_Pin to determine the sensor
                             // number sent to JMRI, so pin D12 will set sensor AR:(12+Sensor_Offset) in JMRI
                             // This would allow one Arduino Sensor channel to set sensors 2-69 and another to 
                             // Set sensors 70-137 for example; this offset can also be negative
#define Sensors_Active_Low 1 // Set Sensors_Active_Low to 1 if sensors are active LOW
                             // Set Sensors_Active_Low to 0 if sensors are active HIGH
#define open_delay 15        // longer delay to get past script initialization
#define delta_delay 4        // Short delay to allow the script to get all the characters
int i;
char  sensor_state [70];     // up to 70 sensors on a Mega2560
char  new_sensor_state ;     // temp to process the possible state change
char  incomingByte = 0;      // working temp for character processing

void setup(){
    Serial.begin(19200);              // Open serial connection.
    while (Serial.available() == 0);  // wait until we get a charater from JMRI
    incomingByte=Serial.read();       // get the first character
    while ((Serial.available() > 0) && (incomingByte != '!')) incomingByte=Serial.read(); //get past !!!
    while ((Serial.available() > 0) ) incomingByte=Serial.read();                       //flush anything else
    delay(open_delay);                 // take a breath
    for ( i=Digital_Sensor_Pin_Start; i<Digital_Sensor_Pin_Max; i++)  {  //Initialize all sensors in JMRI and grab each sensor
       pinMode(i, INPUT_PULLUP);       // define each sensor pin as coming in
       sensor_state[i] = (digitalRead( i ))^Sensors_Active_Low;    // read & save each sensor state & invert if necessary
       Serial.print("A"); Serial.print (char((sensor_state[i]<<7)+i+Sensor_Offset));  // send "A <on/off><snesor #>" to JMRI script
       delay(delta_delay);             // in milliseconds, take a short breath as not to overwhelm JMRI's seraial read
    }

   for ( i=Analogue_Sensor_Pin_Start; i<Analogue_Sensor_Pin_Max; i++)  {  //Initialize all sensors in JMRI and grab each sensor
      Serial.println("\nACS712 Current Sensing Basic Demonstration\nMultiple Readings converted to RMS at 1 second intervals\nValues for quiescent output are determined by Calibration.\n\nCalibrating the sensor:\n");
      adc_zero[i] = determineVQ(i); //Quiescent output voltage - the average voltage ACS712 shows with no load (0 A)
      delay(1000);
   }    
}

void loop()  {
   for ( i=Digital_Sensor_Pin_Start; i<Digital_Sensor_Pin_Max; i++)  {     // scan every sensor over and over for any sensor changes
       new_sensor_state = (digitalRead( i ))^Sensors_Active_Low;  // read & save each sensor state & invert if necessary
       if (new_sensor_state != sensor_state[i] )  {               // check if the sensor changed ->if yes update JMRI
         Serial.print("A"); Serial.print (char((new_sensor_state<<7)+i+Sensor_Offset)); // send "A <on/off><snesor #>" to JMRI script
         sensor_state[i] = new_sensor_state ;                     // save the updated sensor state
         delay(delta_delay);            // in milliseconds, take a short breath as not to overwhelm JMRI's seraial read
       }
   }
   
   for ( i=Analogue_Sensor_Pin_Start; i<Analogue_Sensor_Pin_Max; i++)  {     // scan every sensor over and over for any sensor changes

          float avg_current;
          prev_current = current;
          current = readCurrent(i, adc_zero[i]);
          avg_current = (current + prev_current)/2;
          occupied = avg_current > occupancy_threshold;
  
          Serial.print("Current Sensed:");
          Serial.print(current * 1000 ,1);
          Serial.print(" mA\t\t");
          Serial.print(analogRead(i));
          Serial.print("The block is ");
          if(occupied){
            new_sensor_state = 1;
          } else {
            new_sensor_state = 0;
          }
          
       if (new_sensor_state != sensor_state[i+54] )  {               // check if the sensor changed ->if yes update JMRI
         Serial.print("A"); Serial.print (char((new_sensor_state<<7)+i+Sensor_Offset)); // send "A <on/off><snesor #>" to JMRI script
         sensor_state[i+54] = new_sensor_state ;                     // save the updated sensor state
         delay(delta_delay);            // in milliseconds, take a short breath as not to overwhelm JMRI's seraial read
       }
   }
}


int determineVQ(int PIN) {
  Serial.print("estimating avg. quiscent voltage:");
  long VQ = 0;
  //read 5000 samples to stabilize value
  for (int i=0; i<5000; i++) {
    VQ += analogRead(PIN);
    delay(1);//depends on sampling (on filter capacitor), can be 1/80000 (80kHz) max.
  }
  VQ /= 5000;
  Serial.print(map(VQ, 0, 1023, 0, 5000));
  Serial.println(" mV");
  return int(VQ);
}

float readCurrent(int PIN, float adc_zero)
{
  float currentAcc = 0;
  unsigned int count = 0;
  unsigned long prevMicros = micros() - sampleInterval ;
  while (count < numSamples)
  {
    if (micros() - prevMicros >= sampleInterval)
    {
      float adc_raw = analogRead(PIN) - adc_zero; // Electical offset voltage
      adc_raw /= SENSITIVITY; // convert to amperes
      currentAcc += (adc_raw * adc_raw);
      ++count;
      prevMicros += sampleInterval;
    }
  }
  //https://en.wikipedia.org/wiki/Root_mean_square
  float rms = sqrt((float)currentAcc / (float)numSamples);
  return rms;
}
