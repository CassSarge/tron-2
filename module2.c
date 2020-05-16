///Function definitions for all of module 2
///The main focus of this module is in looping through the ranges provided by using the given parameters.
///The inputs to this module come from Module 1, and it calls Module 3 to get additional IMU data.
/// The outputs go to Module 4 to be displayed in real-time, and are compiled in an array for Module 4 to store.
/// Angle used inside the function are processed and returned as integers corresponding to angles^-1.
/// e.g. 120.2 degrees is processed as int 1202.

#include "module2.h"

int module2(int minAzimuth, int maxAzimuth, int minElevation, int maxElevation, 
            int resolution, int samples_per_orientation, int sampleFrequency, int modeFlag){

// minAzimuth and maxAzimuth: Between 30-160 degrees, expressed as degrees.
// minElevation and maxElevation: Between -60 -> 60 degrees, expressed as degrees.
// Resolution: Size of step, expressed as degrees^-1 (i.e. 1 represents 0.1 degrees.)
// Samples_per_orientation: How many range samples we take at each discrete point before they are averaged
// for a single more accurate value.
// sample_frequency: ???
// mode_flag: [1,2,3] -> [Online, Offline, Demo]


int servoAzi=0,servoEle=0;
int imuInfo[4]; 
int range;
int result;

// As resolution is down to 0.1 degrees expressed as integer 1, all other angles must 
// follow the same convention to avoid using floats.

maxAzimuth    *= 10; // e.g. 160 -> 1600  
minAzimuth    *= 10; // e.g. 30  -> 300
maxElevation  *= 10; // e.g. 60  -> 600
minElevation  *= 10; // e.g. -60 -> -600


// Start the main loop for going through every single point the servo should get data from
// This loop goes along the entire horizontal at one elevation, then jumps to the final horizontal point
// and take a final reading before resetting to the initial horizontal, incrementing elevation, and repeating
// 1) Set elevation
// 2) Set azimuth
// 3) Get and transmit readings
// 4) Increment azimuth by resolution while not up to maximum azimuth, and repeat from 2
// 5) Jump to final azimuth regardless of resolution
// 6) Get and transmit readings
// 7) Increment heading by resolution while not up to maximum azimuth, and repeat from 1
// 8) Jump to final elevation regardless of resolution
// 9) Repeat steps 2 to 6
// _______________________________________________________________________________________

// Start at our initial elevation angle, and then incremement by resolution (in degrees^-1)
for(servoEle = minElevation; servoEle < maxElevation; servoEle += resolution){
      setElevation(servoEle);
     


     // Start at our initial azimuth angle, and then incremement by resolution (in degrees^-1) 
     for(servoAzi = minAzimuth; servoAzi < maxAzimuth; servoAzi += resolution){
        setAzimuth(servoAzi);
      
        // TRANSMITTING DATA HERE//
        range = sampleGetRange(sampleFrequency, samples_per_orientation);  // Get laser range
        sampleGetIMUdata(imuInfo);          // Get IMU data
        sampleSendData(range, servoAzi, servoEle, imuInfo[0], imuInfo[1], imuInfo[2], imuInfo[3]); // Send
        
        
        
     }
    // Get values at the upper edge of azimuth
    setAzimuth(maxAzimuth); // Jump to maximum azimuth from closest position without overflow
    
    // Get laser range
    // Get IMU data
    // TRANSMITTING DATA HERE//
      range = sampleGetRange(sampleFrequency, samples_per_orientation);  // Get laser range
      sampleGetIMUdata(imuInfo);          // Get IMU data
      sampleSendData(range, maxAzimuth,servoEle, imuInfo[0], imuInfo[1], imuInfo[2], imuInfo[3]); // Send    
    
    setAzimuth(minAzimuth);
    // Delay (because it takes longer for azimuth to reset from max to zero)
          
}
 
// Get values at the upper edge of elevation
 
  setElevation(maxElevation);
  for(servoAzi = minAzimuth;servoAzi < maxAzimuth; servoAzi += resolution){
          setAzimuth(servoAzi);               

          // Get laser range
          // Get IMU data
          // TRANSMITTING DATA HERE //
          range = sampleGetRange(sampleFrequency, samples_per_orientation);  // Get laser range
          sampleGetIMUdata(imuInfo);          // Get IMU data
          sampleSendData(range, servoAzi, servoEle, imuInfo[0], imuInfo[1], imuInfo[2], imuInfo[3]); // Send

  }

// Get values from the final boundary point
setAzimuth(maxAzimuth);
          // Get laser range
          // Get IMU data
          // TRANSMITTING DATA HERE //
          range = sampleGetRange(sampleFrequency,samples_per_orientation);  // Get laser range
          sampleGetIMUdata(imuInfo);          // Get IMU data
          sampleSendData(range, servoAzi, servoEle, imuInfo[0], imuInfo[1], imuInfo[2], imuInfo[3]); // Send

// Reset both servos back to their initial positions.
      //initServo();



 
 
 
result = modeFlag; // Testing (get rid of variable not used warnings)
return result;
    
}



void testLaser(void){  
  
  // Enable input capture and interrupts
   TIOS = 0x02; // Enable channel 1 for input capture
   TCTL3= 0x00;
   TCTL4= 0x0C; // Enable capture on rising or falling edge for channel 1 only
   TIE  = 0x02; // Enable channel 1 interrupt
   

   // Enable the LEDs 
   DDRB =   0xFF;
   DDRJ =   0xFF;
   PTJ =    0x00;
   PORTB =  0x00; // Turn LEDs off
   /*
   while(TRUE){
      PORTB = PTT; // Fill the LEDs with what we read from port T
      // Wait forever while we wait for the interrupt to trigger
   }
   */
}

//_____________________________________________________________________________________

/// Initialise the servo. 
/// This function sets up the servo so it can be used later without further setup. 
/// it turns the servo on, and sets it to the position (0,0). 
/// Channel 5 is azimuth, and channel 7 is elevation. 
//_____________________________________________________________________________________
void initServo(void){
// Period*(24*10^6) = clock cycles
// Need 480 000 clock cycles as period
// Need  36 000 clock cycles for 0 point
// 24*10^6/480 000 = 50Hz (frequency of our PWM signal)
// PWMPERx = PeriodClockCycles/Prescalar
// So we choose a prescalar of 8, so that period can be 60 000 (0xEA60), which is within the 16 bit range we need with concatanated channels
// Thus us new formula is Period*(24*10^6)/8 = PWMPERx or PWMDTYx, where we input the high byte in 6, low in 7 etc.

 
       DDRP = 0xFF; // Make Port P an output
    // Set registers
       PWMPOL = 0xFF; // Make polarity 1 for all channels (Duty cycle is spent high)
       PWMCAE = 0x00; // All channels are left aligned
       PWMCTL = 0xC0; // Concatenate channels 6&7 and 4&5 to make them 16 bit registers.(11000000)
       
    // Prescale by factor of 8     
       PWMCLK =   0x00; // Make Clocks A and B the sources
       PWMPRCLK = 0x33; // Prescale both clocks by 8. (00110011)
       
    // Set period for both channels
       PWMPER4 = 0xEA; // Set high byte
       PWMPER5 = 0x60; // Set low byte
       PWMPER6 = 0xEA; // Set high byte
       PWMPER7 = 0x60; // Set low byte
    // Now both pin 5 and pin 7 have a period of 60 000*8 clock cycles = 20ms  
       
    // Set duty cycle for both channels to be their 0 degree point
    // = 36 000/8 = 4500 = 0x1194
       PWMDTY4 = 0x11; // Set high byte
       PWMDTY5 = 0x94; // Set low byte
       PWMDTY6 = 0x11; // Set high byte
       PWMDTY7 = 0x94; // Set low byte
     
       PWME = 0xA0;    //Turn on PWM channel 5 and 7

    // These will run the PWM channels until theyre disabled
}

//_____________________________________________________________________________________
/// Set the azimuth servo to the given angle. 
/// Angle input can be between 30 and 160 in the format 300 to 1600
/// 0.9ms duty cycle --> -90, 2.1ms duty cycle --> 90.  
/// Therefore we map the duty cycle range of 1.2ms over the angle range of 180 degrees. 
/// PWMDTYx = (0.9ms + (((angle input)+90)/180)*1.2ms)*24*10^6. 
/// This formula simplifies into PWMDTY = 21600 + 16*(angle input + 900)
//_____________________________________________________________________________________
void setAzimuth(int angle){
  
  
  int dutyCycle = 0;
  unsigned char highByte;
  unsigned char lowByte;

  // Check that the elevation entered is a legal value
  if(angle<MIN_AZ || angle>MAX_AZ){
    sampleAbort(); // Abort operation if there is an illegal value
  }


  // Our formula is general for -90 to +90, which we must map to from 30 to 160 for azimuth.
  // We will call -90 degress our zero point, meaning we subtract 90 degrees from every input to make it
  // fit our formula. E.g. input of 30 -> -60, input of 160 -> 70. 

  // Apply the formula for finding duty cycle with our specified angle, dividing by 8 to align with clock prescalar
  dutyCycle = 2*(angle) + 2700;
  
  lowByte = dutyCycle & 0xFF; // Acquire low byte of duty cycle through bitmasking
  highByte= dutyCycle >> 8;   // Acquire high byte of duty cycle through bit shifting
  
  PWMDTY4 = highByte;  // Set high byte 
  PWMDTY5 = lowByte;   // Set low byte
  
}

//_____________________________________________________________________________________
/// Set the elevation servo to the given angle. 
/// Angle input can be between -60 and 60. 
/// This is within range of -90 to 90 (servo limits) and does not need to be remapped. 
/// 0.9ms duty cycle --> -90, 2.1ms duty cycle --> 90.  
/// Therefore we map the duty cycle range of 1.2ms over the angle range of 180 degrees. 
/// PWMDTYx = (0.9ms + (((angle input)+90)/180)*1.2ms)*24*10^6. 
/// This formula simplifies into PWMDTY = 21600 + 16*(angle input + 900)
//_____________________________________________________________________________________

void setElevation(int angle){
  
  int dutyCycle = 0;
  unsigned char highByte;
  unsigned char lowByte;

  // Check that the elevation entered is a legal value
  if(angle<MIN_EL || angle>MAX_EL){
    sampleAbort(); // Abort operation if there is an illegal value
  }


  // Apply the formula for finding duty cycle with our specified angle, dividing by 8 to align with clock prescalar
  dutyCycle = 2*(900+angle) + 2700;

  lowByte = dutyCycle & 0xFF; // Acquire low byte of duty cycle through bitmasking
  highByte= dutyCycle >> 8;   // Acquire high byte of duty cycle through bit shifting
  
  PWMDTY6 = highByte;  // Set high byte 
  PWMDTY7 = lowByte;   // Set low byte
  
}

int sampleGetRange(int sampleFrequency, int samples_per_orientation){
 
 int sum = 0; 
 int range = 0;
 int i=0;
 
 for(i=0; i < samples_per_orientation; i++){  
    sum += sampleFrequency; // TODO: Actually interface with the laser here
 }
 
 range = sum/samples_per_orientation; // Find average range
 
 
 return range; 
  
}


void sampleGetIMUdata(int* imuInfo){
  
 imuInfo[0] = 30;
 imuInfo[1] = 40;
 imuInfo[2] = 50; 
 imuInfo[3] = 60;
  
}

void sampleSendData(int range, int servoAzi, int servoEle, 
                int IMU1, int IMU2, int IMU3, int IMU4){
  
    _FEED_COP();  // Do nothing
  
  
}

void sampleAbort(void){
  
 int ever=1;
 for(;ever;){
   // Wait forever and do nothing after an error
 }
  
}



interrupt 9 void inputChan1ISR (void) {
  
    PORTB = 0xFF; //Turn on all the LEDs
  
}
