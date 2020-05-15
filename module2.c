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

int aziRange;
int aziSteps;
int eleRange;
int eleSteps;
int i,j;

int result;

// As resolution is down to 0.1 degrees expressed as integer 1, all other angles must 
// follow the same convention to avoid using floats.

maxAzimuth    *= 10; // e.g. 160 -> 1600
minAzimuth    *= 10; // e.g. 30  -> 300
maxElevation  *= 10; // e.g. 60  -> 600
minElevation  *= 10; // e.g. -60 -> -600



aziRange = maxAzimuth - minAzimuth;     // Find ranges we need to step through for azimuth
aziSteps = (aziRange/resolution);    // Find the number of steps we take through the azimuth range
                                        // Multiplied by 10 for conversion of resolution to step size
                                        
eleRange = maxElevation - minElevation; // Find ranges we need to step through for elevation
eleSteps = (eleRange/resolution);    // Find the number of steps we take through the elevation range
                                        // Multiplied by 10 for conversion of resolution to step 

// Start at our initial elevation angle, and then incremement by resolution (in degrees^-1)
for(i=minElevation;i<maxElevation;i+=resolution){
     // SetElevation(i)
     


     // Start at our initial azimuth angle, and then incremement by resolution (in degrees^-1) 
     for(j=minAzimuth;j<maxAzimuth;j+=resolution){
        // SetAzimuth(j)
      
      
        // Get laser range
        // Get IMU data
        // Store in array
        // Send 
      
        _FEED_COP();
      
     }
    // Get values at the upper edge of azimuth
    // SetAzimuth(maxAzimuth) // Jump to maximum azimuth from closest position without overflow
    
    // Get laser range
    // Get IMU data
    // Store in array
    // Send
    _FEED_COP(); 
    // SetAzimuth(minAzmuth)
    // Delay (because it takes longer for azimuth to reset from max to zero)
          
}
 
// Get values at the upper edge of elevation
 
// SetElevation(maxElevation)
  for(j=minAzimuth;j<maxAzimuth;j+=resolution){
          // SetAzimuth(j)
        
        
          // Get laser range
          // Get IMU data
          // Store in array
          // Send 
        
          _FEED_COP();
        
  }
 
 
 
result = aziRange + aziSteps + eleRange + eleSteps + sampleFrequency + modeFlag + samples_per_orientation; // Testing (get rid of variable not used warnings)
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
       PWMPER7 = 0x6A; // Set low byte
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
/// Angle input can be between -90 and 90 (this will be remapped later so we will call the lower range 30 and upper range 160)
/// 0.9ms duty cycle --> -90, 2.1ms duty cycle --> 90.  
/// Therefore we map the duty cycle range of 1.2ms over the angle range of 180 degrees. 
/// PWMDTYx = (0.9ms + (((angle input)+90)/180)*1.2ms)*24*10^6. 
/// This formula simplifies into PWMDTY = 21600 + 16*(angle input + 900)
//_____________________________________________________________________________________
void SetAzimuth(int angle){
  
  
  int dutyCycle;
 // int temp;
  unsigned char highByte;
  unsigned char lowByte;
   angle = 0;
  // Apply the formula for finding duty cycle with our specified angle, dividing by 8 to align with clock prescalar
  dutyCycle = 900 + angle;
  dutyCycle = 16*dutyCycle;
  dutyCycle = dutyCycle/8;
  dutyCycle = dutyCycle + 2700;

  // Why does dutyCycle become undefined here??
  
  lowByte = dutyCycle & 0xFF; // Acquire low byte of duty cycle through bitmasking
  highByte= dutyCycle >> 8;   // Acquire high byte of duty cycle through bit shifting
  
  PWMDTY4 = highByte;  // Set high byte 
  PWMDTY5 = lowByte;   // Set low byte
  
}



interrupt 9 void inputChan1ISR (void) {
  
    PORTB = 0xFF; //Turn on all the LEDs
  
}
