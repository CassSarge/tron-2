

#include "main_asm.h" /* interface to the assembly module */
#include "module2.h" // Include the function definitions and defines for this module


/// Main loop
/// Calls module 2 for testing
void main(void){
 
 int result=0;
 
 	EnableInterrupts;
 
  // Initialise the servo
  initServo();

  // Take and transmit measurements for every position
  result = module2(30,160,-60,60, 100, 4, 1,1);



 


    //
  for(;;) {
  //_FEED_COP(); // Feed that dog
  PORTB = PTP;
    
  } /* loop forever */
  /* please make sure that you never leave main */
}


