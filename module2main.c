

#include "main_asm.h" /* interface to the assembly module */
#include "module2.h" // Include the function definitions and defines for this module


/// Main loop
/// Calls module 2 for testing
void main(void){
  
  
  // Initialise the servo
  initServo();
  
 // result = module2(0,160,-60,60, 10, 4, 1,1);

	EnableInterrupts;
 
 
  testLaser();
 
 
 

    //
  for(;;) {
  //_FEED_COP(); // Feed that dog
  PORTB = PTP;
    
  } /* loop forever */
  /* please make sure that you never leave main */
}


/*




*/ 


