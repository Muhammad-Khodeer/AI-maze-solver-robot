#define encoder0PinA  2

volatile unsigned int encoder0Pos = 0;

void setup() { 


  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor

  attachInterrupt(0, doEncoderForward, RISING );  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(0, doEncoderBack, FALLING);  // encoder pin on interrupt 0 - pin 2
} 

void loop(){

}

void doEncoderForward() {
  /* If pinA is high forward. 
   *  else Backword
   */
    encoder0Pos++;
  
}
void doEncoderBack() {
  /* If pinA is high forward. 
   *  else Backword
   */

    encoder0Pos--;
  
}
