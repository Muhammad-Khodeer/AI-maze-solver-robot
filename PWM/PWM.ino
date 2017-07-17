//PWM  works on pins 3, 5, 6, 9, 10, and 11.
int ledPin = 9;      // LED connected to digital pin 9

void setup()

{
  pinMode(ledPin, OUTPUT);   // sets the pin as output

}

void loop()

{

  for(int i = 0 ; i<=255 ; i++){
  analogWrite(ledPin, i);  
  delay(1000);
  }
}

