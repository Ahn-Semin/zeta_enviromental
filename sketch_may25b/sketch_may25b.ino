const int pwm = 2 ;     //initializing pin 2 as ‘pwm’ variable
void setup()
{
     pinMode(pwm,OUTPUT) ;  //Set pin 2 as output
}
void loop()
{
     analogWrite(pwm,25) ;     //setting pwm to 25 
     delay(50) ;      //delay of 50 ms
     analogWrite(pwm,50) ;  
     delay(50) ; 
     analogWrite(pwm,75) ; 
     delay(50) ; 
     analogWrite(pwm,100) ; 
     delay(50) ; 
     analogWrite(pwm,125) ; 
     delay(50) ; 
     analogWrite(pwm,150) ; 
     delay(50) ; 
     analogWrite(pwm,175) ; 
     delay(50) ;
     analogWrite(pwm,200) ; 
     delay(50) ; 
     analogWrite(pwm,225) ; 
     delay(50) ; 
     analogWrite(pwm,250) ; 
}
