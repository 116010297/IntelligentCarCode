#include <Servo.h> 
String comdata = "";

void Rudder(int angle) { 
  for(int i=0;i<50;i++){
    int pulsewidth = map(((angle-90)/1.5527+83), 0, 180, 500, 2500);
    digitalWrite(9, HIGH);   
    delayMicroseconds(pulsewidth);  
    digitalWrite(9, LOW);    
    delayMicroseconds(20000 - pulsewidth);
  }
}

void setup() 
{
  // origin
  // Serial.begin(115200);
  Serial.begin(57600);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9,OUTPUT);
  Rudder(87);
}



void loop()
{

      if(Serial.available())  
      {  
        while(Serial.available()>0)  
        {  
          comdata+=char(Serial.read());  
          delay(2);  
        }  
        if(comdata.length() == 9){
           int pwm1 = comdata.substring(0,3).toInt();
           int pwm2 = comdata.substring(3,6).toInt();
           int pos = comdata.substring(6).toInt();

           if(pos >= 40 && pos <= 130){
              Serial.println("rudder attached");
              Rudder(pos);
           }
           
           Serial.println(pwm1);
           Serial.print(pwm2);
           Serial.print(pos);
//           change speed
           analogWrite(5,pwm1);
           analogWrite(6,pwm2);
           
        }else{
//         stop it
           analogWrite(5,LOW);
           analogWrite(6,LOW); 
        }
        comdata="";  
     }  
 
 }
