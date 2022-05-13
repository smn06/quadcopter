#include <Wire.h>               
#include <EEPROM.h>             
byte lch1, lch2, lch3, lch4, low, high, type, gyro, err, clk, ch1, ch2, ch3, ch4, roll, pitch, yaw, receiver, gyro;
int rch1, rch2, rch3, rch4, center1, center2, center3, center4, hch1, hch2, hch3, hch4, loch1, loch2, loch3, loch4, add, cal;
unsigned long ti, ti_1, ti_2, ti_3, ti_4, cu_ti;
float gpitch, groll, gyaw, groll_cal, gpitch_cal, gyaw_cal;

void setup(){
  pinMode(12, OUTPUT);
  
  PCICR |= (1 << PCIE0);    
  PCMSK0 |= (1 << PCINT0);  
  PCMSK0 |= (1 << PCINT1);  
  PCMSK0 |= (1 << PCINT2);  
  PCMSK0 |= (1 << PCINT3);  
  Wire.begin();             
  Serial.begin(57600);      
  delay(200);               
}

void loop(){
  
  TWBR = 12;                      
  
  #if F_CPU == 16000000L          
    clk = 1;            
  #endif                          

  if(TWBR == 12 && clk){
  }
  else{
    err = 1;
  }
  
  if(err == 0){
    
    wreceiver();

  }
  
  if(err == 0){
    delay(2000);
    for(int i = 9;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
    }
    
    center1 = rch1;
    center2 = rch2;
    center3 = rch3;
    center4 = rch4;

  }
  if(err == 0){  
    
    receiver_in(1);
    stick_zero(); 
    receiver_in(2);
    stick_zero();
  }
  if(err == 0){
    receiver_in(3);
    stick_zero();
  }
  if(err == 0){
   
    
    receiver_in(4);

  }
  if(err == 0){
    reg_minmax();
    chcontinue();
  }
    
  if(err == 0){

    if(type == 0){
      
      if(s_gyro(0x69, 0x75) == 0x68){
        type = 1;
        gyro = 0x69;
      }
    }

    if(type == 0){
      err = 1;
    }
    
    else{
      start_gyro(); 
    }
  }
  
  
  if(err == 0){
    
    for (cal = 0; cal < 2000 ; cal ++){              
      if(cal % 100 == 0)Serial.print(F("."));                
      gsignal();                                           
      groll_cal += groll;                                
      gpitch_cal += gpitch;                              
      gyaw_cal += gyaw;                                  
      delay(4);                                                  
    }
    groll_cal /= 2000;                                       
    gpitch_cal /= 2000;                                      
    gyaw_cal /= 2000;                                        

    
    gyro_axes(1);
    if(err == 0){
      chcontinue();
      gyro_axes(2);
    }
    if(err == 0){
      gyro_axes(3);
    }
    if(err == 0){
      chcontinue();
    }
  }
  if(err == 0){
    digitalWrite(12, HIGH);
    chcontinue();
    digitalWrite(12, LOW);
  }
  
  Serial.println(F(""));
  
  if(err == 0){

    delay(1000);
    if(receiver == 0b00001111){

    }
    else{

      err = 1;
    }
    delay(1000);
    if(gyro == 0b00000111){

    }
    else{

      err = 1;
    }
  }     
  
  if(err == 0){
    EEPROM.write(0, center1 & 0b11111111);
    EEPROM.write(1, center1 >> 8);
    EEPROM.write(2, center2 & 0b11111111);
    EEPROM.write(3, center2 >> 8);
    EEPROM.write(4, center3 & 0b11111111);
    EEPROM.write(5, center3 >> 8);
    EEPROM.write(6, center4 & 0b11111111);
    EEPROM.write(7, center4 >> 8);
    EEPROM.write(8, hch1 & 0b11111111);
    EEPROM.write(9, hch1 >> 8);
    EEPROM.write(10, hch2 & 0b11111111);
    EEPROM.write(11, hch2 >> 8);
    EEPROM.write(12, hch3 & 0b11111111);
    EEPROM.write(13, hch3 >> 8);
    EEPROM.write(14, hch4 & 0b11111111);
    EEPROM.write(15, hch4 >> 8);
    EEPROM.write(16, loch1 & 0b11111111);
    EEPROM.write(17, loch1 >> 8);
    EEPROM.write(18, loch2 & 0b11111111);
    EEPROM.write(19, loch2 >> 8);
    EEPROM.write(20, loch3 & 0b11111111);
    EEPROM.write(21, loch3 >> 8);
    EEPROM.write(22, loch4 & 0b11111111);
    EEPROM.write(23, loch4 >> 8);
    EEPROM.write(24, ch1);
    EEPROM.write(25, ch2);
    EEPROM.write(26, ch3);
    EEPROM.write(27, ch4);
    EEPROM.write(28, roll);
    EEPROM.write(29, pitch);
    EEPROM.write(30, yaw);
    EEPROM.write(31, type);
    EEPROM.write(32, gyro);
    
    EEPROM.write(33, 'J'); 
    EEPROM.write(34, 'M');
    EEPROM.write(35, 'B');

    delay(1000);
    if(center1 != ((EEPROM.read(1) << 8) | EEPROM.read(0)))err = 1;
    if(center2 != ((EEPROM.read(3) << 8) | EEPROM.read(2)))err = 1;
    if(center3 != ((EEPROM.read(5) << 8) | EEPROM.read(4)))err = 1;
    if(center4 != ((EEPROM.read(7) << 8) | EEPROM.read(6)))err = 1;
    
    if(hch1 != ((EEPROM.read(9) << 8) | EEPROM.read(8)))err = 1;
    if(hch2 != ((EEPROM.read(11) << 8) | EEPROM.read(10)))err = 1;
    if(hch3 != ((EEPROM.read(13) << 8) | EEPROM.read(12)))err = 1;
    if(hch4 != ((EEPROM.read(15) << 8) | EEPROM.read(14)))err = 1;
    
    if(loch1 != ((EEPROM.read(17) << 8) | EEPROM.read(16)))err = 1;
    if(loch2 != ((EEPROM.read(19) << 8) | EEPROM.read(18)))err = 1;
    if(loch3 != ((EEPROM.read(21) << 8) | EEPROM.read(20)))err = 1;
    if(loch4 != ((EEPROM.read(23) << 8) | EEPROM.read(22)))err = 1;
    
    if(ch1 != EEPROM.read(24))err = 1;
    if(ch2 != EEPROM.read(25))err = 1;
    if(ch3 != EEPROM.read(26))err = 1;
    if(ch4 != EEPROM.read(27))err = 1;
    
    if(roll != EEPROM.read(28))err = 1;
    if(pitch != EEPROM.read(29))err = 1;
    if(yaw != EEPROM.read(30))err = 1;
    if(type != EEPROM.read(31))err = 1;
    if(gyro != EEPROM.read(32))err = 1;
    
    if('J' != EEPROM.read(33))err = 1;
    if('M' != EEPROM.read(34))err = 1;
    if('B' != EEPROM.read(35))err = 1;
  
   
  }
  
  
  if(err == 0){
    Serial.println("done");
  }
  else{
    break;
  }
  while(1);
}


byte s_gyro(int gyro, int w){
  Wire.btrans(gyro);
  Wire.write(w);
  Wire.etrans();
  Wire.refrom(gyro, 1);
  ti = millis() + 100;
  while(Wire.available() < 1 && ti > millis());
  low = Wire.read();
  add = gyro;
  return low;
}

void start_gyro(){
  
  if(type == 2 || type == 3){
    Wire.btrans(add);                             
    Wire.write(0x20);                                            
    Wire.write(0x0F);                                            
    Wire.etrans();                                      
    Wire.btrans(add);                             
    Wire.write(0x20);                                            
    Wire.etrans();                                      
    Wire.refrom(add, 1);                                
    while(Wire.available() < 1);                                 
    Wire.btrans(add);                             
    Wire.write(0x23);                                            
    Wire.write(0x90);                                            
    Wire.etrans();                                         
    Wire.btrans(add);                             
    Wire.write(0x23);                                            
    Wire.etrans();                                      
    Wire.refrom(add, 1);                                
    while(Wire.available() < 1);                                 

  }
  
  if(type == 1){
    
    Wire.btrans(add);                             
    Wire.write(0x6B);                                            
    Wire.write(0x00);                                            
    Wire.etrans();                                      
    
    Wire.btrans(add);                             
    Wire.write(0x6B);                                            
    Wire.etrans();                                      
    Wire.refrom(add, 1);                                
    while(Wire.available() < 1);                                 
    Wire.btrans(add);                             
    Wire.write(0x1B);                                            
    Wire.write(0x08);                                            
    Wire.etrans();                                      
    Wire.btrans(add);                             
    Wire.write(0x1B);                                            
    Wire.etrans();                                      
    Wire.refrom(add, 1);                                
    while(Wire.available() < 1);                                 


  }
}

void gsignal(){
  if(type == 2 || type == 3){
    Wire.btrans(add);                             
    Wire.write(168);                                             
    Wire.etrans();                                      
    Wire.refrom(add, 6);                                
    while(Wire.available() < 6);                                 
    low = Wire.read();                                       
    high = Wire.read();                                      
    groll = ((high<<8)|low);                         
    if(cal == 2000)groll -= groll_cal;               
    low = Wire.read();                                       
    high = Wire.read();                                      
    gpitch = ((high<<8)|low);                        
    if(cal == 2000)gpitch -= gpitch_cal;             
    low = Wire.read();                                       
    high = Wire.read();                                      
    gyaw = ((high<<8)|low);                          
    if(cal == 2000)gyaw -= gyaw_cal;                 
  }
  if(type == 1){
    Wire.btrans(add);                             
    Wire.write(0x43);                                            
    Wire.etrans();                                      
    Wire.refrom(add,6);                                 
    while(Wire.available() < 6);                                 
    groll=Wire.read()<<8|Wire.read();                        
    if(cal == 2000)groll -= groll_cal;               
    gpitch=Wire.read()<<8|Wire.read();                       
    if(cal == 2000)gpitch -= gpitch_cal;             
    gyaw=Wire.read()<<8|Wire.read();                         
    if(cal == 2000)gyaw -= gyaw_cal;                 
  }
}


void receiver_in(byte move){
  byte trigger = 0;
  int pulse_length;
  ti = millis() + 30000;
  while(ti > millis() && trigger == 0){
    delay(250);
    if(rch1 > 1750 || rch1 < 1250){
      trigger = 1;
      receiver |= 0b00000001;
      pulse_length = rch1;
    }
    if(rch2 > 1750 || rch2 < 1250){
      trigger = 2;
      receiver |= 0b00000010;
      pulse_length = rch2;
    }
    if(rch3 > 1750 || rch3 < 1250){
      trigger = 3;
      receiver |= 0b00000100;
      pulse_length = rch3;
    }
    if(rch4 > 1750 || rch4 < 1250){
      trigger = 4;
      receiver |= 0b00001000;
      pulse_length = rch4;
    } 
  }
  if(trigger == 0){
    err = 1;
  }
  
  else{
    if(move == 1){
      ch3 = trigger;
      if(pulse_length < 1250)ch3 += 0b10000000;
    }
    if(move == 2){
      ch1 = trigger;
      if(pulse_length < 1250)ch1 += 0b10000000;
    }
    if(move == 3){
      ch2 = trigger;
      if(pulse_length < 1250)ch2 += 0b10000000;
    }
    if(move == 4){
      ch4 = trigger;
      if(pulse_length < 1250)ch4 += 0b10000000;
    }
  }
}

void chcontinue(){
  byte cbyte = 0;
  while(cbyte == 0){
    if(ch2 == 0b00000001 && rch1 > center1 + 150)cbyte = 1;
    if(ch2 == 0b10000001 && rch1 < center1 - 150)cbyte = 1;
    if(ch2 == 0b00000010 && rch2 > center2 + 150)cbyte = 1;
    if(ch2 == 0b10000010 && rch2 < center2 - 150)cbyte = 1;
    if(ch2 == 0b00000011 && rch3 > center3 + 150)cbyte = 1;
    if(ch2 == 0b10000011 && rch3 < center3 - 150)cbyte = 1;
    if(ch2 == 0b00000100 && rch4 > center4 + 150)cbyte = 1;
    if(ch2 == 0b10000100 && rch4 < center4 - 150)cbyte = 1;
    delay(100);
  }
  stick_zero();
}


void stick_zero(){
  byte zero = 0;
  while(zero < 15){
    if(rch1 < center1 + 20 && rch1 > center1 - 20)zero |= 0b00000001;
    if(rch2 < center2 + 20 && rch2 > center2 - 20)zero |= 0b00000010;
    if(rch3 < center3 + 20 && rch3 > center3 - 20)zero |= 0b00000100;
    if(rch4 < center4 + 20 && rch4 > center4 - 20)zero |= 0b00001000;
    delay(100);
  }
}


void wreceiver(){
  byte zero = 0;
  ti = millis() + 10000;
  while(ti > millis() && zero < 15){
    if(rch1 < 2100 && rch1 > 900)zero |= 0b00000001;
    if(rch2 < 2100 && rch2 > 900)zero |= 0b00000010;
    if(rch3 < 2100 && rch3 > 900)zero |= 0b00000100;
    if(rch4 < 2100 && rch4 > 900)zero |= 0b00001000;
    delay(500);
  }
  if(zero == 0){
    err = 1;

  }
}


void reg_minmax(){
  byte zero = 0;
  loch1 = rch1;
  loch2 = rch2;
  loch3 = rch3;
  loch4 = rch4;
  while(rch1 < center1 + 20 && rch1 > center1 - 20)delay(250);
  while(zero < 15){
    if(rch1 < center1 + 20 && rch1 > center1 - 20)zero |= 0b00000001;
    if(rch2 < center2 + 20 && rch2 > center2 - 20)zero |= 0b00000010;
    if(rch3 < center3 + 20 && rch3 > center3 - 20)zero |= 0b00000100;
    if(rch4 < center4 + 20 && rch4 > center4 - 20)zero |= 0b00001000;
    if(rch1 < loch1)loch1 = rch1;
    if(rch2 < loch2)loch2 = rch2;
    if(rch3 < loch3)loch3 = rch3;
    if(rch4 < loch4)loch4 = rch4;
    if(rch1 > hch1)hch1 = rch1;
    if(rch2 > hch2)hch2 = rch2;
    if(rch3 > hch3)hch3 = rch3;
    if(rch4 > hch4)hch4 = rch4;
    delay(100);
  }
}


void gyro_axes(byte move){
  byte taxis = 0;
  float groll, gpitch, gyaw;
  
  groll = 0;
  gpitch = 0;
  gyaw = 0;
  gsignal();
  ti = millis() + 10000;    
  while(ti > millis() && groll > -30 && groll < 30 && gpitch > -30 && gpitch < 30 && gyaw > -30 && gyaw < 30){
    gsignal();
    if(type == 2 || type == 3){
      groll += groll * 0.00007;              
      gpitch += gpitch * 0.00007;
      gyaw += gyaw * 0.00007;
    }
    if(type == 1){
      groll += groll * 0.0000611;          
      gpitch += gpitch * 0.0000611;
      gyaw += gyaw * 0.0000611;
    }
    
    delayMicroseconds(3700); 
  }
  
  if((groll < -30 || groll > 30) && gpitch > -30 && gpitch < 30 && gyaw > -30 && gyaw < 30){
    gyro |= 0b00000001;
    if(groll < 0)taxis = 0b10000001;
    else taxis = 0b00000001;
  }
  if((gpitch < -30 || gpitch > 30) && groll > -30 && groll < 30 && gyaw > -30 && gyaw < 30){
    gyro |= 0b00000010;
    if(gpitch < 0)taxis = 0b10000010;
    else taxis = 0b00000010;
  }
  if((gyaw < -30 || gyaw > 30) && groll > -30 && groll < 30 && gpitch > -30 && gpitch < 30){
    gyro |= 0b00000100;
    if(gyaw < 0)taxis = 0b10000011;
    else taxis = 0b00000011;
  }
  
  if(taxis == 0){
    err = 1;
  }
  else
  if(move == 1)roll = taxis;
  if(move == 2)pitch = taxis;
  if(move == 3)yaw = taxis;
  
}


ISR(PCINT0_vect){
  cu_ti = micros();
  
  if(PINB & B00000001){                                        
    if(lch1 == 0){                                   
      lch1 = 1;                                      
      ti_1 = cu_ti;                                  
    }
  }
  else if(lch1 == 1){                                
    lch1 = 0;                                        
    rch1 = cu_ti - ti_1;         
  }
  
  if(PINB & B00000010 ){                                       
    if(lch2 == 0){                                   
      lch2 = 1;                                      
      ti_2 = cu_ti;                                  
    }
  }
  else if(lch2 == 1){                                
    lch2 = 0;                                        
    rch2 = cu_ti - ti_2;         
  }
  
  if(PINB & B00000100 ){                                       
    if(lch3 == 0){                                   
      lch3 = 1;                                      
      ti_3 = cu_ti;                                  
    }
  }
  else if(lch3 == 1){                                
    lch3 = 0;                                        
    rch3 = cu_ti - ti_3;         

  }
  
  if(PINB & B00001000 ){                                       
    if(lch4 == 0){                                   
      lch4 = 1;                                      
      ti_4 = cu_ti;                                  
    }
  }
  else if(lch4 == 1){                                
    lch4 = 0;                                        
    rch4 = cu_ti - ti_4;         
  }
}
