#include <Wire.h>                          
#include <EEPROM.h>                        

float pgain_roll = 1.5, igain_roll = 0.07, dgain_roll = 14.0, pgain_pitch = 1.2, igain_pitch = 0.04, dgain_pitch = 15.0, pgain_yaw = 5.0, igain_yaw = 0.05, dgain_yaw = 0.03;                
int pmax_roll = 300, pmax_yaw = 500, pmax_pitch = 350;                     
boolean al = true, gangle;                
byte lc1, lc2, lc3, lc4, eedata[36], high, low;
volatile int ric1, ric2, ric3, ric4;
int cc1, cc2, cc3, cc4, loop_counter, esc_1, esc_2, esc_3, esc_4, tht, voltage, cali, start, gyroadd, ri[5], temp, acc_axis[4], gyro_axis[4];
float roll_levelad, pitch_levelad, errtemp, pmem_roll, proll_setpoint, groll_input, poutput_roll, plastr_error, pmem_pitch, pid_pitch_setpoint, gpitch_input, poutput_pitch, plastd_error, pmem_yaw, pid_yaw_setpoint, gyaw_input, poutput_yaw, plasty_error, angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
long acc_x, acc_y, acc_z, sum_vec;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer, timer_1, timer_2, timer_3, timer_4, current_time, loop_timer;
double gpitch, groll, gyaw, gcal[4];

void setup(){
  
  
  for(start = 0; start <= 35; start++)eedata[start] = EEPROM.read(start);
  start = 0;                                                                
  gyroadd = eedata[32];                                           

  Wire.begin();                                                             

  TWBR = 12;                                                                
  DDRD |= B11110000;                                                        
  DDRB |= B00110000;                                                        

  digitalWrite(12,HIGH);                                                    

  while(eedata[33] != 'J' || eedata[34] != 'M' || eedata[35] != 'B')delay(10);
  if(eedata[31] == 2 || eedata[31] == 3)delay(10);
  gyro_reg();                                                     

  for (cali = 0; cali < 1250 ; cali ++){                           
    PORTD |= B11110000;                                                     
    delayMicroseconds(1000);                                                
    PORTD &= B00001111;                                                     
    delayMicroseconds(3000);                                                
  }

  
  for (cali = 0; cali < 2000 ; cali ++){                           
    if(cali % 15 == 0)digitalWrite(12, !digitalRead(12));                
    gyro_signalen();                                                        
    gcal[1] += gyro_axis[1];                                       
    gcal[2] += gyro_axis[2];                                       
    gcal[3] += gyro_axis[3];                                       
    
    PORTD |= B11110000;                                                     
    delayMicroseconds(1000);                                                
    PORTD &= B00001111;                                                     
    delay(3);                                                               
  }
  
  gcal[1] /= 2000;                                                 
  gcal[2] /= 2000;                                                 
  gcal[3] /= 2000;                                                 

  PCICR |= (1 << PCIE0);                                                    
  PCMSK0 |= (1 << PCINT0);                                                  
  PCMSK0 |= (1 << PCINT1);                                                  
  PCMSK0 |= (1 << PCINT2);                                                  
  PCMSK0 |= (1 << PCINT3);                                                  

  
  while(ric3 < 990 || ric3 > 1020 || ric4 < 1400){
    ric3 = con_rec(3);                 
    ric4 = con_rec(4);                 
    start ++;                                                               
    
    PORTD |= B11110000;                                                     
    delayMicroseconds(1000);                                                
    PORTD &= B00001111;                                                     
    delay(3);                                                               
    if(start == 125){                                                       
      digitalWrite(12, !digitalRead(12));                                   
      start = 0;                                                            
    }
  }
  start = 0;                                                                
  voltage = (analogRead(0) + 65) * 1.2317;
  loop_timer = micros();                                                    

  digitalWrite(12,LOW);                                                     
}


void loop(){

  groll_input = (groll_input * 0.7) + ((groll / 65.5) * 0.3);   
  gpitch_input = (gpitch_input * 0.7) + ((gpitch / 65.5) * 0.3);
  gyaw_input = (gyaw_input * 0.7) + ((gyaw / 65.5) * 0.3);      
  angle_pitch += gpitch * 0.0000611;                                    
  angle_roll += groll * 0.0000611;                                      

  angle_pitch -= angle_roll * sin(gyaw * 0.000001066);                  
  angle_roll += angle_pitch * sin(gyaw * 0.000001066);                  

  sum_vec = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       
  if(abs(acc_y) < sum_vec){                                        
    angle_pitch_acc = asin((float)acc_y/sum_vec)* 57.296;          
  }
  if(abs(acc_x) < sum_vec){                                        
    angle_roll_acc = asin((float)acc_x/sum_vec)* -57.296;          
  }
  
  
  angle_pitch_acc -= 0.0;                                                   
  angle_roll_acc -= 0.0;                                                    
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               

  pitch_levelad = angle_pitch * 15;                                    
  roll_levelad = angle_roll * 15;                                      

  if(!al){                                                          
    pitch_levelad = 0;                                                 
    roll_levelad = 0;                                                  
  }

  if(ric3 < 1050 && ric4 < 1050)start = 1;
  
  if(start == 1 && ric3 < 1050 && ric4 > 1450){
    start = 2;

    angle_pitch = angle_pitch_acc;                                          
    angle_roll = angle_roll_acc;                                            
    gangle = true;                                                 

    
    pmem_roll = 0;
    plastr_error = 0;
    pmem_pitch = 0;
    plastd_error = 0;
    pmem_yaw = 0;
    plasty_error = 0;
  }
  
  if(start == 2 && ric3 < 1050 && ric4 > 1950)start = 0;

  proll_setpoint = 0;
  
  if(ric1 > 1508)proll_setpoint = ric1 - 1508;
  else if(ric1 < 1492)proll_setpoint = ric1 - 1492;

  proll_setpoint -= roll_levelad;                                   
  proll_setpoint /= 3.0;                                                 

  
  pid_pitch_setpoint = 0;
  
  if(ric2 > 1508)pid_pitch_setpoint = ric2 - 1508;
  else if(ric2 < 1492)pid_pitch_setpoint = ric2 - 1492;

  pid_pitch_setpoint -= pitch_levelad;                                  
  pid_pitch_setpoint /= 3.0;                                                 

  pid_yaw_setpoint = 0;
  
  if(ric3 > 1050){ 
    if(ric4 > 1508)pid_yaw_setpoint = (ric4 - 1508)/3.0;
    else if(ric4 < 1492)pid_yaw_setpoint = (ric4 - 1492)/3.0;
  }
  
  cal_pid();                                                            


  voltage = voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  
  if(voltage < 1000 && voltage > 600)digitalWrite(12, HIGH);


  tht = ric3;                                      

  if (start == 2){                                                          
    if (tht > 1800) tht = 1800;                                   
    esc_1 = tht - poutput_pitch + poutput_roll - poutput_yaw; 
    esc_2 = tht + poutput_pitch + poutput_roll + poutput_yaw; 
    esc_3 = tht + poutput_pitch - poutput_roll - poutput_yaw; 
    esc_4 = tht - poutput_pitch - poutput_roll + poutput_yaw; 

    if (voltage < 1240 && voltage > 800){                   
      esc_1 += esc_1 * ((1240 - voltage)/(float)3500);              
      esc_2 += esc_2 * ((1240 - voltage)/(float)3500);              
      esc_3 += esc_3 * ((1240 - voltage)/(float)3500);              
      esc_4 += esc_4 * ((1240 - voltage)/(float)3500);              
    } 

    if (esc_1 < 1100) esc_1 = 1100;                                         
    if (esc_2 < 1100) esc_2 = 1100;                                         
    if (esc_3 < 1100) esc_3 = 1100;                                         
    if (esc_4 < 1100) esc_4 = 1100;                                         

    if(esc_1 > 2000)esc_1 = 2000;                                           
    if(esc_2 > 2000)esc_2 = 2000;                                           
    if(esc_3 > 2000)esc_3 = 2000;                                           
    if(esc_4 > 2000)esc_4 = 2000;                                           
  }

  else{
    esc_1 = 1000;                                                           
    esc_2 = 1000;                                                           
    esc_3 = 1000;                                                           
    esc_4 = 1000;                                                           
  }
 
  if(micros() - loop_timer > 4050)digitalWrite(12, HIGH);                   

  while(micros() - loop_timer < 4000);                                      
  loop_timer = micros();                                                    
  PORTD |= B11110000;                                                       
  timer_channel_1 = esc_1 + loop_timer;                                     
  timer_channel_2 = esc_2 + loop_timer;                                     
  timer_channel_3 = esc_3 + loop_timer;                                     
  timer_channel_4 = esc_4 + loop_timer;                                     

  gyro_signalen();

  while(PORTD >= 16){                                                       
    esc_loop_timer = micros();                                              
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                
  }
}


ISR(PCINT0_vect){
  current_time = micros();
  
  if(PINB & B00000001){                                                     
    if(lc1 == 0){                                                
      lc1 = 1;                                                   
      timer_1 = current_time;                                               
    }
  }
  else if(lc1 == 1){                                             
    lc1 = 0;                                                     
    ri[1] = current_time - timer_1;                             
  }
  
  if(PINB & B00000010 ){                                                    
    if(lc2 == 0){                                                
      lc2 = 1;                                                   
      timer_2 = current_time;                                               
    }
  }
  else if(lc2 == 1){                                             
    lc2 = 0;                                                     
    ri[2] = current_time - timer_2;                             
  }
  
  if(PINB & B00000100 ){                                                    
    if(lc3 == 0){                                                
      lc3 = 1;                                                   
      timer_3 = current_time;                                               
    }
  }
  else if(lc3 == 1){                                             
    lc3 = 0;                                                     
    ri[3] = current_time - timer_3;                             

  }
  
  if(PINB & B00001000 ){                                                    
    if(lc4 == 0){                                                
      lc4 = 1;                                                   
      timer_4 = current_time;                                               
    }
  }
  else if(lc4 == 1){                                             
    lc4 = 0;                                                     
    ri[4] = current_time - timer_4;                             
  }
}

void gyro_signalen(){
  
  if(eedata[31] == 1){
    Wire.beginTransmission(gyroadd);                                   
    Wire.write(0x3B);                                                       
    Wire.endTransmission();                                                 
    Wire.requestFrom(gyroadd,14);                                      
    
    ric1 = con_rec(1);                 
    ric2 = con_rec(2);                 
    ric3 = con_rec(3);                 
    ric4 = con_rec(4);                 
    
    while(Wire.available() < 14);                                           
    acc_axis[1] = Wire.read()<<8|Wire.read();                               
    acc_axis[2] = Wire.read()<<8|Wire.read();                               
    acc_axis[3] = Wire.read()<<8|Wire.read();                               
    temp = Wire.read()<<8|Wire.read();                               
    gyro_axis[1] = Wire.read()<<8|Wire.read();                              
    gyro_axis[2] = Wire.read()<<8|Wire.read();                              
    gyro_axis[3] = Wire.read()<<8|Wire.read();                              
  }

  if(cali == 2000){
    gyro_axis[1] -= gcal[1];                                       
    gyro_axis[2] -= gcal[2];                                       
    gyro_axis[3] -= gcal[3];                                       
  }
  groll = gyro_axis[eedata[28] & 0b00000011];                      
  if(eedata[28] & 0b10000000)groll *= -1;                          
  gpitch = gyro_axis[eedata[29] & 0b00000011];                     
  if(eedata[29] & 0b10000000)gpitch *= -1;                         
  gyaw = gyro_axis[eedata[30] & 0b00000011];                       
  if(eedata[30] & 0b10000000)gyaw *= -1;                           

  acc_x = acc_axis[eedata[29] & 0b00000011];                           
  if(eedata[29] & 0b10000000)acc_x *= -1;                              
  acc_y = acc_axis[eedata[28] & 0b00000011];                           
  if(eedata[28] & 0b10000000)acc_y *= -1;                              
  acc_z = acc_axis[eedata[30] & 0b00000011];                           
  if(eedata[30] & 0b10000000)acc_z *= -1;                              
}
void cal_pid(){
  
  errtemp = groll_input - proll_setpoint;
  pmem_roll += igain_roll * errtemp;
  if(pmem_roll > pmax_roll)pmem_roll = pmax_roll;
  else if(pmem_roll < pmax_roll * -1)pmem_roll = pmax_roll * -1;

  poutput_roll = pgain_roll * errtemp + pmem_roll + dgain_roll * (errtemp - plastr_error);
  if(poutput_roll > pmax_roll)poutput_roll = pmax_roll;
  else if(poutput_roll < pmax_roll * -1)poutput_roll = pmax_roll * -1;

  plastr_error = errtemp;

  
  errtemp = gpitch_input - pid_pitch_setpoint;
  pmem_pitch += igain_pitch * errtemp;
  if(pmem_pitch > pmax_pitch)pmem_pitch = pmax_pitch;
  else if(pmem_pitch < pmax_pitch * -1)pmem_pitch = pmax_pitch * -1;

  poutput_pitch = pgain_pitch * errtemp + pmem_pitch + dgain_pitch * (errtemp - plastd_error);
  if(poutput_pitch > pmax_pitch)poutput_pitch = pmax_pitch;
  else if(poutput_pitch < pmax_pitch * -1)poutput_pitch = pmax_pitch * -1;

  plastd_error = errtemp;

  
  errtemp = gyaw_input - pid_yaw_setpoint;
  pmem_yaw += igain_yaw * errtemp;
  if(pmem_yaw > pmax_yaw)pmem_yaw = pmax_yaw;
  else if(pmem_yaw < pmax_yaw * -1)pmem_yaw = pmax_yaw * -1;

  poutput_yaw = pgain_yaw * errtemp + pmem_yaw + dgain_yaw * (errtemp - plasty_error);
  if(poutput_yaw > pmax_yaw)poutput_yaw = pmax_yaw;
  else if(poutput_yaw < pmax_yaw * -1)poutput_yaw = pmax_yaw * -1;

  plasty_error = errtemp;
}

int con_rec(byte function){
  byte channel, reverse;                                                       
  int low, center, high, actual;
  int difference;

  channel = eedata[function + 23] & 0b00000111;                           
  if(eedata[function + 23] & 0b10000000)reverse = 1;                      
  else reverse = 0;                                                            

  actual = ri[channel];                                            
  low = (eedata[channel * 2 + 15] << 8) | eedata[channel * 2 + 14];  
  center = (eedata[channel * 2 - 1] << 8) | eedata[channel * 2 - 2]; 
  high = (eedata[channel * 2 + 7] << 8) | eedata[channel * 2 + 6];   

  if(actual < center){                                                         
    if(actual < low)actual = low;                                              
    difference = ((long)(center - actual) * (long)500) / (center - low);       
    if(reverse == 1)return 1500 + difference;                                  
    else return 1500 - difference;                                             
  }
  else if(actual > center){                                                                        
    if(actual > high)actual = high;                                            
    difference = ((long)(actual - center) * (long)500) / (high - center);      
    if(reverse == 1)return 1500 - difference;                                  
    else return 1500 + difference;                                             
  }
  else return 1500;
}

void gyro_reg(){
  
  if(eedata[31] == 1){
    Wire.beginTransmission(gyroadd);                                      
    Wire.write(0x6B);                                                          
    Wire.write(0x00);                                                          
    Wire.endTransmission();                                                    

    Wire.beginTransmission(gyroadd);                                      
    Wire.write(0x1B);                                                          
    Wire.write(0x08);                                                          
    Wire.endTransmission();                                                    

    Wire.beginTransmission(gyroadd);                                      
    Wire.write(0x1C);                                                          
    Wire.write(0x10);                                                          
    Wire.endTransmission();                                                    

    
    Wire.beginTransmission(gyroadd);                                      
    Wire.write(0x1B);                                                          
    Wire.endTransmission();                                                    
    Wire.requestFrom(gyroadd, 1);                                         
    while(Wire.available() < 1);                                               
    if(Wire.read() != 0x08){                                                   
      digitalWrite(12,HIGH);                                                   
      while(1)delay(10);                                                       
    }

    Wire.beginTransmission(gyroadd);                                      
    Wire.write(0x1A);                                                          
    Wire.write(0x03);                                                          
    Wire.endTransmission();                                                    

  }  
}

