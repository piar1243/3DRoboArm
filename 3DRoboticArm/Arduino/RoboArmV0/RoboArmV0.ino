#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <Wire.h>

//thank you to adafruit for that function
//thank you and credit to curious scientist for having the code compiled from digikey for the as5600 encoder that function was implemented in this code
#define stepPin 6
#define dirPin 9
uint8_t bus = 2;
int errorCode;

int magnetStatus = 0; //value of the status register (MD, ML, MH)

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])
float startAngle = 0;
float desiredAngle = 11.65;

float angle_1 = 0;
float angle_2 = 0;
float angle_3 = 0;
float angle_4 = 0;
float angle_5 = 0;
float angle_6 = 0;


float angle_1_zero = 168.66;
float angle_2_zero = 221.75;
float angle_3_zero = 36.04;
float angle_4_zero = 215.33;
float angle_5_zero = 125.68;
float angle_6_zero = 148.54;

float angle_1_goal = 0;
float angle_3_goal = 0;
float angle_4_goal = 0;
float angle_5_goal = 0;
float angle_6_goal = 0;

float angle_goals[6] = {0, 0, 0, 3, 0, 0};
float angle_2_goal = 0;
int checker = 0;

int motor_1_dir = 7; //motor 1 = 3rd controller = 2ndpiv
int motor_20_dir = 2; //motor 2 = 1st + 2nd controller = 1stpiv
int motor_21_dir = 4; //motor 2 = 1st + 2nd controller = 1stpiv
int motor_3_dir = 12; //motor 3 = 5th controller = 2ndrot
int motor_4_dir = 13; //motor 4 = 6th controller = 3rdrot
int motor_5_dir = A2; //motor 5 = 3rdpiv
int motor_6_dir = 8; //motor 6 = 4th controller = 1strot

int motor_1_step = 6;
int motor_20_step = 3;
int motor_21_step = 5;
int motor_3_step = 10;
int motor_4_step = 11;
int motor_5_step = A1;
int motor_6_step = 9;

int mag = A0;

ros::NodeHandle nh;

std_msgs::Float64 outputMessage;

ros::Publisher pub("info_bac", &outputMessage);

void callBackFunction(const std_msgs::Float64MultiArray &inputMessage){
if(checker == 0){
  angle_goals[0] = inputMessage.data[0];
  checker++;
}
else if(checker == 1){
  angle_2_goal = inputMessage.data[0];
  checker++;
}
else if(checker == 2){
  angle_goals[2] = inputMessage.data[0];
  checker++;
}
else if(checker == 3){
  angle_goals[3] = -(inputMessage.data[0]);
  checker++;
}
else if(checker == 4){
  angle_goals[4] = inputMessage.data[0];
  checker++;
}
else if(checker == 5){
  angle_goals[5] = inputMessage.data[0];
  checker++;
  if(checker == 6){
    outputMessage.data = angle_goals[5];
    pub.publish(&outputMessage);
    checker = 0;
  }
}
//if(checker == 6){
//  angle_goals[5] = inputMessage.data[0];
//checker++;
//}
//angle_goals[0] = inputMessage.data[0];
//angle_goals[1] = inputMessage.data[1];
//outputMessage.data = inputMessage.data[0];
//pub.publish(&outputMessage);
 


}

ros::Subscriber<std_msgs::Float64MultiArray> sub("angle_information", &callBackFunction);


void TCA9548A(uint8_t bus) //function from adafruit - https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/arduino-wiring-and-test
{
  //nh.spinOnce();
  Wire.beginTransmission(0x70);
  //nh.spinOnce();// TCA9548A address is 0x70
  Wire.write(1 << bus);
  //delay(5);
  Wire.endTransmission(true);
  //nh.spinOnce();
}



void setup()
{
  delay(150);
  angle_goals[1] = 0;
  //Serial.begin(57600); //start serial - tip: don't use serial if you don't need it (speed considerations)
  Wire.begin(0x70); //start i2C  
  //Wire.setClock(800000L); //fast clock
//  TCA9548A(1);
//  pinMode(stepPin,OUTPUT);
//  pinMode(dirPin,OUTPUT);
  //pinMode(mag,OUTPUT);
//  checkMagnetPresence();
//  ReadRawAngle();

  startAngle = degAngle;

  nh.getHardware()->setBaud(115200);
  nh.initNode();
 
  nh.advertise(pub);
  nh.subscribe(sub);

  nh.spinOnce();
}


void loop(){

  delay(1);
  nh.spinOnce();
  updateAngle();
  //TCA9548A(4);

//    Serial.println(angle_1);
//    Serial.println(angle_2);
//    Serial.println(angle_3);
//    Serial.println(angle_4);
//    Serial.println(angle_5);
//    Serial.println(angle_6);

 turnAllMotors(angle_goals[3-1], 1, motor_1_step, motor_1_dir, angle_1_zero, angle_2_goal, 2, motor_20_step, motor_20_dir, motor_21_step, motor_21_dir, angle_2_zero, angle_goals[4-1], 3, motor_3_step, motor_3_dir, angle_3_zero, angle_goals[6-1], 4, motor_4_step, motor_4_dir, angle_4_zero, angle_goals[5-1], 5, motor_5_step, motor_5_dir, angle_5_zero, angle_goals[1-1], 6, motor_6_step, motor_6_dir, angle_6_zero);

//    TCA9548A(6);
//    turnToAngle(angle_goals[1-1], 6, motor_6_step, motor_6_dir, angle_6_zero); //correct
//    TCA9548A(1);
//    turnToAngle(angle_goals[3-1], 1, motor_1_step, motor_1_dir, angle_1_zero); //correct
//    TCA9548A(2);
//    turnToAngleTwoMotors(angle_goals[2-1], 2, motor_20_step, motor_20_dir, motor_21_step, motor_21_dir, angle_2_zero); //correct
//    TCA9548A(3);
//    turnToAngle(angle_goals[4-1], 3, motor_3_step, motor_3_dir, angle_3_zero); //correct
//    TCA9548A(4);
//    //Serial.println("loooooped1");
//    turnToAngle(angle_goals[6-1], 4, motor_4_step, motor_4_dir, angle_4_zero); //correct
//    TCA9548A(5);
//    //Serial.println("loooooped2");
//    turnToAngle(angle_goals[5-1], 5, motor_5_step, motor_5_dir, angle_5_zero); //correct

}


void turnToAngle(int desAngle, int motor_bus1, int motor_step, int motor_dir, float zero_angle){
  TCA9548A(motor_bus1);
  pinMode(motor_step, OUTPUT);
  pinMode(motor_dir, OUTPUT);
  ReadRawAngle();
  float true_angle = zero_angle + desAngle;
  while(!((true_angle - 0.2 < degAngle) && (degAngle < true_angle + 0.2))){
      //Serial.println("finished");
      ReadRawAngle();
      nh.spinOnce();
      true_angle = zero_angle + desAngle;
      //Serial.println(degAngle);
      //Serial.println("lol");
      //correctAngle();
      if (degAngle > true_angle+0.2){
        digitalWrite(motor_dir,LOW);
        digitalWrite(motor_step,HIGH);
        delayMicroseconds(2000);    // by changing this time delay between the steps we can change the rotation speed
        digitalWrite(motor_step,LOW);
        delayMicroseconds(2000);
       
      }
     if (degAngle < true_angle-0.2) {
        digitalWrite(motor_dir,HIGH);
        digitalWrite(motor_step,HIGH);
        delayMicroseconds(2000);    // by changing this time delay between the steps we can change the rotation speed
        digitalWrite(motor_step,LOW);
        delayMicroseconds(2000);
         
     }
   }
}


void turnToAngleTwoMotors(int desAngle, int motor_bus2, int motor_step1, int motor_dir1, int motor_step2, int motor_dir2, float zero_angle){
  TCA9548A(motor_bus2);
  pinMode(motor_step1, OUTPUT);
  pinMode(motor_dir1, OUTPUT);
  pinMode(motor_step2, OUTPUT);
  pinMode(motor_dir2, OUTPUT);
  ReadRawAngle();
  float true_angle = zero_angle + desAngle;
  while(!((true_angle - 0.2 < degAngle) && (degAngle < true_angle + 0.2))){
      ReadRawAngle();
      nh.spinOnce();
      true_angle = zero_angle + desAngle;
      //Serial.println(degAngle);
      //Serial.println("lol2");
      //correctAngle();
      if (degAngle > true_angle+0.2){
        digitalWrite(motor_dir1,LOW);
        digitalWrite(motor_dir2,LOW);
        digitalWrite(motor_step1,HIGH);
        digitalWrite(motor_step2,HIGH);
        delayMicroseconds(2000);    // by changing this time delay between the steps we can change the rotation speed
        digitalWrite(motor_step1,LOW);
        digitalWrite(motor_step2,LOW);
        delayMicroseconds(2000);
       
      }
     if (degAngle < true_angle-0.2) {
        digitalWrite(motor_dir1,HIGH);
        digitalWrite(motor_dir2,HIGH);
        digitalWrite(motor_step1,HIGH);
        digitalWrite(motor_step2,HIGH);
        delayMicroseconds(2000);    // by changing this time delay between the steps we can change the rotation speed
        digitalWrite(motor_step1,LOW);
        digitalWrite(motor_step2,LOW);
        delayMicroseconds(2000);
         
     }
   }
}

//void turnAllMotors(){
//  
//  if (degAngle > desAngle+1){
//    
//        digitalWrite(motor_dir1,LOW);
//        digitalWrite(motor_dir2,LOW);
//        digitalWrite(motor_step1,HIGH);
//        digitalWrite(motor_step2,HIGH);
//        delayMicroseconds(1000);    // by changing this time delay between the steps we can change the rotation speed
//        digitalWrite(motor_step1,LOW);
//        digitalWrite(motor_step2,LOW);
//        delayMicroseconds(1000);
//        
//      }
//   if (degAngle < desAngle-1) {
//    
//        digitalWrite(motor_dir1,HIGH);
//        digitalWrite(motor_dir2,HIGH);
//        digitalWrite(motor_step1,HIGH);
//        digitalWrite(motor_step2,HIGH);
//        delayMicroseconds(1000);    // by changing this time delay between the steps we can change the rotation speed
//        digitalWrite(motor_step1,LOW);
//        digitalWrite(motor_step2,LOW);
//        delayMicroseconds(1000);
//        
//     }
//  
//}


void ReadRawAngle()
{
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
 
  while(Wire.available() == 0); //wait until it becomes available
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
 
  while(Wire.available() == 0);  
  highbyte = Wire.read();
 
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
 
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  //Serial.println(highbyte + lowbyte);
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625;
 
  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
 
}

void checkMagnetPresence(){
  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading
    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor
    while(Wire.available() == 0); //wait until it becomes available
    magnetStatus = Wire.read(); //Reading the data after the request
    //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }      
  //Serial.println("Magnet found!");
}

void updateAngle(){
    TCA9548A(1);
    checkMagnetPresence();
    ReadRawAngle();
    angle_1 = degAngle;
    //Serial.println(angle_1);
    //delay(1);
    TCA9548A(2);
    checkMagnetPresence();
    ReadRawAngle();
    angle_2 = degAngle;
    //Serial.println(angle_2);
    //delay(1);
    TCA9548A(3);
    checkMagnetPresence();
    ReadRawAngle();
    angle_3 = degAngle;
    //Serial.println(angle_3);
    //delay(1);
    TCA9548A(4);
    checkMagnetPresence();
    ReadRawAngle();
    angle_4 = degAngle;
    //Serial.println(angle_4);
    //delay(1);
    TCA9548A(5);
    checkMagnetPresence();
    ReadRawAngle();
    angle_5 = degAngle;
    //Serial.println(angle_5);
    //delay(1);
    TCA9548A(6);
    checkMagnetPresence();
    ReadRawAngle();
    angle_6 = degAngle;
    //Serial.println(angle_6);
    //delay(1);
}

//void stand(){
//  TCA9548A(6);
//    turnToAngle(120, 6, motor_6_step, motor_6_dir);
//    TCA9548A(1);
//    turnToAngle(60, 1, motor_1_step, motor_1_dir);
//    TCA9548A(2);
//    turnToAngleTwoMotors(299.77, 2, motor_20_step, motor_20_dir, motor_21_step, motor_21_dir);
//    TCA9548A(3);
//    turnToAngle(317.29 , 3, motor_3_step, motor_3_dir);
//    TCA9548A(4);
//    //Serial.println("loooooped1");
//    turnToAngle(351.83, 4, motor_4_step, motor_4_dir);
//    TCA9548A(5);
//    //Serial.println("loooooped2");
//    turnToAngle(121.99, 5, motor_5_step, motor_5_dir);
//}







void turnAllMotors(float desAngle1, int motor_bus1, int motor_step1, int motor_dir1, float zero_angle1, float desAngle2, int motor_bus2, int motor_step21, int motor_dir21, int motor_step22, int motor_dir22, float zero_angle2, float desAngle3, int motor_bus3, int motor_step3, int motor_dir3, float zero_angle3, int desAngle4, int motor_bus4, int motor_step4, int motor_dir4, float zero_angle4, float desAngle5, int motor_bus5, int motor_step5, int motor_dir5, float zero_angle5, float desAngle6, int motor_bus6, int motor_step6, int motor_dir6, float zero_angle6){
  TCA9548A(motor_bus1);
  pinMode(motor_step1, OUTPUT);
  pinMode(motor_dir1, OUTPUT);
  pinMode(motor_step21, OUTPUT);
  pinMode(motor_dir21, OUTPUT);
  pinMode(motor_step22, OUTPUT);
  pinMode(motor_dir22, OUTPUT);
  pinMode(motor_step3, OUTPUT);
  pinMode(motor_dir3, OUTPUT);
  pinMode(motor_step4, OUTPUT);
  pinMode(motor_dir4, OUTPUT);
  pinMode(motor_step5, OUTPUT);
  pinMode(motor_dir5, OUTPUT);
  pinMode(motor_step6, OUTPUT);
  pinMode(motor_dir6, OUTPUT);

  ReadRawAngle();
  int mtrs = 1000;
  int counter2 = 0;
  float true_angle1 = zero_angle1 + desAngle1;
  float true_angle2 = zero_angle2 + desAngle2;
  float true_angle3 = zero_angle3 + desAngle3;
  float true_angle4 = zero_angle4 + desAngle4;
  float true_angle5 = zero_angle5 + desAngle5;
  float true_angle6 = zero_angle6 + desAngle6;
  nh.spinOnce();
  // && (!((true_angle3 - 0.2 < angle_3) && (angle_3 < true_angle3 + 0.2))) && (!((true_angle4 - 0.2 < angle_4) && (angle_4 < true_angle4 + 0.2))) && (!((true_angle5 - 0.2 < angle_5) && (angle_5 < true_angle5 + 0.2))) && (!((true_angle6 - 0.2 < angle_6) && (angle_6 < true_angle6 + 0.2))))
  //while((!((true_angle1 - 0.2 < angle_1) && (angle_1 < true_angle1 + 0.2))) && (!((true_angle2 - 0.2 < angle_2) && (angle_2 < true_angle2 + 0.2)))){    
    while(!((true_angle1 - 0.2 < angle_1) && (angle_1 < true_angle1 + 0.2) && (true_angle2 - 0.2 < angle_2) && (angle_2 < true_angle2 + 0.2) && (true_angle3 - 0.2 < angle_3) && (angle_3 < true_angle3 + 0.2) && (true_angle4 - 0.2 < angle_4) && (angle_4 < true_angle4 + 0.2) && (true_angle5 - 0.2 < angle_5) && (angle_5 < true_angle5 + 0.2) && (true_angle6 - 0.2 < angle_6) && (angle_6 < true_angle6 + 0.2))){
       ReadRawAngle();
       updateAngle();
       nh.spinOnce();
     
       true_angle1 = zero_angle1 + desAngle1;
       true_angle2 = zero_angle2 + desAngle2;
       true_angle3 = zero_angle3 + desAngle3;
       true_angle4 = zero_angle4 + desAngle4;
       true_angle5 = zero_angle5 + desAngle5;
       true_angle6 = zero_angle6 + desAngle6;

      if(!((true_angle1 - 0.2 < angle_1) && (angle_1 < true_angle1 + 0.2))){
        nh.spinOnce();
        //ReadRawAngle();
        //Serial.println(angle_1);
        TCA9548A(1);
        if (angle_1 > true_angle1+0.2){
          digitalWrite(motor_dir1,LOW);
          digitalWrite(motor_step1,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step1,LOW);
          delayMicroseconds(mtrs);
         
        }
       if (angle_1 < true_angle1-0.2) {
          digitalWrite(motor_dir1,HIGH);
          digitalWrite(motor_step1,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step1,LOW);
          delayMicroseconds(mtrs);
           
       }
       counter2++;
     }
     nh.spinOnce();

     if(!((true_angle2 - 0.2 < angle_2) && (angle_2 < true_angle2 + 0.2))){
      nh.spinOnce();
        //Serial.println(angle_2);
      TCA9548A(2);
        if (angle_2 > true_angle2+0.2){
          digitalWrite(motor_dir21,LOW);
          digitalWrite(motor_dir22,LOW);
          digitalWrite(motor_step21,HIGH);
          digitalWrite(motor_step22,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step21,LOW);
          digitalWrite(motor_step22,LOW);
          delayMicroseconds(mtrs);
         
        }
       if (angle_2 < true_angle2-0.2) {
          digitalWrite(motor_dir21,HIGH);
          digitalWrite(motor_dir22,HIGH);
          digitalWrite(motor_step21,HIGH);
          digitalWrite(motor_step22,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step21,LOW);
          digitalWrite(motor_step22,LOW);
          delayMicroseconds(mtrs);
           
       }
       counter2++;
     }
     nh.spinOnce();

     if(!((true_angle3 - 0.2 < angle_3) && (angle_3 < true_angle3 + 0.2))){
      nh.spinOnce();
      TCA9548A(3);
        if (angle_3 > true_angle3+0.2){
          digitalWrite(motor_dir3,LOW);
          digitalWrite(motor_step3,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step3,LOW);
          delayMicroseconds(mtrs);
         
        }
       if (angle_3 < true_angle3-0.2) {
          digitalWrite(motor_dir3,HIGH);
          digitalWrite(motor_step3,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step3,LOW);
          delayMicroseconds(mtrs);
           
       }
       counter2++;
     }
     
     nh.spinOnce();
     if(!((true_angle4 - 0.2 < angle_4) && (angle_4 < true_angle4 + 0.2))){
     nh.spinOnce();
      TCA9548A(4);
        if (angle_4 > true_angle4+0.2){
          digitalWrite(motor_dir4,LOW);
          digitalWrite(motor_step4,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step4,LOW);
          delayMicroseconds(mtrs);
         
        }
       if (angle_4 < true_angle4-0.2) {
          digitalWrite(motor_dir4,HIGH);
          digitalWrite(motor_step4,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step4,LOW);
          delayMicroseconds(mtrs);
           
       }
       counter2++;
     }
     nh.spinOnce();

     if(!((true_angle5 - 0.2 < angle_5) && (angle_5 < true_angle5 + 0.2))){
      //nh.spinOnce();
      TCA9548A(5);
        if (angle_5 > true_angle5+0.2){
          digitalWrite(motor_dir5,LOW);
          digitalWrite(motor_step5,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step5,LOW);
          delayMicroseconds(mtrs);
         
        }
       if (angle_5 < true_angle5-0.2) {
          digitalWrite(motor_dir5,HIGH);
          digitalWrite(motor_step5,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step5,LOW);
          delayMicroseconds(mtrs);
           
       }
       counter2++;
     }
     nh.spinOnce();

     if(!((true_angle6- 0.2 < angle_6) && (angle_6 < true_angle6 + 0.2))){
      //nh.spinOnce();
      TCA9548A(6);
        if (angle_6 > true_angle6+0.2){
          digitalWrite(motor_dir6,LOW);
          digitalWrite(motor_step6,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step6,LOW);
          delayMicroseconds(mtrs);
         
        }
       if (angle_6 < true_angle6-0.2) {
          digitalWrite(motor_dir6,HIGH);
          digitalWrite(motor_step6,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step6,LOW);
          delayMicroseconds(mtrs);
           
       }
       counter2++;
     }
     nh.spinOnce();

     if(counter2 == 0){
      mtrs = 500;
     }
     if(counter2 == 1){
      mtrs = 500;
     }
     if(counter2 > 1){
      mtrs = 1000/(counter2-1);
     }
     counter2 = 0;
     //Serial.println(mtrs);

   }
}
