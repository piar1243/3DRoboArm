#include <ros.h> //includes ROSSerial library
#include <sensor_msgs/JointState.h> //includes the joint states that the arduino subscribes to through Serial
#include <std_msgs/Float64.h> //includes the float64 input format
#include <std_msgs/Float64MultiArray.h> //includes the float64 multi-array format for debugging ROSSerial
#include <Wire.h>

//libraries/as5600 function credit: Adafruit, Arduino, and ROS libraries/code and as5600 encoder tutorial code from curious scientist
#define stepPin 6 //step and dir pins on robotic arm for initialization
#define dirPin 9
uint8_t bus = 2; //defines the bus for TCA multiplexer for the AS5600 with the same I2C adresses

int magnetStatus = 0; //magnet status of AS5600

int lowbyte; //AS5600 angle bits 7:0
word highbyte; //AS5600 angle bits 7:0 11:8
int rawAngle; //AS5600 angle out of 4096 (12 bits)
float degAngle; //AS5600 12 bit angle output out of 360
float startAngle = 0; //debugging variables for single motor rotating test
float desiredAngle = 11.65;

float angle_1 = 0; //these are the robotic arms current angular positions on each axis
float angle_2 = 0;
float angle_3 = 0;
float angle_4 = 0;
float angle_5 = 0;
float angle_6 = 0;


float angle_1_zero = 168.66; //the zeroed angle position of the robotic arm
float angle_2_zero = 221.75;
float angle_3_zero = 36.04;
float angle_4_zero = 215.33;
float angle_5_zero = 125.68;
float angle_6_zero = 148.54;

float angle_1_goal = 0; //these are changed to the goal for the robotic arms current angular position to reach
float angle_2_goal = 0;
float angle_3_goal = 0;
float angle_4_goal = 0;
float angle_5_goal = 0;
float angle_6_goal = 0;

float angle_goals[6] = {0, 0, 0, 3, 0, 0}; //array of angle goals to target

int checker = 0; //vairable for checking all the incoming ROS angles individually

int motor_1_dir = 7; //motor 1 = 3rd controller = 2ndpiv
int motor_20_dir = 2; //motor 2 = 1st + 2nd controller = 1stpiv
int motor_21_dir = 4; //motor 2 = 1st + 2nd controller = 1stpiv
int motor_3_dir = 12; //motor 3 = 5th controller = 2ndrot
int motor_4_dir = 13; //motor 4 = 6th controller = 3rdrot
int motor_5_dir = A2; //motor 5 = 3rdpiv
int motor_6_dir = 8; //motor 6 = 4th controller = 1strot

int motor_1_step = 6; // defines the stepper motor directions for each motor
int motor_20_step = 3;
int motor_21_step = 5;
int motor_3_step = 10;
int motor_4_step = 11;
int motor_5_step = A1;
int motor_6_step = 9;

int mag = A0;

ros::NodeHandle nh; //ros node handle as nh

std_msgs::Float64 outputMessage; //ros output message as outputMessage

ros::Publisher pub("info_bac", &outputMessage); //sets up the publisher

void callBackFunction(const std_msgs::Float64MultiArray &inputMessage){ //call back to check the incoming data
if(checker == 0){ //if on the first input message
  angle_goals[0] = inputMessage.data[0]; // get the first angle position from ROS
  checker++; //loop through reading each output message, and going to read the next position
}
else if(checker == 1){
  angle_2_goal = inputMessage.data[0]; // get the first angle position from ROS
  checker++;
}
else if(checker == 2){
  angle_goals[2] = inputMessage.data[0]; // get the second angle position from ROS
  checker++;
}
else if(checker == 3){
  angle_goals[3] = -(inputMessage.data[0]); // get the third angle position from ROS
  checker++;
}
else if(checker == 4){
  angle_goals[4] = inputMessage.data[0]; // get the fourth angle position from ROS
  checker++;
}
else if(checker == 5){
  angle_goals[5] = inputMessage.data[0]; // get the fifth angle position from ROS
  checker++;
  if(checker == 6){
    outputMessage.data = angle_goals[5]; // get the sixth angle position from ROS
    pub.publish(&outputMessage); //publishes the output message to ensure message was recieved
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

ros::Subscriber<std_msgs::Float64MultiArray> sub("angle_information", &callBackFunction); //subcribes to the array of angle information being sent to always wait for the update


void TCA9548A(uint8_t bus) //function to switch the sensors with the same I2C going to the multiplexer, function from adafruit
{
  //nh.spinOnce();
  Wire.beginTransmission(0x70); //TCA9548A address is 0x70
  //nh.spinOnce();
  Wire.write(1 << bus); //sending byte selects the number bus
  //delay(5);
  Wire.endTransmission(true);
  //nh.spinOnce();
}



void setup() // setup
{
  delay(150); // small delay for debugging
  angle_goals[1] = 0; // sets the angle goals 1 as 0

  Wire.begin(0x70); //start i2C  
  //Wire.setClock(800000L); //fast clock
//  TCA9548A(1);
//  pinMode(stepPin,OUTPUT);
//  pinMode(dirPin,OUTPUT);
  //pinMode(mag,OUTPUT);
//  checkMagnetPresence();
//  ReadRawAngle();

  startAngle = degAngle; // defines the start AS5600 angle

  nh.getHardware()->setBaud(115200); // sets the baud between ROS serial at 115200
  nh.initNode(); //initializes the node handle node
 
  nh.advertise(pub); //checks publish
  nh.subscribe(sub); //checks subscribe

  nh.spinOnce(); //runs through the ROS check for new data
}


void loop(){

  delay(1);
  nh.spinOnce(); //check for new publishing data
  updateAngle(); //update the angles
  //TCA9548A(4);

//debugging ----------------------
//    Serial.println(angle_1);
//    Serial.println(angle_2);
//    Serial.println(angle_3);
//    Serial.println(angle_4);
//    Serial.println(angle_5);
//    Serial.println(angle_6);
//--------------------------------
//updates the position from ROS to the stepper motors
 turnAllMotors(angle_goals[3-1], 1, motor_1_step, motor_1_dir, angle_1_zero, angle_2_goal, 2, motor_20_step, motor_20_dir, motor_21_step, motor_21_dir, angle_2_zero, angle_goals[4-1], 3, motor_3_step, motor_3_dir, angle_3_zero, angle_goals[6-1], 4, motor_4_step, motor_4_dir, angle_4_zero, angle_goals[5-1], 5, motor_5_step, motor_5_dir, angle_5_zero, angle_goals[1-1], 6, motor_6_step, motor_6_dir, angle_6_zero);

//debugging --------------------------------------------------------------------------------
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
// ------------------------------------------------------------------------------------------
}


void turnToAngle(int desAngle, int motor_bus1, int motor_step, int motor_dir, float zero_angle){ //turn to angle moves the stepper motor until the AS5600 position is achieved
  TCA9548A(motor_bus1); //which motor is the function operating on
  pinMode(motor_step, OUTPUT);
  pinMode(motor_dir, OUTPUT);
  ReadRawAngle();
  float true_angle = zero_angle + desAngle; //the true angle to rotate to for the AS5600
  while(!((true_angle - 0.2 < degAngle) && (degAngle < true_angle + 0.2))){ //checks if the position is not acheived then increment the stepper motors
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

//this function is was used to test the dual-motor second axes, and is essentially a vestigal structure, so delete if memory is low ----------------------------------------------------------------------------------
void turnToAngleTwoMotors(int desAngle, int motor_bus2, int motor_step1, int motor_dir1, int motor_step2, int motor_dir2, float zero_angle){ //same as singular motor turning to but, turns to angle moves the 2 stepper motors until the AS5600 position are achieved
  //this function is used for the second axis where there are two stepper motors controlling one axis
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
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//Debugging --------------------------------------------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------------------------------------------------

void ReadRawAngle() //reads the raw angle from the AS5600 (library function from curious scientist as5600 tutorial)
{
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D);
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
 
  while(Wire.available() == 0); //wait until it becomes available
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
 
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  highbyte = highbyte << 8; //shifting to left
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  degAngle = rawAngle * 0.087890625;
 
}

void checkMagnetPresence(){  //checks the magnetic presence of the AS5600 (library function from curious scientist as5600 tutorial)
  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading
    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B);
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor
    while(Wire.available() == 0); //wait until it becomes available
    magnetStatus = Wire.read(); //Reading the data after the request

  }      
}

void updateAngle(){ //update angles function to read the AS5600 position and update the angle
    TCA9548A(1); //changes multiplexer to read from AS5600 #
    checkMagnetPresence();
    ReadRawAngle();
    angle_1 = degAngle; //updates the angle of the # axis --> continues for all axes of the robotic arm 
    //Serial.println(angle_1); //debugging
    //delay(1); //debugging
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

//Debugging ---------------------------------------------------------------------------------------------------

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

// -------------------------------------------------------------------------------------------------------------

//this turn all motors function get the current angle position of the joints and then rotates all the stepper motor until these angle are equivalent to the ones published through ROS
void turnAllMotors(float desAngle1, int motor_bus1, int motor_step1, int motor_dir1, float zero_angle1, float desAngle2, int motor_bus2, int motor_step21, int motor_dir21, int motor_step22, int motor_dir22, float zero_angle2, float desAngle3, int motor_bus3, int motor_step3, int motor_dir3, float zero_angle3, int desAngle4, int motor_bus4, int motor_step4, int motor_dir4, float zero_angle4, float desAngle5, int motor_bus5, int motor_step5, int motor_dir5, float zero_angle5, float desAngle6, int motor_bus6, int motor_step6, int motor_dir6, float zero_angle6){
  TCA9548A(motor_bus1); 
  //defines all the outputs to the stepper motors for updating the angular positions ---------------------------
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
  //------------------------------------------------------------------------------------------------------------

  ReadRawAngle(); //reads the initial joint states
  int mtrs = 1000; //speed at which the stepper motors are incremented (pulse width time)
  int counter2 = 0;
  //find the true angle of each joint taking into account the zeroed AS5600 position ---------------------------
  float true_angle1 = zero_angle1 + desAngle1;
  float true_angle2 = zero_angle2 + desAngle2;
  float true_angle3 = zero_angle3 + desAngle3;
  float true_angle4 = zero_angle4 + desAngle4;
  float true_angle5 = zero_angle5 + desAngle5;
  float true_angle6 = zero_angle6 + desAngle6;
  //------------------------------------------------------------------------------------------------------------
  nh.spinOnce();

  //Debugging --------------------------------------------------------------------------------------------------
  // && (!((true_angle3 - 0.2 < angle_3) && (angle_3 < true_angle3 + 0.2))) && (!((true_angle4 - 0.2 < angle_4) && (angle_4 < true_angle4 + 0.2))) && (!((true_angle5 - 0.2 < angle_5) && (angle_5 < true_angle5 + 0.2))) && (!((true_angle6 - 0.2 < angle_6) && (angle_6 < true_angle6 + 0.2))))
  //while((!((true_angle1 - 0.2 < angle_1) && (angle_1 < true_angle1 + 0.2))) && (!((true_angle2 - 0.2 < angle_2) && (angle_2 < true_angle2 + 0.2)))){   
  //------------------------------------------------------------------------------------------------------------
  
  //checks every joint position, and if it's not within a range of .2 degrees (AS5600 12-bit resolution limited) of the ROS position published, will increment all the stepper motors until it is reached
    while(!((true_angle1 - 0.2 < angle_1) && (angle_1 < true_angle1 + 0.2) && (true_angle2 - 0.2 < angle_2) && (angle_2 < true_angle2 + 0.2) && (true_angle3 - 0.2 < angle_3) && (angle_3 < true_angle3 + 0.2) && (true_angle4 - 0.2 < angle_4) && (angle_4 < true_angle4 + 0.2) && (true_angle5 - 0.2 < angle_5) && (angle_5 < true_angle5 + 0.2) && (true_angle6 - 0.2 < angle_6) && (angle_6 < true_angle6 + 0.2))){
       ReadRawAngle(); //clear angle readings
       updateAngle(); //gets the new AS5600 positions
       nh.spinOnce(); //checks to see if the ROS position changed if angle is constantly being updated for path tracing
     
       true_angle1 = zero_angle1 + desAngle1;
       true_angle2 = zero_angle2 + desAngle2;
       true_angle3 = zero_angle3 + desAngle3;
       true_angle4 = zero_angle4 + desAngle4;
       true_angle5 = zero_angle5 + desAngle5;
       true_angle6 = zero_angle6 + desAngle6;

      if(!((true_angle1 - 0.2 < angle_1) && (angle_1 < true_angle1 + 0.2))){ //checks each angle individually to ensure that the motor needs to be incremented, instead of incrementing all the motors
        nh.spinOnce(); //check for ROS angle updates
        //ReadRawAngle();
        //Serial.println(angle_1);
        TCA9548A(1);
        if (angle_1 > true_angle1+0.2){ //checks for which direction to rotate stepper motor
          digitalWrite(motor_dir1,LOW);
          digitalWrite(motor_step1,HIGH); //rotates stepper motor with PWM to stepper driver
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step1,LOW);
          delayMicroseconds(mtrs);
         
        }
       if (angle_1 < true_angle1-0.2) {  //checks for which direction to rotate stepper motor
          digitalWrite(motor_dir1,HIGH);
          digitalWrite(motor_step1,HIGH); //rotates stepper motor with PWM to stepper driver
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step1,LOW);
          delayMicroseconds(mtrs);
           
       }
       counter2++; //adds to motors incremented counter to adjust speed if needed to acount for timing in pulses of a motor not needing to be incremented anymore
     }
     nh.spinOnce(); //check for ROS angle updates

     if(!((true_angle2 - 0.2 < angle_2) && (angle_2 < true_angle2 + 0.2))){ //checks each angle individually to ensure that the motor needs to be incremented, instead of incrementing all the motors
      nh.spinOnce(); //check for ROS angle updates
        //Serial.println(angle_2);
      TCA9548A(2);
        if (angle_2 > true_angle2+0.2){ //checks for which direction to rotate stepper motor
          digitalWrite(motor_dir21,LOW); // DO NOT ROTATE THESE STEPPER MOTORS OPPOSITE OF EACH OTHER
          digitalWrite(motor_dir22,LOW); 
          digitalWrite(motor_step21,HIGH); //rotates stepper motors (because this axis has 2 opposite facing stepper motors) with PWM to stepper driver
          digitalWrite(motor_step22,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step21,LOW);
          digitalWrite(motor_step22,LOW);
          delayMicroseconds(mtrs);
         
        }
       if (angle_2 < true_angle2-0.2) {  //checks for which direction to rotate stepper motor
          digitalWrite(motor_dir21,HIGH);
          digitalWrite(motor_dir22,HIGH);
          digitalWrite(motor_step21,HIGH);//rotates stepper motors (because this axis has 2 opposite facing stepper motors) with PWM to stepper driver
          digitalWrite(motor_step22,HIGH);
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step21,LOW);
          digitalWrite(motor_step22,LOW);
          delayMicroseconds(mtrs);
           
       }
       counter2++; //adds to motors incremented counter to adjust speed if needed to acount for timing in pulses of a motor not needing to be incremented anymore
     }
     nh.spinOnce(); //check for ROS angle updates

     if(!((true_angle3 - 0.2 < angle_3) && (angle_3 < true_angle3 + 0.2))){ //checks each angle individually to ensure that the motor needs to be incremented, instead of incrementing all the motors
      nh.spinOnce(); //check for ROS angle updates
      TCA9548A(3);
        if (angle_3 > true_angle3+0.2){ //checks for which direction to rotate stepper motor
          digitalWrite(motor_dir3,LOW);
          digitalWrite(motor_step3,HIGH); //rotates stepper motor with PWM to stepper driver
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step3,LOW);
          delayMicroseconds(mtrs);
         
        }
       if (angle_3 < true_angle3-0.2) { //checks for which direction to rotate stepper motor
          digitalWrite(motor_dir3,HIGH);
          digitalWrite(motor_step3,HIGH); //rotates stepper motor with PWM to stepper driver
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step3,LOW);
          delayMicroseconds(mtrs);
           
       }
       counter2++; //adds to motors incremented counter to adjust speed if needed to acount for timing in pulses of a motor not needing to be incremented anymore
     }
     
     nh.spinOnce(); //check for ROS angle updates
     if(!((true_angle4 - 0.2 < angle_4) && (angle_4 < true_angle4 + 0.2))){ //checks each angle individually to ensure that the motor needs to be incremented, instead of incrementing all the motors
     nh.spinOnce(); //check for ROS angle updates
      TCA9548A(4);
        if (angle_4 > true_angle4+0.2){ //checks for which direction to rotate stepper motor
          digitalWrite(motor_dir4,LOW);
          digitalWrite(motor_step4,HIGH); //rotates stepper motor with PWM to stepper driver
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step4,LOW);
          delayMicroseconds(mtrs);
         
        }
       if (angle_4 < true_angle4-0.2) { //checks for which direction to rotate stepper motor
          digitalWrite(motor_dir4,HIGH);
          digitalWrite(motor_step4,HIGH); //rotates stepper motor with PWM to stepper driver
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step4,LOW);
          delayMicroseconds(mtrs);
           
       }
       counter2++; //adds to motors incremented counter to adjust speed if needed to acount for timing in pulses of a motor not needing to be incremented anymore
     }
     nh.spinOnce(); //check for ROS angle updates

     if(!((true_angle5 - 0.2 < angle_5) && (angle_5 < true_angle5 + 0.2))){ //checks each angle individually to ensure that the motor needs to be incremented, instead of incrementing all the motors
      //nh.spinOnce();
      TCA9548A(5);
        if (angle_5 > true_angle5+0.2){ //checks for which direction to rotate stepper motor
          digitalWrite(motor_dir5,LOW);
          digitalWrite(motor_step5,HIGH); //rotates stepper motor with PWM to stepper driver
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step5,LOW);
          delayMicroseconds(mtrs);
         
        }
       if (angle_5 < true_angle5-0.2) { //checks for which direction to rotate stepper motor
          digitalWrite(motor_dir5,HIGH);
          digitalWrite(motor_step5,HIGH); //rotates stepper motor with PWM to stepper driver
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step5,LOW);
          delayMicroseconds(mtrs);
           
       }
       counter2++; //adds to motors incremented counter to adjust speed if needed to acount for timing in pulses of a motor not needing to be incremented anymore
     }
     nh.spinOnce(); //check for ROS angle updates

     if(!((true_angle6- 0.2 < angle_6) && (angle_6 < true_angle6 + 0.2))){ //checks each angle individually to ensure that the motor needs to be incremented, instead of incrementing all the motors
      //nh.spinOnce();
      TCA9548A(6);
        if (angle_6 > true_angle6+0.2){ //checks for which direction to rotate stepper motor
          digitalWrite(motor_dir6,LOW);
          digitalWrite(motor_step6,HIGH); //rotates stepper motor with PWM to stepper driver
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step6,LOW);
          delayMicroseconds(mtrs);
         
        }
       if (angle_6 < true_angle6-0.2) { //checks for which direction to rotate stepper motor
          digitalWrite(motor_dir6,HIGH);
          digitalWrite(motor_step6,HIGH); //rotates stepper motor with PWM to stepper driver
          delayMicroseconds(mtrs);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(motor_step6,LOW);
          delayMicroseconds(mtrs);
           
       }
       counter2++; //adds to motors incremented counter to adjust speed if needed to acount for timing in pulses of a motor not needing to be incremented anymore
     }
     nh.spinOnce(); //once again makes sure the ROS position hasn't changed and if it has it will be updated while still running this loop

     if(counter2 == 0){ //adjusts the speed of the stepper motors proportionally to the change the timing of the stepper motors, as if less stepper motors are being incremented, the time/speed between the changes
      mtrs = 500;
     }
     if(counter2 == 1){
      mtrs = 500;
     }
     if(counter2 > 1){ //algorithm for adjusting speed
      mtrs = 1000/(counter2-1);
     }
     counter2 = 0; //resets after every loop to ensure stepper motors don't get out of line
     //Serial.println(mtrs); //debugging

   }
}
