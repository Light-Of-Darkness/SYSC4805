//SYSC 4805 Robot Project

// SENSOR PINS (can be changed)
// DIGITAL
const int ULTRASONIC_OBJ_L = 4;
const int ULTRASONIC_OBJ_R = 5;
const int ULTRASONIC_EDGE = 6;
const int ULTRASONIC_TRIG = 9;
//reuse the same pin to trigger all ultrasonic sensors

//pins for controlling motor
const int MOTOR_LEFT_F = 12;
const int MOTOR_LEFT_R = 11;
const int MOTOR_RIGHT_F = 8;
const int MOTOR_RIGHT_R = 7;

const int SENSOR_POWER = 13;    //checking if the sensor power supply is ON

// infrared
const int INFRARED_left = 14;
const int INFRARED_middle = 15;
const int INFRARED_right = 16;
const int INFRARED_test = 17;
const int INFRARED_end = 18;

const int IRDETEC_left = 2;
const int IRDETEC_right = 3;

//speed of sound in cm/us
const float SPEED_OF_SOUND = 0.034;

//begin object avoidance once the object is less than 10 cm from one of the sensors
const int OBJ_AVOID_DISTANCE = 15;

//threshold for edge detection
const int FLOOR_DISTANCE = 4;

int pollUltrasonicSensor(int sensorEchoPin);
int objectDetection();
bool edgeDetection();

void forward();
void backward();
void brake();
void left();
void right();

void setup() 
{
  //setting pin modes
  Serial.begin(9600); 
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_OBJ_L, INPUT);
  pinMode(ULTRASONIC_OBJ_R, INPUT);
  pinMode(ULTRASONIC_EDGE, INPUT);
  pinMode(INFRARED_left, INPUT);
  pinMode(INFRARED_middle, INPUT);
  pinMode(INFRARED_right, INPUT);
  pinMode(INFRARED_test, INPUT);
  pinMode(INFRARED_end, INPUT);
  pinMode(IRDETEC_left, INPUT);
  pinMode(IRDETEC_right, INPUT);
  pinMode(SENSOR_POWER, INPUT);
  pinMode(MOTOR_LEFT_F, OUTPUT);
  pinMode(MOTOR_LEFT_R, OUTPUT);
  pinMode(MOTOR_RIGHT_F, OUTPUT);
  pinMode(MOTOR_RIGHT_R, OUTPUT);

  delay(5000);
  
  brake();
}

void loop() 
{
  //delay(2000);
  /* priority is as follows:
   *  
   * SAFETY MECHANISMS HAVE TOP PRIORITY:
   * if the sensors do not have power, the robot should not move
   * edge detection should stop the robot and reroute it
   * object avoidance should kick in if no edge is detected
   * 
   * at this point the robot is in a "safe" environment
   * use IR to follow any robot in front of it
   * if no IR light is present, follow a black line on the ground using down-facing IR emitter/detector pairs
   * if no black line is present, roll forward
   */

  //check if the sensors have power by seeing if power input is 5V
  if(digitalRead(SENSOR_POWER) == HIGH)
  {
    int objD = objectDetection();
    //objD=0;
    bool edgeDetected = edgeDetection();
    //stub bools to emulate IR and line detection
   // bool irDetected = false;
   int IRleft,IRmiddle,IRright,IRtest,IRend,IRDETECleft,IRDETECright;
   IRleft=digitalRead(INFRARED_left);
   IRmiddle=digitalRead(INFRARED_middle);
   IRright=digitalRead(INFRARED_right);
   IRtest=digitalRead(INFRARED_test);
   IRend=digitalRead(INFRARED_end);
   IRDETECleft=digitalRead(IRDETEC_left);
   IRDETECright=digitalRead(IRDETEC_right);
   Serial.println(edgeDetected);
   Serial.println(objD);
   Serial.println(IRleft);
   Serial.println(IRmiddle);
   Serial.println(IRright);
   Serial.println(IRtest);
   Serial.println(IRend);
   Serial.println(IRDETECleft);
   Serial.println(IRDETECright);
    
    //SAFETY CHECKS
    if(edgeDetected)
    {
      Serial.print("\nedgeDetected\n");
      //TODO: rerouting
      //brake as a placeholder
      brake();
    }
    else if(objD != 0)
    {
      if(objD > 0)
      {
        //more space on the right
        //TODO: turn right
        Serial.print("\nobjectDetected:right\n");
        right();
      }
      else
      {
        //more space on the left
        //TODO: turn left
        Serial.print("\nobjectDetected:left\n");
        left();
      }
    }
    else if(IRDETECleft==0||IRDETECright==0){
      if(IRDETECleft==0&&IRDETECright==0){
        Serial.print("\nbothIRDetected:left\n");
        forward();
      }else if(IRDETECleft==0){
        Serial.print("\nleftIRDetected:left\n");
        left();
      }else if(IRDETECright==0){
        Serial.print("\nrightIRDetected:left\n");
        right();
      }else{
        brake();
      }
    }
    else if(IRtest==1||IRleft==1||IRright==1||IRmiddle==1){
      Serial.print("\nmaze\n");
      if(IRend==1&&IRleft==1&&IRmiddle==1&&IRright==1){
        Serial.print("\nmazeEnd\n");
        brake();
      }
      else if(IRleft==1){
        Serial.print("\nmazeLeft\n");
        left();
      }else if(IRmiddle==1){
        Serial.print("\nmazeMiddle\n");
        forward();
      }else if(IRright==1){
        Serial.print("\nmazeRight\n");
        right();
      }else if(IRtest==1){
        left();
      }
    }
    else
    {
      //default locomotion should be to roll forward until a higher-priority check is passed
      forward();
    }
  }
}

//polls the given sensor for distance in centimeters, accurate between 2 and 200 cm
int pollUltrasonicSensor(int sensorEchoPin)
{
  //trigger all ultrasonic sensors by setting the trig pin to HIGH for 10 us
  //set trig pin to low for 2 us before to ensure a cleaner pulse
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  //reads the echo pin to return the sound wave travel time in microseconds
  long duration = pulseIn(sensorEchoPin, HIGH);

  //calculate distance
  int distance = duration * SPEED_OF_SOUND/2;

  return distance;
}

//checks for objects in front using the ultrasonic sensors
//returns the difference in distance between the two front facing ultrasonic sensors
//positive is right, negative is left
//0 means no object detected
//UNTESTED
int objectDetection()
{
  int leftD, rightD, diff;
  leftD = pollUltrasonicSensor(ULTRASONIC_OBJ_L);
  rightD = pollUltrasonicSensor(ULTRASONIC_OBJ_R);

  //above the threshold distance, the object is irrelevant.
  if(leftD > OBJ_AVOID_DISTANCE && rightD > OBJ_AVOID_DISTANCE)
    diff = 0;
  else
  {
    diff = rightD - leftD;
    //if the object is equidistant from both front facing sensors, make diff non-zero so that the function does not falsely return no object detected
    if(diff == 0)
      diff--;
  }

  return diff;
}

void forward()
{
  digitalWrite(MOTOR_LEFT_F, HIGH);
  digitalWrite(MOTOR_LEFT_R, LOW);
  digitalWrite(MOTOR_RIGHT_F, HIGH);
  digitalWrite(MOTOR_RIGHT_R, LOW);
}

void backward()
{
  digitalWrite(MOTOR_LEFT_F, LOW);
  digitalWrite(MOTOR_LEFT_R, HIGH);
  digitalWrite(MOTOR_RIGHT_F, LOW);
  digitalWrite(MOTOR_RIGHT_R, HIGH);
}

void left()
{
  digitalWrite(MOTOR_LEFT_F, LOW);
  digitalWrite(MOTOR_LEFT_R, HIGH);
  digitalWrite(MOTOR_RIGHT_F, HIGH);
  digitalWrite(MOTOR_RIGHT_R, LOW);
}

void right()
{
  digitalWrite(MOTOR_LEFT_F, HIGH);
  digitalWrite(MOTOR_LEFT_R, LOW);
  digitalWrite(MOTOR_RIGHT_F, LOW);
  digitalWrite(MOTOR_RIGHT_R, HIGH);
}

void brake()
{
  digitalWrite(MOTOR_LEFT_F, LOW);
  digitalWrite(MOTOR_LEFT_R, LOW);
  digitalWrite(MOTOR_RIGHT_F, LOW);
  digitalWrite(MOTOR_RIGHT_R, LOW);
}

bool edgeDetection()
{
   int distance = pollUltrasonicSensor(ULTRASONIC_EDGE);
   if(distance > FLOOR_DISTANCE)
   {
    //edge detected
    Serial.print("\nedgeDetection:true\n");
    return true; 
   }
   Serial.print("\nedgeDetection:false\n");
   return false;
}
