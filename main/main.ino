//SYSC 4805 Robot Project

// SENSOR PINS (can be changed)
// DIGITAL
const int ULTRASONIC_OBJ_L = 4;
const int ULTRASONIC_OBJ_R = 5;
const int ULTRASONIC_EDGE = 6;
const int ULTRASONIC_TRIG = 0;
//reuse the same pin to trigger all ultrasonic sensors

// ANALOG
const int INFRARED_1 = A0;
const int INFRARED_2 = A1;
const int INFRARED_3 = A2;

//speed of sound in cm/us
const float SPEED_OF_SOUND = 0.034;

//begin object avoidance once the object is less than 10 cm from one of the sensors
const int OBJ_AVOID_DISTANCE = 10;

int pollUltrasonicSensor(int sensorEchoPin);
int objectDetection();
//TODO: locomotion functions (rolling forward, turning)

void setup() 
{
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_OBJ_L, INPUT);
  pinMode(ULTRASONIC_OBJ_R, INPUT);
  pinMode(ULTRASONIC_EDGE, INPUT);
}

void loop() 
{
  /* priority is as follows:
   *  
   * SAFETY MECHANISMS HAVE TOP PRIORITY:
   * edge detection should stop the robot and reroute it
   * object avoidance should kick in if no edge is detected
   * 
   * at this point the robot is in a "safe" environment
   * use IR to follow any robot in front of it
   * if no IR light is present, follow a black line on the ground using down-facing IR emitter/detector pairs
   * if no black line is present, roll forward
   */
  int objD = objectDetection();
  bool edgeDetected = false; //TODO: replace with edge detection function
  //stub bools to emulate IR and line detection
  bool irDetected = false;
  bool lineDetected = false;
  
  //SAFETY CHECKS
  if(edgeDetected)
  {
    //TODO: rerouting
  }
  else if(objD != 0)
  {
    if(objD > 0)
    {
      //more space on the right
      //TODO: turn right
    }
    else
    {
      //more space on the left
      //TODO: turn left
    }
  }
  else if(irDetected)       //Safety checks passed here, start maze solving
  {
    //TODO: robot following
  }
  else if(lineDetected)
  {
    //TODO: line following
  }
  else
  {
    //default locomotion should be to roll forward until a higher-priority check is passed
    //TODO: move forward
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
      diff++;
  }

  return diff;
}
