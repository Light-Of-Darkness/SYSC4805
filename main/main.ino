//SYSC 4805 Robot Project

// SENSOR PINS (can be changed)
// DIGITAL
const int ULTRASONIC_1 = 4;
const int ULTRASONIC_2 = 5;
const int ULTRASONIC_3 = 6;
const int ULTRASONIC_TRIG = 0;
//reuse the same pin to trigger all ultrasonic sensors

// ANALOG
const int INFRARED_1 = A0;
const int INFRARED_2 = A1;
const int INFRARED_3 = A2;

//speed of sound in cm/us
const float SPEED_OF_SOUND = 0.034;

int pollUltrasonicSensor(int sensorEchoPin);

void setup() 
{
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_1, INPUT);
  pinMode(ULTRASONIC_2, INPUT);
  pinMode(ULTRASONIC_3, INPUT);
}

void loop() 
{
  
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
