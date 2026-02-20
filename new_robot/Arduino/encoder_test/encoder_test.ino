// micro metal gear motor
// 12 counts per revolution (12 cpr)
// multiply 12 by the gear reduction ratio
// 12 * 297.9 = 3575 encoders per revolution

// map function that constrains bad input
// value, origional range, modified range
int cmap(int val, int olow, int ohigh, int mlow, int mhigh)
{
  return constrain(map(val, olow, ohigh, mlow, mhigh), mlow, mhigh);
}

// struct to make motor control easier
struct Motor
{
  uint8_t fpin;        // fwd pin
  uint8_t rpin;        // reverse pin
  void speed(int val); // function setting speed
};

// speed function for setting motor speed
void Motor::speed(int val)
{
  int map_speed = cmap(abs(val), 0, 100, 0, 255);
  // check if its forward or reverse
  if (val > 0)
  {
    analogWrite(fpin, map_speed);
    analogWrite(rpin, 0);
  }
  else
  {
    analogWrite(fpin, 0);
    analogWrite(rpin, map_speed);
  }
}

// Motor FL{14, 15}, BL{12, 13};
// Motor FR{10, 11}, BR{8, 9};
Motor FL{15, 14}, BL{9, 8};
Motor FR{13, 12}, BR{11, 10};



// global
const uint8_t direction_pin = 26;
const uint8_t encoder_pin = 27;

// variable to store the current encoders
volatile int64_t encoders = 0;

// encoder interupt function
void enc_update(){
  if(digitalRead(direction_pin) == HIGH){
    encoders++;
  } else {
    encoders--;
  }
}


void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(direction_pin, INPUT);
  pinMode(encoder_pin, INPUT);
  attachInterrupt(encoder_pin,enc_update, RISING);

  pinMode(21, INPUT);
  while(digitalRead(21)) delay(10);

  int encCPR = 3575/4;
  float wheel_diameter = 6.1;
  float encPerCm = encCPR / (wheel_diameter * 3.1415);
  int distanceCM = 20;

  int target = encPerCm * distanceCM;
  
  FL.speed(40);
  FR.speed(40);
  BL.speed(40);
  BR.speed(40);

  encoders = 0;
  while(encoders < target){
    Serial.print("Enc: ");
    Serial.println(encoders);
  }

  FL.speed(0);
  FR.speed(0);
  BL.speed(0);
  BR.speed(0);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Enc: ");
  Serial.println(encoders);
  delay(5);


}
