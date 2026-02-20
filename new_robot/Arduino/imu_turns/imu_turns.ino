#include <Wire.h>
#include <Adafruit_BNO055.h>


int bnoID = 55;
int bnoADDR = 0x28;

Adafruit_BNO055 bno(bnoID, bnoADDR);
sensors_event_t sensorData;


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


int get_direction(){
  bno.getEvent(&sensorData);
  int ret_deg = 0;
  int stop_thresh = 60;
  do{
    ret_deg = (int)sensorData.orientation.x;
    if(stop_thresh == 0){
      break;
    }
    stop_thresh--;
  }while(ret_deg == 0);

  //Serial.print("stop_thresh: ");
  //Serial.println(stop_thresh);
  //Serial.print("RET_DEG: ");
  //Serial.println(ret_deg);
  return ret_deg;
}


// negative = left turn
// positive = right turn
void turn_imu(int deg){
  /*
  float start_deg = 0;
  int thresh = 50;
  while(start_deg < 0.2 && start_deg > -0.2){
    start_deg = get_direction();
    if(thresh == 0){
      break;
    }
    thresh--;
  }
  */
  int start_deg = get_direction();
  int stop_thresh = 60;
  while(start_deg == 0){
    start_deg = get_direction();
    if(stop_thresh == 0){
      break;
    }
    stop_thresh--;
  }

  Serial.print("START DIRECTION: ");
  Serial.println(start_deg);
  

  // stop degree threshold
  int sdt = 10;
  if(deg < 0){
    // left turn
    // make deg positive just cuz its easier
    deg *= -1;
    
    if(start_deg-deg < 0){
      deg = (start_deg - deg) + 360;
    }
    else{
      deg = start_deg - deg;
    }

    Serial.print("LEFT TURN TARGET: ");
    Serial.println(deg);

    int curr_deg;
    do{
      curr_deg = get_direction();
      FL.speed(-60);
      BL.speed(-60);
      FR.speed(60);
      BR.speed(60);
    }while( ! (curr_deg < deg+sdt && curr_deg > deg-sdt));
    Serial.print("LEFT TURN STOP: ");
    Serial.println(curr_deg);
  }
  else{
    // right turn
    deg = (start_deg + deg) % 360;

    Serial.print("RIGHT TURN TARGET: ");
    Serial.println(deg);


    int curr_deg;
    do{
      curr_deg = get_direction();
      FL.speed(60);
      BL.speed(60);
      FR.speed(-60);
      BR.speed(-60);
    }while( ! (curr_deg < deg+sdt && curr_deg > deg-sdt));

    Serial.print("RIGHT TURN STOP: ");
    Serial.println(curr_deg);
  }

  FL.speed(0);
  BL.speed(0);
  FR.speed(0);
  BR.speed(0);
  
}


void setup() {
  Serial.begin(115200);
  delay(2000);
  Wire.setSDA(16);
  Wire.setSCL(17);
  Wire.begin();
  if(bno.begin(OPERATION_MODE_IMUPLUS) == 0){
  //if(bno.begin() == 0){
    Serial.println("IMU Failed");
    while(1); // dont proceed
  }

  pinMode(21, INPUT);
  pinMode(20, INPUT);

  
  
}
void loop() {
  if(digitalRead(20) == 0){
    turn_imu(-90);

    tone(22, 300, 300);
    delay(300);
  }

  if(digitalRead(21) == 0){
    turn_imu(90);

    tone(22, 700, 300);
    delay(300);
  }
}
