
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_APDS9960.h"
#include <Adafruit_BNO055.h>

#define NEOPIN 7
#define NUMPIXELS 24
#define BRIGHT 100


#define COMMAND_LINE_TRACE 0
#define COMMAND_LEFT_GREEN 1
#define COMMAND_RIGHT_GREEN 2
#define COMMAND_DOUBLE_GREEN 3



int bnoID = 55;
int bnoADDR = 0x28;

Adafruit_BNO055 bno(bnoID, bnoADDR);
sensors_event_t sensorData;




Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIN, NEO_GBR + NEO_KHZ800);

// Slightly different, this makes the rainbow equally distributed throughout the ring
void rainbowCycle(uint8_t wait) {
  // 5 cycles of all colors on wheel, you can adjust the number 
  // of cycles by changing the first value (e.g., 1*65536 for one cycle)
  for(long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    for(int i=0; i<pixels.numPixels(); i++) { // For each pixel in the ring
      // Offset pixel hue by an amount proportional to its position in the ring
      int pixelHue = firstPixelHue + (i * 65536L / pixels.numPixels());
      // Set the pixel color using the HSV color wheel (HSV is better for rainbows)
      pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(pixelHue)));
    }
    if(digitalRead(21) == 0)
    {
      break;
    }
    pixels.show(); // Update the LED ring
    delay(wait);  // Pause for a moment
  }
}

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

Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_APDS9960 apds;

/* PRINT FUNCTIONS */
// char arry version
void print(int line_number, char s[])
{
  display.setCursor(0, line_number * 6);
  display.print(s);
  // display.display();
}






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



// negative = left turn
// positive = right turn
void drag_turn_imu(int deg){
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
      FL.speed(-40);
      BL.speed(-40);
      FR.speed(70);
      BR.speed(70);
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
      FL.speed(70);
      BL.speed(70);
      FR.speed(-40);
      BR.speed(-40);
    }while( ! (curr_deg < deg+sdt && curr_deg > deg-sdt));

    Serial.print("RIGHT TURN STOP: ");
    Serial.println(curr_deg);
  }

  FL.speed(0);
  BL.speed(0);
  FR.speed(0);
  BR.speed(0);
  
}


void forward_encoders(int distanceCM){
  int encCPR = 3575/4;
  float wheel_diameter = 6.1;
  float encPerCm = encCPR / (wheel_diameter * 3.1415);

  int target = encPerCm * distanceCM;
  
  FL.speed(40);
  FR.speed(40);
  BL.speed(40);
  BR.speed(40);

  encoders = 0;
  while(encoders < target){
    //Serial.print("Enc: ");
    //Serial.println(encoders);
    delay(2);
  }

  FL.speed(0);
  FR.speed(0);
  BL.speed(0);
  BR.speed(0);
}





void setup() {
  Serial.begin(115200);

  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);

  pixels.begin();
  pixels.setBrightness(50);

  pinMode(direction_pin, INPUT);
  pinMode(encoder_pin, INPUT);
  attachInterrupt(encoder_pin,enc_update, RISING);

  
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();
  if(bno.begin(OPERATION_MODE_IMUPLUS) == 0){
  //if(bno.begin() == 0){
    Serial.println("IMU Failed");
    while(1); // dont proceed
  }
  

  delay(2000);
  tone(22, 700, 200);
  delay(200);
  
  Serial.println("Press gp20 to begin");
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  while (digitalRead(21) == 1) // while not pressed
  {
    rainbowCycle(5);
  }
  while(digitalRead(21) == 0);  // while pressed, wait for release
  Serial1.write("s");


  // SETUP REGULAR LIGHTING
  //pixels.setBrightness(50);
  for(int i = 0; i < NUMPIXELS; i++){
    pixels.setPixelColor(i, pixels.Color(BRIGHT, BRIGHT, BRIGHT));
  }
  pixels.show();
  
}

int stop_motors = 0;
int FLs = 0, FRs = 0, BLs = 0, BRs = 0;
int move_command = 0;
void loop() {

  if(digitalRead(21) == 0 || digitalRead(20) == 0)
  {
    stop_motors = !stop_motors;
    delay(40);
  }
  
  if(Serial1.available()){
    Serial.print("data recieved: ");
    Serial.print(Serial1.available());
    Serial.println(" bytes");

    move_command = Serial1.parseInt();
    

    FLs = Serial1.parseInt();
    FRs = Serial1.parseInt();
    BLs = Serial1.parseInt();
    BRs = Serial1.parseInt();

    if(stop_motors){
      FLs = FRs = BLs = BRs = 0;
    }

    // check the move command
    if(move_command == COMMAND_LINE_TRACE) {
      // line trace
      FL.speed(FLs);
      FR.speed(FRs);
      BL.speed(BLs);
      BR.speed(BRs);
    }
    else if(move_command == COMMAND_LEFT_GREEN){
      // left turn green

      
      forward_encoders(5);
      
      tone(22, 200, 200);
      delay(200);

      turn_imu(-90);

      FL.speed(0);
      FR.speed(0);
      BL.speed(0);
      BR.speed(0);
      tone(22, 200, 200);
      delay(200);

      forward_encoders(5);

      Serial1.write("c");
      delay(100);
      
    }
     else if(move_command == COMMAND_RIGHT_GREEN){
      // right turn green
      
      forward_encoders(5);
      
      tone(22, 500, 200);
      delay(200);

      turn_imu(90);

      FL.speed(0);
      FR.speed(0);
      BL.speed(0);
      BR.speed(0);
      tone(22, 500, 200);
      delay(200);

      forward_encoders(5);
      
      Serial1.write("c");
      delay(100);
      
    }
     else if(move_command == COMMAND_DOUBLE_GREEN){
      // double turn green
      FL.speed(0);
      FR.speed(0);
      BL.speed(0);
      BR.speed(0);
      tone(22, 800, 200);
      delay(200);

      turn_imu(180);

      FL.speed(0);
      FR.speed(0);
      BL.speed(0);
      BR.speed(0);
      tone(22, 800, 200);
      delay(200);

      forward_encoders(5);

      Serial1.write("c");
      delay(100);
      
    }

    // clear buffer???
    while(Serial1.available()) Serial1.read();
  }

}
