int cmap(int val, int olow, int ohigh, int mlow, int mhigh) {
  return constrain(map(val, olow, ohigh, mlow, mhigh), mlow, mhigh);
}
struct Motor {
  uint8_t fpin;
  uint8_t rpin;
  void speed(int val);
} Lmotor{9, 8}, Rmotor{10, 11};
void Motor::speed(int val) {
  int map_speed = cmap(abs(val), 0, 100, 0, 255);
  if (val > 0) {
    analogWrite(fpin, map_speed);
    analogWrite(rpin, 0);
  }
  else {
    analogWrite(fpin, 0);
    analogWrite(rpin, map_speed);
  }
}
//000000000000000000000000000000000000000000000000
//
//
//
//////
//
void setup() {
  Serial.begin(115200);

  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);

  delay(2000);

  Serial.println("Serial is up and running!");

  Serial.println("Press gp20 to begin");
  pinMode(22, INPUT);
  while (digitalRead(20));
  Serial.println("Starting SOON");
  Serial1.write('s');
  delay(1000);
}
//
void loop() {
  //Serial.println("run");
  //Serial.println(Serial1.available());
  if (Serial1.available()) {
    Serial.println("run2");
    // read in the int
    int lm = Serial1.parseInt();
    int rm = Serial1.parseInt();
    int green = Serial1.parseInt();
    Serial.print("Read Data: ");
    Serial.println(lm);
    //Serial1.read();
    Serial.println(rm);

    if (green == 0) {
      Lmotor.speed(lm);
      Rmotor.speed(rm);
    }
    else {
      if (green == 2) {
        tone(22, 1000, 10000);
      }
      if((green==1)||(green==-1)){
        Lmotor.speed(20);
        Rmotor.speed(20);
        delay(200);
        Lmotor.speed(0);
        Rmotor.speed(0);
        }
    //  tone(22, 500, 500);
      //
      Lmotor.speed(0);
      Rmotor.speed(0);
      delay(1000);
    }//
    // flush the remaining bytes
    while (Serial1.available()) {
      Serial1.read();
    }

    // wait for a short time
    //delay(400);
    //0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
    // send data
    //char out_data = 'a';
    // Serial.print("Send Data: ");
    // Serial.println(out_data);
    // Serial1.print(out_data);



  }


}
