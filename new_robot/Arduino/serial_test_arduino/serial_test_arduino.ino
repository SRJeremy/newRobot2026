int FL = 0, FR = 0, BL = 0, BR = 0;

void setup() {
  Serial.begin(115200);

  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);

  delay(2000);

  Serial.println("Serial is up and running!");
  tone(22, 500, 300);
  delay(300);
  Serial.println("Press gp20 to begin");
  pinMode(20, INPUT);
  while (digitalRead(20));
  Serial.println("Starting SOON");
  Serial1.write('s');
  //delay(1000);
}

void loop() {
  if(Serial1.available()){
    Serial.print("data recieved: ");
    Serial.print(Serial1.available());
    Serial.println(" bytes");

    FL = Serial1.parseInt();
    FR = Serial1.parseInt();
    BL = Serial1.parseInt();
    BR = Serial1.parseInt();
//    FL = Serial1.read();
//    if(Serial1.available())
//      FR = Serial1.read();
//    if(Serial1.available())
//      BL = Serial1.read();
//    if(Serial1.available())
//      BR = Serial1.read();


    Serial.print("Front Left:  ");
    Serial.println(FL);
    Serial.print("Front Right: ");
    Serial.println(FR);
    Serial.print("Back Left:   ");
    Serial.println(BL);
    Serial.print("Back Right:  ");
    Serial.println(BR);

    /*
    while(Serial1.available()){
      Serial.print("REMOVED: ");
      Serial.println(Serial1.read());
    }
    */

    
  }

}
