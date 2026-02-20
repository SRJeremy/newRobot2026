void setup() {
  Serial.begin(115200);

  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);

  delay(2000);
  
  Serial.println("Press gp20 to begin");
  pinMode(20, INPUT);
  while (digitalRead(20));
  Serial1.write("start");
}


char buff[100];
char echo[100];

void loop() {
  if(Serial1.available()){
    Serial.print("data received: ");
    delay(100); // let all data come in
    
    int ct = 0;
    while(Serial1.available()){
      buff[ct++] = Serial1.read();
    }

    buff[ct] = '\0';
    Serial.println(buff);
    
    
    sprintf(echo, "received message %c", buff[0]);
    Serial.print("Sending back response: ");
    Serial.println(echo);

    Serial1.write(echo, strlen(echo)+1);

    Serial.println();
  }

}
