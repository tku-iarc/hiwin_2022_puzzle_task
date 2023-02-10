bool button = 0;
bool old_button = 0;

void setup() {
  pinMode(8,INPUT_PULLUP);
  Serial.begin(9600);

}

void loop() {
  button = digitalRead(8);


  if(button){
//    Serial.println("no");
      old_button=0;
  }else if(old_button==0){
    old_button = 1;
    Serial.println("yes");
    //Serial.println("yes");
    delay(100);
  }
  delay(5);

}
