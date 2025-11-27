unsigned long last_encoder_time = 0;
//编码器
void Encoder() {

  int aState = digitalRead(32);  // A 相
  int bState = digitalRead(33);  // B 相

  if (aState != aLastState) {
    if (bState != aState) {

      encoderPos += 1;
      // ledcWrite(channel, 255 - 0.35 * encoderPos);
    } else {

      encoderPos -= 1;
      // ledcWrite(channel, 255 - 0.35 * encoderPos);
    }
    // Serial.println(encoderPos);
    // 蜂鸣提示
    // Turn_Beep();

  }

  aLastState = aState;
//  last_encoder_time = millis();
  
  //限幅
  if (encoderPos > 360) encoderPos = 0;
  else if (encoderPos < 0) encoderPos = 360;
}


void BreathLed() {
  uint16_t st = 60;
  while (st--) {
    encoderPos += 2;
    // ledcWrite(channel, 255 - 0.35 * encoderPos);
  }
  Turn_Beep();
  st = encoderPos / 2;
  while (st--) {
    encoderPos -= 2;
    // ledcWrite(channel, 255 - 0.35 * encoderPos);
  }
  Press_Beep();
}

void FlashLed() {
  uint16_t st = 120 / flash_freq;
  while (st--) {
    encoderPos += flash_freq;
    // ledcWrite(channel, 255 - 0.35 * encoderPos);
  }
  Turn_Beep();
  st = encoderPos / flash_freq;
  while (st--) {
    encoderPos -= flash_freq;
    // ledcWrite(channel, 255 - 0.35 * encoderPos);
  }
  Turn_Beep();
}

void Turn_Beep() {
  uint16_t i;
  for(i=0;i<100;i++)
  {
    Beep_on;
    delayMicroseconds(110);
    Beep_off;
    delayMicroseconds(90);
  } 
    
}
void Turn_Beep1() {
  uint16_t i;
  for (i = 0; i < 50; i++) {
    Beep_on;
    delayMicroseconds(i);
    Beep_off;
    delayMicroseconds(3*i);
  }  
}
void Press_Beep() {
  uint16_t j;
  for (j = 0; j < 80; j++) {
    Beep_on;
    delayMicroseconds(70);
    Beep_off;
    delayMicroseconds(30);
  }
}
