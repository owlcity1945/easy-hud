
void doubleclick()
{
  double_flag = 1;
  int nb = 3;
  while (nb--)BreathLed();
  double_flag = 0;
  click_flag = 0;
}
void click()
{
  click_flag = 1;
  // int nf = 30;
  // while (nf--)FlashLed();
  // click_flag = 0;
}
void longPressStart()
{
  long_flag = 1;
  Press_Beep();
  uint16_t st = encoderPos;
  while (st--) {
    encoderPos -= 1;
    Turn_Beep();
    // ledcWrite(channel, 255 - 0.35 * encoderPos);
  }
  long_flag = 0;
}
void duringLongPress()
{
  if (button.isLongPressed())
  {
    delay(50);
    Serial.println("duringLongPress:");
  }
}
void longPressStop()
{
  Serial.println("longPressStop");
}
