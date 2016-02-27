void send_rf_data()
{
  //Serial.println("Entering send_rf_data");
  char msg[13];
  uint8_t recv;
  while (SRF.available()) {
    recv = SRF.read();
    //Serial.write(recv);
  }
  if (CT1) {
    snprintf(msg, 13, "a%c%cPWRA%05d", PANID[0], PANID[1], emontx.power1);
    //Serial.print(msg);
    //Serial.println();
    SRF.write((const unsigned char *)msg, 12);
  }
  if (CT2) {
    snprintf(msg, 13, "a%c%cPWRB%05d", PANID[0], PANID[1], emontx.power2);
    //Serial.print(msg);
    //Serial.println();
    SRF.write((const unsigned char *)msg, 12);
  }
  if (CT3) {
    snprintf(msg, 13, "a%c%cPWRC%05d", PANID[0], PANID[1], emontx.power3);
    //Serial.print(msg);
    //Serial.println();
    SRF.write((const unsigned char *)msg, 12);
  }
  if (CT4) {
    snprintf(msg, 13, "a%c%cPWRD%05d", PANID[0], PANID[1], emontx.power4);
    //Serial.print(msg);
    //Serial.println();
    SRF.write((const unsigned char *)msg, 12);
  }
  //Serial.println("Leaving send_rf_data");
}

void send_temp_data()
{
  char msg[13];
  uint8_t recv;
  int rem;
  while (SRF.available()) {
    recv = SRF.read();
    //Serial.write(recv);
  }
  rem = (int)((int)(emontx.temp * 10 + 0.5) % 10);
  snprintf(msg, 13, "a%c%cTEMP%03d.%d", PANID[0], PANID[1], (int)emontx.temp, abs(rem));
  //Serial.print(msg);
  //Serial.println();
  SRF.write((const unsigned char *)msg, 12);
}

