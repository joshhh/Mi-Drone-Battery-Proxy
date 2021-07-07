uint8_t writeByte(byte b) {
  return WireBat.i2c_write(b);
}

uint8_t beginTransmission(byte address) {
  if(WireBat.i2c_start())
    return writeByte((address << 1) | 0);
  else
    return 1;
}

void endTransmission(bool sendStop) {
  if(sendStop)
    WireBat.i2c_stop();
  else
    WireBat.i2c_repstart();
}

uint8_t writeBytes(byte bytes[], uint8_t len) {
  uint8_t status = 0;
  for(uint8_t i= 0; i < len; i ++) {
    status = writeByte(bytes[i]);
    if (status)
      break;
  }
  return status;
}

uint8_t requestFrom(byte address) {
  if(WireBat.i2c_start())
    return writeByte((address << 1) | 1);
  else
    return 1;
}

uint8_t readByte(bool last) {
  return WireBat.i2c_read(!last);
}

uint8_t readByte() {
  return readByte(false);
}
