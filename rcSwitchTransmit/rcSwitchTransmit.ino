/*
  Example for different sending methods
  
  https://github.com/sui77/rc-switch/
  https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
  ----------------------------------------------------------
  Mod by Little Satan. Have Fun!
  ----------------------------------------------------------
*/
#include <SmartRC_CC1101.h>
#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();
SmartRC_CC1101 myRadio;

void setup() {
  Serial.begin(9600);

  myRadio.setSpiPin(18, 19, 23, 21);
  myRadio.Init();

  if (myRadio.getCC1101()){       // Check the CC1101 Spi connection.
  Serial.println("Connection OK");
  }else{
  Serial.println("Connection Error");
  }

  myRadio.setMHZ(433.90);
 
  mySwitch.enableTransmit(22);

  myRadio.SetTx();
}

void loop() {
  mySwitch.send(4835521, 24);
  delay(10000);
}