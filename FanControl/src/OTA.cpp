#include "OTA.h"
#include "Config.h"
#include <ArduinoOTA.h>

void OTASetup::init() {
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
}

void OTASetup::handle() {
    ArduinoOTA.handle();
}
