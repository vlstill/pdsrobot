#include "axx.hpp"
#include <Arduino.h>

void setup() {
    axx::main();
}

void loop() {
    Serial.println( "A++ ERROR: main ended!" );
    delay( 1000 );
}
