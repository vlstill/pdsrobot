#include "axx.hpp"
#include <Arduino.h>

void setup() {
    Serial.begin( 9600 );
    Serial.println( "A++ INFO: init" );
    axx::main();
}

void loop() {
    Serial.println( "A++ ERROR: main ended!" );
    delay( 1000 );
}
