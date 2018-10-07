#include <Arduino.h>

namespace axx {

void main();

} // namespace axx

void setup() {
    axx::main();
}

void loop() {
    Serial.println( "A++ ERROR: main ended!" );
    delay( 1000 );
}
