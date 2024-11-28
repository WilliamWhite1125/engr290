#include state_control.h;

void setup(){
    State=INIT;
}
void loop() {
    handleState();
}
