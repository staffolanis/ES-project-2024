#include <miosix.h>
#include <cstdio>

using namespace std;
using namespace miosix;

typedef Gpio<GPIOD_BASE, 12> greenLED; // Define GPIO pin for green LED

int main() {
    iprintf("Main function\n");

    // blinking led as experiment
    greenLED::mode(Mode::OUTPUT);
    while(true){
        greenLED::high();
        Thread::sleep(500);
        greenLED::low();
        Thread::sleep(500);
    }
}