#include <miosix.h>
#include <cstdio>

#define USB_FREQ 48000000

using namespace std;
using namespace miosix;


typedef Gpio<GPIOD_BASE, 12> greenLED; // Define GPIO pin for green LED

// function to calculate PLL's parameters
void setParameters(float in_f, float out_f_target) {
    float usb_check = USB_FREQ;
    int N = 0, M = 0, P = 0, Q = 0;

    for (int n = 50; n <= 432; n++) {
        for (int m = 2; m <= 63; m++) {
            float vco = in_f * n / m;
            for (int q = 2; q <= 15; q++) {
                for (int p = 2; p<= 8; p+=2) {
                    float out_f = vco / p;
                    float out_usb = vco / q;
                    float diff_usb = (float)USB_FREQ - out_usb;

                    if ((out_f == out_f_target) && (diff_usb <= usb_check && diff_usb >= 0)) {
                        usb_check = diff_usb;
                        N = n;
                        M = m;
                        P = p;
                        Q = q;
                        iprintf("out_f=%d, usb_f=%d\nn:%d, m:%d, p:%d, q:%d\n", (int)out_f, (int)out_usb, N,M,P,Q);
                    }
                }
            }
        }
    }

    // calculating values with the found configuration
    float vco = in_f * N / M;
    float out_f = vco / P;
    float out_usb = vco / Q;

    iprintf("Configurazione finale:\n n: %d\n m: %d\n p: %d\n q: %d\n", N,M,P,Q);
    iprintf("output freq: %d\n", (int)out_f);
    iprintf("usb freq: %d\n", (int)out_usb);
}

int main() {
    // debug parameters' calculation
    setParameters(8000000.0, 168000000.0);

    // blinking led as experiment
    greenLED::mode(Mode::OUTPUT);
    while(true){
        greenLED::high();
        Thread::sleep(500);
        greenLED::low();
        Thread::sleep(500);
    }
}