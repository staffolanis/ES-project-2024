#include <miosix.h>
#include <cstdio>

#define USB_FREQ 48000000

using namespace std;
using namespace miosix;

typedef Gpio<GPIOD_BASE, 12> greenLED; // Define GPIO pin for green LED

struct Parameters {
    int N;
    int M;
    int P;
    int Q;

    int errorF;
    int errorFUSB;

    Parameters() : N(0), M(0), P(0), Q(0), errorF(0), errorFUSB(0) {}
};

template<int _InF, int _OutFTarget>
Parameters setParameters() {
    Parameters params;
    constexpr float UsbCheck = (float)USB_FREQ;
    constexpr float InF = (float)_InF;
    constexpr float OutFTarget = (float)_OutFTarget;
    float usb_check = UsbCheck;

    for (int n = 50; n < 433; n++) {
        for (int m = 2; m < 64; m++) {
            float vco = InF * n / m;
            for (int q = 2; q < 16; q++) {
                for (int p = 2; p < 10; p += 2) {
                    float out_f = vco / p;
                    float out_usb = vco / q;
                    float diff_usb = (float)USB_FREQ - out_usb;

                    if ((out_f == OutFTarget) && (diff_usb <= usb_check && diff_usb >= 0.0)) {
                        usb_check = diff_usb;
                        params.N = n;
                        params.M = m;
                        params.P = p;
                        params.Q = q;

                        //TODO: pipo -> check frequence error micro
                        /*params.errorF = (int)(out_f-OutFTarget);
                        if(params.errorF<0) params.errorF *= -1;
                        params.errorF /= OutFTarget;*/
                        params.errorFUSB = (int)((diff_usb/(float)USB_FREQ)*10000);
                    }
                }
            }
        }
    }
    return params;
}

int main() {
    // debug parameters' calculation
    Parameters result = setParameters<8000000, 100000000>();
    iprintf("Configurazione finale:\n n: %d\n m: %d\n p: %d\n q: %d\n", result.N ,result.M ,result.P ,result.Q );
    iprintf("F error: %d/100\n", result.errorF);
    iprintf("F_USB error: %d/100\n", result.errorFUSB);

    // blinking led as experiment
    greenLED::mode(Mode::OUTPUT);
    while(true){
        greenLED::high();
        Thread::sleep(500);
        greenLED::low();
        Thread::sleep(500);
    }
}