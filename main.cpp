#include <miosix.h>
using namespace miosix;
int main()
{
    for(;;)
    {
        ledOn();
        delayMs(1000);
        ledOff();
        delayMs(1000);
    }
}