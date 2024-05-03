#include <miosix.h>
using namespace miosix;
int main()
{
    for(;;)
    {
        ledOn();
        delayMs(5000);
        ledOff();
        delayMs(5000);
    }
}