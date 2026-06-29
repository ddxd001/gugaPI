#include "services/fault.h"

namespace services {

static volatile FaultCode g_faultCode = FAULT_NONE;

void Fault_Init(void)
{
    g_faultCode = FAULT_NONE;
}

void Fault_Set(FaultCode code)
{
    if (code != FAULT_NONE) {
        g_faultCode = code;
    }
}

FaultCode Fault_Get(void)
{
    return g_faultCode;
}

void Fault_Clear(void)
{
    g_faultCode = FAULT_NONE;
}

void Fault_Panic(FaultCode code)
{
    Fault_Set(code);

    while (1) {
        /*
         * Future board support can blink an error LED or print the code here.
         * Keep the loop simple so panic behavior is deterministic.
         */
    }
}

} /* namespace services */