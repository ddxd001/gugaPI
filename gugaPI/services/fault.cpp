#include "services/fault.h"

namespace services {

/*
 * Two fault notions:
 *  - g_currentFault: the most recent fault code (informational).
 *  - g_latchedFault / g_hasFault: the FIRST fault since the last Fault_Clear,
 *    so a transient blip is not erased by a later, possibly less-relevant one.
 *  - g_faultCount: cumulative number of Fault_Set(non-none) calls; preserved
 *    across Fault_Clear as a diagnostic counter.
 */
static volatile FaultCode g_currentFault = FAULT_NONE;
static volatile FaultCode g_latchedFault = FAULT_NONE;
static volatile bool g_hasFault = false;
static volatile uint32_t g_faultCount = 0U;
static FaultPanicHandler g_panicHandler = nullptr;

void Fault_Init(void)
{
    g_currentFault = FAULT_NONE;
    g_latchedFault = FAULT_NONE;
    g_hasFault = false;
    g_faultCount = 0U;
    g_panicHandler = nullptr;
}

void Fault_Set(FaultCode code)
{
    if (code != FAULT_NONE) {
        g_currentFault = code;
        g_faultCount++;
        if (!g_hasFault) {
            g_latchedFault = code;
            g_hasFault = true;
        }
    }
}

FaultCode Fault_Get(void)
{
    return g_hasFault ? g_latchedFault : g_currentFault;
}

bool Fault_HasFault(void)
{
    return g_hasFault;
}

uint32_t Fault_GetCount(void)
{
    return g_faultCount;
}

void Fault_Clear(void)
{
    g_currentFault = FAULT_NONE;
    g_latchedFault = FAULT_NONE;
    g_hasFault = false;
    /* g_faultCount is intentionally preserved as a cumulative diagnostic. */
}

void Fault_SetPanicHandler(FaultPanicHandler handler)
{
    g_panicHandler = handler;
}

void Fault_Panic(FaultCode code)
{
    Fault_Set(code);

    if (g_panicHandler != nullptr) {
        g_panicHandler(code);
    }

    while (1) {
        /*
         * If a panic handler is registered it owns the observable loop; if it
         * returns, hang deterministically here.
         */
    }
}

} /* namespace services */
