#include "algorithm/filter.h"

namespace algorithm {

static float ClampAlpha(float alpha)
{
    if (alpha < 0.0f) {
        return 0.0f;
    }

    if (alpha > 1.0f) {
        return 1.0f;
    }

    return alpha;
}

void LowPassFilter_Init(LowPassFilter *filter, float alpha)
{
    if (filter == 0) {
        return;
    }

    filter->alpha = ClampAlpha(alpha);
    filter->value = 0.0f;
    filter->initialized = false;
}

void LowPassFilter_Reset(LowPassFilter *filter)
{
    if (filter == 0) {
        return;
    }

    filter->value = 0.0f;
    filter->initialized = false;
}

float LowPassFilter_Update(LowPassFilter *filter, float input)
{
    if (filter == 0) {
        return input;
    }

    if (!filter->initialized) {
        filter->value = input;
        filter->initialized = true;
        return filter->value;
    }

    filter->value = (filter->alpha * input) +
                    ((1.0f - filter->alpha) * filter->value);
    return filter->value;
}

float LowPassFilter_Get(const LowPassFilter *filter)
{
    if (filter == 0) {
        return 0.0f;
    }

    return filter->value;
}

} /* namespace algorithm */