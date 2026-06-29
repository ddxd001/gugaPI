#ifndef ALGORITHM_FILTER_H_
#define ALGORITHM_FILTER_H_

#include <stdbool.h>

namespace algorithm {

struct LowPassFilter {
    float alpha;
    float value;
    bool initialized;
};

void LowPassFilter_Init(LowPassFilter *filter, float alpha);
void LowPassFilter_Reset(LowPassFilter *filter);
float LowPassFilter_Update(LowPassFilter *filter, float input);
float LowPassFilter_Get(const LowPassFilter *filter);

} /* namespace algorithm */

#endif /* ALGORITHM_FILTER_H_ */