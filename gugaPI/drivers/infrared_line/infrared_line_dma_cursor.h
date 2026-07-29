#ifndef DRIVERS_INFRARED_LINE_INFRARED_LINE_DMA_CURSOR_H_
#define DRIVERS_INFRARED_LINE_INFRARED_LINE_DMA_CURSOR_H_

#include <stdbool.h>
#include <stdint.h>

namespace drivers {

struct InfraredLineDmaCursor {
    uint64_t producer;
    uint64_t consumer;
    uint32_t maximum_lag;
    uint32_t overwrite_count;
    uint32_t dropped_bytes;
};

inline uint32_t InfraredLineDmaCursor_Lag(
    const InfraredLineDmaCursor *cursor)
{
    if ((cursor == 0) || (cursor->producer < cursor->consumer)) {
        return 0U;
    }
    const uint64_t lag = cursor->producer - cursor->consumer;
    return (lag > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(lag);
}

/* Publishes one consistent hardware producer snapshot. A regressing snapshot
 * means DMA crossed the repeat boundary before its wrap event was accounted
 * for; defer it rather than exposing old buffer bytes. */
inline bool InfraredLineDmaCursor_Publish(
    InfraredLineDmaCursor *cursor,
    uint64_t producer,
    uint16_t capacity)
{
    if ((cursor == 0) || (capacity == 0U) ||
        (producer < cursor->producer)) {
        return false;
    }
    cursor->producer = producer;
    const uint32_t lag = InfraredLineDmaCursor_Lag(cursor);
    if (lag > cursor->maximum_lag) {
        cursor->maximum_lag = lag;
    }
    if (lag <= capacity) {
        return true;
    }
    if (cursor->overwrite_count != UINT32_MAX) {
        cursor->overwrite_count++;
    }
    const uint64_t dropped = static_cast<uint64_t>(cursor->dropped_bytes) + lag;
    cursor->dropped_bytes = (dropped > UINT32_MAX)
        ? UINT32_MAX : static_cast<uint32_t>(dropped);
    cursor->consumer = cursor->producer;
    return false;
}

inline bool InfraredLineDmaCursor_ReadIndex(
    InfraredLineDmaCursor *cursor,
    uint64_t cycle_base,
    uint16_t capacity,
    uint16_t *index)
{
    if ((cursor == 0) || (index == 0) || (capacity == 0U) ||
        (cursor->consumer >= cursor->producer) ||
        (cursor->consumer < cycle_base)) {
        return false;
    }
    *index = static_cast<uint16_t>(
        (cursor->consumer - cycle_base) % capacity);
    cursor->consumer++;
    return true;
}

} /* namespace drivers */

#endif /* DRIVERS_INFRARED_LINE_INFRARED_LINE_DMA_CURSOR_H_ */
