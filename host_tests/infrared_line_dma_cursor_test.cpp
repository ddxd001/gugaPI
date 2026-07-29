#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "drivers/infrared_line/infrared_line_dma_cursor.h"

int main(void)
{
    drivers::InfraredLineDmaCursor cursor = {};
    assert(drivers::InfraredLineDmaCursor_Publish(&cursor, 15U, 128U));
    for (uint16_t expected = 0U; expected < 15U; expected++) {
        uint16_t index = 0U;
        assert(drivers::InfraredLineDmaCursor_ReadIndex(
            &cursor, 0U, 128U, &index));
        assert(index == expected);
    }

    /* Delayed foreground service across multiple hardware repeats still maps
     * absolute consumer positions to the correct circular byte. */
    assert(drivers::InfraredLineDmaCursor_Publish(&cursor, 140U, 128U));
    uint16_t index = 0U;
    assert(drivers::InfraredLineDmaCursor_ReadIndex(
        &cursor, 0U, 128U, &index));
    assert(index == 15U);

    /* A snapshot from the old side of a wrap must be deferred. */
    assert(!drivers::InfraredLineDmaCursor_Publish(&cursor, 127U, 128U));
    assert(cursor.producer == 140U);

    /* More than one whole buffer of lag is unrecoverable: discard all unread
     * bytes and make the loss visible instead of parsing overwritten memory. */
    assert(!drivers::InfraredLineDmaCursor_Publish(&cursor, 300U, 128U));
    assert(cursor.overwrite_count == 1U);
    assert(cursor.consumer == cursor.producer);
    assert(cursor.dropped_bytes == 284U);

    puts("infrared line dma cursor ok");
    return 0;
}
