#ifndef APP_FRAM_LAYOUT_H_
#define APP_FRAM_LAYOUT_H_

#include <stdint.h>

namespace app {

/*
 * FM24CL64B 8 KiB layout. The record formats are intentionally incompatible
 * with the former ConfigStore/SeqStore layout.
 */
static const uint16_t FRAM_CONFIG_BANK_SIZE = 0x0400U;
static const uint16_t FRAM_CONFIG_BANK_A_ADDRESS = 0x0000U;
static const uint16_t FRAM_CONFIG_BANK_B_ADDRESS = 0x0400U;

static const uint16_t FRAM_SEQ_HEADER_ADDRESS = 0x0800U;
static const uint16_t FRAM_SEQ_HEADER_SIZE = 8U;
static const uint16_t FRAM_SEQ_SLOT_ADDRESS = 0x0808U;
static const uint16_t FRAM_SEQ_SLOT_SIZE = 766U;
static const uint8_t FRAM_SEQ_SLOT_COUNT = 8U;
static const uint8_t FRAM_SEQ_MAX_INSTRS = 54U;
static const uint8_t FRAM_SEQ_INSTR_SIZE = 14U;

static const uint16_t FRAM_SELF_TEST_ADDRESS = 0x1FF8U;
static const uint16_t FRAM_SIZE_BYTES = 0x2000U;

static_assert(FRAM_CONFIG_BANK_A_ADDRESS + FRAM_CONFIG_BANK_SIZE ==
              FRAM_CONFIG_BANK_B_ADDRESS,
              "ConfigStore banks must be contiguous");
static_assert(FRAM_CONFIG_BANK_B_ADDRESS + FRAM_CONFIG_BANK_SIZE ==
              FRAM_SEQ_HEADER_ADDRESS,
              "ConfigStore must end at the SeqStore header");
static_assert(FRAM_SEQ_HEADER_ADDRESS + FRAM_SEQ_HEADER_SIZE ==
              FRAM_SEQ_SLOT_ADDRESS,
              "SeqStore slots must follow the header");
static_assert(FRAM_SEQ_SLOT_SIZE ==
              1U + 1U + 4U +
              FRAM_SEQ_MAX_INSTRS * FRAM_SEQ_INSTR_SIZE + 4U,
              "SeqStore slot format size mismatch");
static_assert(FRAM_SEQ_SLOT_ADDRESS +
              FRAM_SEQ_SLOT_COUNT * FRAM_SEQ_SLOT_SIZE ==
              FRAM_SELF_TEST_ADDRESS,
              "SeqStore must end at the self-test area");
static_assert(FRAM_SELF_TEST_ADDRESS + 8U == FRAM_SIZE_BYTES,
              "FRAM layout must consume exactly 8 KiB");

} /* namespace app */

#endif /* APP_FRAM_LAYOUT_H_ */
