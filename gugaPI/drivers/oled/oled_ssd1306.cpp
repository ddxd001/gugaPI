#include "drivers/oled/oled_ssd1306.h"

namespace drivers {
namespace {

static const uint8_t kCommandControlByte = 0x00U;
static const uint8_t kDataControlByte = 0x40U;
static const uint16_t kChunkBytes = I2C_DIAG_MAX_BLOCK_WRITE_BYTES;
static const uint8_t kTextCharWidth = 6U;

static const uint8_t kFont5x7[96][5] = {
    { 0x00U, 0x00U, 0x00U, 0x00U, 0x00U }, /*   */
    { 0x00U, 0x00U, 0x5FU, 0x00U, 0x00U }, /* ! */
    { 0x00U, 0x07U, 0x00U, 0x07U, 0x00U }, /* " */
    { 0x14U, 0x7FU, 0x14U, 0x7FU, 0x14U }, /* # */
    { 0x24U, 0x2AU, 0x7FU, 0x2AU, 0x12U }, /* $ */
    { 0x23U, 0x13U, 0x08U, 0x64U, 0x62U }, /* % */
    { 0x36U, 0x49U, 0x55U, 0x22U, 0x50U }, /* & */
    { 0x00U, 0x05U, 0x03U, 0x00U, 0x00U }, /* ' */
    { 0x00U, 0x1CU, 0x22U, 0x41U, 0x00U }, /* ( */
    { 0x00U, 0x41U, 0x22U, 0x1CU, 0x00U }, /* ) */
    { 0x14U, 0x08U, 0x3EU, 0x08U, 0x14U }, /* * */
    { 0x08U, 0x08U, 0x3EU, 0x08U, 0x08U }, /* + */
    { 0x00U, 0x50U, 0x30U, 0x00U, 0x00U }, /* , */
    { 0x08U, 0x08U, 0x08U, 0x08U, 0x08U }, /* - */
    { 0x00U, 0x60U, 0x60U, 0x00U, 0x00U }, /* . */
    { 0x20U, 0x10U, 0x08U, 0x04U, 0x02U }, /* / */
    { 0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU }, /* 0 */
    { 0x00U, 0x42U, 0x7FU, 0x40U, 0x00U }, /* 1 */
    { 0x42U, 0x61U, 0x51U, 0x49U, 0x46U }, /* 2 */
    { 0x21U, 0x41U, 0x45U, 0x4BU, 0x31U }, /* 3 */
    { 0x18U, 0x14U, 0x12U, 0x7FU, 0x10U }, /* 4 */
    { 0x27U, 0x45U, 0x45U, 0x45U, 0x39U }, /* 5 */
    { 0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U }, /* 6 */
    { 0x01U, 0x71U, 0x09U, 0x05U, 0x03U }, /* 7 */
    { 0x36U, 0x49U, 0x49U, 0x49U, 0x36U }, /* 8 */
    { 0x06U, 0x49U, 0x49U, 0x29U, 0x1EU }, /* 9 */
    { 0x00U, 0x36U, 0x36U, 0x00U, 0x00U }, /* : */
    { 0x00U, 0x56U, 0x36U, 0x00U, 0x00U }, /* ; */
    { 0x08U, 0x14U, 0x22U, 0x41U, 0x00U }, /* < */
    { 0x14U, 0x14U, 0x14U, 0x14U, 0x14U }, /* = */
    { 0x00U, 0x41U, 0x22U, 0x14U, 0x08U }, /* > */
    { 0x02U, 0x01U, 0x51U, 0x09U, 0x06U }, /* ? */
    { 0x32U, 0x49U, 0x79U, 0x41U, 0x3EU }, /* @ */
    { 0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU }, /* A */
    { 0x7FU, 0x49U, 0x49U, 0x49U, 0x36U }, /* B */
    { 0x3EU, 0x41U, 0x41U, 0x41U, 0x22U }, /* C */
    { 0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU }, /* D */
    { 0x7FU, 0x49U, 0x49U, 0x49U, 0x41U }, /* E */
    { 0x7FU, 0x09U, 0x09U, 0x09U, 0x01U }, /* F */
    { 0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU }, /* G */
    { 0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU }, /* H */
    { 0x00U, 0x41U, 0x7FU, 0x41U, 0x00U }, /* I */
    { 0x20U, 0x40U, 0x41U, 0x3FU, 0x01U }, /* J */
    { 0x7FU, 0x08U, 0x14U, 0x22U, 0x41U }, /* K */
    { 0x7FU, 0x40U, 0x40U, 0x40U, 0x40U }, /* L */
    { 0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU }, /* M */
    { 0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU }, /* N */
    { 0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU }, /* O */
    { 0x7FU, 0x09U, 0x09U, 0x09U, 0x06U }, /* P */
    { 0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU }, /* Q */
    { 0x7FU, 0x09U, 0x19U, 0x29U, 0x46U }, /* R */
    { 0x46U, 0x49U, 0x49U, 0x49U, 0x31U }, /* S */
    { 0x01U, 0x01U, 0x7FU, 0x01U, 0x01U }, /* T */
    { 0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU }, /* U */
    { 0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU }, /* V */
    { 0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU }, /* W */
    { 0x63U, 0x14U, 0x08U, 0x14U, 0x63U }, /* X */
    { 0x07U, 0x08U, 0x70U, 0x08U, 0x07U }, /* Y */
    { 0x61U, 0x51U, 0x49U, 0x45U, 0x43U }, /* Z */
    { 0x00U, 0x7FU, 0x41U, 0x41U, 0x00U }, /* [ */
    { 0x02U, 0x04U, 0x08U, 0x10U, 0x20U }, /* \ */
    { 0x00U, 0x41U, 0x41U, 0x7FU, 0x00U }, /* ] */
    { 0x04U, 0x02U, 0x01U, 0x02U, 0x04U }, /* ^ */
    { 0x40U, 0x40U, 0x40U, 0x40U, 0x40U }, /* _ */
    { 0x00U, 0x01U, 0x02U, 0x04U, 0x00U }, /* ` */
    { 0x20U, 0x54U, 0x54U, 0x54U, 0x78U }, /* a */
    { 0x7FU, 0x48U, 0x44U, 0x44U, 0x38U }, /* b */
    { 0x38U, 0x44U, 0x44U, 0x44U, 0x20U }, /* c */
    { 0x38U, 0x44U, 0x44U, 0x48U, 0x7FU }, /* d */
    { 0x38U, 0x54U, 0x54U, 0x54U, 0x18U }, /* e */
    { 0x08U, 0x7EU, 0x09U, 0x01U, 0x02U }, /* f */
    { 0x0CU, 0x52U, 0x52U, 0x52U, 0x3EU }, /* g */
    { 0x7FU, 0x08U, 0x04U, 0x04U, 0x78U }, /* h */
    { 0x00U, 0x44U, 0x7DU, 0x40U, 0x00U }, /* i */
    { 0x20U, 0x40U, 0x44U, 0x3DU, 0x00U }, /* j */
    { 0x7FU, 0x10U, 0x28U, 0x44U, 0x00U }, /* k */
    { 0x00U, 0x41U, 0x7FU, 0x40U, 0x00U }, /* l */
    { 0x7CU, 0x04U, 0x18U, 0x04U, 0x78U }, /* m */
    { 0x7CU, 0x08U, 0x04U, 0x04U, 0x78U }, /* n */
    { 0x38U, 0x44U, 0x44U, 0x44U, 0x38U }, /* o */
    { 0x7CU, 0x14U, 0x14U, 0x14U, 0x08U }, /* p */
    { 0x08U, 0x14U, 0x14U, 0x18U, 0x7CU }, /* q */
    { 0x7CU, 0x08U, 0x04U, 0x04U, 0x08U }, /* r */
    { 0x48U, 0x54U, 0x54U, 0x54U, 0x20U }, /* s */
    { 0x04U, 0x3FU, 0x44U, 0x40U, 0x20U }, /* t */
    { 0x3CU, 0x40U, 0x40U, 0x20U, 0x7CU }, /* u */
    { 0x1CU, 0x20U, 0x40U, 0x20U, 0x1CU }, /* v */
    { 0x3CU, 0x40U, 0x30U, 0x40U, 0x3CU }, /* w */
    { 0x44U, 0x28U, 0x10U, 0x28U, 0x44U }, /* x */
    { 0x0CU, 0x50U, 0x50U, 0x50U, 0x3CU }, /* y */
    { 0x44U, 0x64U, 0x54U, 0x4CU, 0x44U }, /* z */
    { 0x00U, 0x08U, 0x36U, 0x41U, 0x00U }, /* { */
    { 0x00U, 0x00U, 0x7FU, 0x00U, 0x00U }, /* | */
    { 0x00U, 0x41U, 0x36U, 0x08U, 0x00U }, /* } */
    { 0x10U, 0x08U, 0x08U, 0x10U, 0x08U }, /* ~ */
    { 0x00U, 0x00U, 0x00U, 0x00U, 0x00U }
};

bool IsConfigValid(const OledSsd1306Config *config)
{
    return (config != 0) && (config->bus != 0) &&
           (config->width == 128U) && (config->height == 32U) &&
           (config->i2c_address >= I2C_DIAG_MIN_7BIT_ADDRESS) &&
           (config->i2c_address <= I2C_DIAG_MAX_7BIT_ADDRESS);
}

uint8_t PageCount(const OledSsd1306Config *config)
{
    return static_cast<uint8_t>(config->height / 8U);
}

DriverStatus WriteControlBlock(const OledSsd1306Config *config,
                               uint8_t control,
                               const uint8_t *data,
                               uint16_t length)
{
    return I2cDiag_WriteReg8Block(config->bus,
                                  config->i2c_address,
                                  control,
                                  data,
                                  length);
}

DriverStatus FlushData(const OledSsd1306Config *config,
                       const uint8_t *data,
                       uint16_t length)
{
    if (length == 0U) {
        return DRIVER_OK;
    }

    return WriteControlBlock(config, kDataControlByte, data, length);
}

DriverStatus WriteCommands(const OledSsd1306Config *config,
                           const uint8_t *commands,
                           uint16_t length)
{
    if ((commands == 0) || (length == 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    uint16_t offset = 0U;
    while (offset < length) {
        uint16_t chunk = static_cast<uint16_t>(length - offset);
        if (chunk > kChunkBytes) {
            chunk = kChunkBytes;
        }

        const DriverStatus status = WriteControlBlock(config,
                                                      kCommandControlByte,
                                                      &commands[offset],
                                                      chunk);
        if (status != DRIVER_OK) {
            return status;
        }
        offset = static_cast<uint16_t>(offset + chunk);
    }

    return DRIVER_OK;
}

DriverStatus SetWindowRange(const OledSsd1306Config *config,
                            uint8_t start_col,
                            uint8_t end_col,
                            uint8_t start_page,
                            uint8_t end_page)
{
    const uint8_t commands[] = {
        0x20U, 0x00U,
        0x21U, start_col, end_col,
        0x22U, start_page, end_page
    };

    return WriteCommands(config, commands, sizeof(commands));
}

DriverStatus SetWindow(const OledSsd1306Config *config)
{
    return SetWindowRange(config,
                          0U,
                          static_cast<uint8_t>(config->width - 1U),
                          0U,
                          static_cast<uint8_t>(PageCount(config) - 1U));
}

const uint8_t *FontGlyph(char character)
{
    uint8_t code = static_cast<uint8_t>(character);
    if ((code < 0x20U) || (code > 0x7EU)) {
        code = static_cast<uint8_t>('?');
    }

    return kFont5x7[code - 0x20U];
}

DriverStatus WriteRepeated(OledSsd1306Context *ctx, uint8_t pattern)
{
    uint8_t data[kChunkBytes];
    for (uint16_t i = 0U; i < kChunkBytes; i++) {
        data[i] = pattern;
    }

    DriverStatus status = SetWindow(ctx->config);
    if (status != DRIVER_OK) {
        return status;
    }

    uint16_t remaining =
        static_cast<uint16_t>(ctx->config->width * PageCount(ctx->config));
    while (remaining > 0U) {
        uint16_t chunk = remaining;
        if (chunk > kChunkBytes) {
            chunk = kChunkBytes;
        }

        status = WriteControlBlock(ctx->config, kDataControlByte, data, chunk);
        if (status != DRIVER_OK) {
            return status;
        }

        remaining = static_cast<uint16_t>(remaining - chunk);
    }

    return DRIVER_OK;
}

bool IsContextReady(const OledSsd1306Context *ctx)
{
    return (ctx != 0) && (ctx->initialized) && IsConfigValid(ctx->config);
}

} /* namespace */

DriverStatus OledSsd1306_Init(OledSsd1306Context *ctx,
                              const OledSsd1306Config *config)
{
    if ((ctx == 0) || (!IsConfigValid(config))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->initialized = false;

    DriverStatus status = I2cDiag_ProbeAddress(config->bus,
                                               config->i2c_address);
    if (status != DRIVER_OK) {
        return status;
    }

    const uint8_t init_commands[] = {
        0xAEU,
        0xD5U, 0x80U,
        0xA8U, 0x1FU,
        0xD3U, 0x00U,
        0x40U,
        0x8DU, 0x14U,
        0x20U, 0x00U,
        0xA1U,
        0xC8U,
        0xDAU, 0x02U,
        0x81U, 0x8FU,
        0xD9U, 0xF1U,
        0xDBU, 0x40U,
        0xA4U,
        0xA6U
    };

    status = WriteCommands(config, init_commands, sizeof(init_commands));
    if (status != DRIVER_OK) {
        return status;
    }

    ctx->initialized = true;

    status = OledSsd1306_Clear(ctx);
    if (status != DRIVER_OK) {
        ctx->initialized = false;
        return status;
    }

    return OledSsd1306_SetDisplayOn(ctx, true);
}

DriverStatus OledSsd1306_Probe(OledSsd1306Context *ctx)
{
    if ((ctx == 0) || (!IsConfigValid(ctx->config))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    return I2cDiag_ProbeAddress(ctx->config->bus, ctx->config->i2c_address);
}

DriverStatus OledSsd1306_Clear(OledSsd1306Context *ctx)
{
    if (!IsContextReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    return WriteRepeated(ctx, 0x00U);
}

DriverStatus OledSsd1306_Fill(OledSsd1306Context *ctx, uint8_t pattern)
{
    if (!IsContextReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    return WriteRepeated(ctx, pattern);
}

DriverStatus OledSsd1306_DrawChecker(OledSsd1306Context *ctx)
{
    if (!IsContextReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    DriverStatus status = SetWindow(ctx->config);
    if (status != DRIVER_OK) {
        return status;
    }

    uint8_t data[kChunkBytes];
    for (uint8_t page = 0U; page < PageCount(ctx->config); page++) {
        uint16_t written = 0U;
        while (written < ctx->config->width) {
            uint16_t chunk = static_cast<uint16_t>(ctx->config->width - written);
            if (chunk > kChunkBytes) {
                chunk = kChunkBytes;
            }

            for (uint16_t i = 0U; i < chunk; i++) {
                const uint16_t column = static_cast<uint16_t>(written + i);
                data[i] = (((column / 8U) + page) & 1U) ? 0xAAU : 0x55U;
            }

            status = WriteControlBlock(ctx->config,
                                       kDataControlByte,
                                       data,
                                       chunk);
            if (status != DRIVER_OK) {
                return status;
            }
            written = static_cast<uint16_t>(written + chunk);
        }
    }

    return DRIVER_OK;
}

DriverStatus OledSsd1306_SetDisplayOn(OledSsd1306Context *ctx, bool on)
{
    if (!IsContextReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    const uint8_t command = on ? 0xAFU : 0xAEU;
    return WriteCommands(ctx->config, &command, 1U);
}

DriverStatus OledSsd1306_SetInvert(OledSsd1306Context *ctx, bool invert)
{
    if (!IsContextReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    const uint8_t command = invert ? 0xA7U : 0xA6U;
    return WriteCommands(ctx->config, &command, 1U);
}

DriverStatus OledSsd1306_DrawString(OledSsd1306Context *ctx,
                                    uint8_t row,
                                    uint8_t col,
                                    const char *text)
{
    if (!IsContextReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (text == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const uint8_t page_count = PageCount(ctx->config);
    const uint8_t max_cols =
        static_cast<uint8_t>(ctx->config->width / kTextCharWidth);
    if ((row >= page_count) || (col >= max_cols)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const uint8_t start_col =
        static_cast<uint8_t>(col * kTextCharWidth);
    DriverStatus status = SetWindowRange(
        ctx->config,
        start_col,
        static_cast<uint8_t>(ctx->config->width - 1U),
        row,
        row);
    if (status != DRIVER_OK) {
        return status;
    }

    uint8_t data[kChunkBytes];
    uint16_t used = 0U;
    uint16_t remaining =
        static_cast<uint16_t>(ctx->config->width - start_col);

    while ((*text != '\0') && (remaining >= kTextCharWidth)) {
        const uint8_t *glyph = FontGlyph(*text);
        for (uint8_t i = 0U; i < 5U; i++) {
            data[used] = glyph[i];
            used++;
            if (used == kChunkBytes) {
                status = FlushData(ctx->config, data, used);
                if (status != DRIVER_OK) {
                    return status;
                }
                used = 0U;
            }
        }

        data[used] = 0x00U;
        used++;
        if (used == kChunkBytes) {
            status = FlushData(ctx->config, data, used);
            if (status != DRIVER_OK) {
                return status;
            }
            used = 0U;
        }

        text++;
        remaining = static_cast<uint16_t>(remaining - kTextCharWidth);
    }

    return FlushData(ctx->config, data, used);
}

DriverStatus OledSsd1306_WriteBuffer(OledSsd1306Context *ctx,
                                     const uint8_t *buffer,
                                     uint16_t length)
{
    if ((!IsContextReady(ctx)) || (buffer == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const uint16_t expected =
        static_cast<uint16_t>(ctx->config->width * PageCount(ctx->config));
    if (length != expected) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = SetWindow(ctx->config);
    if (status != DRIVER_OK) {
        return status;
    }

    uint16_t offset = 0U;
    while (offset < length) {
        uint16_t chunk = static_cast<uint16_t>(length - offset);
        if (chunk > kChunkBytes) {
            chunk = kChunkBytes;
        }

        status = WriteControlBlock(ctx->config,
                                   kDataControlByte,
                                   &buffer[offset],
                                   chunk);
        if (status != DRIVER_OK) {
            return status;
        }
        offset = static_cast<uint16_t>(offset + chunk);
    }

    return DRIVER_OK;
}

bool OledSsd1306_IsReady(const OledSsd1306Context *ctx)
{
    return IsContextReady(ctx);
}

} /* namespace drivers */
