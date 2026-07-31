#include "drivers/oled/oled_ssd1306.h"

namespace drivers {
namespace {

static const uint8_t kCommandControlByte = 0x00U;
static const uint8_t kDataControlByte = 0x40U;
static const uint16_t kChunkBytes = 32U;
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
           (config->dma_tx != 0) &&
           (config->width == 128U) && (config->height == 32U) &&
           I2cController_IsConfigValid(config->bus) &&
           I2cController_IsAddressValid(config->i2c_address);
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
    return I2cController_Write(config->bus,
                               config->i2c_address,
                               &control,
                               1U,
                               data,
                               length);
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

const uint8_t *FontGlyph(char character)
{
    uint8_t code = static_cast<uint8_t>(character);
    if ((code < 0x20U) || (code > 0x7EU)) {
        code = static_cast<uint8_t>('?');
    }

    return kFont5x7[code - 0x20U];
}

DriverStatus ClearDisplayBlocking(const OledSsd1306Config *config)
{
    uint8_t data[kChunkBytes];
    for (uint16_t i = 0U; i < kChunkBytes; i++) {
        data[i] = 0U;
    }

    const uint8_t window_commands[] = {
        0x20U, 0x00U,
        0x21U, 0x00U, 0x7FU,
        0x22U, 0x00U, 0x03U
    };
    DriverStatus status = WriteCommands(config,
                                         window_commands,
                                         sizeof(window_commands));
    if (status != DRIVER_OK) {
        return status;
    }

    uint16_t remaining =
        static_cast<uint16_t>(config->width * PageCount(config));
    while (remaining > 0U) {
        uint16_t chunk = remaining;
        if (chunk > kChunkBytes) {
            chunk = kChunkBytes;
        }

        status = WriteControlBlock(config, kDataControlByte, data, chunk);
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

void FillBytes(uint8_t *data, uint16_t length, uint8_t value)
{
    for (uint16_t i = 0U; i < length; i++) {
        data[i] = value;
    }
}

void CopyBytes(uint8_t *destination,
               const uint8_t *source,
               uint16_t length)
{
    for (uint16_t i = 0U; i < length; i++) {
        destination[i] = source[i];
    }
}

uint8_t PageMask(uint8_t first_page, uint8_t last_page)
{
    uint8_t mask = 0U;
    for (uint8_t page = first_page; page <= last_page; page++) {
        mask = static_cast<uint8_t>(mask | (1U << page));
    }
    return mask;
}

void MarkDirty(OledSsd1306Context *ctx,
               uint8_t page,
               uint8_t first_column,
               uint8_t last_column)
{
    ctx->dirty_pages = static_cast<uint8_t>(
        ctx->dirty_pages | (1U << page));
    if (first_column < ctx->dirty_first_column) {
        ctx->dirty_first_column = first_column;
    }
    if (last_column > ctx->dirty_last_column) {
        ctx->dirty_last_column = last_column;
    }
}

void MarkChangedPages(OledSsd1306Context *ctx,
                      const uint8_t *buffer,
                      uint16_t length)
{
    const uint16_t width = ctx->config->width;
    const uint8_t pages = PageCount(ctx->config);
    for (uint8_t page = 0U; page < pages; page++) {
        const uint16_t start = static_cast<uint16_t>(page * width);
        bool changed = false;
        uint8_t first_column = ctx->config->width;
        uint8_t last_column = 0U;
        for (uint16_t i = 0U; i < width; i++) {
            if ((start + i < length) &&
                (ctx->framebuffer[start + i] != buffer[start + i])) {
                changed = true;
                if (i < first_column) {
                    first_column = static_cast<uint8_t>(i);
                }
                last_column = static_cast<uint8_t>(i);
            }
        }
        if (changed) {
            MarkDirty(ctx, page, first_column, last_column);
        }
    }
}

void RestoreActivePages(OledSsd1306Context *ctx)
{
    if ((ctx->dirty_pages == 0U) ||
        (ctx->active_first_column < ctx->dirty_first_column)) {
        ctx->dirty_first_column = ctx->active_first_column;
    }
    if ((ctx->dirty_pages == 0U) ||
        (ctx->active_last_column > ctx->dirty_last_column)) {
        ctx->dirty_last_column = ctx->active_last_column;
    }
    ctx->dirty_pages = static_cast<uint8_t>(ctx->dirty_pages |
                                             ctx->active_pages);
    ctx->active_pages = 0U;
    ctx->flush_phase = OLED_FLUSH_IDLE;
}

void PrepareActiveFlush(OledSsd1306Context *ctx)
{
    const uint8_t pages = PageCount(ctx->config);
    uint8_t first_page = 0U;
    while (((ctx->dirty_pages & (1U << first_page)) == 0U) &&
           (first_page + 1U < pages)) {
        first_page++;
    }

    uint8_t last_page = static_cast<uint8_t>(pages - 1U);
    while (((ctx->dirty_pages & (1U << last_page)) == 0U) &&
           (last_page > first_page)) {
        last_page--;
    }

    ctx->active_first_page = first_page;
    ctx->active_last_page = last_page;
    ctx->active_first_column = ctx->dirty_first_column;
    ctx->active_last_column = ctx->dirty_last_column;
    ctx->active_pages = PageMask(first_page, last_page);
    const uint16_t column_count = static_cast<uint16_t>(
        ctx->active_last_column - ctx->active_first_column + 1U);
    ctx->active_data_length = static_cast<uint16_t>(
        (last_page - first_page + 1U) * column_count + 1U);

    ctx->command_buffer[0] = kCommandControlByte;
    ctx->command_buffer[1] = 0x20U;
    ctx->command_buffer[2] = 0x00U;
    ctx->command_buffer[3] = 0x21U;
    ctx->command_buffer[4] = ctx->active_first_column;
    ctx->command_buffer[5] = ctx->active_last_column;
    ctx->command_buffer[6] = 0x22U;
    ctx->command_buffer[7] = first_page;
    ctx->command_buffer[8] = last_page;

    ctx->dma_buffer[0] = kDataControlByte;
    uint16_t destination = 1U;
    for (uint8_t page = first_page; page <= last_page; page++) {
        const uint16_t offset = static_cast<uint16_t>(
            page * ctx->config->width + ctx->active_first_column);
        CopyBytes(&ctx->dma_buffer[destination],
                  &ctx->framebuffer[offset],
                  column_count);
        destination = static_cast<uint16_t>(destination + column_count);
    }
}

DriverStatus StartAsyncBlock(OledSsd1306Context *ctx,
                             const uint8_t *data,
                             uint16_t length)
{
    return I2cController_AsyncWriteStart(ctx->config->bus,
                                         ctx->config->dma_tx,
                                         ctx->config->i2c_address,
                                         data,
                                         length);
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
    ctx->flush_phase = OLED_FLUSH_IDLE;
    ctx->dirty_pages = 0U;
    ctx->dirty_first_column = config->width;
    ctx->dirty_last_column = 0U;
    ctx->active_pages = 0U;
    ctx->active_first_page = 0U;
    ctx->active_last_page = 0U;
    ctx->active_first_column = 0U;
    ctx->active_last_column = 0U;
    ctx->active_data_length = 0U;
    ctx->last_flush_status = DRIVER_OK;
    FillBytes(ctx->framebuffer, OLED_SSD1306_FRAME_BYTES, 0U);

    DriverStatus status = I2cController_Probe(config->bus,
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

    status = ClearDisplayBlocking(config);
    if (status != DRIVER_OK) {
        return status;
    }

    ctx->initialized = true;
    status = OledSsd1306_SetDisplayOn(ctx, true);
    if (status != DRIVER_OK) {
        ctx->initialized = false;
    }
    return status;
}

DriverStatus OledSsd1306_Probe(OledSsd1306Context *ctx)
{
    if ((ctx == 0) || (!IsConfigValid(ctx->config))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    return I2cController_Probe(ctx->config->bus, ctx->config->i2c_address);
}

DriverStatus OledSsd1306_Clear(OledSsd1306Context *ctx)
{
    if (!IsContextReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    return OledSsd1306_Fill(ctx, 0x00U);
}

DriverStatus OledSsd1306_Fill(OledSsd1306Context *ctx, uint8_t pattern)
{
    if (!IsContextReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    const uint8_t pages = PageCount(ctx->config);
    for (uint8_t page = 0U; page < pages; page++) {
        const uint16_t start = static_cast<uint16_t>(
            page * ctx->config->width);
        bool changed = false;
        uint8_t first_column = ctx->config->width;
        uint8_t last_column = 0U;
        for (uint16_t i = 0U; i < ctx->config->width; i++) {
            if (ctx->framebuffer[start + i] != pattern) {
                ctx->framebuffer[start + i] = pattern;
                changed = true;
                if (i < first_column) {
                    first_column = static_cast<uint8_t>(i);
                }
                last_column = static_cast<uint8_t>(i);
            }
        }
        if (changed) {
            MarkDirty(ctx, page, first_column, last_column);
        }
    }
    return DRIVER_OK;
}

DriverStatus OledSsd1306_DrawChecker(OledSsd1306Context *ctx)
{
    if (!IsContextReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    for (uint8_t page = 0U; page < PageCount(ctx->config); page++) {
        bool changed = false;
        uint8_t first_column = ctx->config->width;
        uint8_t last_column = 0U;
        for (uint16_t column = 0U; column < ctx->config->width; column++) {
            const uint8_t value =
                ((((column / 8U) + page) & 1U) != 0U) ? 0xAAU : 0x55U;
            const uint16_t index = static_cast<uint16_t>(
                page * ctx->config->width + column);
            if (ctx->framebuffer[index] != value) {
                ctx->framebuffer[index] = value;
                changed = true;
                if (column < first_column) {
                    first_column = static_cast<uint8_t>(column);
                }
                last_column = static_cast<uint8_t>(column);
            }
        }
        if (changed) {
            MarkDirty(ctx, page, first_column, last_column);
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

    uint16_t framebuffer_col = static_cast<uint16_t>(col * kTextCharWidth);
    uint16_t remaining = static_cast<uint16_t>(
        ctx->config->width - framebuffer_col);
    bool changed = false;
    uint8_t first_changed_column = ctx->config->width;
    uint8_t last_changed_column = 0U;

    while ((*text != '\0') && (remaining >= kTextCharWidth)) {
        const uint8_t *glyph = FontGlyph(*text);
        for (uint8_t i = 0U; i < 5U; i++) {
            const uint16_t index = static_cast<uint16_t>(
                row * ctx->config->width + framebuffer_col);
            if (ctx->framebuffer[index] != glyph[i]) {
                ctx->framebuffer[index] = glyph[i];
                changed = true;
                if (framebuffer_col < first_changed_column) {
                    first_changed_column = static_cast<uint8_t>(
                        framebuffer_col);
                }
                last_changed_column = static_cast<uint8_t>(framebuffer_col);
            }
            framebuffer_col++;
        }
        const uint16_t spacer_index = static_cast<uint16_t>(
            row * ctx->config->width + framebuffer_col);
        if (ctx->framebuffer[spacer_index] != 0U) {
            ctx->framebuffer[spacer_index] = 0U;
            changed = true;
            if (framebuffer_col < first_changed_column) {
                first_changed_column = static_cast<uint8_t>(framebuffer_col);
            }
            last_changed_column = static_cast<uint8_t>(framebuffer_col);
        }
        framebuffer_col++;

        text++;
        remaining = static_cast<uint16_t>(remaining - kTextCharWidth);
    }
    if (changed) {
        MarkDirty(ctx, row, first_changed_column, last_changed_column);
    }
    return DRIVER_OK;
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

    MarkChangedPages(ctx, buffer, length);
    CopyBytes(ctx->framebuffer, buffer, length);
    return DRIVER_OK;
}

DriverStatus OledSsd1306_Service(OledSsd1306Context *ctx)
{
    if (!IsContextReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    if (ctx->flush_phase == OLED_FLUSH_IDLE) {
        if (ctx->dirty_pages == 0U) {
            return DRIVER_OK;
        }

        PrepareActiveFlush(ctx);
        const DriverStatus status = StartAsyncBlock(
            ctx,
            ctx->command_buffer,
            OLED_SSD1306_WINDOW_COMMAND_BYTES);
        if (status != DRIVER_OK) {
            ctx->active_pages = 0U;
            ctx->last_flush_status = status;
            return status;
        }

        ctx->dirty_pages = static_cast<uint8_t>(ctx->dirty_pages &
                                                 ~ctx->active_pages);
        ctx->dirty_first_column = ctx->config->width;
        ctx->dirty_last_column = 0U;
        ctx->flush_phase = OLED_FLUSH_WINDOW;
        ctx->last_flush_status = DRIVER_ERROR_BUSY;
        return DRIVER_ERROR_BUSY;
    }

    DriverStatus status = I2cController_AsyncWritePoll(
        ctx->config->bus,
        ctx->config->dma_tx);
    if (status == DRIVER_ERROR_BUSY) {
        return status;
    }
    if (status != DRIVER_OK) {
        RestoreActivePages(ctx);
        ctx->last_flush_status = status;
        return status;
    }

    if (ctx->flush_phase == OLED_FLUSH_WINDOW) {
        status = StartAsyncBlock(ctx,
                                 ctx->dma_buffer,
                                 ctx->active_data_length);
        if (status != DRIVER_OK) {
            RestoreActivePages(ctx);
            ctx->last_flush_status = status;
            return status;
        }
        ctx->flush_phase = OLED_FLUSH_DATA;
        return DRIVER_ERROR_BUSY;
    }

    ctx->active_pages = 0U;
    ctx->flush_phase = OLED_FLUSH_IDLE;
    ctx->last_flush_status = DRIVER_OK;
    return DRIVER_OK;
}

void OledSsd1306_HandleI2cInterrupt(OledSsd1306Context *ctx)
{
    if (IsContextReady(ctx)) {
        I2cController_AsyncWriteHandleInterrupt(ctx->config->bus);
    }
}

void OledSsd1306_HandleDmaFault(OledSsd1306Context *ctx)
{
    if (IsContextReady(ctx)) {
        I2cController_AsyncWriteHandleDmaFault(ctx->config->bus,
                                               ctx->config->dma_tx);
    }
}

bool OledSsd1306_HasPendingFlush(const OledSsd1306Context *ctx)
{
    return IsContextReady(ctx) &&
        ((ctx->dirty_pages != 0U) ||
         (ctx->flush_phase != OLED_FLUSH_IDLE));
}

bool OledSsd1306_IsReady(const OledSsd1306Context *ctx)
{
    return IsContextReady(ctx);
}

} /* namespace drivers */
