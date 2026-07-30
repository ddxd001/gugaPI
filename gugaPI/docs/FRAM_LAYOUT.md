# gugaPI FRAM layout

The FM24CL64B contains 8192 bytes. The current layout is a clean format and
does not migrate any ConfigStore or SeqStore data written by older firmware.

| Address | Size | Owner |
| --- | ---: | --- |
| `0x0000..0x03FF` | 1024 | ConfigStore bank A |
| `0x0400..0x07FF` | 1024 | ConfigStore bank B |
| `0x0800..0x0807` | 8 | SeqStore format header |
| `0x0808..0x0B05` | 766 | Sequence slot 0 |
| `0x0B06..0x0E03` | 766 | Sequence slot 1 |
| `0x0E04..0x1101` | 766 | Sequence slot 2 |
| `0x1102..0x13FF` | 766 | Sequence slot 3 |
| `0x1400..0x16FD` | 766 | Sequence slot 4 |
| `0x16FE..0x19FB` | 766 | Sequence slot 5 |
| `0x19FC..0x1CF9` | 766 | Sequence slot 6 |
| `0x1CFA..0x1FF7` | 766 | Sequence slot 7 |
| `0x1FF8..0x1FFF` | 8 | Non-destructive FRAM self-test |

All boundaries are defined in `app/fram_layout.h` and protected by compile-time
assertions.

## ConfigStore

Each 1 KiB bank contains a 16-byte header, up to 1004 bytes of explicitly
serialized payload, and a CRC32 immediately after the payload. The header
contains the `GCF1` magic, format version, payload length, generation, and a
single-byte commit state.

`param save` always writes the inactive bank, verifies it, and commits the
state byte last. Loading selects the newest valid generation with wrap-safe
comparison. An interrupted save therefore leaves the previous bank usable.

The current payload is 305 bytes and includes all chassis, sensor, infrared,
DM-G6220, and ball-balance parameters.

## SeqStore

The 8-byte header contains the `GSQ1` magic, format version, slot count, maximum
instruction count, and CRC8. Each slot contains:

```text
state(1) + count(1) + generation(4) + instructions(54 * 14) + CRC32(4)
```

A slot save invalidates its state first, writes and verifies the record, and
commits the state byte last. A power loss can invalidate the slot being saved,
but cannot corrupt the other slots or ConfigStore.

When the `GSQ1` header is absent after a successful FRAM read, firmware
invalidates all eight slots and creates the new header. Read errors never
trigger formatting. Use `fram format confirm` in an idle `dev-running` state
to deliberately invalidate both configuration banks and all sequence slots.
