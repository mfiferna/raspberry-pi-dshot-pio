// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ----- //
// dshot //
// ----- //

#define dshot_wrap_target 0
#define dshot_wrap 3
#define dshot_pio_version 0

static const uint16_t dshot_program_instructions[] = {
            //     .wrap_target
    0x6028, //  0: out    x, 8                       
    0xa20b, //  1: mov    pins, !null            [2] 
    0xa201, //  2: mov    pins, x                [2] 
    0xa003, //  3: mov    pins, null                 
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program dshot_program = {
    .instructions = dshot_program_instructions,
    .length = 4,
    .origin = -1,
    .pio_version = dshot_pio_version,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = 0x0
#endif
};

static inline pio_sm_config dshot_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + dshot_wrap_target, offset + dshot_wrap);
    return c;
}
#endif

