#ifndef DSHOT_H_
#define DSHOT_H_

#include <stdint.h>
#include <stdbool.h>

// TODO: Look at which of these are actually important
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"

#include "dshot_encoder.pio.h"
#include "pio_loader.h"


// Very much stolen directly from:
// https://github.com/cadouthat/pico-dshot/blob/main/src/dshot_encoder.cpp

const uint16_t MIN_THROTTLE_COMMAND = 48;
const uint16_t MAX_THROTTLE_COMMAND = 2047;

struct DShotEncoder {
    uint dshot_gpio;
    PIO pio;
    uint pio_offset;
    int pio_sm = -1;
}

/**
 * Initialize a DShotEncoder "object"
 * 
 * @param this Pointer to DShotEncoder
 * @param dshot_gpio GPIO pin
 * @param pio Pio instance to be used
*/
void init(struct DShotEncoder *this, uint dshot_gpio, PIO pio);
/**
 * Setup dshot, but do not output data yet.
 * 
 * @param this Pointer to DShotEncoder
 * @param enable_repeat If true, the PIO will continuously output the last provided value at 8000 frames per second.
*/
bool setup_dshot (struct DShotEncoder *this, bool enable_repeat);

/**
 * Generates a frame given a throttle and telemetry value and send command
 * 
 * Frames are organized in the following 16 bit pattern: SSSSSSSSSSSTCCCC
 *  (S) 11 bit throttle
 *  (T) 1 bit telemetry request
 *  (C) 4 bit Cyclic Redundancy Check (CRC) (calculated in this function)
 * 
 * @param this Pointer to DShotEncoder
 * @param throttle Throttle value (11 bits) 
 * @param telemetry Telemetry value (1 bit), true (1) if telemetry should be used, false (0) if not. Defaults to false
*/
void send_cmd(struct DShotEncoder *this, uint16_t throttle, bool telemetry=false);

/**
 * Stop generating output (until next send command)
 * 
 * @param this Pointer to DShotEncoder
*/
void stop(struct DShotEncoder *this);

#endif  // DSHOT_H_