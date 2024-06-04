#include "dshot.h"

/**
 * Initialize a DShotEncoder "object"
 * 
 * @param this Pointer to DShotEncoder
 * @param dshot_gpio GPIO pin
 * @param pio Pio instance to be used
*/
void init(struct DShotEncoder *this, uint dshot_gpio, PIO pio) {
    this->dshot_gpio = dshot_gpio;
    this->pio = pio;
}

/**
 * Setup dshot, but do not output data yet.
 * 
 * @param this Pointer to DShotEncoder
 * @param enable_repeat If true, the PIO will continuously output the last provided value at 8000 frames per second.
*/
bool setup_dshot (struct DShotEncoder *this, bool enable_repeat) {
    this->pio_sm = pio_claim_unused_sm(this->pio, /*required=*/false);
    
    if (this->pio_sm < 0) {
        return false;
    }

    // TODO: Find out what this is???
    pio_program_t* dshot_encoder_program; 

    if (!pio_loader_add_or_get_offset(this->pio, &dshot_encoder_program, &this->pio_offset)) {
        pio_sm_unclaim(this->pio, this->pio_sm);
        this->pio_sm = -1;
        return false;
    }

    dshot_encoder_program_init(this->pio, this->pio_sm, this->pio_offset, this->dshot_gpio, enable_repeat);

    // Look at enabling 3d mode (for bidirectional). 3d mode packet must be sent 6 times
    // Also have to save settings after changing anything. Must be sent 6 times as well
    // See: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/ 

    return true;
}

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
void send_cmd(struct DShotEncoder *this, uint16_t throttle, bool telemetry=false) {
    // Shift everything over by one place then append telem
    uint16_t data = (throttle << 1) | telemetry;

    // CRC calculation
    uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;

    // Shift and include checksum
    uint16_t c = (data << 4) | crc;

    pio_sm_put_blocking(this->pio, this->pio_sm, c);
}

/**
 * Stop generating output (until next send command)
 * 
 * @param this Pointer to DShotEncoder
*/
void stop(struct DShotEncoder *this) {
    pio_sm_put_blocking(this->pio, this->pio_sm, 0);
} 