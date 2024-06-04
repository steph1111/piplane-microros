#ifndef PIO_LOADER_H_
#define PIO_LOADER_H_

#include "hardware/pio.h"

const int MAX_PROGRAMS_PER_PIO = 4;

struct PioProgram {
    const pio_program_t* program;
    uint offset;
};

PioProgram pio0_programs[MAX_PROGRAMS_PER_PIO];
int pio0_program_count = 0;

PioProgram pio1_programs[MAX_PROGRAMS_PER_PIO];
int pio1_program_count = 0;

bool pio_loader_add_or_get_offset(PIO pio, const pio_program_t* program, uint* offset);

#endif  // PIO_LOADER_H_