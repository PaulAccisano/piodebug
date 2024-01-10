// RP2040 MicroPython PIO Debugger
// MIT license; Copyright (c) 2024 Paul Accisano

typedef unsigned int uint;
typedef unsigned char byte;

struct pio_state
{
    uint x;
    uint y;
    uint osr;
    uint isr;
    uint fifo[8];
    byte osc;
    byte isc;
    byte tx;
    byte rx;
    byte addr;
    byte irq;
};

void get_state(int pio, int sm, struct pio_state *state);
uint step(int pio, int sm, uint max_steps, uint breakpoints);