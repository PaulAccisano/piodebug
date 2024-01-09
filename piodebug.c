// RP2040 MicroPython PIO Debugger
// MIT license; Copyright (c) 2024 Paul Accisano

#include "piodebug.h"
#include "lib/pico-sdk/src/rp2040/hardware_regs/include/hardware/regs/pio.h"
#include "lib/pico-sdk/src/rp2040/hardware_regs/include/hardware/regs/dma.h"
#include "lib/pico-sdk/src/rp2040/hardware_regs/include/hardware/regs/dreq.h"

#define _u(x) ((uint)(x))
#define DMA_BASE _u(0x50000000)
#define PIO0_BASE _u(0x50200000)
#define PIO1_BASE _u(0x50300000)
#define NUM_DMA_CHANNELS _u(12)
#define REG(reg) (*(volatile uint*)(reg))

void *memset(void *blk, int c, int n) {    
    byte *dst = blk;
    while (n-- > 0)
    {
        *dst++ = (byte)c;
    }

    return blk;
}

const uint asm_in_isr           = 0b0100000011000000;
const uint asm_in_null          = 0b0100000001100000;
const uint asm_in_osr           = 0b0100000011100000;
const uint asm_in_x             = 0b0100000000100000;
const uint asm_in_y             = 0b0100000001000000;
const uint asm_jmp              = 0b0000000000000000;
const uint asm_jmp_not_osre     = 0b0000000011100000;
const uint asm_mov_isr_isr      = 0b1010000011000110;
const uint asm_mov_osr_inv_isr  = 0b1010000011101110;
const uint asm_mov_osr_isr      = 0b1010000011100110;
const uint asm_mov_osr_x        = 0b1010000011100001;
const uint asm_mov_osr_inv_osr  = 0b1010000011101111;
const uint asm_mov_x_osr        = 0b1010000000100111;
const uint asm_out_null         = 0b0110000001100000;
const uint asm_out_pc           = 0b0110000010100000;
const uint asm_pull             = 0b1000000010100000;
const uint asm_push_iffull      = 0b1000000001100000;
const uint asm_set_x            = 0b1110000000100000;
const uint asm_wait_0_irq       = 0b0010000001000000;

__attribute__((always_inline)) static inline
void sm_exec(uint SMx, uint instr)
{
    REG(SMx + PIO_SM0_INSTR_OFFSET) = instr;
    // Wait at least 2 cycles for instruction to execute
    __asm("nop");
    __asm("nop");
}

void get_state_destructive(uint PIOx, uint sm, struct pio_state *state, uint *original_shiftctrl, uint *original_side_en);
void restore_state(uint PIOx, uint sm, struct pio_state const *state, uint original_shiftctrl, uint original_side_en);

// Gets the state of a PIO State Machine
void get_state(int pio, int sm, struct pio_state *state)
{
    // Base addresses for the PIO
    const uint PIOx = pio ? PIO1_BASE : PIO0_BASE;

    // Pause the state machine if it's active
    uint was_active = REG(PIOx + PIO_CTRL_OFFSET) & (1 << (PIO_CTRL_SM_ENABLE_LSB + sm));
    REG(PIOx + PIO_CTRL_OFFSET) &= ~(1 << (PIO_CTRL_SM_ENABLE_LSB + sm));

    // Find any active DMA channels with this state machine as their DREQ and pause them
    uint paused_dma_channels = 0;
    uint tx_dreq = ((pio ? DREQ_PIO1_TX0 : DREQ_PIO0_TX0) + sm) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB;
    uint rx_dreq = ((pio ? DREQ_PIO1_RX0 : DREQ_PIO0_RX0) + sm) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB;
    for(int i = 0; i < NUM_DMA_CHANNELS; i++)
    {
        uint ctrl = DMA_BASE + DMA_CH0_AL1_CTRL_OFFSET + i * (DMA_CH1_AL1_CTRL_OFFSET - DMA_CH0_AL1_CTRL_OFFSET);
        uint dreq = REG(ctrl) & DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS;
        if (dreq == tx_dreq || dreq == rx_dreq)
        {
            if ((REG(ctrl) & DMA_CH0_CTRL_TRIG_BUSY_BITS) && (REG(ctrl) & DMA_CH0_CTRL_TRIG_EN_BITS))
            {
                REG(ctrl) &= ~DMA_CH0_CTRL_TRIG_EN_BITS;
                paused_dma_channels |= (1 << i);
            }
        }
    }

    // Get the state destructively
    uint original_shiftctrl, original_side_en;
    get_state_destructive(PIOx, sm, state, &original_shiftctrl, &original_side_en);

    // Restore the original state
    restore_state(PIOx, sm, state, original_shiftctrl, original_side_en);

    // Restart any DMA channels we paused
    if (paused_dma_channels)
    {
        for(int i = 0; i < NUM_DMA_CHANNELS; i++)
        {
            if (paused_dma_channels & (1 << i))
            {
                const uint ctrl = DMA_BASE + DMA_CH0_AL1_CTRL_OFFSET + i * (DMA_CH1_AL1_CTRL_OFFSET - DMA_CH0_AL1_CTRL_OFFSET);
                REG(ctrl) |= DMA_CH0_CTRL_TRIG_EN_BITS;
            }
        }
    }

    // Unpause the state machine
    REG(PIOx + PIO_CTRL_OFFSET) |= was_active;

    return;
}

void get_state_destructive(uint PIOx, uint sm, struct pio_state *state, uint *original_shiftctrl, uint *original_side_en)
{
    // Base addresses for the SM block
    const uint SMx = PIOx + (PIO_SM1_CLKDIV_OFFSET - PIO_SM0_CLKDIV_OFFSET) * sm;
    
    // Back up shiftctrl and side_en settings
    *original_shiftctrl = REG(SMx + PIO_SM0_SHIFTCTRL_OFFSET);
    *original_side_en = REG(SMx + PIO_SM0_EXECCTRL_OFFSET) & PIO_SM0_EXECCTRL_SIDE_EN_BITS;

    // Set SHIFTCTRL settings for easy readout:
    // PULL_THRESH = 0, PUSH_THRESH = 0, OUT_SHIFTDIR = 0 (left), IN_SHIFTDIR = 1 (right), AUTOPULL = 0, AUTOPUSH = 1
    // DO NOT modify FJOIN yet, as it will clear the FIFO!
    REG(SMx + PIO_SM0_SHIFTCTRL_OFFSET) = 
          (0 << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB)
        | (0 << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB)
        | (0 << PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_LSB)
        | (1 << PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_LSB)
        | (0 << PIO_SM0_SHIFTCTRL_AUTOPULL_LSB)
        | (1 << PIO_SM0_SHIFTCTRL_AUTOPUSH_LSB)
        | (*original_shiftctrl & (PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS | PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS));
    
    // Set side_en, which will prevent executed instructions from sidesetting
    REG(SMx + PIO_SM0_EXECCTRL_OFFSET) |= PIO_SM0_EXECCTRL_SIDE_EN_BITS;

    // Store current address and jump to address 0
    state->addr = REG(SMx + PIO_SM0_ADDR_OFFSET);
    sm_exec(SMx, asm_jmp);

    // Since we may or may not have a tx fifo, read out osr through the ADDR register.
    // We can determine osc in the process.
    for (int i = 32; i > 0;)
    {
        // Check if the output shift counter is saturated
        if (!state->osc)
        {
            sm_exec(SMx, asm_jmp_not_osre | 2);
            if (REG(SMx + PIO_SM0_ADDR_OFFSET) != 2)
            {
                state->osc = i;
            }
        }

        // Read one bit at a time until we've deteremined the output shift counter,
        // then up to 5 bits at a time until 32 bits have been read
        int chunk = !state->osc ? 1 : i < 5 ? i : 5;
        sm_exec(SMx, asm_out_pc | chunk);
        state->osr = (state->osr << chunk) | REG(SMx + PIO_SM0_ADDR_OFFSET);
        i -= chunk;
    }

    // Read out the tx fifo the same way.
    while(!(REG(PIOx + PIO_FSTAT_OFFSET) & (1 << (PIO_FSTAT_TXEMPTY_LSB + sm))))
    {
        sm_exec(SMx, asm_pull);
        for (int i = 32; i > 0; i -= 5)
        {
            int chunk = i < 5 ? i : 5;
            sm_exec(SMx, asm_out_pc | chunk);
            state->fifo[state->tx] = (state->fifo[state->tx] << chunk) | REG(SMx + PIO_SM0_ADDR_OFFSET);
        }
        state->tx++;
    } 

    // Read out the rx fifo directly
    while(!(REG(PIOx + PIO_FSTAT_OFFSET) & (1 << (PIO_FSTAT_RXEMPTY_LSB + sm))))
    {
        state->fifo[7 - state->rx] = REG(PIOx + PIO_RXF0_OFFSET + sm * 4);
        state->rx++;
    }

    // With the fifo queues read out, we can now safely disable FJOIN
    REG(SMx + PIO_SM0_SHIFTCTRL_OFFSET) &= ~(PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS | PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    // Figure out isc by rotating isr one bit at at a time until autopush triggers
    state->isc = 32;
    sm_exec(SMx, asm_push_iffull);
    while (REG(PIOx + PIO_FSTAT_OFFSET) & (1 << (PIO_FSTAT_RXEMPTY_LSB + sm)))
    {
        sm_exec(SMx, asm_in_isr | 1);
        state->isc--;
    }

    // Read out isr and undo the rotations
    state->isr = REG(PIOx + PIO_RXF0_OFFSET + sm * 4);
    state->isr = (state->isr >> state->isc) | (state->isr << (32 - state->isc));

    // Read out x and y
    sm_exec(SMx, asm_in_x);
    state->x = REG(PIOx + PIO_RXF0_OFFSET + sm * 4);
    sm_exec(SMx, asm_in_y);
    state->y = REG(PIOx + PIO_RXF0_OFFSET + sm * 4);

    // Get IRQ status
    for (int i = 0; i < 8; i++)
    {
        sm_exec(SMx, asm_wait_0_irq | i);
        uint stalled = (REG(SMx + PIO_SM0_EXECCTRL_OFFSET) & PIO_SM0_EXECCTRL_EXEC_STALLED_BITS);
        state->irq |= (stalled >> PIO_SM0_EXECCTRL_EXEC_STALLED_LSB) << i;
    }
}

void restore_state(uint PIOx, uint sm, struct pio_state const *state, uint original_shiftctrl, uint original_side_en)
{
    // Base addresses for the SM block
    const uint SMx = PIOx + (PIO_SM1_CLKDIV_OFFSET - PIO_SM0_CLKDIV_OFFSET) * sm;

    // First we need to compute the un-shifted version of osr which
    // will be loaded first so that osc can be restored later
    uint load_osr;
    int out_shift_right = (original_shiftctrl & PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS);
    uint expected_osr_bits = out_shift_right ? (0xFFFFFFFF >> state->osc) : (0xFFFFFFFF << state->osc);
    int complex_osr = (state->osr & ~expected_osr_bits);
    if (complex_osr)
    {
        // In 'normal' situations, the bits on the opposite side of the output shift counter
        // will all be 0. It is possible to get the machine into a state where they're not.
        // However, it's definitely impossible for osr to have a 1 at both ends if the output
        // counter is more than 0.
        if (state->osc > 0 && (state->osr & 1) == 1 && (state->osr >> 31) == 1)
        {
            load_osr = 0xDEADBEEF; // Should never happen.
        }
        else if ((state->osc & 1) == 0)
        {
            load_osr = state->osr;
        }
        else
        {
            load_osr = (state->osr & 1) ? state->osr << 1 : state->osr >> 1;
        }
    }
    else
    {
        load_osr = out_shift_right ? (state->osr << state->osc) : (state->osr >> state->osc);
    }
    
    // Temporarily store x value in osr so we can use x to recreate the rx fifo and osr values
    sm_exec(SMx, asm_mov_osr_x);

    // Restore FJOIN_RX before restoring rx fifo
    REG(SMx + PIO_SM0_SHIFTCTRL_OFFSET) |= (original_shiftctrl & PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    // Since we might not have a tx fifo anymore, recreate rx fifo words 5 bits at a time through set
    for (int i = 0; i < state->rx; i++)
    {
        for (int j = 0; j < 32; j += 5)
        {
            sm_exec(SMx, asm_set_x | ((state->fifo[7 - i] >> j) & 0b11111));
            sm_exec(SMx, asm_in_x | (j < 27 ? 5 : 2));
        }
    }

    // Recreate osr value the same way.
    // Disable autopush to prevent pushing it into the rx fifo.
    REG(SMx + PIO_SM0_SHIFTCTRL_OFFSET) &= ~PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS;
    for (int i = 0; i < 32; i += 5)
    {
        sm_exec(SMx, asm_set_x | ((load_osr >> i) & 0b11111));
        sm_exec(SMx, asm_in_x | (i < 27 ? 5 : 2));
    }

    // Put x back before restoring osr
    sm_exec(SMx, asm_mov_x_osr);

    // If osr is even, invert it temporarily so we have a source of 1's to shift in
    sm_exec(SMx, (load_osr & 1) == 0 ? asm_mov_osr_inv_isr : asm_mov_osr_isr);
    
    // Restore isr and input counter by shifting in one bit at at time
    for (int i = 0; i < 32; i++)
    {
        sm_exec(SMx, (state->isr & (1 << i)) ? (asm_in_osr | 1) : (asm_in_null | 1));   
        if (i == 31 - state->isc)
        {
            // Reset isc to zero here so we reach the correct isc value when we're done
            sm_exec(SMx, asm_mov_isr_isr);
        }
    }

    // Un-invert osr if we inverted it
    if ((load_osr & 1) == 0)
    {
        sm_exec(SMx, asm_mov_osr_inv_osr);
    }

    // Restore osc
    if (complex_osr)
    {
        // To restore a complex osr, jiggle osr left and right to restore the output counter
        if ((state->osr & 1) == (state->osc & 1))
        {
            REG(SMx + PIO_SM0_SHIFTCTRL_OFFSET) |= PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS;
        }
        
        for (int i = 0; i < state->osc; i++)
        {
            sm_exec(SMx, asm_out_null | 1);
            REG(SMx + PIO_SM0_SHIFTCTRL_OFFSET) ^= PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS;
        }
    }
    else if (state->osc > 0)
    {
        // For normal osr values, we can restore osc directly
        REG(SMx + PIO_SM0_SHIFTCTRL_OFFSET) |= (original_shiftctrl & PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS);
        sm_exec(SMx, asm_out_null | (state->osc & 0b11111));
    }

    // Restore FJOIN_TX. No need to worry about the rx fifo; if FJOIN_TX was set, then the rx fifo must have been empty.
    REG(SMx + PIO_SM0_SHIFTCTRL_OFFSET) |= (original_shiftctrl & PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS);

    // Restore tx fifo
    for (int i = 0; i < state->tx; i++)
    {
        REG(PIOx + PIO_TXF0_OFFSET + sm * 4) = state->fifo[i];
    }

    // Jump back to the original address
    sm_exec(SMx, asm_jmp | state->addr);
    
    // Restore settings
    REG(SMx + PIO_SM0_SHIFTCTRL_OFFSET) = original_shiftctrl;
    REG(SMx + PIO_SM0_EXECCTRL_OFFSET) &= (~PIO_SM0_EXECCTRL_SIDE_EN_BITS | original_side_en);
}


uint step(int pio, int sm, uint max_steps, uint breakpoints)
{
    // Base addresses for the SM block
    const uint PIOx = pio ? PIO1_BASE : PIO0_BASE;
    const uint SMx = PIOx + (PIO_SM1_CLKDIV_OFFSET - PIO_SM0_CLKDIV_OFFSET) * sm;

    // Ensure the state machine is paused; return 0 if it's running
    if (REG(PIOx + PIO_CTRL_OFFSET) & (1 << (PIO_CTRL_SM_ENABLE_LSB + sm)))
    {
        return 0;
    }

    // Backup the original clock divider, then set it to the slowest possible speed
    uint original_clkdiv = REG(SMx + PIO_SM0_CLKDIV_OFFSET);
    REG(SMx + PIO_SM0_CLKDIV_OFFSET) = 0xFFFF0000;

    // Restart clock divider from its initial phase for consistent timing.
    // The clock divider's enable pulse will be sent on the following cycle,
    // and since the machine is paused, it will not be able to advance.
    REG(PIOx + PIO_CTRL_OFFSET) |= (1 << (PIO_CTRL_CLKDIV_RESTART_LSB + sm));
    __asm("nop");

    // Unpause the state machine. Now, it will advance exactly one step
    // each time we restart the clock divider, so long as we don't take
    // more than 2^16 CPU cycles between clock divider restarts.
    REG(PIOx + PIO_CTRL_OFFSET) |= (1 << (PIO_CTRL_SM_ENABLE_LSB + sm));

    // Step the state machine until we hit a breakpoint or the maximum number of steps
    uint steps_taken = 1;
    for (; steps_taken <= max_steps; steps_taken++)
    {
        // Restart the clock divider to take another step
        REG(PIOx + PIO_CTRL_OFFSET) |= (1 << (PIO_CTRL_CLKDIV_RESTART_LSB + sm));
        __asm("nop");

        // Check if the current address is a breakpoint
        if (breakpoints & (1 << REG(SMx + PIO_SM0_ADDR_OFFSET)))
        {
            break;
        }
    }

    // Pause the state machine again and restore the original clock divider
    REG(PIOx + PIO_CTRL_OFFSET) &= ~(1 << (PIO_CTRL_SM_ENABLE_LSB + sm));
    REG(SMx + PIO_SM0_CLKDIV_OFFSET) = original_clkdiv;
    REG(PIOx + PIO_CTRL_OFFSET) |= (1 << (PIO_CTRL_CLKDIV_RESTART_LSB + sm));

    return steps_taken;
}