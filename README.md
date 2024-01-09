# piodebug
This package enables introspection and debugging of RP2040 PIO State Machines.

## Methods

* `sm_get_state(id)` - Non-destructively reads the current state of a state machine, where `id` is the Micropython state machine identifier (0-7). Returns a dictionary containing the four registers, two shift counters, two FIFO queues, the current program address, and an IRQ bitfield, with each bit indicating the status of the corresponding PIO IRQ.

* `sm_step(id, max_steps, breakpoints)` - Single-steps the state machine for `max_steps` cycles, or until a breakpoint is hit. `breakpoints` is a 32-bit integer where each bit corresponds to a state machine instruction address. If a bit is set, the state machine will stop when it reaches that address. Note that the addresses are absolute, not relative to the start of the program.

* `sm_print_state(id, program, state, breakpoints)` - Prints the current state of the state machine in a human-readable format, along with the state machine program itself. `program` is an `@rp2.asm_pio` annotated PIO program from which an `rp2.StateMachine` has been instantiated. `state` is the dictionary returned by `sm_get_state()`. `breakpoints` is a 32-bit integer where each bit corresponds to a state machine instruction address.

* `sm_interactive_debug(id, program)` - Starts an interactive debugging session for the state machine. `program` is an `@rp2.asm_pio` annotated PIO program from which an `rp2.StateMachine` has been instantiated.

* `asm_pio_decode(instruction, side_set_bits, side_set_opt, addr)` - Decodes a PIO instruction into its corresponding Micropython command. `instruction` is the 16-bit instruction word, `side_set_bits` is the number of bits devoted to sideset, and `side_set_opt` is whether sideset is optional. `addr` is the optional instruction address. If `addr` is provided, `jmp` instructions will decode to `jmp(<condition>, +X)` or `jmp(<condition>, -X)`, where `X` is the relative number of instructions to jump. Otherwise, `jmp` instructions will decode to `jmp(<condition>, <address>)` where `address` is the absolute address of the jump target.

## Limitations

If a state machine is stalled on an instruction executed using `rp2.StateMachine.exec`, `sm_get_state` will cause the instruction to be lost. This is because the RP2040 does not provide a way to read executing instructions executed this way, so there is no way to restore it.

## Language Support

`sm_get_state` and `sm_step` are implemented in C, and are only dependent on a few of the pico-sdk headers, making them easy to use in a C/C++ project. `sm_print_state` and `sm_interactive_debug` are implemented in Micropython.

## Example

```python
import rp2
import piodebug

@rp2.asm_pio(out_shiftdir = PIO.SHIFT_RIGHT, in_shiftdir = PIO.SHIFT_RIGHT, autopush=True)
def addition():
    <...>

sm_addition = rp2.StateMachine(0, addition)

# Send some numbers to add
sm_addition.put(7777)
sm_addition.put(3333)
sm_addition.put(12345)
sm_addition.put(543210)

# Step past the pulls to make room
piodebug.sm_step(0, 3, 0)

# Send two more
sm_addition.put(2040)
sm_addition.put(4020)

# Debug
piodebug.sm_interactive_debug(0, addition)
```

Output:
```text
12     pull()                               |  X:   0xFFFFE19E (-7778)
13     mov(x, invert(osr))                  |  11111111111111111110000110011110
14     pull()                               | 
15 >   mov(isr, invert(null))               |  Y:   0x00000000 (0)
16     out(y, 1)                            |  00000000000000000000000000000000
17     jmp(not_y, +8)                       | 
18     in_(x, 1)                            |  OSR: 0x00000D05 (3333)
19     mov(y, invert(osr))                  |  00000000000000000000110100000101
20     mov(osr, invert(x))                  |                                  
21     mov(x, y)                            |  ISR: 0x00000000 (0)
22     out(y, 1)                            |  00000000000000000000000000000000
23     jmp(not_y, +7)                       |                                  
24     jmp(x_dec, +6)                       | 
25     mov(y, invert(osr))                  |        TX               RX
26     mov(osr, invert(x))                  |  [       12345 ]  [             ]
27     mov(x, y)                            |  [      543210 ]  [             ]
28     in_(osr, 1)                          |  [        2040 ]  [             ]
29     out(null, 1)                         |  [        4020 ]  [             ]
30     mov(y, isr)                          | 
31     jmp(y_dec, -15)                      |  IRQs:  0  1  2  3  4  5  6  7
                                            |                               

Enter command ('?' for help, blank to repeat last): ?
Commands:
  s - step one cycle
  n - step until PC changes (skips past delay)
  l - step until PC returns to current value
  r - step until next breakpoint
  b - toggle breakpoint
  q - quit

Enter command ('?' for help, blank to repeat last): 
```
