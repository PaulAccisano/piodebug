# RP2040 MicroPython PIO Debugger
# MIT license; Copyright (c) 2024 Paul Accisano

import rp2
import piodebug

# PIO program which pulls two numbers from tx and pushes their sum to rx in O(1) time
@rp2.asm_pio(out_shiftdir = rp2.PIO.SHIFT_RIGHT, in_shiftdir = rp2.PIO.SHIFT_RIGHT, autopush=True)
def addition():
    # osr is the first number, x is the inverted second number, isr is the output
    pull()
    mov(x, invert(osr))
    pull()

    # preload isr with all 1s. When autopush triggers, it will set isr to 0, which will signal we're done.
    mov(isr, invert(null))
    
    # Check first bit of osr.
    label("loop")
    out(y, 1)
    jmp(not_y, "zero")

    # If first bit is 1, the output bit is the first bit of x (inverted second number)
    in_(x, 1)

    # Check if we need to carry
    mov(y, invert(osr))
    mov(osr, invert(x))
    mov(x, y)
    out(y, 1)
    jmp(not_y, "check")
    jmp(x_dec, "check") # Not possible for x to be 0 here.

    # if first bit is 0, the output bit is the inverted first bit of x.
    label("zero")
    mov(y, invert(osr))
    mov(osr, invert(x))
    mov(x, y)
    in_(osr, 1)
    out(null, 1)

    # Check if we're done
    label("check")
    mov(y, isr)
    jmp(y_dec, "loop")
    
    # Set irq 5 for demo purposes :)
    irq(5)
    
sm_addition = rp2.StateMachine(0, addition)
sm_addition.exec("irq(clear, 5)")

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

# Get the state dictionary
print(piodebug.sm_get_state(0))

# Start debug session
piodebug.sm_interactive_debug(0, addition)
