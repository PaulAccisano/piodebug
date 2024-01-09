# RP2040 MicroPython PIO Debugger
# MIT license; Copyright (c) 2024 Paul Accisano

# Decode a single PIO instruction
def asm_pio_decode(instruction, side_set_bits = 0, side_set_opt = False, addr = None):
    arg = (instruction >> 0) & 0b11111
    mod = (instruction >> 5) & 0b111
    dss = (instruction >> 8) & 0b11111
    op = (instruction >> 13) & 0b111
    ret = ""
    if op == 0b000: # JMP
        jmp_cond = ["", "not_x, ", "x_dec, ", "not_y, ", "y_dec, ", "x_not_y, ", "pin, ", "not_osre, "][mod]
        jmp_target = str(arg) if addr is None else f"+{arg-addr}" if arg > addr else f"{arg-addr}"
        ret = f"jmp({jmp_cond}{jmp_target})"
    elif op == 0b001: # WAIT
        wait_cond = ["gpio", "pin", "irq", "#"][mod & 0b11]
        arg_str = str(arg) if (mod & 0b11) != 2 else '#' if (arg & 0b1000) else f"rel({arg & 0b111})" if (arg & 0b10000) else str(arg)
        ret = f"wait({mod >> 2}, {wait_cond}, {arg_str})"
    elif op == 0b010: # IN
        in_src = ["pins", "x", "y", "null", "#", "#", "isr", "osr"][mod]
        ret = f"in_({in_src}, {32 if not arg else arg})"
    elif op == 0b011: # OUT
        out_dest = ["pins", "x", "y", "null", "pindirs", "pc", "isr", "exec"][mod]
        ret = f"out({out_dest}, {32 if not arg else arg})"
    elif op == 0b100: # PUSH/PULL
        push_pull = ["push", "pull"][mod >> 2]
        args = ["noblock", "", "iffull, noblock", "iffull", "noblock", "", "ifempty, noblock", "ifempty"][mod]
        ret = f"{push_pull}({args}{', #' if arg else ''})"
    elif op == 0b101: # MOV
        mov_dest = ["pins", "x", "y", "#", "exec", "pc", "isr", "osr"][mod]
        mov_src = ["pins", "x", "y", "null", "#", "status", "isr", "osr"][arg & 0b111]
        mov_src_op = [f"{mov_src}", f"invert({mov_src})", f"reverse({mov_src})", "#"][arg >> 3]
        ret = f"mov({mov_dest}, {mov_src_op})"
    elif op == 0b110: # IRQ
        arg_str = '#' if (arg & 0b1000) else f"rel({arg & 0b111})" if (arg & 0b10000) else str(arg)
        mod_str = ["", "block, ", "clear, ", "#", "#", "#", "#", "#"][mod]
        ret = f"irq({mod_str}{arg_str})"
    elif op == 0b111: # SET
        set_dest = ["pins", "x", "y", "#", "pindirs", "#", "#", "#"][mod]
        ret = f"set({set_dest}, {arg})"

    delay = dss & (0b11111 >> side_set_bits)
    if side_set_bits > 0:
        if side_set_opt:
            side_set_bits = 0 if dss >> 4 == 1 else side_set_bits - 1
            dss &= 0b1111

    if side_set_bits > 0:
        ret += f".side({dss >> (5 - side_set_bits)})"
    
    if delay > 0:
        ret += f"[{delay}]"
    
    return ret

# Print out a pretty version of the state machine's program and state
def print_state(id, program, state, breakpoints = 0):
    PROG_DATA = 0
    PROG_OFFSET_PIO0 = 1
    PROG_OFFSET_PIO1 = 2
    PROG_EXECCTRL = 3
    PROG_SHIFTCTRL = 4
    PROG_OUT_PINS = 5
    PROG_SET_PINS = 6
    PROG_SIDESET_PINS = 7

    EXECCTRL_SIDE_EN_BITS = 0x40000000
    SHIFTCTRL_FJOIN_TX_BITS = 0x40000000
    SHIFTCTRL_FJOIN_RX_BITS = 0x80000000
    SHIFTCTRL_IN_SHIFTDIR_BITS = 0x00040000
    SHIFTCTRL_OUT_SHIFTDIR_BITS = 0x00080000

    ssb = len(program[PROG_SIDESET_PINS]) if program[PROG_SIDESET_PINS] else 0
    sso = program[PROG_EXECCTRL] & EXECCTRL_SIDE_EN_BITS
    tx_size = 8 if program[PROG_SHIFTCTRL] & SHIFTCTRL_FJOIN_TX_BITS else 0 if program[PROG_SHIFTCTRL] & SHIFTCTRL_FJOIN_RX_BITS else 4
    rx_size = 8 - tx_size
    instructions = [asm_pio_decode(instr, side_set_bits=ssb, side_set_opt=sso, addr=index) for index, instr in enumerate(program[PROG_DATA])]
    offset = program[PROG_OFFSET_PIO0] if (id >> 2) == 0 else program[PROG_OFFSET_PIO1]
    addr = state['addr'] - offset

    # Display registers in hex, signed decimal, and binary
    toSigned = lambda x: ((x & 0xffffffff) ^ 0x80000000) - 0x80000000
    out_shiftdir = program[PROG_SHIFTCTRL] & SHIFTCTRL_OUT_SHIFTDIR_BITS 
    in_shiftdir = program[PROG_SHIFTCTRL] & SHIFTCTRL_IN_SHIFTDIR_BITS
    state_display_lines = [
        f" X:   0x{state['x']:08X} ({toSigned(state['x'])})",
        f" {state['x']:032b}",
        "", 
        f" Y:   0x{state['y']:08X} ({toSigned(state['y'])})",
        f" {state['y']:032b}",
        "",
        f" OSR: 0x{state['osr']:08X} ({toSigned(state['osr'])})",
        f" {state['osr']:032b}",
        f" {('*' * state['osc']) + (' ' * (32 - state['osc'])) if out_shiftdir else (' ' * (32 - state['osc'])) + ('*' * state['osc'])}",
        f" ISR: 0x{state['isr']:08X} ({toSigned(state['isr'])})",
        f" {state['isr']:032b}",
        f" {('*' * state['isc']) + (' ' * (32 - state['isc'])) if in_shiftdir else (' ' * (32 - state['isc'])) + ('*' * state['isc'])}",
        "",
        f"       TX               RX"
    ] + [
        (f" [{toSigned(state['tx_fifo'][i]) if i < len(state['tx_fifo']) else '' : 12} ]" if i < tx_size else " " * 19) + " " + \
        (f" [{toSigned(state['rx_fifo'][i]) if i < len(state['rx_fifo']) else '' : 12} ]" if i < rx_size else " " * 19)
        for i in range(max(tx_size, rx_size))
    ] + [
        "",
        f" IRQs:  0  1  2  3  4  5  6  7",
        f"        {'  '.join(['*' if state['irq'] & (1 << i) else ' ' for i in range(8)])}"
    ]

    # Compute target if active instruction is a jump
    jmp = [-1, -1]
    if 0 <= addr < len(program[PROG_DATA]) and program[PROG_DATA][addr] >> 13 == 0b000:
        jmp = sorted([addr, program[PROG_DATA][addr] & 0b11111])
    
    # Display the program on the left, and the state on the right
    instructions += [''] * (len(state_display_lines) - len(instructions))
    lines = [
        f"{i:02}{'*' if breakpoints & (1 << i) else ' '}" + \
        f"{'>' if i == addr else ' '} " + \
        f"{'.-' if i == jmp[0] else '`-' if i == jmp[1] else '| ' if jmp[0] < i < jmp[1] else '  ' }" + \
        f"{instructions[i]:<36} |{state_display_lines[i]}"
        for i in range(len(instructions))
    ]
    
    print('\n'.join(lines))

def interactive_debug(id, program):
    PROG_OFFSET_PIO0 = 1
    PROG_OFFSET_PIO1 = 2

    last_command = ''
    breakpoints = 0
    offset = program[PROG_OFFSET_PIO0] if (id >> 2) == 0 else program[PROG_OFFSET_PIO1]

    while True:
        if last_command != '?':
            print("\n\n")
            state = sm_get_state(id)
            print_state(id, program, state, breakpoints)

        command = input("\nEnter command ('?' for help, blank to repeat last): ")
        if command == '':
            command = last_command or '?'

        try:
            if command == '?':
                print("Commands:")
                print("  s - step one cycle")
                print("  n - step until PC changes (skips past delay)")
                print("  l - step until PC returns to current value")
                print("  r - step until next breakpoint")
                print("  b - toggle breakpoint")
                print("  q - quit")

            elif command == 's':
                sm_step(id, 1, 0)

            elif command == 'n':
                sm_step(id, 0xFFFFFFFF, ~(1 << state['addr']))

            elif command == 'l':
                sm_step(id, 0xFFFFFFFF, 1 << state['addr'])
                
            elif command == 'r':
                sm_step(id, 0xFFFFFFFF, breakpoints)
                
            elif command == 'b':
                bp_line = int(input("Enter breakpoint address: "))
                breakpoints ^= 1 << (bp_line + offset)
                
            elif command == 'q':
                break

        except KeyboardInterrupt:
            pass
        
        last_command = command
