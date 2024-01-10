// RP2040 MicroPython PIO Debugger
// MIT license; Copyright (c) 2024 Paul Accisano

#include "piodebug.h"
#include "py/dynruntime.h"

// This is the function which will be called from Python, as get_state(x)
STATIC mp_obj_t py_get_state(mp_obj_t pio_sm_obj) {
    // Extract the integer from the MicroPython input object
    mp_int_t pio_sm = mp_obj_get_int(pio_sm_obj);
    
    // Call the fuction
    struct pio_state state = {0};
    get_state(pio_sm / 4, pio_sm % 4, &state);

    // Convert the result to a MicroPython dictionary object and return it
    mp_obj_t dict = mp_obj_new_dict(8);
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_x), mp_obj_new_int_from_uint(state.x));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_y), mp_obj_new_int_from_uint(state.y));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_osr), mp_obj_new_int_from_uint(state.osr));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_isr), mp_obj_new_int_from_uint(state.isr));
    
    mp_obj_t fifo[8];
    for (int i = 0; i < state.tx; i++)
    {
        fifo[i] = mp_obj_new_int_from_uint(state.fifo[i]);
    }
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_tx_fifo), mp_obj_new_list(state.tx, fifo));

    for (int i = 0; i < state.rx; i++)
    {
        fifo[i] = mp_obj_new_int_from_uint(state.fifo[7 - i]);
    }
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_rx_fifo), mp_obj_new_list(state.rx, fifo));

    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_osc), mp_obj_new_int(state.osc));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_isc), mp_obj_new_int(state.isc));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_addr), mp_obj_new_int(state.addr));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_irq), mp_obj_new_int(state.irq));
    
    return dict;
}

STATIC mp_obj_t py_step(mp_obj_t pio_sm_obj, mp_obj_t max_steps_obj, mp_obj_t breakpoints_obj) {
    // Extract the integers from the MicroPython input objects
    mp_int_t pio_sm = mp_obj_get_int(pio_sm_obj);
    mp_uint_t max_steps = mp_obj_get_int(max_steps_obj);
    mp_uint_t breakpoints = mp_obj_get_int(breakpoints_obj);

    // Call the fuction
    uint steps_taken = step(pio_sm / 4, pio_sm % 4, max_steps, breakpoints);

    // Convert the result to a MicroPython integer object and return it
    return mp_obj_new_int_from_uint(steps_taken);
}

// Define a Python reference to the functions above
STATIC MP_DEFINE_CONST_FUN_OBJ_1(get_state_obj, py_get_state);
STATIC MP_DEFINE_CONST_FUN_OBJ_3(step_obj, py_step);

// This is the entry point and is called when the module is imported
mp_obj_t mpy_init(mp_obj_fun_bc_t *self, size_t n_args, size_t n_kw, mp_obj_t *args) {
    // This must be first, it sets up the globals dict and other things
    MP_DYNRUNTIME_INIT_ENTRY

    // Make the functions available in the module's namespace
    mp_store_global(MP_QSTR_sm_get_state, MP_OBJ_FROM_PTR(&get_state_obj));
    mp_store_global(MP_QSTR_sm_step, MP_OBJ_FROM_PTR(&step_obj));

    // This must be last, it restores the globals dict
    MP_DYNRUNTIME_INIT_EXIT
}