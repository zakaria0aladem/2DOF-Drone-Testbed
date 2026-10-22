@ exc_return.s - exception-return trampolines for the emulator.
@
@ The emulator gives every exception handler an LR of 0x1FFFFFxx, where xx is
@ the low byte of the real EXC_RETURN value (E1, E9, ED, F1, F9, FD). So code
@ that tests LR bits (FreeRTOS, fault handlers) sees the bits it expects.
@ When the handler returns (bx lr / pop {pc}) it lands here; this code tells
@ the emulator which exception finished (a store to the NOTIFY register) and
@ then unstacks the frame like the hardware does.
@
@ Assembled once into exc_return.bin (see make_asm.py) and loaded at 0x1FFFF800.
        .syntax unified
        .thumb
        .equ NOTIFY, 0x1FFFF400

        .org 0
ret_common:                         @ r0 = EXC_RETURN low byte; r0-r3, r12, lr are free
        ldr     r1, =NOTIFY
        str     r0, [r1]
        tst     r0, #4              @ return to the process stack?
        beq     1f
        mrs     r1, control
        orr     r1, r1, #2
        msr     control, r1
        isb
1:      tst     r0, #0x10           @ bit4 set = basic frame, clear = FP frame
        bne     basic
        add     r1, sp, #32
        vldmia  r1, {s0-s15}
        ldr     r1, [sp, #96]
        vmsr    fpscr, r1
        ldr     r0, [sp, #28]
        ldr     r1, [sp, #24]
        orr     r1, r1, #1
        tst     r0, #0x200          @ stack was realigned on entry?
        bne     fp_pad
        str     r1, [sp, #100]
        msr     APSR_nzcvqg, r0
        ldmia   sp!, {r0-r3, r12, lr}
        add     sp, sp, #76
        pop     {pc}
fp_pad: str     r1, [sp, #104]
        msr     APSR_nzcvqg, r0
        ldmia   sp!, {r0-r3, r12, lr}
        add     sp, sp, #80
        pop     {pc}
basic:  ldr     r0, [sp, #28]
        ldr     r1, [sp, #24]
        orr     r1, r1, #1
        tst     r0, #0x200
        bne     b_pad
        str     r1, [sp, #28]
        msr     APSR_nzcvqg, r0
        ldmia   sp!, {r0-r3, r12, lr}
        add     sp, sp, #4
        pop     {pc}
b_pad:  str     r1, [sp, #32]
        msr     APSR_nzcvqg, r0
        ldmia   sp!, {r0-r3, r12, lr}
        add     sp, sp, #8
        pop     {pc}
        .ltorg

s_e1:   movs    r0, #0xE1
        b       ret_common
s_e9:   movs    r0, #0xE9
        b       ret_common
s_ed:   movs    r0, #0xED
        b       ret_common
s_f1:   movs    r0, #0xF1
        b       ret_common
s_f9:   movs    r0, #0xF9
        b       ret_common
s_fd:   movs    r0, #0xFD
        b       ret_common

        .org 0x7E0                  @ = 0x1FFFFFE0
        b       s_e1
        .org 0x7E8
        b       s_e9
        .org 0x7EC
        b       s_ed
        .org 0x7F0
        b       s_f1
        .org 0x7F8
        b       s_f9
        .org 0x7FC
        b       s_fd
        .org 0x800
