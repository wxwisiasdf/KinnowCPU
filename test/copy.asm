;
; Copy t0 to whatever t1 points to, but then increment t0
;

.section text

start:
.global start
    la t0, 0 ; data=0
    la t1, 0 ; ram=0
    jal jmp_test
jmp_test:
    addi t0, t0, 4
    addi t1, t1, 4
    mov long [t1 + 0], t0
    jalr zero, lr, 0

.section data
.dl 0
.section bss
.dl 0
