;
; Perform a jump and increment t0
;

.section text

start:
.global start
    la sp, 0
    la lr, 0
    jal jmp_test
jmp_test:
    addi t0, t0, 1
    jalr zero, lr, 0
