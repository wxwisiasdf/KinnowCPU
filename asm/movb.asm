;
; Serial test
;

.define base_port 0xF8000040

.section text

start:
.global start
    ; Print HELLO\r\n on the serial
    la t0, base_port
    la t1, msg
.loop:
    mov s0, byte [t1]
    beq s0, .end
    jal putc
    addi t1, t1, 1
    b .loop
.end:
; Then proceed to hang
.hang:
    hlt
    b .hang

putc:
.global putc
.wait: ; Wait until serial port returns 0
    mov s1, byte [t0]
    bne s1, .wait
    mov byte [t0 + 4], s0
    ret

msg:
    .ds "Hello world!"
    .db 0
