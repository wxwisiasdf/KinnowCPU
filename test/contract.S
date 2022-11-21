	.file	"contract.cpp"
	.section text
	.section	.rodata
	.align 4
.LC0:
	.string	"contract.cpp"
	.align 4
.LC1:
	.string	"fun"
	.align 4
.LC2:
	.string	"v > 0"
	.align 4
.LC3:
	.string	"default"
	.section text
	.align 2
	.global	_Z3funi
	.type	_Z3funi, @function
_Z3funi:
.LFB0:
	;compare a0=0
	mov	t1, a0
	sub	sp, sp, 44
.LCFI0:
	ble	.L4
	add	a0, zero, zero
	sub	a0, a0, t1
	ret
.L4:
	sub	t0, zero, -28
	add	a0, tp, t0
	la	t1, .LC0
	mov	long [a0], t1
	la	t1, .LC1
	la	t0, .LC3
	mov	long [a0 + 4], t1
	la	t1, .LC2
	mov	long [a0 + 8], t1
	mov	long [a0 + 12], t0
	mov	long [a0 + 16], t0
	mov	long [a0 + 20], 3
	mov	long [a0 + 24], 0
	call	_Z25handle_contract_violationRKNSt12experimental18contract_violationE
	call	_ZSt9terminatev
.LFE0:
	.size	_Z3funi, .-_Z3funi
	.section	.eh_frame,"aw",@progbits
.Lframe1:
	.4byte	.LECIE1-.LSCIE1
.LSCIE1:
	.4byte	0
	.byte	0x3
	.string	""
	.byte	0x1
	.byte	0x7c
	.byte	0x24
	.byte	0xc
	.byte	0x1e
	.byte	0
	.byte	0x11
	.byte	0x24
	.byte	0x7f
	.align 4
.LECIE1:
.LSFDE1:
	.4byte	.LEFDE1-.LASFDE1
.LASFDE1:
	.4byte	.LASFDE1-.Lframe1
	.4byte	.LFB0
	.4byte	.LFE0-.LFB0
	.byte	0x4
	.4byte	.LCFI0-.LFB0
	.byte	0xe
	.byte	0x2c
	.align 4
.LEFDE1:
	.ident	"GCC: (GNU) 13.0.0 20221118 (experimental)"
