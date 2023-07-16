`ifndef DEFINES_SV
`define DEFINES_SV

/// @param a Value
/// @param b Amount rotated
/// @param n Size of value
`define ROR(a, b, n) ((a >> b) | (a << (n - b)))

`define VOID [1:0]

`endif
