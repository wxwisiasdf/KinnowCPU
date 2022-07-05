
# Shortcuts

## Trans size
See the truth value for trans_size, if we apply the shift as `(1 << (~trans_size))`, we can get 1, 2 and 4 from this, so all the cases would be covered now multiplying by 8 we can obtain 8, 16 and 32, so in order to do this efficiently. Now for obtaining the bitmask that is to be applied, we can simply do: `(1 << ((1 << (~trans_size)) * 8))`, yielding 256, 65535 and the maximum 32-bit value possible now we simply subtract by one and we obtain the bitmask! - Simple as that, we are confident that the verilog compiler will be able to optimize the multiplication and subtraction into shifts or some other semantics, but these are provided for readability, remember that when shifting the 32-bit register into a 33-bit one, and performing a subtrction will always yield a 32'hFFFFFFFF value due to how unsigned numbers work.

Translation size can be converted onto a shift value given the following truth table:
```
-------------------------
| Size   | I  | O  | V  |
|--------|----|----|----|
| Byte   | 11 | 00 | 0  |
| Int    | 10 | 01 | 1  |
| Long   | 01 | 10 | 2  |
| ????   | 00 | 11 | 3  |
-------------------------
```
