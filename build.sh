rm system.o
iverilog system.v -o system.o || exit
vvp system.o >log.txt || exit

rm system.json
yosys \
    -p "hierarchy -top limn2600_cpu" \
    -p "aigmap" \
    -p "write_json system.json" \
    -p "show -prefix system -notitle -colors 2 -width -format dot"
    system.v || exit
xdot system.dot
