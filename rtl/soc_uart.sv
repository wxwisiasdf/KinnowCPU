///////////////////////////////////////////////////////////////////////////////
//
// System UART
//
///////////////////////////////////////////////////////////////////////////////
module soc_uart
#( // Parameter
    parameter DATA_WIDTH = 32
)
( // Interface
    input clk, // Clock
    input we, // Write enable
    input ce, // Command enable
    input oe, // Output enable
    output reg rdy, // Ready
    input [7:0] offset, // Offset
    input [7:0] data_in,
    output reg [DATA_WIDTH - 1:0] data_out
);
    always @(posedge clk) begin
        data_out <= 0;
        rdy <= 0;
        if(ce) begin
            if(offset == 8'h00) begin
                if(oe) begin
                    $display("%m: READ_CMD <%b>", data_out);
                    data_out <= 32'h00000000;
                end
                if(we) begin
                    $display("%m: WRITE CMD <%b> %c", data_in[7:0], data_in[7:0]);
                end
            end else if(offset == 8'h04) begin
                if(oe) begin
                    $display("%m: READ <%b>", data_out);
                    data_out <= 32'hFFFF;
                end
                if(we) begin
                    $display("%m: WRITE 0x%h<%b> %c", data_in[7:0], data_in[7:0], data_in[7:0]);
                end
            end
            rdy <= 1;
        end
    end

    initial begin

    end
endmodule
