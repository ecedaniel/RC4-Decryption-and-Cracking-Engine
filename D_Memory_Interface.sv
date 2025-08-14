//  This module is a simplified memory interface without the reading interface.

module D_Memory_Interface (
    input logic clk, nreset,
    // general interface
    input logic [4:0] addr_in,
    output logic [4:0] addr_mem,
    // write interface
    input logic wr_start,
    input logic [7:0] wr_data_in,
    output logic wr_done,
    output logic wr_enable,
    output logic [7:0] wr_mem
);

parameter IDLE      = 8'b0000_0000;
parameter WR_SETUP  = 8'b0001_0000;
parameter WRITE     = 8'b0010_0001;
parameter WR_DONE   = 8'b0011_0010;

logic [7:0] state;

assign wr_done = state[1];      // 1'b1 during WR_DONE state
assign wr_enable = state[0];    // 1'b1 during WRITE state

always_ff @(posedge clk) begin
    if (~nreset) begin
        state <= IDLE;
        addr_mem <= 5'b0;
        wr_mem <= 8'b0;
    end else begin 
        case (state)
            IDLE: begin
                if (wr_start) begin
                    state <= WR_SETUP;
                end
                else begin  
                    state <= IDLE;
                end
            end

            WR_SETUP: begin
                // Captures address and data when starting write
                addr_mem <= addr_in;
                wr_mem <= wr_data_in;
                state <= WRITE;
            end

            WRITE: begin
                // toggles wr_enable for one clk cycle
                state <= WR_DONE;
            end

            WR_DONE: begin
                // flags the wr_done for one cycle
                state <= IDLE;
            end

            default: begin  
                state <= IDLE;
            end
        endcase
    end
end

endmodule