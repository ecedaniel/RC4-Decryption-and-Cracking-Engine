// This module is an interface for writing data to a memory and reading data from memory.
// set addr and wr_data (if writing) before or at same time wr_start is asserted.
// set addr before or at the same time rd_start is asserted.
// read data after rd_done is asserted.

module Memory_Interface (
    input logic clk, nreset,
    // general interface
    input logic [7:0] addr_in,
    output logic [7:0] addr_mem,
    // write interface 
    input logic wr_start,           // start wr flag
    input logic [7:0] wr_data_in,
    output logic wr_done,
    output logic wr_enable,         // write enable to memory (toggled by state encoding)
    output logic [7:0] wr_mem,
    // Read interface
    input logic rd_start,           // start rd flag
    input logic [7:0] rd_mem,       // Data from memory 
    output logic rd_done,
    output logic [7:0] rd_data_out  // Captured data from memory
);

parameter IDLE      = 8'b0000_0000;
parameter WR_SETUP  = 8'b0001_0000;
parameter WRITE     = 8'b0010_0001;
parameter WR_DONE   = 8'b0011_0010;
parameter RD_SETUP  = 8'b0100_0000;
parameter READ_WAIT = 8'b0101_0000;
parameter READ_WAIT_2 = 8'b0110_0000;
parameter READ      = 8'b0111_0000;
parameter RD_DONE   = 8'b1000_1000;

logic [7:0] state;

assign wr_done = state[1];    // 1'b1 during WR_DONE state
assign rd_done = state[3];    // 1'b1 during RD_DONE state
assign wr_enable = state[0];  // 1'b1 during WRITE state

always_ff @(posedge clk) begin
    if (~nreset) begin 
        state <= IDLE;
        addr_mem <= 8'b0;
        wr_mem <= 8'b0;
        rd_data_out <= 8'b0;
    end else begin
        case (state)
            IDLE: begin
                if (wr_start) begin
                    state <= WR_SETUP;
                end
                else if (rd_start) begin
                    state <= RD_SETUP;
                end
                else begin
                    state <= IDLE;
                end
            end

            WR_SETUP: begin
                // Capture address and data when starting write
                addr_mem <= addr_in;
                wr_mem <= wr_data_in;
                state <= WRITE;
            end

            WRITE: begin
                // wr enabled for one clk cycle
                state <= WR_DONE;
            end
            
            WR_DONE: begin
                // wr_done asserted for one clk cycle
                state <= IDLE;
            end
            
            RD_SETUP: begin
                // Capture address when starting read
                addr_mem <= addr_in;
                state <= READ_WAIT;
            end

            READ_WAIT: begin
                state <= READ_WAIT_2;
            end

            READ_WAIT_2: begin
                state <= READ;
            end
        
            READ: begin
                rd_data_out <= rd_mem;
                state <= RD_DONE;
            end
            
            RD_DONE: begin
                // rd_done asserted for one clk cycle.
                state <= IDLE;
            end
            
            default: begin
                state <= IDLE;
            end
        endcase
    end
end

endmodule



