// This module is an interface for reading data from encrypted memory (ROM).
// It uses a simplified version of Memory_Interface without the writing interface

module E_Memory_Interface (
    input logic clk, nreset,
    // general interface
    input logic [4:0] addr_in,
    output logic [4:0] addr_mem,
    // Read interface
    input logic rd_start,
    input logic [7:0] rd_mem,
    output logic rd_done,
    output logic [7:0] rd_data_out
);

parameter IDLE        = 8'b0000_0000;
parameter RD_SETUP    = 8'b0001_0000;
parameter READ_WAIT   = 8'b0010_0000;
parameter READ_WAIT_2 = 8'b0011_0000;
parameter READ        = 8'b0100_0000;
parameter RD_DONE     = 8'b0101_1000;

logic [7:0] state;

assign rd_done = state[3];    // 1'b1 during RD_DONE state

always_ff @(posedge clk) begin
    if (~nreset) begin 
        state <= IDLE;
        addr_mem <= 5'b0;
        rd_data_out <= 8'b0;
    end else begin
        case (state)
            IDLE: begin
                if (rd_start) begin
                    state <= RD_SETUP;
                end
                else begin
                    state <= IDLE;
                end
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
                // Capture the data from memory
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
