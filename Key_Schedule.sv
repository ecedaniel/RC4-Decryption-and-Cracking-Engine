// Key_Schedule.sv - RC4 Key Scheduling Algorithm (Second Loop)
// Implements: j = 0; for i = 0 to 255 { j = (j + s[i] + secret_key[i mod keylength]) mod 256; swap s[i] and s[j] }

module Key_Schedule (
    input  logic        clk,
    input  logic        nreset,
    input  logic        start,              // Start key scheduling flag
    output logic        finish,             // Finish flag when key scheduling finished
    output logic        busy,               // Busy flag (not really used)
    
    // Memory Interface - Write
    output logic        wr_start,           // Write start flag
    output logic [7:0]  addr_out,           // Address output
    output logic [7:0]  wr_data_out,        // Write data output
    input  logic        wr_done,            // Write done flag
    
    // Memory Interface - Read
    output logic        rd_start,           // Read start flag
    input  logic        rd_done,            // Read done flag
    input  logic [7:0]  rd_data_in,         // Read data input
    
    // Secret key input (24-bit key)
    input  logic [23:0] secret_key
);
    // states
    parameter IDLE         =    10'b000000_0000;
    parameter TRANS_READ_SI =   10'b000001_0100;
    parameter READ_SI      =    10'b000010_0001;
    parameter WAIT_READ_SI =    10'b000011_0001;
    parameter CALC_J       =    10'b000100_0001;
    parameter CALC_J_2     =    10'b000101_0101;
    parameter READ_SJ      =    10'b000110_0001;
    parameter WAIT_READ_SJ =    10'b000111_0001;
    parameter TRANS_WRITE_SI =  10'b001000_1000;
    parameter WRITE_SI     =    10'b001001_0001;
    parameter WAIT_WRITE_SI =   10'b001010_0001;
    parameter TRANS_WRITE_SJ =  10'b001011_1000;
    parameter WRITE_SJ     =    10'b001111_0001;
    parameter WAIT_WRITE_SJ =   10'b010001_0001;
    parameter CHECK_DONE   =    10'b010010_0000;
    parameter FINISHED     =    10'b010011_0010;

    // Internal signals
    logic [9:0] state;
    logic [7:0] i_counter;          // Loop counter (0 to 255)
    logic [7:0] j_accumulator;      // j accumulator
    logic [7:0] s_i_value;          // Temp storage for s[i]
    logic [7:0] key_byte;           // Current key byte based on i mod 3

    // Outputs from state encoding
    assign wr_start = state[3];
    assign rd_start = state[2];
    assign finish = state[1];  // 1'b1 during FINISHED state
    assign busy = state[0];    // 1'b1 during all active states

    Key_Byte_Selector key_byte_selector_ksa (
        .secret_key(secret_key),
        .i_counter(i_counter),
        .key_byte(key_byte)
    );

    // Main Key_Schedule FSM
    always_ff @(posedge clk) begin
        if (~nreset) begin
            state <= IDLE;
            i_counter <= 8'd0;
            j_accumulator <= 8'd0;
            s_i_value <= 8'd0;
            addr_out <= 8'd0;
            wr_data_out <= 8'd0;
        end else begin
            case (state)
                IDLE: begin
                    if (start) begin
                        // Reset all counters
                        i_counter <= 8'd0;
                        j_accumulator <= 8'd0;
                        // Reset temporary storage
                        s_i_value <= 8'd0;
                        // Reset all interface signals
                        addr_out <= 8'd0;
                        wr_data_out <= 8'd0;
                        // Start first read operation
                        state <= TRANS_READ_SI;
                    end else begin
                        state <= IDLE;
                    end
                end
                TRANS_READ_SI: begin
                    // rd_start assigned 1'b1
                    state <= READ_SI;
                end

                READ_SI: begin
                    // rd_start assigned 1'b0
                    state <= WAIT_READ_SI;
                end

                WAIT_READ_SI: begin
                    if (rd_done) begin
                        // Capture s[i] value
                        s_i_value <= rd_data_in;
                        state <= CALC_J;
                    end
                end

                CALC_J: begin
                    // Calculate j = (j + s[i] + secret_key[i mod 3]) mod 256
                    // The mod 256 happens from the nature of a 8 bit adder
                    j_accumulator <=  (j_accumulator + s_i_value + key_byte);
                    state <= CALC_J_2;
                end

                CALC_J_2: begin
                    // rd_start is assigned 1'b1
                    addr_out <= j_accumulator;
                    state <= READ_SJ;
                end

                READ_SJ: begin
                    // rd_start is assigned 1'b0
                    state <= WAIT_READ_SJ;
                end

                WAIT_READ_SJ: begin
                    if (rd_done) begin
                        state <= TRANS_WRITE_SI;
                    end
                end

                TRANS_WRITE_SI: begin
                    // wr_start is assigned 1'b1
                     // Write s[j] value to address i
                    addr_out <= i_counter;
                    wr_data_out <= rd_data_in;  // s[j] value
                    state <= WRITE_SI;
                end

                WRITE_SI: begin
                    // wr_start is assigned 1'b0
                    state <= WAIT_WRITE_SI;
                end

                WAIT_WRITE_SI: begin
                    if (wr_done) begin
                        state <= TRANS_WRITE_SJ;
                    end
                end

                TRANS_WRITE_SJ: begin
                    // wr_start is assigned 1'b1
                    // Write s[i] value to address j
                    addr_out <= j_accumulator;
                    wr_data_out <= s_i_value;
                    state <= WRITE_SJ;
                end

                WRITE_SJ: begin
                    // wr_start is assigned 1'b0
                    state <= WAIT_WRITE_SJ;
                end

                WAIT_WRITE_SJ: begin
                    if (wr_done) begin
                        state <= CHECK_DONE;
                    end
                end

                CHECK_DONE: begin
                    if (i_counter == 8'd255) begin
                        // Finished all 256 iterations
                        state <= FINISHED;
                    end else begin
                        // Progress to next iteration
                        i_counter <= i_counter + 8'd1;
                        // Start reading s[i+1]
                        addr_out <= i_counter + 8'd1;
                        state <= TRANS_READ_SI;
                    end
                end

                FINISHED: begin
                    state <= IDLE;
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule