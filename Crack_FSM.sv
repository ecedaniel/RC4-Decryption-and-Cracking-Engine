module Crack_FSM (
    input  logic        clk,                    // System clock
    input  logic        nreset,                 // Active-low reset signal
    input  logic        test_finish,            // Test sequence completion signal
    input  logic        test_message_valid,     // Valid message found signal
    input  logic        test_message_invalid,   // Invalid message signal
    input  logic        keyspace_exhausted,     // Key generator exhausted signal
    input  logic [23:0] current_crack_key,      // Current key being tested
    
    output logic        test_start,             // Start test sequence signal
    output logic        crack_ack,              // Acknowledgment signal
    output logic        increment_key,          // Increment key signal
    output logic        crack_success,          // Cracking success flag
    output logic        crack_failure,          // Cracking failure flag
    output logic [23:0] successful_key,         // Successful key output
    output logic [2:0]  crack_state             // Current crack state (for debug)
);

    // Cracking FSM states
    parameter CRACK_IDLE = 3'b000;
    parameter CRACK_START_PIPELINE = 3'b001;
    parameter CRACK_WAIT_PIPELINE = 3'b010; 
    parameter CRACK_CHECK_RESULT = 3'b011;
    parameter CRACK_SUCCESS = 3'b100;
    parameter CRACK_NEXT_KEY = 3'b101;
    parameter CRACK_FAILURE = 3'b110;

    // Task 3 - Cracking FSM Controller
    // Coordinates key iteration and test FSM execution
    always_ff @(posedge clk) begin
        if (~nreset) begin
            crack_state <= CRACK_IDLE;
            test_start <= 1'b0;
            crack_ack <= 1'b0;
            increment_key <= 1'b0;
            crack_success <= 1'b0;
            crack_failure <= 1'b0;
            successful_key <= 24'h000000;
        end else begin
            case (crack_state)
                CRACK_IDLE: begin
                    // Start cracking automatically after reset
                    crack_state <= CRACK_START_PIPELINE;
                    test_start <= 1'b1;
                    crack_ack <= 1'b0;
                    increment_key <= 1'b0;
                end
                
                CRACK_START_PIPELINE: begin
                    test_start <= 1'b0;
                    crack_state <= CRACK_WAIT_PIPELINE;
                end
                
                CRACK_WAIT_PIPELINE: begin
                    if (test_finish) begin
                        crack_state <= CRACK_CHECK_RESULT;
                    end
                end
                
                CRACK_CHECK_RESULT: begin
                    if (test_message_valid) begin
                        // Valid message found - success!
                        successful_key <= current_crack_key;  // Capture the winning key
                        crack_state <= CRACK_SUCCESS;
                        crack_success <= 1'b1;
                        crack_ack <= 1'b1;
                    end else if (test_message_invalid) begin
                        // Invalid message - try next key
                        if (keyspace_exhausted) begin
                            crack_state <= CRACK_FAILURE;
                            crack_failure <= 1'b1;
                            crack_ack <= 1'b1;
                        end else begin
                            crack_state <= CRACK_NEXT_KEY;
                            crack_ack <= 1'b1;
                        end
                    end
                end
                
                CRACK_SUCCESS: begin
                    // Stay in success state - display result
                    // Wait for test FSM to acknowledge before clearing crack_ack
                    if (!test_message_valid) begin
                        crack_ack <= 1'b0;
                    end
                end
                
                CRACK_NEXT_KEY: begin
                    // Wait for test FSM to acknowledge before incrementing key
                    if (!test_message_invalid) begin
                        // Test FSM has acknowledged, increment key and start next test
                        increment_key <= 1'b1;
                        crack_ack <= 1'b0;
                        crack_state <= CRACK_START_PIPELINE;
                        test_start <= 1'b1;
                    end
                end
                
                CRACK_FAILURE: begin
                    // Stay in failure state - no valid key found
                    // Wait for test FSM to acknowledge before clearing crack_ack
                    if (!test_message_invalid) begin
                        crack_ack <= 1'b0;
                    end
                end
                
                default: begin
                    crack_state <= CRACK_IDLE;
                end
            endcase
            
            // Reset increment_key after one cycle
            if (increment_key) begin
                increment_key <= 1'b0;
            end
        end
    end

endmodule 