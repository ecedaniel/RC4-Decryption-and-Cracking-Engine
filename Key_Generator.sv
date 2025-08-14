module Key_Generator (
    input  logic        clk,
    input  logic        nreset,
    input  logic        increment,          // Pulse to increment to next key
    output logic [23:0] current_key,        // Current key being tested
    output logic        keyspace_exhausted  // High when all keys tested
);

    parameter KEY_START = 22'h000000;
    parameter KEY_END = 22'h3FFFFF;

    logic [21:0] key_counter;
    
    assign current_key = {2'b00, key_counter};
    assign keyspace_exhausted = (key_counter == KEY_END);
    
    always_ff @(posedge clk) begin
        if (~nreset) begin
            key_counter <= KEY_START;
        end else if (increment && ~keyspace_exhausted) begin
            key_counter <= key_counter + 22'd1;
        end
    end

endmodule 