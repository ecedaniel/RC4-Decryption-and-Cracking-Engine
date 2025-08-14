// Key_Byte_Selector.sv - Module for selecting key bytes based on index modulo 3

module Key_Byte_Selector (
    input  logic [23:0] secret_key,
    input  logic [7:0]  i_counter, // counter variable
    output logic [7:0]  key_byte
);

    // secret_key[23:16] - first byte
    // secret_key[15:8] - second byte
    // secret_key[7:0] - third byte
    
    // secret_key[i mod keylength]
    // keylength == 3.

    always_comb begin
        case (i_counter % 3)
            8'd0: key_byte = secret_key[23:16];
            8'd1: key_byte = secret_key[15:8];
            8'd2: key_byte = secret_key[7:0];
            default: key_byte = 8'd0;
        endcase
    end

endmodule 