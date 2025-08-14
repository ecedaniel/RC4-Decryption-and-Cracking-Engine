module ksa (
    input  logic        CLOCK_50,           // Clock pin
    input  logic [3:0]  KEY,                // push button switches
    input  logic [9:0]  SW,                 // slider switches
    output logic [9:0]  LEDR,               // red lights
    output logic [6:0]  HEX0,
    output logic [6:0]  HEX1,
    output logic [6:0]  HEX2,
    output logic [6:0]  HEX3,
    output logic [6:0]  HEX4,
    output logic [6:0]  HEX5
);

    // FSM Selection Parameters
    parameter FSM_SAI = 2'b00;     // S_Array_Init
    parameter FSM_TEST = 2'b01;    // Test FSM
    parameter FSM_KSA = 2'b10;     // Key_Schedule
    parameter FSM_DECRYPT = 2'b11; // RC4_Decrypt

    // Clock and reset signals
    logic clk, reset_n;
    assign clk = CLOCK_50;
    assign reset_n = KEY[3];
    // Task 3: Use cracking key instead of switches
    assign secret_key = current_crack_key;

    // s_memory signals
    logic [7:0] s_address, s_q, s_data;    // Memory address, read data, write data
    logic s_wren;                          // Memory write enable
    
    // e_memory signals (ROM - encrypted message)
    logic [4:0] e_address;                 // 5-bit address for 32 bytes
    logic [7:0] e_q;                       // Read data from encrypted ROM
    
    // d_memory signals (RAM - decrypted message)
    logic [4:0] d_address;                 // 5-bit address for 32 bytes  
    logic [7:0] d_q, d_data;               // Read data, write data
    logic d_wren;                          // Write enable
    
    // E_Memory_Interface signals (for encrypted ROM)
    logic e_rd_start;                      // Read start signal to E_Memory_Interface
    logic [4:0] e_addr_in;                 // Address input to E_Memory_Interface
    logic [4:0] e_addr_mem;                // Address output from E_Memory_Interface
    logic e_rd_done;                       // Read done signal from E_Memory_Interface
    logic [7:0] e_rd_data_out;             // Read data output from E_Memory_Interface
    
    // D_Memory_Interface signals (for decrypted RAM)
    logic d_wr_start;                      // Write start signal to D_Memory_Interface
    logic [4:0] d_addr_in;                 // Address input to D_Memory_Interface
    logic [4:0] d_addr_mem;                // Address output from D_Memory_Interface
    logic [7:0] d_wr_data_in;              // Write data input to D_Memory_Interface
    logic d_wr_done;                       // Write done signal from D_Memory_Interface
    logic d_wr_enable;                     // Write enable output from D_Memory_Interface
    logic [7:0] d_wr_mem;                  // Write data output from D_Memory_Interface
    
    // Memory_Interface signals for s_memory - ONLY THESE SIGNALS MATTER:
    // Write interface - Important signals:
    logic wr_start;                        // wr_start - pulse high to start write operation
    logic [7:0] addr_in;                   // addr_in - address for read/write operations  
    logic [7:0] wr_data_in;                // wr_data_in - data to write to memory
    logic wr_done;                         // wr_done - high when write operation complete
    
    // Read interface - Important signals:
    logic rd_start;                        // rd_start - pulse high to start read operation
    logic rd_done;                         // rd_done - high when read operation complete
    logic [7:0] rd_data_out;               // rd_data_out - data read from memory
    
    // FSM Selection and Control
    logic [1:0] active_fsm;                // Current active FSM selector
    
    // S_Array_Init (SAI) interface signals
    logic SAI_wr_start;                    // Write start from S_Array_Init
    logic [7:0] SAI_addr_out;              // Address from S_Array_Init
    logic [7:0] SAI_wr_data_out;           // Write data from S_Array_Init
    logic SAI_rd_start;                    // Read start from S_Array_Init
    
    // Test FSM interface signals
    logic TEST_wr_start;                   // Write start from Test FSM (always 0)
    logic [7:0] TEST_addr_out;             // Address from Test FSM
    logic [7:0] TEST_wr_data_out;          // Write data from Test FSM (always 0)
    logic TEST_rd_start;                   // Read start from Test FSM
    
    // Key_Schedule interface signals
    logic KSA_wr_start;                    // Write start from Key_Schedule
    logic [7:0] KSA_addr_out;              // Address from Key_Schedule
    logic [7:0] KSA_wr_data_out;           // Write data from Key_Schedule
    logic KSA_rd_start;                    // Read start from Key_Schedule
    
    // RC4_Decrypt interface signals
    logic DECRYPT_wr_start;                // Write start from RC4_Decrypt (to s_memory)
    logic [7:0] DECRYPT_addr_out;          // Address from RC4_Decrypt (to s_memory)
    logic [7:0] DECRYPT_wr_data_out;       // Write data from RC4_Decrypt (to s_memory)
    logic DECRYPT_rd_start;                // Read start from RC4_Decrypt (to s_memory)
    
    // RC4_Decrypt to E_Memory_Interface signals
    logic DECRYPT_e_rd_start;              // Read start from RC4_Decrypt to E_Memory_Interface
    logic [4:0] DECRYPT_e_addr_out;        // Address from RC4_Decrypt to E_Memory_Interface
    
    // RC4_Decrypt to D_Memory_Interface signals
    logic DECRYPT_d_wr_start;              // Write start from RC4_Decrypt to D_Memory_Interface
    logic [4:0] DECRYPT_d_addr_out;        // Address from RC4_Decrypt to D_Memory_Interface
    logic [7:0] DECRYPT_d_wr_data_out;     // Write data from RC4_Decrypt to D_Memory_Interface
    
    // Memory_Interface internal signals (connected to s_memory)
    logic [7:0] addr_mem;                  // addr_mem - address output to memory
    logic [7:0] wr_mem;                    // wr_mem - data output to memory
    logic wr_enable;                       // wr_enable - write enable output to memory
    
    // S_Array_Init signals
    logic init_start, init_finish;         // Start and finish signals for S_Array_Init module
    
    // Key_Schedule signals
    logic ksa_start, ksa_finish;           // Start and finish signals for Key_Schedule module

    logic [23:0] secret_key;               // 24-bit secret key for RC4
    
    // RC4_Decrypt signals
    logic decrypt_start, decrypt_finish;   // Start and finish signals for RC4_Decrypt module

    logic invalid_message_flag;            // Invalid message flag from RC4_Decrypt module
    
    // Test_FSM signals
    logic test_start, test_finish;         // Start and finish signals for Test_FSM module
    logic test_busy;                       // Busy signal from Test_FSM module
    

    
    // Task 3 - Cracking FSM signals

    logic crack_success, crack_failure;
    logic [23:0] current_crack_key;
    logic [23:0] successful_key;        // Capture the successful key
    logic keyspace_exhausted;
    logic increment_key;
    logic crack_ack;
    logic test_message_valid, test_message_invalid;
    
    logic [2:0] crack_state;

    // Memory instantiation
    s_memory s_memory_ksa (
        .address(s_address),                // Memory address input (0-255)
        .clock(CLOCK_50),                   // System clock
        .data(s_data),                      // Data to write to memory
        .wren(s_wren),                      // Write enable signal - high to write
        .q(s_q)                             // Data output from memory (read data)
    );

    // Encrypted message ROM (32x8-bit, read-only)
    e_memory e_memory_ksa (
        .address(e_address),                // 5-bit address input (0-31)
        .clock(CLOCK_50),                   // System clock
        .q(e_q)                             // Data output from ROM (read data)
    );

    // Decrypted message RAM (32x8-bit, write-capable)
    d_memory d_memory_ksa (
        .address(d_address),                // 5-bit address input (0-31)
        .clock(CLOCK_50),                   // System clock
        .data(d_data),                      // Data to write to memory
        .wren(d_wren),                      // Write enable signal - high to write
        .q(d_q)                             // Data output from memory (read data)
    );

    // Memory Interface instantiation for s_memory - SIMPLIFIED INTERFACE
    Memory_Interface MI_ksa (
        .clk(CLOCK_50),                     // System clock
        .nreset(reset_n),                   // Active-low reset signal
        
        // General interface - IMPORTANT: addr_in used for both read and write
        .addr_in(addr_in),                  // addr_in - address input for operations
        .addr_mem(addr_mem),                // addr_mem - address output to memory
        
        // Write interface - IMPORTANT SIGNALS:
        .wr_start(wr_start),                // wr_start - pulse high to begin write
        .wr_data_in(wr_data_in),            // wr_data_in - data input for write
        .wr_done(wr_done),                  // wr_done - high when write complete
        .wr_enable(wr_enable),              // wr_enable - write enable to memory
        .wr_mem(wr_mem),                    // wr_mem - data output to memory
        
        // Read interface - IMPORTANT SIGNALS:
        .rd_start(rd_start),                // rd_start - pulse high to begin read
        .rd_mem(s_q),                       // rd_mem - data input from memory
        .rd_done(rd_done),                  // rd_done - high when read complete
        .rd_data_out(rd_data_out)           // rd_data_out - captured read data
    );
    
    // E_Memory_Interface instantiation for encrypted memory (ROM)
    E_Memory_Interface E_MI_ksa (
        .clk(CLOCK_50),                     // System clock
        .nreset(reset_n),                   // Active-low reset signal
        
        // General interface
        .addr_in(e_addr_in),                // addr_in - address input for read operations
        .addr_mem(e_addr_mem),              // addr_mem - address output to memory
        
        // Read interface
        .rd_start(e_rd_start),              // rd_start - pulse high to begin read
        .rd_mem(e_q),                       // rd_mem - data input from memory
        .rd_done(e_rd_done),                // rd_done - high when read complete
        .rd_data_out(e_rd_data_out)         // rd_data_out - captured read data
    );
    
    // D_Memory_Interface instantiation for decrypted memory (RAM)
    D_Memory_Interface D_MI_ksa (
        .clk(CLOCK_50),                     // System clock
        .nreset(reset_n),                   // Active-low reset signal
        
        // General interface
        .addr_in(d_addr_in),                // addr_in - address input for write operations
        .addr_mem(d_addr_mem),              // addr_mem - address output to memory
        
        // Write interface
        .wr_start(d_wr_start),              // wr_start - pulse high to begin write
        .wr_data_in(d_wr_data_in),          // wr_data_in - data input for write
        .wr_done(d_wr_done),                // wr_done - high when write complete
        .wr_enable(d_wr_enable),            // wr_enable - write enable to memory
        .wr_mem(d_wr_mem)                   // wr_mem - data output to memory
    );
    
    // S_Array_Init instantiation
    S_Array_Init S_Init (
        .clk(CLOCK_50),                     // System clock
        .nreset(reset_n),                   // Active-low reset signal
        .start(init_start),                 // Start initialization signal from test FSM
        .finish(init_finish),               // Finish signal when initialization complete
        .busy(),                            // Busy signal (not connected/used)
        // Write interface
        .wr_start(SAI_wr_start),            // Write start signal from S_Array_Init
        .addr_out(SAI_addr_out),            // Address from S_Array_Init
        .wr_data_out(SAI_wr_data_out),      // Write data from S_Array_Init
        .wr_done(wr_done),                  // Write done signal to S_Array_Init
        // Read interface (template - unused in this module)
        .rd_start(SAI_rd_start),            // Read start from S_Array_Init (tied to 0)
        .rd_done(rd_done),                  // Read done to S_Array_Init (unused)
        .rd_data_in(rd_data_out)            // Read data to S_Array_Init (unused)
    );
    
    // Key_Schedule instantiation
    Key_Schedule KSA_Module (
        .clk(CLOCK_50),                     // System clock
        .nreset(reset_n),                   // Active-low reset signal
        .start(ksa_start),                  // Start key scheduling signal from test FSM
        .finish(ksa_finish),                // Finish signal when key scheduling complete
        .busy(),                            // Busy signal from Key_Schedule (unused)
        // Write interface
        .wr_start(KSA_wr_start),            // Write start signal from Key_Schedule
        .addr_out(KSA_addr_out),            // Address from Key_Schedule
        .wr_data_out(KSA_wr_data_out),      // Write data from Key_Schedule
        .wr_done(wr_done),                  // Write done signal to Key_Schedule
        // Read interface
        .rd_start(KSA_rd_start),            // Read start from Key_Schedule
        .rd_done(rd_done),                  // Read done to Key_Schedule
        .rd_data_in(rd_data_out),           // Read data to Key_Schedule
        // Secret key
        .secret_key(secret_key)             // 24-bit secret key
    );
    
    // RC4_Decrypt signals

    
    // RC4_Decrypt instantiation
    RC4_Decrypt Decrypt_Module (
        .clk(CLOCK_50),                     // System clock
        .nreset(reset_n),                   // Active-low reset signal
        .start(decrypt_start),              // Start decryption signal from test FSM
        .finish(decrypt_finish),            // Finish signal when decryption complete
        .busy(),                            // Busy signal from RC4_Decrypt (unused)
        .message_valid(),                   // Message valid signal from RC4_Decrypt (unused)
        .invalid_message_flag(invalid_message_flag), // Invalid message flag signal from RC4_Decrypt
        // S-memory interface (via Memory_Interface)
        .wr_start(DECRYPT_wr_start),        // Write start signal from RC4_Decrypt
        .addr_out(DECRYPT_addr_out),        // Address from RC4_Decrypt
        .wr_data_out(DECRYPT_wr_data_out),  // Write data from RC4_Decrypt
        .wr_done(wr_done),                  // Write done signal to RC4_Decrypt
        .rd_start(DECRYPT_rd_start),        // Read start from RC4_Decrypt
        .rd_done(rd_done),                  // Read done to RC4_Decrypt
        .rd_data_in(rd_data_out),           // Read data to RC4_Decrypt
        // Encrypted message interface (via E_Memory_Interface)
        .e_rd_start(DECRYPT_e_rd_start),    // Read start to E_Memory_Interface
        .e_addr_out(DECRYPT_e_addr_out),    // Address to E_Memory_Interface
        .e_rd_done(e_rd_done),              // Read done from E_Memory_Interface
        .e_rd_data_in(e_rd_data_out),       // Read data from E_Memory_Interface
        // Decrypted message interface (via D_Memory_Interface)
        .d_wr_start(DECRYPT_d_wr_start),    // Write start to D_Memory_Interface
        .d_addr_out(DECRYPT_d_addr_out),    // Address to D_Memory_Interface
        .d_wr_data_out(DECRYPT_d_wr_data_out), // Write data to D_Memory_Interface
        .d_wr_done(d_wr_done)               // Write done from D_Memory_Interface
    );
    
    // Key_Generator instantiation
    Key_Generator Key_Gen (
        .clk(CLOCK_50),                     // System clock
        .nreset(reset_n),                   // Active-low reset signal
        .increment(increment_key),          // Increment key signal
        .current_key(current_crack_key),    // Current key being tested
        .keyspace_exhausted(keyspace_exhausted) // Keyspace exhausted flag
    );
    
    // Test_FSM debug signals
    logic [7:0] test_state_debug;           // Debug: Current test state
    
    // Test_FSM instantiation
    Test_FSM Test_Controller (
        .clk(CLOCK_50),                     // System clock
        .nreset(reset_n),                   // Active-low reset signal
        .start(test_start),                 // Start test sequence signal
        .finish(test_finish),               // Finish signal when test complete
        .busy(test_busy),                   // Busy signal from Test_FSM
        // S_Array_Init interface
        .init_start(init_start),            // Start signal to S_Array_Init
        .init_finish(init_finish),          // Finish signal from S_Array_Init
        // Key_Schedule interface
        .ksa_start(ksa_start),              // Start signal to Key_Schedule
        .ksa_finish(ksa_finish),            // Finish signal from Key_Schedule
        // RC4_Decrypt interface
        .decrypt_start(decrypt_start),      // Start signal to RC4_Decrypt
        .decrypt_finish(decrypt_finish),    // Finish signal from RC4_Decrypt
        .invalid_message_flag(invalid_message_flag), // Invalid message flag from RC4_Decrypt
        // FSM Selection Control
        .active_fsm(active_fsm),            // Active FSM selector output
        // Task 3 - Result communication
        .crack_ack(crack_ack),              // Acknowledgment from cracking FSM
        .test_message_valid(test_message_valid),     // Message was valid
        .test_message_invalid(test_message_invalid), // Message was invalid
        // Debug
        .test_state_debug(test_state_debug) // Current test state for debugging
    );
    
    // Connect E_Memory_Interface to e_memory
    assign e_address = e_addr_mem;          // Memory address from E_Memory_Interface
    assign e_addr_in = DECRYPT_e_addr_out;  // Address input from RC4_Decrypt
    assign e_rd_start = DECRYPT_e_rd_start; // Read start from RC4_Decrypt
    
    // Connect D_Memory_Interface to d_memory
    assign d_address = d_addr_mem;          // Memory address from D_Memory_Interface
    assign d_data = d_wr_mem;               // Memory write data from D_Memory_Interface
    assign d_wren = d_wr_enable;            // Memory write enable from D_Memory_Interface
    assign d_addr_in = DECRYPT_d_addr_out;  // Address input from RC4_Decrypt
    assign d_wr_start = DECRYPT_d_wr_start; // Write start from RC4_Decrypt
    assign d_wr_data_in = DECRYPT_d_wr_data_out; // Write data from RC4_Decrypt
    
    // Test FSM signal assignments (these represent the test FSM's outputs)
    assign TEST_wr_start = 1'b0;            // Test FSM never writes
    assign TEST_addr_out = 8'b0;            // Test FSM doesn't use memory addresses
    assign TEST_wr_data_out = 8'b0;         // Test FSM never writes data
    assign TEST_rd_start = 1'b0;            // Test FSM doesn't directly read memory
    
    // FSM Multiplexer - Controls which FSM drives the Memory_Interface
    // This can be easily extended by adding more cases
    always_comb begin
        case (active_fsm)
            FSM_SAI: begin
                // S_Array_Init controls Memory_Interface
                wr_start = SAI_wr_start;
                addr_in = SAI_addr_out;
                wr_data_in = SAI_wr_data_out;
                rd_start = SAI_rd_start;
            end
            FSM_TEST: begin
                // Test FSM controls Memory_Interface
                wr_start = TEST_wr_start;
                addr_in = TEST_addr_out;
                wr_data_in = TEST_wr_data_out;
                rd_start = TEST_rd_start;
            end
            FSM_KSA: begin
                // Key_Schedule controls Memory_Interface
                wr_start = KSA_wr_start;
                addr_in = KSA_addr_out;
                wr_data_in = KSA_wr_data_out;
                rd_start = KSA_rd_start;
            end
            FSM_DECRYPT: begin
                // RC4_Decrypt controls Memory_Interface (for s_memory access)
                wr_start = DECRYPT_wr_start;
                addr_in = DECRYPT_addr_out;
                wr_data_in = DECRYPT_wr_data_out;
                rd_start = DECRYPT_rd_start;
            end
            
            default: begin
                // Default to 0's
                wr_start = 1'd0;
                addr_in = 8'b0;
                wr_data_in = 8'b0;
                rd_start = 1'd0;
            end
        endcase
    end
    
    // Connect Memory Interface to s_memory
    // Protocol: Memory_Interface handles the memory control internally
    // We just connect its outputs to the memory inputs
    assign s_address = addr_mem;           // Memory address from Memory_Interface
    assign s_data = wr_mem;                // Memory write data from Memory_Interface
    assign s_wren = wr_enable;             // Memory write enable from Memory_Interface
    

    
    // Crack_FSM instantiation
    Crack_FSM Crack_Controller (
        .clk(CLOCK_50),                     // System clock
        .nreset(reset_n),                   // Active-low reset signal
        .test_finish(test_finish),          // Test sequence completion signal
        .test_message_valid(test_message_valid),     // Valid message found signal
        .test_message_invalid(test_message_invalid), // Invalid message signal
        .keyspace_exhausted(keyspace_exhausted),     // Key generator exhausted signal
        .current_crack_key(current_crack_key),       // Current key being tested
        .test_start(test_start),            // Start test sequence signal
        .crack_ack(crack_ack),              // Acknowledgment signal
        .increment_key(increment_key),      // Increment key signal
        .crack_success(crack_success),      // Cracking success flag
        .crack_failure(crack_failure),      // Cracking failure flag
        .successful_key(successful_key),    // Successful key output
        .crack_state(crack_state)           // Current crack state (for debug)
    );
    
    // Task 3 - Status indicators for cracking with detailed debug
    assign LEDR[9] = crack_success;                              // SUCCESS: Valid message found
    assign LEDR[8] = crack_failure;                              // FAILURE: No valid key found  
    assign LEDR[7] = init_finish;                                // Debug: S_Array_Init finished
    assign LEDR[6] = ksa_finish;                                 // Debug: Key_Schedule finished  
    assign LEDR[5] = decrypt_finish;                             // Debug: RC4_Decrypt finished
    assign LEDR[4] = test_finish;                                // Debug: Test FSM finished
    assign LEDR[3] = test_busy;                                  // Debug: Test FSM busy
    assign LEDR[2:0] = crack_state;                              // Debug: Current cracking state
    
    // Task 3 - HEX displays show current key being tested (24-bit key on 6 displays)
    // Use successful_key when in success state, otherwise current_crack_key
    logic [23:0] display_key;
    assign display_key = crack_success ? successful_key : current_crack_key;
    
    // HEX0 - Bits 3:0 of display key using SevenSegmentDisplayDecoder
    SevenSegmentDisplayDecoder hex0_decoder(.ssOut(HEX0), .nIn(display_key[3:0]));
    
    // HEX1 - Bits 7:4 of display key using SevenSegmentDisplayDecoder            
    SevenSegmentDisplayDecoder hex1_decoder(.ssOut(HEX1), .nIn(display_key[7:4]));
    
    // HEX2 - Bits 11:8 of display key using SevenSegmentDisplayDecoder
    SevenSegmentDisplayDecoder hex2_decoder(.ssOut(HEX2), .nIn(display_key[11:8]));
    
    // HEX3 - Bits 15:12 of display key using SevenSegmentDisplayDecoder
    SevenSegmentDisplayDecoder hex3_decoder(.ssOut(HEX3), .nIn(display_key[15:12]));
    
    // HEX4 - Bits 19:16 of display key using SevenSegmentDisplayDecoder
    SevenSegmentDisplayDecoder hex4_decoder(.ssOut(HEX4), .nIn(display_key[19:16]));
    
    // HEX5 - Bits 23:20 of display key using SevenSegmentDisplayDecoder
    SevenSegmentDisplayDecoder hex5_decoder(.ssOut(HEX5), .nIn(display_key[23:20]));

endmodule