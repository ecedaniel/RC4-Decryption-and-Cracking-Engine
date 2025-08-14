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
    assign secret_key = {14'b0, SW[9:0]};  // Key from switches: upper 14 bits = 0, lower 10 bits = SW[9:0]

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
    logic ksa_busy;                        // Busy signal from Key_Schedule module
    logic [23:0] secret_key;               // 24-bit secret key for RC4
    
    // RC4_Decrypt signals
    logic decrypt_start, decrypt_finish;   // Start and finish signals for RC4_Decrypt module
    logic decrypt_busy;                    // Busy signal from RC4_Decrypt module
    
    // Test state machine parameters - SIMPLIFIED SEQUENTIAL ENCODING
    parameter TEST_IDLE         = 8'd0;  // Idle state 
    parameter TEST_START_INIT   = 8'd1;  // Starting initialization state
    parameter TEST_WAIT_INIT    = 8'd2;  // Waiting for initialization to complete
    parameter TEST_WAIT_KEY1    = 8'd3;  // Waiting for KEY[1] press before KSA
    parameter TEST_START_KSA    = 8'd4;  // Starting key scheduling state
    parameter TEST_WAIT_KSA     = 8'd5;  // Waiting for key scheduling to complete
    parameter TEST_WAIT_KEY2    = 8'd6;  // Waiting for KEY[2] press before Decrypt
    parameter TEST_START_DECRYPT = 8'd7; // Starting decryption state
    parameter TEST_WAIT_DECRYPT = 8'd8;  // Waiting for decryption to complete
    parameter TEST_START_READ   = 8'd9;  // Starting first read after decryption
    parameter TEST_WAIT_READ    = 8'd10; // Waiting for read to complete
    parameter TEST_COMPLETE     = 8'd11; // Complete state
    
    logic [7:0] test_state;                // Current state of test FSM
    logic [7:0] read_addr;                 // Current address for reading memory
    logic [7:0] current_data;              // Current data read from memory
    logic key1_press_detected;            // Latched KEY[1] press detection
    logic key2_press_detected;            // Latched KEY[2] press detection
    
    // Key synchronization and edge detection with longer hold detection
    // KEY[1] synchronization
    logic key1_sync1, key1_sync2, key1_pressed;
    logic [7:0] key1_hold_counter;
    logic key1_held_long;
    
    // KEY[2] synchronization
    logic key2_sync1, key2_sync2, key2_pressed;
    logic [7:0] key2_hold_counter;
    logic key2_held_long;
    
    // Synchronize KEY[1] and detect press (active low)
    always_ff @(posedge clk) begin
        if (~reset_n) begin
            key1_sync1 <= 1'b1;
            key1_sync2 <= 1'b1;
            key1_hold_counter <= 8'b0;
            key1_held_long <= 1'b0;
        end else begin
            key1_sync1 <= KEY[1];
            key1_sync2 <= key1_sync1;
            
            // Count how long KEY[1] is held low (pressed)
            if (~key1_sync2) begin  // KEY[1] is pressed (active low)
                if (key1_hold_counter < 8'hFF) begin
                    key1_hold_counter <= key1_hold_counter + 1'b1;
                end
                if (key1_hold_counter >= 8'd10) begin  // Held for at least 10 cycles
                    key1_held_long <= 1'b1;
                end
            end else begin
                key1_hold_counter <= 8'b0;
                key1_held_long <= 1'b0;
            end
        end
    end
    
    // Synchronize KEY[2] and detect press (active low)
    always_ff @(posedge clk) begin
        if (~reset_n) begin
            key2_sync1 <= 1'b1;
            key2_sync2 <= 1'b1;
            key2_hold_counter <= 8'b0;
            key2_held_long <= 1'b0;
        end else begin
            key2_sync1 <= KEY[2];
            key2_sync2 <= key2_sync1;
            
            // Count how long KEY[2] is held low (pressed)
            if (~key2_sync2) begin  // KEY[2] is pressed (active low)
                if (key2_hold_counter < 8'hFF) begin
                    key2_hold_counter <= key2_hold_counter + 1'b1;
                end
                if (key2_hold_counter >= 8'd10) begin  // Held for at least 10 cycles
                    key2_held_long <= 1'b1;
                end
            end else begin
                key2_hold_counter <= 8'b0;
                key2_held_long <= 1'b0;
            end
        end
    end
    
    // Detect falling edge or long hold for both keys
    assign key1_pressed = (key1_sync2 & ~key1_sync1) | key1_held_long;
    assign key2_pressed = (key2_sync2 & ~key2_sync1) | key2_held_long;

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
        .busy(ksa_busy),                    // Busy signal from Key_Schedule
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
    
    // RC4_Decrypt instantiation
    RC4_Decrypt Decrypt_Module (
        .clk(CLOCK_50),                     // System clock
        .nreset(reset_n),                   // Active-low reset signal
        .start(decrypt_start),              // Start decryption signal from test FSM
        .finish(decrypt_finish),            // Finish signal when decryption complete
        .busy(decrypt_busy),                // Busy signal from RC4_Decrypt
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
    assign TEST_addr_out = read_addr;       // Current read address
    assign TEST_wr_data_out = 8'b0;         // Test FSM never writes data
    assign TEST_rd_start = (test_state == TEST_START_READ) ? 1'b1 : 1'b0;
    
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
                // Default to SAI for safety
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
    
    // Test state machine - starts initialization, then key scheduling, then decryption
    // Protocol: Set address first, then send start signal, wait for done
    // Controls active_fsm to switch between different FSMs
    always_ff @(posedge clk) begin
        if (~reset_n) begin
            test_state <= TEST_IDLE;
            init_start <= 1'b0;
            ksa_start <= 1'b0;
            decrypt_start <= 1'b0;
            read_addr <= 8'b0;
            current_data <= 8'b0;
            key1_press_detected <= 1'b0;
            key2_press_detected <= 1'b0;
            active_fsm <= FSM_SAI;  // Start with S_Array_Init
        end else begin
            // Latch key press detection
            if (key1_pressed && (test_state == TEST_WAIT_KEY1)) begin
                key1_press_detected <= 1'b1;
            end
            if (key2_pressed && (test_state == TEST_WAIT_KEY2)) begin
                key2_press_detected <= 1'b1;
            end
            
            case (test_state)
                TEST_IDLE: begin
                    test_state <= TEST_START_INIT;
                    init_start <= 1'b1;
                    active_fsm <= FSM_SAI;  // S_Array_Init is active
                end
                
                TEST_START_INIT: begin
                    init_start <= 1'b0;
                    test_state <= TEST_WAIT_INIT;
                    active_fsm <= FSM_SAI;  // S_Array_Init remains active
                end
                
                TEST_WAIT_INIT: begin
                    if (init_finish) begin
                        // Initialization complete, wait for KEY[1] press
                        test_state <= TEST_WAIT_KEY1;
                        active_fsm <= FSM_SAI;  // Keep SAI active (no memory operations)
                    end else begin
                        active_fsm <= FSM_SAI;  // S_Array_Init remains active
                    end
                end
                
                TEST_WAIT_KEY1: begin
                    // Wait for KEY[1] press before starting KSA
                    if (key1_press_detected) begin
                        test_state <= TEST_START_KSA;
                        ksa_start <= 1'b1;
                        key1_press_detected <= 1'b0;  // Clear the latch
                        active_fsm <= FSM_KSA;  // Switch to Key_Schedule FSM
                    end else begin
                        active_fsm <= FSM_SAI;  // Keep SAI active (no operations)
                    end
                end
                
                TEST_START_KSA: begin
                    ksa_start <= 1'b0;
                    test_state <= TEST_WAIT_KSA;
                    active_fsm <= FSM_KSA;  // Key_Schedule remains active
                end
                
                TEST_WAIT_KSA: begin
                    if (ksa_finish) begin
                        // Key scheduling complete, wait for KEY[2] press
                        test_state <= TEST_WAIT_KEY2;
                        active_fsm <= FSM_SAI;  // Switch to idle state
                    end else begin
                        active_fsm <= FSM_KSA;  // Key_Schedule remains active
                    end
                end
                
                TEST_WAIT_KEY2: begin
                    // Wait for KEY[2] press before starting Decryption
                    if (key2_press_detected) begin
                        test_state <= TEST_START_DECRYPT;
                        decrypt_start <= 1'b1;
                        key2_press_detected <= 1'b0;  // Clear the latch
                        active_fsm <= FSM_DECRYPT;  // Switch to Decryption FSM
                    end else begin
                        active_fsm <= FSM_SAI;  // Keep SAI active (no operations)
                    end
                end
                
                TEST_START_DECRYPT: begin
                    decrypt_start <= 1'b0;
                    test_state <= TEST_WAIT_DECRYPT;
                    active_fsm <= FSM_DECRYPT;  // RC4_Decrypt remains active
                end
                
                TEST_WAIT_DECRYPT: begin
                    if (decrypt_finish) begin
                        // Decryption complete, switch to test FSM for reading
                        test_state <= TEST_START_READ;
                        active_fsm <= FSM_TEST;  // Switch to Test FSM for reading
                    end else begin
                        active_fsm <= FSM_DECRYPT;  // RC4_Decrypt remains active
                    end
                end

                TEST_START_READ: begin
                    // This state can be used later for verification reading
                    test_state <= TEST_COMPLETE;
                    active_fsm <= FSM_TEST;
                end
                
                TEST_COMPLETE: begin
                    // All operations complete
                    active_fsm <= FSM_SAI;  // Default to SAI
                end

                default: begin
                    // Don't change state in default case - hold current state for debugging
                    // This prevents unexpected resets
                    active_fsm <= FSM_SAI;  // Default to SAI
                end
            endcase
        end
    end
	
	
   SevenSegmentDisplayDecoder seg0 (.ssOut(HEX0), .nIn(secret_key[3:0]));
	SevenSegmentDisplayDecoder seg1 (.ssOut(HEX1), .nIn(secret_key[7:4]));
	SevenSegmentDisplayDecoder seg2 (.ssOut(HEX2), .nIn(secret_key[11:8]));
	SevenSegmentDisplayDecoder seg3 (.ssOut(HEX3), .nIn(secret_key[15:12]));
	SevenSegmentDisplayDecoder seg4 (.ssOut(HEX4), .nIn(secret_key[19:16]));
	SevenSegmentDisplayDecoder seg5 (.ssOut(HEX5), .nIn(secret_key[23:20]));

endmodule