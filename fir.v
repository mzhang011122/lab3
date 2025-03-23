/*
FIR Filter Module with AXI4-Lite Control and AXI-Stream Data Interface
Implements a configurable FIR filter with:
- Coefficient programming via AXI4-Lite
- Data streaming via AXI-Stream
- BRAM storage for coefficients and data samples
- Configurable data/tap widths and number of taps
*/

module fir #(
    parameter pADDR_WIDTH = 12,       // Address bus width for BRAM interfaces
    parameter pDATA_WIDTH = 32,       // Data bus width for all interfaces
    parameter Tape_Num    = 11        // Number of filter coefficients (taps)
) (
    // [AXI4-Lite Interface Signals Description]
    // These signals handle configuration and control register access
    // Used for setting coefficients, data length, and control signals
    output  wire                     awready,     // Write address channel ready
    output  wire                     wready,      // Write data channel ready
    input   wire                     awvalid,     // Write address valid
    input   wire [(pADDR_WIDTH-1):0] awaddr,      // Write address
    input   wire                     wvalid,      // Write data valid
    input   wire [(pDATA_WIDTH-1):0] wdata,       // Write data
    output  wire                     arready,     // Read address channel ready
    input   wire                     rready,      // Read data channel ready
    input   wire                     arvalid,     // Read address valid
    input   wire [(pADDR_WIDTH-1):0] araddr,      // Read address
    output  wire                     rvalid,      // Read data valid
    output  wire [(pDATA_WIDTH-1):0] rdata,       // Read data

    // [AXI-Stream Interface Signals Description]
    // These signals handle the data streaming interface
    // ss = slave stream (input), sm = master stream (output)
    input   wire                     ss_tvalid,   // Input data valid
    input   wire [(pDATA_WIDTH-1):0] ss_tdata,    // Input data
    input   wire                     ss_tlast,    // Input data last flag
    output  wire                     ss_tready,   // Input data ready
    input   wire                     sm_tready,   // Output data ready
    output  wire                     sm_tvalid,   // Output data valid
    output  wire [(pDATA_WIDTH-1):0] sm_tdata,    // Output data
    output  wire                     sm_tlast,    // Output data last flag

    // [BRAM Interface for Coefficient Storage]
    // Interface for coefficient memory (taps)
    output  reg [3:0]               tap_WE,      // Tap write enable (4 bits for 32-bit data)
    output  wire                     tap_EN,      // Tap memory enable
    output  wire [(pDATA_WIDTH-1):0] tap_Di,      // Tap data input
    output  wire [(pADDR_WIDTH-1):0] tap_A,       // Tap address
    input   wire [(pDATA_WIDTH-1):0] tap_Do,      // Tap data output

    // [BRAM Interface for Data Sample Storage]
    // Interface for data sample memory
    output  reg [3:0]               data_WE,     // Data write enable
    output  wire                     data_EN,     // Data memory enable
    output  wire [(pDATA_WIDTH-1):0] data_Di,     // Data input
    output  wire [(pADDR_WIDTH-1):0] data_A,      // Data address
    input   wire [(pDATA_WIDTH-1):0] data_Do,     // Data output

    // [Clock and Reset]
    input   wire                     axis_clk,    // System clock
    input   wire                     axis_rst_n   // Active-low reset
);

// [Register Address Mapping]
// Memory map for AXI4-Lite interface
localparam
    ADDR_AP_CTRL = 12'h00,          // Control register address
    ADDR_datalength = 12'h10,       // Data length register address
    ADDR_tapparameters = 12'hFF;    // Base address for tap coefficients

// [Internal Signal Declarations]
// Control signals for AXI4-Lite interface
reg aw_ready;                       // Internal write address ready
reg w_ready;                        // Internal write data ready
reg ar_ready;                       // Internal read address ready
reg [pDATA_WIDTH-1:0] r_data;       // Internal read data buffer
reg [pADDR_WIDTH-1:0] aw_addr;      // Buffered write address
reg r_valid;                        // Read data valid flag

// BRAM control signals
reg tapen;                          // Tap memory enable
reg [pADDR_WIDTH-1:0] tapa;         // Tap address
reg [3:0] datawe;                   // Data write enable
reg dataen;                         // Data memory enable
reg [pADDR_WIDTH-1:0] dataa;        // Data address

// Filter computation signals
reg smtvalid;                       // Master stream valid
reg smtlast;                        // Master stream last flag
reg [pDATA_WIDTH-1:0] ymult;        // Multiplier result
reg [pDATA_WIDTH-1:0] yout;         // Accumulator output
reg sstready = 1'b0;                // Slave stream ready

// [Signal Assignments]
// Connecting internal registers to output ports
assign ss_tready = sstready;
assign sm_tlast = smtlast;
assign sm_tdata = yout;
assign sm_tvalid = smtvalid;
assign tap_EN = tapen;
assign tap_Di = wdata;
assign tap_A = tapa;
assign data_EN = dataen;
assign data_Di = ss_tdata & {32{sstready}};
assign data_A = dataa;
assign awready = aw_ready;
assign wready = w_ready;
assign arready = ar_ready;
assign rvalid = r_valid;
assign rdata = r_data;

// [BRAM Address Management]
// Manages circular buffer addressing for data samples using a state machine
// Implements wrap-around behavior at address 0x28 (40 bytes = 10 samples * 4 bytes)
reg bramstate;

// [Stream Control State Machine]
// Handles data packet boundaries and filter operation states
// Manages tlast signal propagation and operation status flags
reg last;
reg int_rd_done;
reg int_rd_idle;
reg firstate;
reg int_rd_start;

// [BRAM Write Enable Generation]
// Generates write enables for coefficient and data BRAMs
// Combines valid signals from AXI interfaces and internal state
reg data_we_en;
reg tap_we_en;
always @(*) begin
    data_we_en = (sstready && ss_tvalid) || ~bramstate;
    tap_we_en  = awvalid && wvalid && awaddr[7];
    data_WE = {4{data_we_en}};
    tap_WE  = {4{tap_we_en}};
end

// [Stream Ready Control]
// Manages the slave stream ready signal and filter initialization
// Controls when the filter is ready to accept new input data
reg [pADDR_WIDTH-1:0] data_addr = 12'h20;
always @(posedge axis_clk) begin
    if (axis_rst_n == 1'b0) begin
        sstready <= 1'b0;
        firstate <= 1'b0;
    end 
    else if (int_rd_start) begin
        sstready <= 1'b1;
        firstate <= 1'b1;
    end 
end
// [Data Length Register]
// Stores the number of samples to process
// Set via AXI4-Lite interface before filter operation
reg [pDATA_WIDTH-1:0] data_length;
// [AXI4-Lite Read Interface]
// Handles read transactions for status registers and coefficient memory
// Multiplexes between control registers and coefficient memory access
always @(*) begin
    case (araddr)
        ADDR_AP_CTRL: begin
            // Control register mapping
            r_data[0] <= int_rd_start;   // Start bit
            r_data[1] <= int_rd_done;    // Done status
            r_data[2] <= int_rd_idle;    // Idle status
        end
        ADDR_datalength: begin
            r_data <= data_length;       // Data length register
        end
        default: begin
            r_data <= tap_Do;            // Coefficient memory read
        end
    endcase
end

// [Read Transaction Management]
// Manages read valid and ready signals
// Implements basic handshaking protocol for AXI4-Lite reads
always @(posedge axis_clk) begin
    if (!axis_rst_n) begin
        {r_valid, ar_ready} <= 0;
    end else if (!r_valid && arvalid) begin
        r_valid <= 1;
    end else if (rready && r_valid) begin
        r_valid <= 0;
    end
end

// [BRAM Enable Control]
// Always enable BRAMs during operation
// Simplified control assuming always-active memory access
always @(posedge axis_clk) begin
    if (axis_rst_n == 1'b0) begin
        tapen <= 1'b1;
        dataen <= 1'b1;
    end
end

// [Address Multiplexing]
// Selects between write and read addresses for coefficient BRAM
// Prioritizes write operations over read operations
always @(*) begin
    if (awvalid == 1'b1 && awaddr[7] == 1'b1) begin
        tapa <= awaddr[6:0];           // Use write address during writes
    end
    else if (arvalid == 1'b1 && araddr[7] == 1'b1) begin
        tapa <= araddr[6:0];          // Use read address during reads
    end
end




// [AXI4-Lite Write Interface]
// Handles write transactions to control registers
// Processes filter start commands and data length configuration
always @(posedge axis_clk) begin
    if (!axis_rst_n) begin
        last <= 0;
        smtlast <= 0;
        int_rd_done <= 0;
    end else begin
        // Manage last signal propagation through the pipeline
        smtlast <= (last && smtvalid) ? 1 : (smtlast && smtvalid) ? 0 : smtlast;
        last <= (ss_tlast && sstready) ? 1 : (smtlast && smtvalid) ? 0 : last;
        
        // Update operation status flags
        if (smtlast && smtvalid) begin
            int_rd_done <= 1;
            int_rd_idle <= 1;
            firstate <= 0;
        end
    end
    if (axis_rst_n == 1'b0) begin
        data_length <= 0;
        w_ready <= 1'b1;
        aw_ready <= 1'b1;
        int_rd_idle <= 1'b0;
    end 
    else if (awvalid && wvalid) begin
        case (awaddr)
            ADDR_AP_CTRL: begin
                if (wdata == 32'h0000_0001)
                    int_rd_start <= 1'b1;  // Start filter operation
                int_rd_idle <= 1'b0;       // Clear idle status
            end
            ADDR_datalength: begin
                data_length <= wdata;      // Update data length
            end
        endcase
    end
end

// [FIR Computation Core]
// Implements the multiply-accumulate (MAC) operations
// Controls the filter datapath including coefficient/data addressing
// and pipeline timing management
reg smstate;
always @(posedge axis_clk) begin
    if (axis_rst_n == 1'b0) begin
        smtvalid <= 1'b0;
        yout <= 32'h0;
        ymult <= 32'h0;
        smstate <= 1'b0;
        bramstate <= 0;
        dataa <= 0;
    end else begin
        if (!bramstate) begin
            // Circular buffer implementation for data samples
            dataa <= (dataa == 12'h28) ? 0 : dataa + 4;
            bramstate <= (dataa == 12'h28);
        end
    end 
        if (int_rd_start)begin
            tapa <= 12'h00;
            dataa <= 12'h00;
            int_rd_start <= 1'b0;
            end
        if (firstate) begin
            case (tapa)
                12'h00: begin
                    // Initial MAC operation
                    ymult <= tap_Do * data_Do;
                    dataa <= (dataa == 12'h28) ? 12'h00 : dataa + 12'h04;
                    tapa <= tapa + 12'h04;
                    sstready <= 1'b0;
                    if (smstate) begin
                        smtvalid <= 1'b1;
                    end else begin
                        smstate <= 1'b1;
                    end
                    yout <= ymult + yout;
                end
                12'h04: begin
                    // Subsequent MAC operations
                    yout <= ymult;
                    dataa <= (dataa == 12'h28) ? 12'h00 : dataa + 12'h04;
                    ymult <= tap_Do * data_Do;
                    tapa <= tapa + 12'h04;
                    smtvalid <= 1'b0;
                end
                12'h28: begin
                    // Final MAC operation in sequence
                    ymult <= tap_Do * data_Do;
                    yout <= ymult + yout;
                    tapa <= 12'h00;
                    if (~last)
                        sstready <= 1'b1;
                end
                default: begin
                    // Regular MAC operations
                    ymult <= tap_Do * data_Do;
                    yout <= ymult + yout;
                    dataa <= (dataa == 12'h28) ? 12'h00 : dataa + 12'h04;
                    tapa <= tapa + 12'h04;
                end
            endcase
    end
end

endmodule