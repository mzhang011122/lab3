module fir #(
    parameter pADDR_WIDTH = 12,       // Parameter for address width
    parameter pDATA_WIDTH = 32,       // Parameter for data width
    parameter Tape_Num    = 11        // Parameter for the number of taps
)
(
    // AXI4-Lite Interface signals
    output  wire                     awready,     // Write address ready
    output  wire                     wready,      // Write data ready
    input   wire                     awvalid,     // Write address valid
    input   wire [(pADDR_WIDTH-1):0] awaddr,      // Write address
    input   wire                     wvalid,      // Write data valid
    input   wire [(pDATA_WIDTH-1):0] wdata,       // Write data
    output  wire                     arready,     // Read address ready
    input   wire                     rready,      // Read data ready
    input   wire                     arvalid,     // Read address valid
    input   wire [(pADDR_WIDTH-1):0] araddr,      // Read address
    output  wire                     rvalid,      // Read data valid
    output  wire [(pDATA_WIDTH-1):0] rdata,       // Read data    

    // AXI-Stream Interface signals
    input   wire                     ss_tvalid,   // Slave stream valid
    input   wire [(pDATA_WIDTH-1):0] ss_tdata,    // Slave stream data
    input   wire                     ss_tlast,    // Slave stream last
    output  wire                     ss_tready,   // Slave stream ready
    input   wire                     sm_tready,   // Master stream ready
    output  wire                     sm_tvalid,   // Master stream valid
    output  wire [(pDATA_WIDTH-1):0] sm_tdata,    // Master stream data
    output  wire                     sm_tlast,    // Master stream last
    
    // BRAM Interface for tap RAM
    output  wire [3:0]               tap_WE,      // Write enable for tap RAM
    output  wire                     tap_EN,      // Enable signal for tap RAM
    output  wire [(pDATA_WIDTH-1):0] tap_Di,      // Data input for tap RAM
    output  wire [(pADDR_WIDTH-1):0] tap_A,       // Address for tap RAM
    input   wire [(pDATA_WIDTH-1):0] tap_Do,      // Data output from tap RAM

    // BRAM Interface for data RAM
    output  wire [3:0]               data_WE,     // Write enable for data RAM
    output  wire                     data_EN,     // Enable signal for data RAM
    output  wire [(pDATA_WIDTH-1):0] data_Di,     // Data input for data RAM
    output  wire [(pADDR_WIDTH-1):0] data_A,      // Address for data RAM
    input   wire [(pDATA_WIDTH-1):0] data_Do,     // Data output from data RAM

    // Clock and Reset
    input   wire                     axis_clk,    // Clock signal
    input   wire                     axis_rst_n   // Active-low reset signal
);

    // Local parameters for register address mapping
    localparam
        ADDR_AP_CTRL = 12'h00,          // Address for control register
        ADDR_datalength = 12'h10,       // Address for data length register
        ADDR_tapparameters = 12'hFF;    // Address for tap parameters register
    
    // Output signal declarations
    reg aw_ready;                       // Internal signal for write address ready
    reg w_ready;                        // Internal signal for write data ready
    reg ar_ready;                       // Internal signal for read address ready
    reg [pDATA_WIDTH-1:0] r_data;       // Internal signal for read data
    reg [pADDR_WIDTH-1:0] aw_addr;      // Internal signal for write address
    reg r_valid;                        // Internal signal for read data valid
    reg tapen;                          // Enable signal for tap RAM
    reg [pADDR_WIDTH-1:0] tapa;         // Address signal for tap RAM
    reg [3:0] datawe;                   // Write enable signal for data RAM
    reg dataen;                         // Enable signal for data RAM
    reg [pADDR_WIDTH-1:0] dataa;        // Address signal for data RAM
    reg smtvalid;                       // Master stream valid signal
    reg smtlast;                        // Master stream last signal

    // Signal assignments for outputs
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

    // State machine for BRAM 0 operations
    reg bramstate;
    always @(posedge axis_clk) begin
        if (axis_rst_n == 1'b0) begin
            bramstate <= 0;              // Reset BRAM state
            dataa <= 12'h0;              // Reset data address
        end else begin
            if (bramstate == 1'b0) begin
                if (dataa == 12'h28) begin
                    bramstate <= 1'b1;    // Change BRAM state
                end else begin
                    dataa <= dataa + 12'h04; // Increment address
                end
            end
        end
    end

    // State machine to handle last input and output state
    reg last;
    always @(posedge axis_clk) begin
        if (axis_rst_n == 1'b0) begin
            last <= 0;                   // Reset last signal
            smtlast<=1'b0;               // Reset master stream last signal
        end else begin
            if (ss_tlast && sstready) begin
                last <= 1'b1;            // Set last when slave stream last is high
            end
            else if( smtlast && smtvalid) begin
                int_task_ap_done <= 1'b1; // Mark task as done
                int_ap_idle <= 1'b1;     // Set idle state
                firstate<=1'b0;          // Reset first state
                smstate<=1'b0;           // Reset state machine
            end
            else if (last && smtvalid) begin
                smtlast<=1'b1;           // Set master stream last signal
                last<=1'b0;              // Clear last signal
            end
        end
    end
    
    reg smstate;
    // FIR filter calculation process
    reg [pDATA_WIDTH-1:0] ymult;         // Multiplier output
    reg [pDATA_WIDTH-1:0] yout;          // Filter output
    always @(posedge axis_clk) begin
        if (axis_rst_n == 1'b0) begin
            smtvalid <= 1'b0;            // Reset master stream valid signal
            yout <= 32'h0;               // Reset output
            ymult <= 32'h0;              // Reset multiplier output          
            smtlast <= 1'b0;             // Reset master stream last signal
            smstate<=1'b0;               // Reset state machine
        end else begin
        if (firstate) begin
            case (tapa)
                12'h00: begin
                    ymult <= tap_Do * data_Do;  // Multiply tap and data
                    dataa <= (dataa == 12'h28) ? 12'h00 : dataa + 12'h04; // Update data address
                    tapa <= tapa + 12'h04;      // Update tap address
                    sstready <= 1'b0;           // Clear slave stream ready signal
                    if (smstate) begin 
                        smtvalid <= 1'b1;       // Set master stream valid signal
                    end else begin 
                        smstate<=1'b1 ;         // Update state machine
                    end
                    yout <= ymult + yout;       // Accumulate output
                end
                12'h04: begin
                    yout <= ymult;              // Update output
                    dataa <= (dataa == 12'h28) ? 12'h00 : dataa + 12'h04; // Update data address
                    ymult <= tap_Do * data_Do; // Multiply tap and data
                    tapa <= tapa + 12'h04;      // Update tap address
                    smtvalid <= 1'b0;           // Clear master stream valid signal
                end
                12'h28: begin
                    ymult <= tap_Do * data_Do;  // Multiply tap and data
                    yout <= ymult + yout;       // Accumulate output
                    tapa <= 12'h00;             // Reset tap address
                    if (~last)
                    sstready <= 1'b1;           // Set slave stream ready signal
                end
                default: begin
                    ymult <= tap_Do * data_Do;  // Multiply tap and data
                    yout <= ymult + yout;       // Accumulate output
                    dataa <= (dataa == 12'h28) ? 12'h00 : dataa + 12'h04; // Update data address
                    tapa <= tapa + 12'h04;      // Update tap address
                end
            endcase
        end
      end
    end

    // Generate write enable signal for data RAM
    assign data_WE ={4{(sstready && ss_tvalid)||~bramstate}};
    
    // Slave stream ready signal and first state logic
    reg sstready=1'b0;
    assign ss_tready = sstready;
    reg firstate;
    reg [pADDR_WIDTH-1:0] data_addr = 12'h20;
    always @(posedge axis_clk) begin
        if (axis_rst_n == 1'b0) begin
            sstready <= 1'b0;            // Reset slave stream ready signal
            firstate <= 1'b0;            // Reset first state
        end else if (int_ap_start) begin
            sstready <= 1'b1;            // Set slave stream ready signal
            firstate <= 1'b1;            // Set first state
            tapa <= 12'h00;              // Reset tap address
            dataa <= 12'h00;             // Reset data address
            int_ap_start <= 1'b0;        // Clear start signal
        end 
    end

    // Enable signals for tap and data RAM
    always @(posedge axis_clk) begin
        if (axis_rst_n == 1'b0) begin
            tapen <= 1'b1;               // Enable tap RAM
            dataen <= 1'b1;              // Enable data RAM
        end
    end

    // Write enable signal generation for tap RAM
    assign tap_WE={4{awvalid && wvalid && awaddr[7]}};
    always @(*) begin
        if (awvalid == 1'b1&&awaddr[7]==1'b1) begin
            tapa<=awaddr[6:0];           // Update tap address
        end
        else if (arvalid == 1'b1&&araddr[7]==1'b1)begin
            tapa<=araddr[6:0];           // Update tap address
        end
    end

    // Register to hold data length
    reg [pDATA_WIDTH-1:0] data_length;
    // Write logic for AXI4-Lite write transactions
    always @(posedge axis_clk) begin
        if (axis_rst_n == 1'b0) begin
            data_length <= 0;            // Reset data length
            w_ready<=1'b1;               // Set write data ready signal
            aw_ready<=1'b1;              // Set write address ready signal
           end else if (awvalid && wvalid) begin
            case (awaddr)
                ADDR_AP_CTRL: begin
                    if (wdata == 32'h0000_0001)
                    int_ap_start <= 1'b1; // Start the FIR process
                    int_ap_idle <= 1'b0;  // Clear idle signal  
                end
                ADDR_datalength: begin
                    data_length <= wdata; // Update data length
                end
                default: begin
                end
            endcase
        end
    end

    // AXI4-Lite read logic
    reg int_ap_start;
    reg int_task_ap_done;
    reg int_ap_idle;
    always @(*) begin
    case (araddr)
    ADDR_AP_CTRL: begin
                        r_data[0] <= int_ap_start;       // Start signal
                        r_data[1] <= int_task_ap_done;   // Task done signal
                        r_data[2] <= int_ap_idle;        // Idle signal
                    end
                    ADDR_datalength: begin
                        r_data <= data_length;           // Data length
                    end
                    default: begin
                        r_data <= tap_Do;                // Default data
                    end
      endcase
    end
    always @(posedge axis_clk) begin
        if (axis_rst_n == 1'b0) begin
            r_data <= 0;                // Reset read data
            int_task_ap_done <= 1'b0;  // Reset task done signal
            int_ap_idle <= 1'b0;       // Reset idle signal
            r_valid <= 1'b0;           // Clear read valid signal
            ar_ready<=1'b1;            // Set read address ready signal
        end else if ( ~r_valid && arvalid) begin
            r_valid <= ~r_valid;       // Toggle read valid signal
        end else if (rready && r_valid) begin    
            r_valid <= 1'b0;           // Clear read valid signal
        end
    end
endmodule
