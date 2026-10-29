`timescale 1ns / 1ps

module APB (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] paddr,
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output wire        pready,

    output wire        start,
    output wire        soft_reset,
    output reg  [31:0] dma_src_addr,
    output reg  [31:0] dma_dst_addr,
    output wire [15:0] matrix_rows,
    output wire [15:0] matrix_cols,

    input  wire        done,
    input  wire        busy,
    input  wire        error
);
    localparam ADDR_CTRL    = 6'h00; 
    localparam ADDR_STATUS  = 6'h01; 
    localparam ADDR_DMA_SRC = 6'h02; 
    localparam ADDR_DMA_DST = 6'h03; 
    localparam ADDR_MATRIX  = 6'h04; 

    wire [5:0] word_addr = paddr[7:2]; 

    wire apb_valid = psel && penable;  
    wire apb_write = apb_valid && pwrite;
    wire apb_read  = apb_valid && !pwrite;

    assign pready = 1'b1;

    //NPU_CTRL (0x00)
    reg start_pending;
    reg ctrl_reset_bit;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_pending  <= 1'b0;
            ctrl_reset_bit <= 1'b0;
        end else begin
            if (apb_write && word_addr == ADDR_CTRL)
                ctrl_reset_bit <= pwdata[1];
            if (apb_write && word_addr == ADDR_CTRL && pwdata[0])
                start_pending <= 1'b1;
            else
                start_pending <= 1'b0;
        end
    end

    assign start      = start_pending;
    assign soft_reset = ctrl_reset_bit;

    // DMA_SRC_ADDR (0x08) / DMA_DST_ADDR (0x0C) 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dma_src_addr <= 32'h0;
            dma_dst_addr <= 32'h0;
        end else if (apb_write) begin
            if (word_addr == ADDR_DMA_SRC) dma_src_addr <= pwdata;
            if (word_addr == ADDR_DMA_DST) dma_dst_addr <= pwdata;
        end
    end

    // MATRIX_CFG (0x10) 
    reg [31:0] matrix_cfg_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            matrix_cfg_reg <= 32'h0;
        else if (apb_write && word_addr == ADDR_MATRIX)
            matrix_cfg_reg <= pwdata;
    end

    assign matrix_rows = matrix_cfg_reg[15:0];
    assign matrix_cols = matrix_cfg_reg[31:16];

    // NPU_STATUS (0x04) 
    wire [31:0] status_word = {29'b0, error, busy, done};

    // Read data mux
    always @(*) begin
        case (word_addr)
            ADDR_CTRL:    prdata = {30'b0, ctrl_reset_bit, start_pending};
            ADDR_STATUS:  prdata = status_word;
            ADDR_DMA_SRC: prdata = dma_src_addr;
            ADDR_DMA_DST: prdata = dma_dst_addr;
            ADDR_MATRIX:  prdata = matrix_cfg_reg;
            default:      prdata = 32'hDEAD_BEEF;
        endcase
    end

endmodule

