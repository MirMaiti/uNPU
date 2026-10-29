`timescale 1ns / 1ps

module unpu_top #(
    parameter int ADDR_WIDTH     = 32,
    parameter int DATA_WIDTH     = 32,
    parameter int TIMEOUT_CYCLES = 32'd65535,  // 0 = watchdog disabled
    parameter int N              = 4,          
    parameter int ACT_W          = 8,
    parameter int WGT_W          = 8,
    parameter int PROD_W         = 16,
    parameter int PSUM_W         = 18
) (
    input  wire clk,
    input  wire rst_n,

    // ---- APB slave (control plane) ----
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] paddr,
    input  wire [31:0] pwdata,
    output wire [31:0] prdata,
    output wire        pready,

    // ---- native SRAM master (data plane) ----
    output wire [ADDR_WIDTH-1:0] dma_addr,
    output wire [DATA_WIDTH-1:0] dma_wdata,
    input  wire [DATA_WIDTH-1:0] dma_rdata,
    output wire [3:0]            dma_wstrb,
    output wire                  dma_valid,
    input  wire                  dma_ready     
);
    wire        start, soft_reset;
    wire [31:0] dma_src_addr, dma_dst_addr;
    wire [15:0] matrix_rows, matrix_cols;
    wire        done, busy, error;

    APB u_apb (
        .clk(clk), .rst_n(rst_n),
        .psel(psel), .penable(penable), .pwrite(pwrite),
        .paddr(paddr), .pwdata(pwdata), .prdata(prdata), .pready(pready),
        .start(start), .soft_reset(soft_reset),
        .dma_src_addr(dma_src_addr), .dma_dst_addr(dma_dst_addr),
        .matrix_rows(matrix_rows), .matrix_cols(matrix_cols),
        .done(done), .busy(busy), .error(error)
    );

    wire [DATA_WIDTH-1:0] cu_data;
    wire                   cu_data_valid, cu_data_ready;
    wire                   cu_load_data, cu_start_compute;
    wire                   cu_busy, cu_done;
    wire [DATA_WIDTH-1:0]  cu_result_data;
    wire                   cu_result_valid, cu_result_ready;

    unpu_dma #(
        .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
    ) u_dma (
        .clk(clk), .rst_n(rst_n),
        .start(start), .soft_reset(soft_reset),
        .src_addr(dma_src_addr), .dst_addr(dma_dst_addr),
        .cfg_rows(matrix_rows), .cfg_cols(matrix_cols),
        .busy(busy), .done(done), .error(error),
        .dma_addr(dma_addr), .dma_wdata(dma_wdata), .dma_rdata(dma_rdata),
        .dma_wstrb(dma_wstrb), .dma_valid(dma_valid), .dma_ready(dma_ready),
        .cu_data(cu_data), .cu_data_valid(cu_data_valid), .cu_data_ready(cu_data_ready),
        .cu_load_data(cu_load_data), .cu_start_compute(cu_start_compute),
        .cu_busy(cu_busy), .cu_done(cu_done),
        .cu_result_data(cu_result_data), .cu_result_valid(cu_result_valid),
        .cu_result_ready(cu_result_ready)
    );

    unpu_cu #(
        .N(N), .ACT_W(ACT_W), .WGT_W(WGT_W), .PROD_W(PROD_W), .PSUM_W(PSUM_W), .DATA_W(DATA_WIDTH)
    ) u_cu (
        .clk(clk), .rst_n(rst_n), .soft_reset(soft_reset),
        .cu_data(cu_data), .cu_data_valid(cu_data_valid), .cu_data_ready(cu_data_ready),
        .cu_load_data(cu_load_data), .cu_start_compute(cu_start_compute),
        .cu_busy(cu_busy), .cu_done(cu_done),
        .cu_result_data(cu_result_data), .cu_result_valid(cu_result_valid),
        .cu_result_ready(cu_result_ready)
    );

endmodule