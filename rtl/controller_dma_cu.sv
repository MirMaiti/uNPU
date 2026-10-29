`timescale 1ns / 1ps
module unpu_cu #(
    parameter int N      = 4,   
    parameter int ACT_W  = 8,
    parameter int WGT_W  = 8,
    parameter int PROD_W = 16,
    parameter int PSUM_W = 18,
    parameter int DATA_W = 32
) (
    input  logic clk,
    input  logic rst_n,
    input  logic soft_reset,       
    input  logic [DATA_W-1:0] cu_data,
    input  logic              cu_data_valid,
    output logic              cu_data_ready,
    input  logic              cu_load_data,      
    input  logic              cu_start_compute,  // 1-cycle pulse after last load word handed off
    output logic              cu_busy,           // not used for sequencing by unpu_dma_ctrl
    output logic              cu_done,
    output logic [DATA_W-1:0] cu_result_data,
    output logic              cu_result_valid,
    input  logic              cu_result_ready
);

    localparam int WEIGHT_WORDS = (N * N) / 4;              // 4
    localparam int INPUT_WORDS  = (N * N) / 4;               // 4
    localparam int LOAD_WORDS   = WEIGHT_WORDS + INPUT_WORDS; // 8
    localparam int OUT_WORDS    = N * N;                      // 16

    logic              wshift, act_en;
    logic [31:0]       weight_word, act_word;
    logic [PSUM_W-1:0] arr_psum_seed [N-1:0];
    logic [PSUM_W-1:0] arr_psum_out  [N-1:0];
    logic              out_valid;
    logic [1:0]        load_cnt_dbg;

    genvar gi;
    generate
        for (gi = 0; gi < N; gi++) begin : g_seed
            assign arr_psum_seed[gi] = '0;   
        end
    endgenerate


   
    logic [3:0] word_cnt;   

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            word_cnt <= 4'd0;
        else if (soft_reset)
            word_cnt <= 4'd0;
        else if (!cu_load_data)
            word_cnt <= 4'd0;
        else if (cu_data_valid && cu_data_ready)
            word_cnt <= word_cnt + 4'd1;
    end

    wire loading_words  = cu_load_data && (word_cnt < LOAD_WORDS[3:0]);
    wire is_weight_word = (word_cnt < WEIGHT_WORDS[3:0]);

    assign cu_data_ready = loading_words;               
    assign wshift        = cu_data_valid && cu_data_ready &&  is_weight_word;
    assign act_en        = cu_data_valid && cu_data_ready && !is_weight_word;
    assign weight_word   = cu_data;                     // valid only while wshift is asserted
    assign act_word      = cu_data;                     // valid only while act_en is asserted

    logic cu_load_data_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cu_load_data_d <= 1'b0;
        else
            cu_load_data_d <= cu_load_data;
    end
    wire round_start = cu_load_data && !cu_load_data_d;

    logic [DATA_W-1:0] result_buf [OUT_WORDS-1:0];
    logic [2:0]        out_row_cnt;   // 0..N
    logic              done_all;

    integer c;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_row_cnt <= 3'd0;
            done_all    <= 1'b0;
        end else if (soft_reset) begin
            out_row_cnt <= 3'd0;
            done_all    <= 1'b0;
        end else if (round_start) begin
            out_row_cnt <= 3'd0;
            done_all    <= 1'b0;
        end else if (out_valid && (out_row_cnt < N[2:0])) begin
            for (c = 0; c < N; c = c + 1)
                result_buf[out_row_cnt * N + c] <= {{(DATA_W - PSUM_W){1'b0}}, arr_psum_out[c]};
            if (out_row_cnt == N[2:0] - 3'd1)
                done_all <= 1'b1;
            out_row_cnt <= out_row_cnt + 3'd1;
        end
    end

    assign cu_done = done_all;
    logic [4:0] result_ptr;   // 0..OUT_WORDS

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            result_ptr <= 5'd0;
        else if (soft_reset)
            result_ptr <= 5'd0;
        else if (round_start)
            result_ptr <= 5'd0;
        else if (cu_result_valid && cu_result_ready)
            result_ptr <= result_ptr + 5'd1;
    end

    assign cu_result_valid = done_all && (result_ptr < OUT_WORDS[4:0]);
    assign cu_result_data  = result_buf[result_ptr[3:0]];

    assign cu_busy = loading_words
                   || (out_row_cnt != 3'd0 && out_row_cnt < N[2:0])
                   || (done_all && (result_ptr < OUT_WORDS[4:0]));


    systolic_array_dip_4x4 #(
        .N(N), .ACT_W(ACT_W), .WGT_W(WGT_W), .PROD_W(PROD_W), .PSUM_W(PSUM_W)
    ) u_array (
        .clk(clk), .rst_n(rst_n),
        .wshift(wshift), .act_en(act_en),
        .weight_word(weight_word), .act_word(act_word),
        .psum_seed(arr_psum_seed),
        .load_clr (soft_reset | round_start),
        .psum_out(arr_psum_out), .out_valid(out_valid),
        .load_cnt_dbg(load_cnt_dbg)
    );

endmodule
