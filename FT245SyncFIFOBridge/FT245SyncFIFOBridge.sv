/*
FT245SyncFIFOBridge v1.0

Copyright 2019 Fujii Naomichi

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

`timescale 1 ps / 1 ps
module FT245SyncFIFOBridge 
    #(
        parameter DATA_WIDTH = 8,
        parameter EMPTY_WIDTH = 1,
        parameter BE_WIDTH = 1,
        parameter MIN_ARBITRATION_CYCLES = 8,
        parameter MAX_ARBITRATION_CYCLES = 1024
    )
    (
        input  wire                   clk,                  //     clk.clk
        input  wire                   reset,                //   reset.reset
        input  wire [DATA_WIDTH-1:0]  sink_data,            //    sink.data
        input  wire [EMPTY_WIDTH-1:0] sink_empty,           //        .empty
        input  wire                   sink_endofpacket,     //        .endofpacket
        output wire                   sink_ready,           //        .ready
        input  wire                   sink_startofpacket,   //        .startofpacket
        input  wire                   sink_valid,           //        .valid
        output wire [DATA_WIDTH-1:0]  source_data,          //  source.data
        output wire [EMPTY_WIDTH-1:0] source_empty,         //        .empty
        output wire                   source_endofpacket,   //        .endofpacket
        input  wire                   source_ready,         //        .ready
        output wire                   source_startofpacket, //        .startofpacket
        output wire                   source_valid,         //        .valid
        inout  wire [DATA_WIDTH-1:0]  ft_data,              // conduit.data
        inout  wire [BE_WIDTH-1:0]    ft_be,                //        .byteenable
        input  wire                   ft_txe_n,             //        .txe_n
        input  wire                   ft_rxf_n,             //        .rxf_n
        output reg                    ft_oe_n,              //        .oe_n
        output reg                    ft_rd_n,              //        .rd_n
        output reg                    ft_wr_n               //        .wr_n
    );
    
    // Rx/Tx arbitration signals
    reg rx_tx_select = 1'b0; // 0->Tx, 1->Rx
    reg rx_enabled = 1'b0;
    reg tx_enabled = 1'b0;
    
    // Rx/Tx arbitration logic
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            rx_enabled <= 1'b0;
            tx_enabled <= 1'b0;
        end
        else begin
            if (rx_tx_select == 1'b0) begin
                rx_enabled <= 1'b0;
            end else if (tx_enabled == 1'b0) begin
                rx_enabled <= 1'b1;
            end
            if (rx_tx_select == 1'b1) begin
                tx_enabled <= 1'b0;
            end else if (rx_enabled == 1'b0) begin
                tx_enabled <= 1'b1;
            end
        end
    end
    
    // Rx/Tx arbitration algorithm
    wire arb_active = rx_tx_select ? source_valid : sink_valid;
    reg arb_min_cycle = 1'b1;
    reg [$clog2(MAX_ARBITRATION_CYCLES)-1:0] arb_counter = '0;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            arb_min_cycle <= 1'b1;
            arb_counter <= '0;
        end
        else begin
            if ((~arb_active & ~arb_min_cycle) | (arb_counter == (MAX_ARBITRATION_CYCLES - 1))) begin
                arb_min_cycle <= 1'b1;
                arb_counter <= '0;
                rx_tx_select <= ~rx_tx_select;
            end else begin
                if (arb_counter == (MIN_ARBITRATION_CYCLES - 1)) begin
                    arb_min_cycle <= 1'b0;
                end
                arb_counter <= arb_counter + 1;
            end
        end
    end
    
    /*function automatic void set_rx_enable (bit new_value);
        rx_enabled = new_value;
    endfunction
    
    function automatic void set_tx_enable (bit new_value);
        tx_enabled = new_value;
    endfunction*/
    
    // Latched FT245's signals
    reg internal_txe_n = 1'b1;
    reg internal_wr_n = 1'b1;
    reg internal_rxf_n = 1'b1;
    reg internal_oe_n = 1'b1;
    reg internal_rd_n = 1'b1;
    
    // Reordering symbols
    wire [DATA_WIDTH-1:0] sink_reordered_data = swap_bytes(sink_data);
    wire [BE_WIDTH-1:0] sink_byte_enable = sink_endofpacket ? ('1 >> sink_empty) : '1;

    // Transmitter
    reg tx_delayled_enabled = 1'b0;
    reg tx_hold_pending = 1'b0;
    reg tx_back_pending = 1'b0;
    reg tx_front_pending_negedge = 1'b0;
    reg tx_front_pending = 1'b0;
    wire tx_push_to_hold = ~sink_ready & sink_valid;
    wire tx_push_to_front = tx_back_pending & ((~internal_txe_n & ~internal_wr_n) | ~tx_front_pending);
    wire tx_push_to_back = (sink_ready & sink_valid) | (tx_hold_pending & (~tx_back_pending | tx_push_to_front));
    
    reg [DATA_WIDTH-1:0] tx_hold_data = '0;
    reg [DATA_WIDTH-1:0] tx_back_data = '0;
    reg [DATA_WIDTH-1:0] tx_front_data = '0;
    reg [BE_WIDTH-1:0] tx_hold_be = '0;
    reg [BE_WIDTH-1:0] tx_back_be = '0;
    reg [BE_WIDTH-1:0] tx_front_be = '0;

    assign ft_data = ~ft_wr_n ? tx_front_data : 'z;
    assign ft_be = ~ft_wr_n ? tx_front_be : 'z;
    assign sink_ready = tx_enabled & ~internal_txe_n & (~tx_hold_pending | tx_delayled_enabled);
    //tx_enabled & ~(tx_hold_pending ? (delayed_txe_n | internal_txe_n) : internal_txe_n);
    
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            internal_txe_n <= 1'b1;
            internal_wr_n <= 1'b1;
            tx_delayled_enabled <= 1'b0;
            tx_hold_pending <= 1'b0;
            tx_back_pending <= 1'b0;
            tx_front_pending <= 1'b0;
        end
        else begin
            internal_txe_n <= ft_txe_n;
            internal_wr_n <= ft_wr_n;
            tx_delayled_enabled <= tx_enabled & ~internal_txe_n;
            tx_front_pending <= tx_front_pending_negedge;
            
            if (tx_push_to_back == 1'b1) begin
                tx_back_pending <= 1'b1;
            end
            else if (tx_push_to_front == 1'b1) begin
                tx_back_pending <= 1'b0;
            end

            if (tx_push_to_back == 1'b1) begin
                tx_back_data <= tx_hold_pending ? tx_hold_data : sink_reordered_data;
                tx_back_be <= tx_hold_pending ? tx_hold_be : sink_byte_enable;
                tx_hold_pending <= 1'b0;
            end
            else begin
                if (tx_push_to_hold == 1'b1) begin
                    tx_hold_data <= sink_reordered_data;
                    tx_hold_be <= sink_byte_enable;
                    tx_hold_pending <= 1'b1;
                end
            end
        end
    end
    
    always @(negedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            ft_wr_n <= 1'b1;
            tx_front_pending_negedge <= 1'b0;
        end
        else begin
            if (tx_push_to_front == 1'b1) begin
                tx_front_data <= tx_back_data;
                tx_front_be <= tx_back_be;
                tx_front_pending_negedge <= 1'b1;
                ft_wr_n <= ~tx_enabled | internal_txe_n;
            end
            else if (~internal_txe_n & ~internal_wr_n) begin
                tx_front_pending_negedge <= 1'b0;
                ft_wr_n <= 1'b1;
            end
            else begin
                ft_wr_n <= ~tx_enabled | internal_txe_n | ~tx_front_pending_negedge;
            end
        end
    end
    
    // Receiver
    reg rx_pending_back = 1'b0;
    reg rx_pending_front = 1'b0;
    wire rx_invalidate_back = internal_rxf_n & ~internal_rd_n;
    reg rx_push_to_back_n = 1'b1;
    wire rx_push_to_back = ~rx_push_to_back_n;
    wire rx_push_to_front = (rx_pending_back & ~rx_invalidate_back) & (source_valid | ~rx_pending_front);
    
    reg [DATA_WIDTH-1:0] rx_back_data = '0;
    reg [DATA_WIDTH-1:0] rx_front_data = '0;
    reg [BE_WIDTH-1:0] rx_back_be = '0;
    reg [BE_WIDTH-1:0] rx_front_be = '0;

    reg delayed_source_ready = 1'b0;
    wire rx_read_enable = rx_enabled & delayed_source_ready & ~internal_rxf_n;
    
    assign source_data = swap_bytes(rx_front_data);
    assign source_empty = be_to_empty(rx_front_be);
    assign source_endofpacket = source_valid;
    assign source_startofpacket = source_valid;
    assign source_valid = rx_pending_front & delayed_source_ready;
    
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            internal_rxf_n <= 1'b1;
            internal_oe_n <= 1'b1;
            internal_rd_n <= 1'b1;
        end
        else begin
            internal_rxf_n <= ft_rxf_n;
            internal_oe_n <= ft_oe_n;
            internal_rd_n <= ft_rd_n;
            delayed_source_ready <= source_ready;
            
            if (rx_push_to_back == 1'b1) begin
                rx_back_data <= ft_data;
                rx_back_be <= ft_be;
                rx_pending_back <= 1'b1;
            end else if (rx_push_to_front | rx_invalidate_back) begin
                rx_pending_back <= 1'b0;
            end
            
            if (rx_push_to_front == 1'b1) begin
                rx_front_data <= rx_back_data;
                rx_front_be <= rx_back_be;
                rx_pending_front <= 1'b1;
            end else if (source_valid == 1'b1) begin
                rx_pending_front <= 1'b0;
            end
        end
    end
    
    always @(negedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            ft_oe_n <= 1'b1;
            ft_rd_n <= 1'b1;
            rx_push_to_back_n <= 1'b1;
        end
        else begin
            ft_oe_n <= ~rx_read_enable;
            ft_rd_n <= ~rx_read_enable | internal_oe_n;
            rx_push_to_back_n <= ~rx_read_enable | internal_oe_n;
            
        end
    end
    
    // Convert byte enable to empty signal
    function int be_to_empty([BE_WIDTH-1:0] in);
        casex(in)
            4'b1xxx : be_to_empty = BE_WIDTH - 4;
            4'b01xx : be_to_empty = BE_WIDTH - 3;
            4'b001x : be_to_empty = BE_WIDTH - 2;
            4'b0001 : be_to_empty = BE_WIDTH - 1;
            default : be_to_empty = 0;
        endcase
    endfunction
    
    // Swap bytes
    function [DATA_WIDTH-1:0] swap_bytes([DATA_WIDTH-1:0] in);
        begin
            // For System Verilog 1800-2012
            //swap_bytes = {<< 8 {in}};
            
            // For older versions
            if (DATA_WIDTH == 8) begin
                swap_bytes = in;
            end
            else if (DATA_WIDTH == 16) begin
                swap_bytes = {in[7:0], in[15:8]};
            end
            else if (DATA_WIDTH == 32) begin
                swap_bytes = {in[7:0], in[15:8], in[23:16], in[31:24]};
            end
            else begin
                swap_bytes = '0;
            end
        end
    endfunction
endmodule
