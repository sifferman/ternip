// Copyright (c) 2026 Ethan Sifferman
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of
//    conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of
//    conditions and the following disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific prior written
//    permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// ternip_add
//
// Saturating fixed-point add.
//
// Computes y_o = a_i + b_i and clamps to FixedPointPrecision.
// This is combinational: drive a_i and b_i, then use y_o in the same cycle.

module ternip_add #(
    parameter ternip_pkg::ternip_cfg_t Cfg = `TERNIP_CFG,
    parameter int FixedPointPrecision = Cfg.FixedPointPrecision,
    localparam type fixed_point_t = logic signed [FixedPointPrecision-1:0]
) (
    input  fixed_point_t a_i,
    input  fixed_point_t b_i,
    output fixed_point_t y_o
);

localparam fixed_point_t FixedPointMin = ternip_pkg::fixed_point_min(FixedPointPrecision);
localparam fixed_point_t FixedPointMax = ternip_pkg::fixed_point_max(FixedPointPrecision);

logic signed [FixedPointPrecision:0] internal;
always_comb begin
    internal = a_i + b_i;
    if (internal > FixedPointMax) internal = FixedPointMax;
    if (internal < FixedPointMin) internal = FixedPointMin;
    y_o = fixed_point_t'(internal);
end

endmodule
