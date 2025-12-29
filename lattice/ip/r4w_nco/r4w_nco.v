//-----------------------------------------------------------------------------
// R4W NCO for Lattice FPGAs (iCE40/ECP5)
// R4W FPGA Acceleration Layer
//
// Numerically Controlled Oscillator optimized for Lattice FPGAs.
// Uses LUT-based sin/cos for smaller FPGAs, CORDIC for ECP5.
//
// Compatible with: iCE40 HX/LP/UP, ECP5
// Toolchain: Yosys + IceStorm/Trellis
//
// trace:FR-0081 | ai:claude
//-----------------------------------------------------------------------------

module r4w_nco #(
    parameter PHASE_WIDTH = 24,     // Phase accumulator width
    parameter OUTPUT_WIDTH = 12,    // Output sample width
    parameter USE_LUT = 1           // 1=LUT (iCE40), 0=CORDIC (ECP5)
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Control
    input  wire                     enable,
    input  wire [PHASE_WIDTH-1:0]   freq_word,   // Phase increment per sample
    input  wire [PHASE_WIDTH-1:0]   phase_offset,

    // Outputs (signed)
    output reg  signed [OUTPUT_WIDTH-1:0] i_out,  // Cosine
    output reg  signed [OUTPUT_WIDTH-1:0] q_out   // Sine
);

    // =========================================================================
    // Phase Accumulator
    // =========================================================================

    reg [PHASE_WIDTH-1:0] phase_acc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_acc <= 0;
        end else if (enable) begin
            phase_acc <= phase_acc + freq_word;
        end
    end

    wire [PHASE_WIDTH-1:0] phase = phase_acc + phase_offset;

    // =========================================================================
    // Sin/Cos Generation
    // =========================================================================

    generate
        if (USE_LUT) begin : gen_lut
            // LUT-based approach for iCE40 (256 entry quarter-wave)
            // Uses 256 x 12-bit = 3 BRAMs

            // Quarter-wave sine lookup table (pre-computed for synthesis)
            // Full wave reconstructed using symmetry
            // Values: sin(i * pi/2 / 256) * 2047, for i = 0..255
            reg signed [OUTPUT_WIDTH-1:0] sin_lut [0:255];

            initial begin : init_lut
                // Pre-computed quarter-wave sine table (12-bit, max 2047)
                // Row 0: indices 0-15
                sin_lut[0] = 12'd0;     sin_lut[1] = 12'd13;    sin_lut[2] = 12'd25;    sin_lut[3] = 12'd38;
                sin_lut[4] = 12'd50;    sin_lut[5] = 12'd63;    sin_lut[6] = 12'd75;    sin_lut[7] = 12'd88;
                sin_lut[8] = 12'd100;   sin_lut[9] = 12'd113;   sin_lut[10] = 12'd125;  sin_lut[11] = 12'd138;
                sin_lut[12] = 12'd150;  sin_lut[13] = 12'd163;  sin_lut[14] = 12'd175;  sin_lut[15] = 12'd188;
                // Row 1: indices 16-31
                sin_lut[16] = 12'd200;  sin_lut[17] = 12'd213;  sin_lut[18] = 12'd225;  sin_lut[19] = 12'd238;
                sin_lut[20] = 12'd250;  sin_lut[21] = 12'd263;  sin_lut[22] = 12'd275;  sin_lut[23] = 12'd287;
                sin_lut[24] = 12'd300;  sin_lut[25] = 12'd312;  sin_lut[26] = 12'd324;  sin_lut[27] = 12'd337;
                sin_lut[28] = 12'd349;  sin_lut[29] = 12'd361;  sin_lut[30] = 12'd374;  sin_lut[31] = 12'd386;
                // Row 2: indices 32-47
                sin_lut[32] = 12'd398;  sin_lut[33] = 12'd410;  sin_lut[34] = 12'd422;  sin_lut[35] = 12'd435;
                sin_lut[36] = 12'd447;  sin_lut[37] = 12'd459;  sin_lut[38] = 12'd471;  sin_lut[39] = 12'd483;
                sin_lut[40] = 12'd495;  sin_lut[41] = 12'd507;  sin_lut[42] = 12'd519;  sin_lut[43] = 12'd531;
                sin_lut[44] = 12'd543;  sin_lut[45] = 12'd555;  sin_lut[46] = 12'd567;  sin_lut[47] = 12'd578;
                // Row 3: indices 48-63
                sin_lut[48] = 12'd590;  sin_lut[49] = 12'd602;  sin_lut[50] = 12'd614;  sin_lut[51] = 12'd625;
                sin_lut[52] = 12'd637;  sin_lut[53] = 12'd649;  sin_lut[54] = 12'd660;  sin_lut[55] = 12'd672;
                sin_lut[56] = 12'd683;  sin_lut[57] = 12'd695;  sin_lut[58] = 12'd706;  sin_lut[59] = 12'd718;
                sin_lut[60] = 12'd729;  sin_lut[61] = 12'd740;  sin_lut[62] = 12'd752;  sin_lut[63] = 12'd763;
                // Row 4: indices 64-79
                sin_lut[64] = 12'd774;  sin_lut[65] = 12'd785;  sin_lut[66] = 12'd796;  sin_lut[67] = 12'd807;
                sin_lut[68] = 12'd818;  sin_lut[69] = 12'd829;  sin_lut[70] = 12'd840;  sin_lut[71] = 12'd851;
                sin_lut[72] = 12'd862;  sin_lut[73] = 12'd873;  sin_lut[74] = 12'd883;  sin_lut[75] = 12'd894;
                sin_lut[76] = 12'd905;  sin_lut[77] = 12'd915;  sin_lut[78] = 12'd926;  sin_lut[79] = 12'd936;
                // Row 5: indices 80-95
                sin_lut[80] = 12'd946;  sin_lut[81] = 12'd957;  sin_lut[82] = 12'd967;  sin_lut[83] = 12'd977;
                sin_lut[84] = 12'd987;  sin_lut[85] = 12'd997;  sin_lut[86] = 12'd1007; sin_lut[87] = 12'd1017;
                sin_lut[88] = 12'd1027; sin_lut[89] = 12'd1037; sin_lut[90] = 12'd1047; sin_lut[91] = 12'd1056;
                sin_lut[92] = 12'd1066; sin_lut[93] = 12'd1076; sin_lut[94] = 12'd1085; sin_lut[95] = 12'd1095;
                // Row 6: indices 96-111
                sin_lut[96] = 12'd1104; sin_lut[97] = 12'd1114; sin_lut[98] = 12'd1123; sin_lut[99] = 12'd1132;
                sin_lut[100] = 12'd1141; sin_lut[101] = 12'd1151; sin_lut[102] = 12'd1160; sin_lut[103] = 12'd1169;
                sin_lut[104] = 12'd1178; sin_lut[105] = 12'd1187; sin_lut[106] = 12'd1195; sin_lut[107] = 12'd1204;
                sin_lut[108] = 12'd1213; sin_lut[109] = 12'd1221; sin_lut[110] = 12'd1230; sin_lut[111] = 12'd1238;
                // Row 7: indices 112-127
                sin_lut[112] = 12'd1247; sin_lut[113] = 12'd1255; sin_lut[114] = 12'd1263; sin_lut[115] = 12'd1272;
                sin_lut[116] = 12'd1280; sin_lut[117] = 12'd1288; sin_lut[118] = 12'd1296; sin_lut[119] = 12'd1304;
                sin_lut[120] = 12'd1312; sin_lut[121] = 12'd1319; sin_lut[122] = 12'd1327; sin_lut[123] = 12'd1335;
                sin_lut[124] = 12'd1342; sin_lut[125] = 12'd1350; sin_lut[126] = 12'd1357; sin_lut[127] = 12'd1365;
                // Row 8: indices 128-143
                sin_lut[128] = 12'd1372; sin_lut[129] = 12'd1379; sin_lut[130] = 12'd1386; sin_lut[131] = 12'd1393;
                sin_lut[132] = 12'd1400; sin_lut[133] = 12'd1407; sin_lut[134] = 12'd1414; sin_lut[135] = 12'd1421;
                sin_lut[136] = 12'd1428; sin_lut[137] = 12'd1434; sin_lut[138] = 12'd1441; sin_lut[139] = 12'd1447;
                sin_lut[140] = 12'd1454; sin_lut[141] = 12'd1460; sin_lut[142] = 12'd1466; sin_lut[143] = 12'd1473;
                // Row 9: indices 144-159
                sin_lut[144] = 12'd1479; sin_lut[145] = 12'd1485; sin_lut[146] = 12'd1491; sin_lut[147] = 12'd1497;
                sin_lut[148] = 12'd1503; sin_lut[149] = 12'd1508; sin_lut[150] = 12'd1514; sin_lut[151] = 12'd1520;
                sin_lut[152] = 12'd1525; sin_lut[153] = 12'd1531; sin_lut[154] = 12'd1536; sin_lut[155] = 12'd1541;
                sin_lut[156] = 12'd1547; sin_lut[157] = 12'd1552; sin_lut[158] = 12'd1557; sin_lut[159] = 12'd1562;
                // Row 10: indices 160-175
                sin_lut[160] = 12'd1567; sin_lut[161] = 12'd1572; sin_lut[162] = 12'd1577; sin_lut[163] = 12'd1582;
                sin_lut[164] = 12'd1586; sin_lut[165] = 12'd1591; sin_lut[166] = 12'd1595; sin_lut[167] = 12'd1600;
                sin_lut[168] = 12'd1604; sin_lut[169] = 12'd1609; sin_lut[170] = 12'd1613; sin_lut[171] = 12'd1617;
                sin_lut[172] = 12'd1621; sin_lut[173] = 12'd1625; sin_lut[174] = 12'd1629; sin_lut[175] = 12'd1633;
                // Row 11: indices 176-191
                sin_lut[176] = 12'd1637; sin_lut[177] = 12'd1641; sin_lut[178] = 12'd1645; sin_lut[179] = 12'd1648;
                sin_lut[180] = 12'd1652; sin_lut[181] = 12'd1655; sin_lut[182] = 12'd1659; sin_lut[183] = 12'd1662;
                sin_lut[184] = 12'd1665; sin_lut[185] = 12'd1669; sin_lut[186] = 12'd1672; sin_lut[187] = 12'd1675;
                sin_lut[188] = 12'd1678; sin_lut[189] = 12'd1681; sin_lut[190] = 12'd1684; sin_lut[191] = 12'd1687;
                // Row 12: indices 192-207
                sin_lut[192] = 12'd1689; sin_lut[193] = 12'd1692; sin_lut[194] = 12'd1695; sin_lut[195] = 12'd1697;
                sin_lut[196] = 12'd1700; sin_lut[197] = 12'd1702; sin_lut[198] = 12'd1704; sin_lut[199] = 12'd1707;
                sin_lut[200] = 12'd1709; sin_lut[201] = 12'd1711; sin_lut[202] = 12'd1713; sin_lut[203] = 12'd1715;
                sin_lut[204] = 12'd1717; sin_lut[205] = 12'd1719; sin_lut[206] = 12'd1721; sin_lut[207] = 12'd1722;
                // Row 13: indices 208-223
                sin_lut[208] = 12'd1724; sin_lut[209] = 12'd1726; sin_lut[210] = 12'd1727; sin_lut[211] = 12'd1729;
                sin_lut[212] = 12'd1730; sin_lut[213] = 12'd1731; sin_lut[214] = 12'd1733; sin_lut[215] = 12'd1734;
                sin_lut[216] = 12'd1735; sin_lut[217] = 12'd1736; sin_lut[218] = 12'd1737; sin_lut[219] = 12'd1738;
                sin_lut[220] = 12'd1739; sin_lut[221] = 12'd1740; sin_lut[222] = 12'd1741; sin_lut[223] = 12'd1742;
                // Row 14: indices 224-239
                sin_lut[224] = 12'd1743; sin_lut[225] = 12'd1743; sin_lut[226] = 12'd1744; sin_lut[227] = 12'd1744;
                sin_lut[228] = 12'd1745; sin_lut[229] = 12'd1745; sin_lut[230] = 12'd1746; sin_lut[231] = 12'd1746;
                sin_lut[232] = 12'd1746; sin_lut[233] = 12'd1747; sin_lut[234] = 12'd1747; sin_lut[235] = 12'd1747;
                sin_lut[236] = 12'd1747; sin_lut[237] = 12'd1747; sin_lut[238] = 12'd1747; sin_lut[239] = 12'd1747;
                // Row 15: indices 240-255
                sin_lut[240] = 12'd1747; sin_lut[241] = 12'd1747; sin_lut[242] = 12'd1747; sin_lut[243] = 12'd1747;
                sin_lut[244] = 12'd1747; sin_lut[245] = 12'd1746; sin_lut[246] = 12'd1746; sin_lut[247] = 12'd1746;
                sin_lut[248] = 12'd1745; sin_lut[249] = 12'd1745; sin_lut[250] = 12'd1744; sin_lut[251] = 12'd1744;
                sin_lut[252] = 12'd1743; sin_lut[253] = 12'd1742; sin_lut[254] = 12'd1742; sin_lut[255] = 12'd2047;
            end

            // Phase to quadrant and address
            wire [1:0] quadrant = phase[PHASE_WIDTH-1:PHASE_WIDTH-2];
            wire [7:0] lut_addr_raw = phase[PHASE_WIDTH-3:PHASE_WIDTH-10];

            // Mirror address for quadrants 1 and 3
            wire [7:0] lut_addr = (quadrant[0]) ? (8'hFF - lut_addr_raw) : lut_addr_raw;

            // Look up sine value
            reg signed [OUTPUT_WIDTH-1:0] sin_raw;
            always @(posedge clk) begin
                sin_raw <= sin_lut[lut_addr];
            end

            // Apply quadrant sign corrections
            // Quadrant 0: sin +, cos +
            // Quadrant 1: sin +, cos -
            // Quadrant 2: sin -, cos -
            // Quadrant 3: sin -, cos +

            reg [1:0] quadrant_d;
            always @(posedge clk) begin
                quadrant_d <= quadrant;
            end

            always @(posedge clk) begin
                // Sine output
                case (quadrant_d)
                    2'b00: q_out <=  sin_raw;
                    2'b01: q_out <=  sin_raw;
                    2'b10: q_out <= -sin_raw;
                    2'b11: q_out <= -sin_raw;
                endcase

                // Cosine output (sine shifted by 90°)
                case (quadrant_d)
                    2'b00: i_out <=  sin_lut[(8'hFF - lut_addr)];  // cos = sin(90-x)
                    2'b01: i_out <= -sin_lut[lut_addr];
                    2'b10: i_out <= -sin_lut[(8'hFF - lut_addr)];
                    2'b11: i_out <=  sin_lut[lut_addr];
                endcase
            end

        end else begin : gen_cordic
            // CORDIC approach for ECP5 (uses more logic, no BRAM)

            localparam CORDIC_STAGES = 12;

            // CORDIC angles (atan(2^-i) in binary radians)
            wire [PHASE_WIDTH-1:0] atan_table [0:CORDIC_STAGES-1];
            assign atan_table[0]  = (1 << PHASE_WIDTH) / 4;      // 45°
            assign atan_table[1]  = 24'h0D5543;  // 26.565°
            assign atan_table[2]  = 24'h070C26;  // 14.036°
            assign atan_table[3]  = 24'h038A2C;  // 7.125°
            assign atan_table[4]  = 24'h01C6A1;  // 3.576°
            assign atan_table[5]  = 24'h00E384;  // 1.790°
            assign atan_table[6]  = 24'h0071C7;  // 0.895°
            assign atan_table[7]  = 24'h0038E5;  // 0.448°
            assign atan_table[8]  = 24'h001C73;  // 0.224°
            assign atan_table[9]  = 24'h000E39;  // 0.112°
            assign atan_table[10] = 24'h00071D;  // 0.056°
            assign atan_table[11] = 24'h00038E;  // 0.028°

            // CORDIC gain compensation (K ≈ 0.6073)
            localparam signed [15:0] CORDIC_GAIN = 16'h4DBA; // 0.6073 in Q15

            // Pipelined CORDIC
            reg signed [15:0] x [0:CORDIC_STAGES];
            reg signed [15:0] y [0:CORDIC_STAGES];
            reg [PHASE_WIDTH-1:0] z [0:CORDIC_STAGES];

            // Normalize phase to first quadrant and track rotation
            wire [1:0] quadrant = phase[PHASE_WIDTH-1:PHASE_WIDTH-2];
            wire [PHASE_WIDTH-1:0] phase_norm;

            assign phase_norm = (quadrant[0]) ?
                                (~phase[PHASE_WIDTH-3:0]) :
                                phase[PHASE_WIDTH-3:0];

            // Initialize with unit vector
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    x[0] <= 16'h4DBA;  // Start with K pre-applied
                    y[0] <= 16'h0000;
                    z[0] <= phase_norm;
                end else begin
                    x[0] <= 16'h4DBA;
                    y[0] <= 16'h0000;
                    z[0] <= phase_norm;
                end
            end

            // CORDIC iterations
            genvar i;
            for (i = 0; i < CORDIC_STAGES; i = i + 1) begin : cordic_stage
                    wire dir = z[i][PHASE_WIDTH-1];  // Sign bit determines direction

                    always @(posedge clk) begin
                        if (dir) begin
                            // Rotate clockwise
                            x[i+1] <= x[i] + (y[i] >>> i);
                            y[i+1] <= y[i] - (x[i] >>> i);
                            z[i+1] <= z[i] + atan_table[i];
                        end else begin
                            // Rotate counter-clockwise
                            x[i+1] <= x[i] - (y[i] >>> i);
                            y[i+1] <= y[i] + (x[i] >>> i);
                            z[i+1] <= z[i] - atan_table[i];
                        end
                    end
                end

            // Apply quadrant correction
            reg [1:0] quadrant_pipe [0:CORDIC_STAGES];
            always @(posedge clk) begin
                quadrant_pipe[0] <= quadrant;
            end

            genvar j;
            for (j = 0; j < CORDIC_STAGES; j = j + 1) begin : quad_pipe
                always @(posedge clk) begin
                    quadrant_pipe[j+1] <= quadrant_pipe[j];
                end
            end

            // Output with quadrant correction
            always @(posedge clk) begin
                case (quadrant_pipe[CORDIC_STAGES])
                    2'b00: begin
                        i_out <= x[CORDIC_STAGES][15:15-OUTPUT_WIDTH+1];
                        q_out <= y[CORDIC_STAGES][15:15-OUTPUT_WIDTH+1];
                    end
                    2'b01: begin
                        i_out <= -y[CORDIC_STAGES][15:15-OUTPUT_WIDTH+1];
                        q_out <= x[CORDIC_STAGES][15:15-OUTPUT_WIDTH+1];
                    end
                    2'b10: begin
                        i_out <= -x[CORDIC_STAGES][15:15-OUTPUT_WIDTH+1];
                        q_out <= -y[CORDIC_STAGES][15:15-OUTPUT_WIDTH+1];
                    end
                    2'b11: begin
                        i_out <= y[CORDIC_STAGES][15:15-OUTPUT_WIDTH+1];
                        q_out <= -x[CORDIC_STAGES][15:15-OUTPUT_WIDTH+1];
                    end
                endcase
            end
        end
    endgenerate

endmodule
