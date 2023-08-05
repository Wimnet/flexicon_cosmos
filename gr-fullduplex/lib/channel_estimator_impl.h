/* -*- c++ -*- */
/*
 * Copyright 2022 gr-fullduplex author.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_FULLDUPLEX_CHANNEL_ESTIMATOR_IMPL_H
#define INCLUDED_FULLDUPLEX_CHANNEL_ESTIMATOR_IMPL_H

#include <fullduplex/channel_estimator.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <fftw3.h>
#include <cmath>
#include <complex>
#include <gnuradio/expj.h>
#include <gnuradio/math.h>
#include <volk/volk.h>
#include <stdlib.h>
#include <queue>

namespace gr
{
  namespace fullduplex
  {

    class channel_estimator_impl : public channel_estimator
    {
    private:
      bool d_debug;
      uint64_t d_frame_len;
      uint64_t d_estim_pkt_prd;
      unsigned int d_pkt_counter = 0; // Is this the global packet count? YES IT IS
      uint64_t d_packet_count;        // Is this the "local" packet count for averaging?
      uint64_t d_ofdm_sym_len;
      uint64_t d_n_subcarriers;
      uint64_t d_num_packet;
      uint64_t d_cp_len;
      uint64_t d_active_subcarriers;
      uint64_t d_pilot_pos;
      float d_premultiplier;
      bool d_requesting_estimate;

      std::queue<std::vector<gr_complex>> packet_queue;

      std::vector<gr_complex> d_old_est = std::vector<gr_complex>(d_active_subcarriers);

      // Tx Pilots
      std::vector<gr_complex> d_sig_pilot = std::vector<gr_complex>(2 * d_ofdm_sym_len);
      void load_pilots()
      {
        d_sig_pilot[0] = gr_complex(0.040406, 0.060609);
        d_sig_pilot[1] = gr_complex(-0.133884, -0.004820);
        d_sig_pilot[2] = gr_complex(-0.201962, 0.020896);
        d_sig_pilot[3] = gr_complex(0.143982, 0.031263);
        d_sig_pilot[4] = gr_complex(0.040620, 0.036383);
        d_sig_pilot[5] = gr_complex(0.052793, -0.005394);
        d_sig_pilot[6] = gr_complex(0.026307, 0.013965);
        d_sig_pilot[7] = gr_complex(-0.007893, -0.099068);
        d_sig_pilot[8] = gr_complex(0.025783, -0.048774);
        d_sig_pilot[9] = gr_complex(-0.039561, 0.024610);
        d_sig_pilot[10] = gr_complex(0.041165, 0.057548);
        d_sig_pilot[11] = gr_complex(0.004856, 0.035321);
        d_sig_pilot[12] = gr_complex(-0.004402, -0.082023);
        d_sig_pilot[13] = gr_complex(0.088905, 0.018569);
        d_sig_pilot[14] = gr_complex(-0.027376, 0.026346);
        d_sig_pilot[15] = gr_complex(-0.069822, 0.029407);
        d_sig_pilot[16] = gr_complex(0.064940, -0.001568);
        d_sig_pilot[17] = gr_complex(0.020500, 0.018805);
        d_sig_pilot[18] = gr_complex(-0.014237, 0.070996);
        d_sig_pilot[19] = gr_complex(-0.001405, 0.088381);
        d_sig_pilot[20] = gr_complex(0.008563, -0.057910);
        d_sig_pilot[21] = gr_complex(-0.000118, 0.030549);
        d_sig_pilot[22] = gr_complex(-0.024889, 0.006510);
        d_sig_pilot[23] = gr_complex(-0.012755, -0.018807);
        d_sig_pilot[24] = gr_complex(-0.003316, 0.014938);
        d_sig_pilot[25] = gr_complex(-0.051644, -0.045193);
        d_sig_pilot[26] = gr_complex(0.000706, 0.061681);
        d_sig_pilot[27] = gr_complex(0.014943, -0.014239);
        d_sig_pilot[28] = gr_complex(0.001189, 0.040681);
        d_sig_pilot[29] = gr_complex(0.006336, 0.040932);
        d_sig_pilot[30] = gr_complex(0.019915, 0.068689);
        d_sig_pilot[31] = gr_complex(0.093277, 0.038685);
        d_sig_pilot[32] = gr_complex(-0.004283, 0.011496);
        d_sig_pilot[33] = gr_complex(-0.057756, 0.033024);
        d_sig_pilot[34] = gr_complex(0.039470, -0.024211);
        d_sig_pilot[35] = gr_complex(-0.050999, -0.012346);
        d_sig_pilot[36] = gr_complex(-0.059725, 0.047809);
        d_sig_pilot[37] = gr_complex(0.050409, 0.023301);
        d_sig_pilot[38] = gr_complex(-0.011958, 0.020206);
        d_sig_pilot[39] = gr_complex(0.034603, -0.027731);
        d_sig_pilot[40] = gr_complex(0.031000, -0.087483);
        d_sig_pilot[41] = gr_complex(-0.030065, -0.041756);
        d_sig_pilot[42] = gr_complex(0.060306, -0.083687);
        d_sig_pilot[43] = gr_complex(0.104423, -0.039095);
        d_sig_pilot[44] = gr_complex(0.033828, 0.034169);
        d_sig_pilot[45] = gr_complex(0.064052, 0.005027);
        d_sig_pilot[46] = gr_complex(-0.011716, 0.016074);
        d_sig_pilot[47] = gr_complex(-0.094371, 0.013032);
        d_sig_pilot[48] = gr_complex(-0.049077, -0.036747);
        d_sig_pilot[49] = gr_complex(-0.047277, -0.056654);
        d_sig_pilot[50] = gr_complex(0.033899, 0.050394);
        d_sig_pilot[51] = gr_complex(-0.018856, 0.034317);
        d_sig_pilot[52] = gr_complex(-0.087672, -0.017493);
        d_sig_pilot[53] = gr_complex(-0.017110, 0.025498);
        d_sig_pilot[54] = gr_complex(0.004363, 0.057428);
        d_sig_pilot[55] = gr_complex(-0.078640, 0.048100);
        d_sig_pilot[56] = gr_complex(-0.045850, -0.065748);
        d_sig_pilot[57] = gr_complex(0.021961, 0.038584);
        d_sig_pilot[58] = gr_complex(-0.021962, 0.050281);
        d_sig_pilot[59] = gr_complex(0.000151, -0.062674);
        d_sig_pilot[60] = gr_complex(0.049271, 0.005338);
        d_sig_pilot[61] = gr_complex(0.012328, 0.025628);
        d_sig_pilot[62] = gr_complex(-0.048764, -0.007294);
        d_sig_pilot[63] = gr_complex(-0.031369, -0.010145);
        d_sig_pilot[64] = gr_complex(-0.130264, -0.202031);
        d_sig_pilot[65] = gr_complex(-0.028743, -0.038895);
        d_sig_pilot[66] = gr_complex(0.062022, -0.061130);
        d_sig_pilot[67] = gr_complex(-0.050846, 0.019249);
        d_sig_pilot[68] = gr_complex(0.061210, -0.014848);
        d_sig_pilot[69] = gr_complex(0.037897, -0.086170);
        d_sig_pilot[70] = gr_complex(0.043598, 0.016134);
        d_sig_pilot[71] = gr_complex(0.048883, -0.069215);
        d_sig_pilot[72] = gr_complex(0.004538, -0.000094);
        d_sig_pilot[73] = gr_complex(-0.042622, -0.050426);
        d_sig_pilot[74] = gr_complex(0.050437, 0.048491);
        d_sig_pilot[75] = gr_complex(-0.013121, -0.020034);
        d_sig_pilot[76] = gr_complex(-0.055236, 0.073672);
        d_sig_pilot[77] = gr_complex(0.091193, -0.071442);
        d_sig_pilot[78] = gr_complex(-0.035037, -0.006278);
        d_sig_pilot[79] = gr_complex(0.048797, 0.036206);
        d_sig_pilot[80] = gr_complex(0.056707, -0.010447);
        d_sig_pilot[81] = gr_complex(0.094041, 0.007664);
        d_sig_pilot[82] = gr_complex(-0.022433, 0.015645);
        d_sig_pilot[83] = gr_complex(0.077089, 0.069136);
        d_sig_pilot[84] = gr_complex(0.094844, 0.058516);
        d_sig_pilot[85] = gr_complex(-0.021084, -0.007160);
        d_sig_pilot[86] = gr_complex(0.016825, -0.016351);
        d_sig_pilot[87] = gr_complex(-0.111656, 0.070613);
        d_sig_pilot[88] = gr_complex(0.058866, -0.004409);
        d_sig_pilot[89] = gr_complex(-0.018961, -0.013593);
        d_sig_pilot[90] = gr_complex(-0.008811, -0.095763);
        d_sig_pilot[91] = gr_complex(0.000615, -0.038290);
        d_sig_pilot[92] = gr_complex(-0.009615, -0.027485);
        d_sig_pilot[93] = gr_complex(-0.001512, -0.000066);
        d_sig_pilot[94] = gr_complex(-0.027170, -0.055685);
        d_sig_pilot[95] = gr_complex(0.048042, 0.004387);
        d_sig_pilot[96] = gr_complex(-0.024965, 0.017075);
        d_sig_pilot[97] = gr_complex(-0.063603, 0.051473);
        d_sig_pilot[98] = gr_complex(0.029463, 0.021157);
        d_sig_pilot[99] = gr_complex(0.041957, -0.020252);
        d_sig_pilot[100] = gr_complex(-0.006771, -0.021998);
        d_sig_pilot[101] = gr_complex(0.001506, -0.025735);
        d_sig_pilot[102] = gr_complex(-0.120350, -0.069415);
        d_sig_pilot[103] = gr_complex(-0.057970, 0.068349);
        d_sig_pilot[104] = gr_complex(0.024356, 0.031926);
        d_sig_pilot[105] = gr_complex(-0.082850, -0.112458);
        d_sig_pilot[106] = gr_complex(-0.058152, 0.027099);
        d_sig_pilot[107] = gr_complex(-0.058055, 0.053410);
        d_sig_pilot[108] = gr_complex(-0.029366, -0.018271);
        d_sig_pilot[109] = gr_complex(0.082018, -0.022356);
        d_sig_pilot[110] = gr_complex(0.051042, 0.008649);
        d_sig_pilot[111] = gr_complex(0.070567, -0.023733);
        d_sig_pilot[112] = gr_complex(-0.039593, 0.029115);
        d_sig_pilot[113] = gr_complex(-0.036036, -0.051537);
        d_sig_pilot[114] = gr_complex(-0.041044, 0.056777);
        d_sig_pilot[115] = gr_complex(-0.068268, 0.100477);
        d_sig_pilot[116] = gr_complex(0.017798, 0.081437);
        d_sig_pilot[117] = gr_complex(0.116592, -0.029581);
        d_sig_pilot[118] = gr_complex(0.079385, 0.026213);
        d_sig_pilot[119] = gr_complex(-0.028811, 0.042502);
        d_sig_pilot[120] = gr_complex(-0.011590, 0.101784);
        d_sig_pilot[121] = gr_complex(-0.074467, -0.023819);
        d_sig_pilot[122] = gr_complex(0.034481, 0.007164);
        d_sig_pilot[123] = gr_complex(-0.014139, -0.078225);
        d_sig_pilot[124] = gr_complex(0.043685, 0.038386);
        d_sig_pilot[125] = gr_complex(-0.000088, 0.119134);
        d_sig_pilot[126] = gr_complex(-0.027655, -0.226222);
        d_sig_pilot[127] = gr_complex(-0.024021, -0.031702);
        d_sig_pilot[128] = gr_complex(0.000000, -0.020203);
        d_sig_pilot[129] = gr_complex(0.097672, 0.071144);
        d_sig_pilot[130] = gr_complex(0.019840, -0.030841);
        d_sig_pilot[131] = gr_complex(0.009679, 0.071967);
        d_sig_pilot[132] = gr_complex(-0.090857, -0.023126);
        d_sig_pilot[133] = gr_complex(0.025508, 0.040340);
        d_sig_pilot[134] = gr_complex(0.043303, 0.111353);
        d_sig_pilot[135] = gr_complex(0.044132, -0.071977);
        d_sig_pilot[136] = gr_complex(0.048930, 0.007399);
        d_sig_pilot[137] = gr_complex(0.002283, -0.048282);
        d_sig_pilot[138] = gr_complex(0.009285, 0.012107);
        d_sig_pilot[139] = gr_complex(0.009089, 0.070968);
        d_sig_pilot[140] = gr_complex(-0.024343, 0.018754);
        d_sig_pilot[141] = gr_complex(-0.010187, -0.043914);
        d_sig_pilot[142] = gr_complex(0.036320, -0.079988);
        d_sig_pilot[143] = gr_complex(0.024966, -0.017825);
        d_sig_pilot[144] = gr_complex(-0.016644, 0.004836);
        d_sig_pilot[145] = gr_complex(-0.061349, -0.113091);
        d_sig_pilot[146] = gr_complex(-0.002406, -0.012651);
        d_sig_pilot[147] = gr_complex(-0.092443, 0.044658);
        d_sig_pilot[148] = gr_complex(-0.035966, -0.057870);
        d_sig_pilot[149] = gr_complex(0.026993, -0.025015);
        d_sig_pilot[150] = gr_complex(-0.076922, 0.010921);
        d_sig_pilot[151] = gr_complex(-0.005580, -0.009283);
        d_sig_pilot[152] = gr_complex(-0.005684, -0.007189);
        d_sig_pilot[153] = gr_complex(-0.029279, 0.053604);
        d_sig_pilot[154] = gr_complex(0.058967, 0.056754);
        d_sig_pilot[155] = gr_complex(0.048875, -0.033311);
        d_sig_pilot[156] = gr_complex(0.061370, 0.006763);
        d_sig_pilot[157] = gr_complex(-0.036133, 0.034443);
        d_sig_pilot[158] = gr_complex(0.036457, 0.015133);
        d_sig_pilot[159] = gr_complex(0.056143, 0.014110);
        d_sig_pilot[160] = gr_complex(-0.089998, 0.001972);
        d_sig_pilot[161] = gr_complex(0.027220, -0.042137);
        d_sig_pilot[162] = gr_complex(0.046770, -0.019674);
        d_sig_pilot[163] = gr_complex(0.009731, 0.010883);
        d_sig_pilot[164] = gr_complex(0.033738, 0.103871);
        d_sig_pilot[165] = gr_complex(0.032109, 0.015406);
        d_sig_pilot[166] = gr_complex(-0.097561, -0.008145);
        d_sig_pilot[167] = gr_complex(-0.084566, -0.009540);
        d_sig_pilot[168] = gr_complex(-0.000182, 0.041839);
        d_sig_pilot[169] = gr_complex(0.070325, -0.019368);
        d_sig_pilot[170] = gr_complex(0.042014, -0.025838);
        d_sig_pilot[171] = gr_complex(-0.008297, -0.007201);
        d_sig_pilot[172] = gr_complex(0.023680, -0.048501);
        d_sig_pilot[173] = gr_complex(-0.054197, 0.083501);
        d_sig_pilot[174] = gr_complex(-0.004478, -0.046444);
        d_sig_pilot[175] = gr_complex(-0.019855, 0.041784);
        d_sig_pilot[176] = gr_complex(0.027719, 0.063685);
        d_sig_pilot[177] = gr_complex(0.044830, 0.027557);
        d_sig_pilot[178] = gr_complex(0.040455, -0.052966);
        d_sig_pilot[179] = gr_complex(-0.001996, 0.018877);
        d_sig_pilot[180] = gr_complex(-0.111876, 0.104163);
        d_sig_pilot[181] = gr_complex(-0.006097, -0.052669);
        d_sig_pilot[182] = gr_complex(-0.057640, -0.028465);
        d_sig_pilot[183] = gr_complex(0.031732, 0.056968);
        d_sig_pilot[184] = gr_complex(-0.021862, 0.030893);
        d_sig_pilot[185] = gr_complex(-0.054635, 0.112473);
        d_sig_pilot[186] = gr_complex(0.038739, 0.031105);
        d_sig_pilot[187] = gr_complex(-0.030816, 0.082688);
        d_sig_pilot[188] = gr_complex(0.096766, -0.071055);
        d_sig_pilot[189] = gr_complex(-0.017665, -0.032652);
        d_sig_pilot[190] = gr_complex(-0.014963, 0.079782);
        d_sig_pilot[191] = gr_complex(0.039790, -0.048247);
        d_sig_pilot[192] = gr_complex(-0.044830, 0.000000);
        d_sig_pilot[193] = gr_complex(0.040344, -0.117638);
        d_sig_pilot[194] = gr_complex(0.017281, 0.015165);
        d_sig_pilot[195] = gr_complex(-0.115405, 0.049897);
        d_sig_pilot[196] = gr_complex(-0.131420, 0.058648);
        d_sig_pilot[197] = gr_complex(0.010338, -0.029019);
        d_sig_pilot[198] = gr_complex(0.036591, 0.017527);
        d_sig_pilot[199] = gr_complex(-0.068350, -0.031204);
        d_sig_pilot[200] = gr_complex(0.169722, 0.014532);
        d_sig_pilot[201] = gr_complex(0.118870, -0.003230);
        d_sig_pilot[202] = gr_complex(-0.063166, -0.013763);
        d_sig_pilot[203] = gr_complex(0.060838, -0.003376);
        d_sig_pilot[204] = gr_complex(-0.047103, -0.107732);
        d_sig_pilot[205] = gr_complex(0.070375, -0.102229);
        d_sig_pilot[206] = gr_complex(0.045635, 0.015452);
        d_sig_pilot[207] = gr_complex(-0.054460, -0.027486);
        d_sig_pilot[208] = gr_complex(0.029685, -0.100571);
        d_sig_pilot[209] = gr_complex(0.032047, -0.008412);
        d_sig_pilot[210] = gr_complex(-0.028420, -0.049466);
        d_sig_pilot[211] = gr_complex(0.002438, 0.044282);
        d_sig_pilot[212] = gr_complex(0.031825, -0.049340);
        d_sig_pilot[213] = gr_complex(-0.026322, 0.003111);
        d_sig_pilot[214] = gr_complex(-0.041349, -0.031904);
        d_sig_pilot[215] = gr_complex(-0.091725, 0.025389);
        d_sig_pilot[216] = gr_complex(-0.038709, -0.014498);
        d_sig_pilot[217] = gr_complex(0.009775, -0.097232);
        d_sig_pilot[218] = gr_complex(0.044382, -0.055806);
        d_sig_pilot[219] = gr_complex(-0.009670, -0.047379);
        d_sig_pilot[220] = gr_complex(-0.017307, -0.017740);
        d_sig_pilot[221] = gr_complex(0.050513, -0.026235);
        d_sig_pilot[222] = gr_complex(0.007184, 0.062341);
        d_sig_pilot[223] = gr_complex(-0.016914, 0.042322);
        d_sig_pilot[224] = gr_complex(-0.015441, -0.030544);
        d_sig_pilot[225] = gr_complex(0.023454, 0.019921);
        d_sig_pilot[226] = gr_complex(0.032053, -0.029849);
        d_sig_pilot[227] = gr_complex(-0.039034, -0.038610);
        d_sig_pilot[228] = gr_complex(0.099329, 0.051077);
        d_sig_pilot[229] = gr_complex(-0.033288, -0.028012);
        d_sig_pilot[230] = gr_complex(0.017257, 0.006862);
        d_sig_pilot[231] = gr_complex(0.124475, 0.031907);
        d_sig_pilot[232] = gr_complex(-0.034772, -0.013219);
        d_sig_pilot[233] = gr_complex(0.020136, 0.052856);
        d_sig_pilot[234] = gr_complex(-0.060309, -0.013017);
        d_sig_pilot[235] = gr_complex(-0.027873, -0.112628);
        d_sig_pilot[236] = gr_complex(-0.071755, 0.076056);
        d_sig_pilot[237] = gr_complex(0.006072, 0.017385);
        d_sig_pilot[238] = gr_complex(0.000221, -0.001064);
        d_sig_pilot[239] = gr_complex(-0.051305, -0.002841);
        d_sig_pilot[240] = gr_complex(-0.019861, -0.002177);
        d_sig_pilot[241] = gr_complex(0.027264, -0.021577);
        d_sig_pilot[242] = gr_complex(0.020808, -0.046434);
        d_sig_pilot[243] = gr_complex(-0.035043, 0.052870);
        d_sig_pilot[244] = gr_complex(0.028610, 0.023931);
        d_sig_pilot[245] = gr_complex(0.044961, 0.019442);
        d_sig_pilot[246] = gr_complex(-0.020900, -0.056648);
        d_sig_pilot[247] = gr_complex(-0.101833, -0.050838);
        d_sig_pilot[248] = gr_complex(0.014269, -0.001896);
        d_sig_pilot[249] = gr_complex(-0.039636, 0.030524);
        d_sig_pilot[250] = gr_complex(0.015859, 0.011837);
        d_sig_pilot[251] = gr_complex(0.044485, 0.016893);
        d_sig_pilot[252] = gr_complex(0.057088, -0.028764);
        d_sig_pilot[253] = gr_complex(0.025546, 0.022265);
        d_sig_pilot[254] = gr_complex(-0.031174, 0.054318);
        d_sig_pilot[255] = gr_complex(-0.021391, 0.048997);
        d_sig_pilot[256] = gr_complex(0.040406, 0.060609);
        d_sig_pilot[257] = gr_complex(-0.133884, -0.004820);
        d_sig_pilot[258] = gr_complex(-0.201962, 0.020896);
        d_sig_pilot[259] = gr_complex(0.143982, 0.031263);
        d_sig_pilot[260] = gr_complex(0.040620, 0.036383);
        d_sig_pilot[261] = gr_complex(0.052793, -0.005394);
        d_sig_pilot[262] = gr_complex(0.026307, 0.013965);
        d_sig_pilot[263] = gr_complex(-0.007893, -0.099068);
        d_sig_pilot[264] = gr_complex(0.025783, -0.048774);
        d_sig_pilot[265] = gr_complex(-0.039561, 0.024610);
        d_sig_pilot[266] = gr_complex(0.041165, 0.057548);
        d_sig_pilot[267] = gr_complex(0.004856, 0.035321);
        d_sig_pilot[268] = gr_complex(-0.004402, -0.082023);
        d_sig_pilot[269] = gr_complex(0.088905, 0.018569);
        d_sig_pilot[270] = gr_complex(-0.027376, 0.026346);
        d_sig_pilot[271] = gr_complex(-0.069822, 0.029407);
        d_sig_pilot[272] = gr_complex(0.064940, -0.001568);
        d_sig_pilot[273] = gr_complex(0.020500, 0.018805);
        d_sig_pilot[274] = gr_complex(-0.014237, 0.070996);
        d_sig_pilot[275] = gr_complex(-0.001405, 0.088381);
        d_sig_pilot[276] = gr_complex(0.008563, -0.057910);
        d_sig_pilot[277] = gr_complex(-0.000118, 0.030549);
        d_sig_pilot[278] = gr_complex(-0.024889, 0.006510);
        d_sig_pilot[279] = gr_complex(-0.012755, -0.018807);
        d_sig_pilot[280] = gr_complex(-0.003316, 0.014938);
        d_sig_pilot[281] = gr_complex(-0.051644, -0.045193);
        d_sig_pilot[282] = gr_complex(0.000706, 0.061681);
        d_sig_pilot[283] = gr_complex(0.014943, -0.014239);
        d_sig_pilot[284] = gr_complex(0.001189, 0.040681);
        d_sig_pilot[285] = gr_complex(0.006336, 0.040932);
        d_sig_pilot[286] = gr_complex(0.019915, 0.068689);
        d_sig_pilot[287] = gr_complex(0.093277, 0.038685);
        d_sig_pilot[288] = gr_complex(-0.004283, 0.011496);
        d_sig_pilot[289] = gr_complex(-0.057756, 0.033024);
        d_sig_pilot[290] = gr_complex(0.039470, -0.024211);
        d_sig_pilot[291] = gr_complex(-0.050999, -0.012346);
        d_sig_pilot[292] = gr_complex(-0.059725, 0.047809);
        d_sig_pilot[293] = gr_complex(0.050409, 0.023301);
        d_sig_pilot[294] = gr_complex(-0.011958, 0.020206);
        d_sig_pilot[295] = gr_complex(0.034603, -0.027731);
        d_sig_pilot[296] = gr_complex(0.031000, -0.087483);
        d_sig_pilot[297] = gr_complex(-0.030065, -0.041756);
        d_sig_pilot[298] = gr_complex(0.060306, -0.083687);
        d_sig_pilot[299] = gr_complex(0.104423, -0.039095);
        d_sig_pilot[300] = gr_complex(0.033828, 0.034169);
        d_sig_pilot[301] = gr_complex(0.064052, 0.005027);
        d_sig_pilot[302] = gr_complex(-0.011716, 0.016074);
        d_sig_pilot[303] = gr_complex(-0.094371, 0.013032);
        d_sig_pilot[304] = gr_complex(-0.049077, -0.036747);
        d_sig_pilot[305] = gr_complex(-0.047277, -0.056654);
        d_sig_pilot[306] = gr_complex(0.033899, 0.050394);
        d_sig_pilot[307] = gr_complex(-0.018856, 0.034317);
        d_sig_pilot[308] = gr_complex(-0.087672, -0.017493);
        d_sig_pilot[309] = gr_complex(-0.017110, 0.025498);
        d_sig_pilot[310] = gr_complex(0.004363, 0.057428);
        d_sig_pilot[311] = gr_complex(-0.078640, 0.048100);
        d_sig_pilot[312] = gr_complex(-0.045850, -0.065748);
        d_sig_pilot[313] = gr_complex(0.021961, 0.038584);
        d_sig_pilot[314] = gr_complex(-0.021962, 0.050281);
        d_sig_pilot[315] = gr_complex(0.000151, -0.062674);
        d_sig_pilot[316] = gr_complex(0.049271, 0.005338);
        d_sig_pilot[317] = gr_complex(0.012328, 0.025628);
        d_sig_pilot[318] = gr_complex(-0.048764, -0.007294);
        d_sig_pilot[319] = gr_complex(-0.031369, -0.010145);
        /*d_sig_pilot[320] = gr_complex(0.040406, 0.060609);
        d_sig_pilot[321] = gr_complex(-0.133884, -0.004820);
        d_sig_pilot[322] = gr_complex(-0.201962, 0.020896);
        d_sig_pilot[323] = gr_complex(0.143982, 0.031263);
        d_sig_pilot[324] = gr_complex(0.040620, 0.036383);
        d_sig_pilot[325] = gr_complex(0.052793, -0.005394);
        d_sig_pilot[326] = gr_complex(0.026307, 0.013965);
        d_sig_pilot[327] = gr_complex(-0.007893, -0.099068);
        d_sig_pilot[328] = gr_complex(0.025783, -0.048774);
        d_sig_pilot[329] = gr_complex(-0.039561, 0.024610);
        d_sig_pilot[330] = gr_complex(0.041165, 0.057548);
        d_sig_pilot[331] = gr_complex(0.004856, 0.035321);
        d_sig_pilot[332] = gr_complex(-0.004402, -0.082023);
        d_sig_pilot[333] = gr_complex(0.088905, 0.018569);
        d_sig_pilot[334] = gr_complex(-0.027376, 0.026346);
        d_sig_pilot[335] = gr_complex(-0.069822, 0.029407);
        d_sig_pilot[336] = gr_complex(0.064940, -0.001568);
        d_sig_pilot[337] = gr_complex(0.020500, 0.018805);
        d_sig_pilot[338] = gr_complex(-0.014237, 0.070996);
        d_sig_pilot[339] = gr_complex(-0.001405, 0.088381);
        d_sig_pilot[340] = gr_complex(0.008563, -0.057910);
        d_sig_pilot[341] = gr_complex(-0.000118, 0.030549);
        d_sig_pilot[342] = gr_complex(-0.024889, 0.006510);
        d_sig_pilot[343] = gr_complex(-0.012755, -0.018807);
        d_sig_pilot[344] = gr_complex(-0.003316, 0.014938);
        d_sig_pilot[345] = gr_complex(-0.051644, -0.045193);
        d_sig_pilot[346] = gr_complex(0.000706, 0.061681);
        d_sig_pilot[347] = gr_complex(0.014943, -0.014239);
        d_sig_pilot[348] = gr_complex(0.001189, 0.040681);
        d_sig_pilot[349] = gr_complex(0.006336, 0.040932);
        d_sig_pilot[350] = gr_complex(0.019915, 0.068689);
        d_sig_pilot[351] = gr_complex(0.093277, 0.038685);
        d_sig_pilot[352] = gr_complex(-0.004283, 0.011496);
        d_sig_pilot[353] = gr_complex(-0.057756, 0.033024);
        d_sig_pilot[354] = gr_complex(0.039470, -0.024211);
        d_sig_pilot[355] = gr_complex(-0.050999, -0.012346);
        d_sig_pilot[356] = gr_complex(-0.059725, 0.047809);
        d_sig_pilot[357] = gr_complex(0.050409, 0.023301);
        d_sig_pilot[358] = gr_complex(-0.011958, 0.020206);
        d_sig_pilot[359] = gr_complex(0.034603, -0.027731);
        d_sig_pilot[360] = gr_complex(0.031000, -0.087483);
        d_sig_pilot[361] = gr_complex(-0.030065, -0.041756);
        d_sig_pilot[362] = gr_complex(0.060306, -0.083687);
        d_sig_pilot[363] = gr_complex(0.104423, -0.039095);
        d_sig_pilot[364] = gr_complex(0.033828, 0.034169);
        d_sig_pilot[365] = gr_complex(0.064052, 0.005027);
        d_sig_pilot[366] = gr_complex(-0.011716, 0.016074);
        d_sig_pilot[367] = gr_complex(-0.094371, 0.013032);
        d_sig_pilot[368] = gr_complex(-0.049077, -0.036747);
        d_sig_pilot[369] = gr_complex(-0.047277, -0.056654);
        d_sig_pilot[370] = gr_complex(0.033899, 0.050394);
        d_sig_pilot[371] = gr_complex(-0.018856, 0.034317);
        d_sig_pilot[372] = gr_complex(-0.087672, -0.017493);
        d_sig_pilot[373] = gr_complex(-0.017110, 0.025498);
        d_sig_pilot[374] = gr_complex(0.004363, 0.057428);
        d_sig_pilot[375] = gr_complex(-0.078640, 0.048100);
        d_sig_pilot[376] = gr_complex(-0.045850, -0.065748);
        d_sig_pilot[377] = gr_complex(0.021961, 0.038584);
        d_sig_pilot[378] = gr_complex(-0.021962, 0.050281);
        d_sig_pilot[379] = gr_complex(0.000151, -0.062674);
        d_sig_pilot[380] = gr_complex(0.049271, 0.005338);
        d_sig_pilot[381] = gr_complex(0.012328, 0.025628);
        d_sig_pilot[382] = gr_complex(-0.048764, -0.007294);
        d_sig_pilot[383] = gr_complex(-0.031369, -0.010145);*/
        d_sig_pilot[384] = gr_complex(-0.130264, -0.202031);
        d_sig_pilot[385] = gr_complex(-0.028743, -0.038895);
        d_sig_pilot[386] = gr_complex(0.062022, -0.061130);
        d_sig_pilot[387] = gr_complex(-0.050846, 0.019249);
        d_sig_pilot[388] = gr_complex(0.061210, -0.014848);
        d_sig_pilot[389] = gr_complex(0.037897, -0.086170);
        d_sig_pilot[390] = gr_complex(0.043598, 0.016134);
        d_sig_pilot[391] = gr_complex(0.048883, -0.069215);
        d_sig_pilot[392] = gr_complex(0.004538, -0.000094);
        d_sig_pilot[393] = gr_complex(-0.042622, -0.050426);
        d_sig_pilot[394] = gr_complex(0.050437, 0.048491);
        d_sig_pilot[395] = gr_complex(-0.013121, -0.020034);
        d_sig_pilot[396] = gr_complex(-0.055236, 0.073672);
        d_sig_pilot[397] = gr_complex(0.091193, -0.071442);
        d_sig_pilot[398] = gr_complex(-0.035037, -0.006278);
        d_sig_pilot[399] = gr_complex(0.048797, 0.036206);
        d_sig_pilot[400] = gr_complex(0.056707, -0.010447);
        d_sig_pilot[401] = gr_complex(0.094041, 0.007664);
        d_sig_pilot[402] = gr_complex(-0.022433, 0.015645);
        d_sig_pilot[403] = gr_complex(0.077089, 0.069136);
        d_sig_pilot[404] = gr_complex(0.094844, 0.058516);
        d_sig_pilot[405] = gr_complex(-0.021084, -0.007160);
        d_sig_pilot[406] = gr_complex(0.016825, -0.016351);
        d_sig_pilot[407] = gr_complex(-0.111656, 0.070613);
        d_sig_pilot[408] = gr_complex(0.058866, -0.004409);
        d_sig_pilot[409] = gr_complex(-0.018961, -0.013593);
        d_sig_pilot[410] = gr_complex(-0.008811, -0.095763);
        d_sig_pilot[411] = gr_complex(0.000615, -0.038290);
        d_sig_pilot[412] = gr_complex(-0.009615, -0.027485);
        d_sig_pilot[413] = gr_complex(-0.001512, -0.000066);
        d_sig_pilot[414] = gr_complex(-0.027170, -0.055685);
        d_sig_pilot[415] = gr_complex(0.048042, 0.004387);
        d_sig_pilot[416] = gr_complex(-0.024965, 0.017075);
        d_sig_pilot[417] = gr_complex(-0.063603, 0.051473);
        d_sig_pilot[418] = gr_complex(0.029463, 0.021157);
        d_sig_pilot[419] = gr_complex(0.041957, -0.020252);
        d_sig_pilot[420] = gr_complex(-0.006771, -0.021998);
        d_sig_pilot[421] = gr_complex(0.001506, -0.025735);
        d_sig_pilot[422] = gr_complex(-0.120350, -0.069415);
        d_sig_pilot[423] = gr_complex(-0.057970, 0.068349);
        d_sig_pilot[424] = gr_complex(0.024356, 0.031926);
        d_sig_pilot[425] = gr_complex(-0.082850, -0.112458);
        d_sig_pilot[426] = gr_complex(-0.058152, 0.027099);
        d_sig_pilot[427] = gr_complex(-0.058055, 0.053410);
        d_sig_pilot[428] = gr_complex(-0.029366, -0.018271);
        d_sig_pilot[429] = gr_complex(0.082018, -0.022356);
        d_sig_pilot[430] = gr_complex(0.051042, 0.008649);
        d_sig_pilot[431] = gr_complex(0.070567, -0.023733);
        d_sig_pilot[432] = gr_complex(-0.039593, 0.029115);
        d_sig_pilot[433] = gr_complex(-0.036036, -0.051537);
        d_sig_pilot[434] = gr_complex(-0.041044, 0.056777);
        d_sig_pilot[435] = gr_complex(-0.068268, 0.100477);
        d_sig_pilot[436] = gr_complex(0.017798, 0.081437);
        d_sig_pilot[437] = gr_complex(0.116592, -0.029581);
        d_sig_pilot[438] = gr_complex(0.079385, 0.026213);
        d_sig_pilot[439] = gr_complex(-0.028811, 0.042502);
        d_sig_pilot[440] = gr_complex(-0.011590, 0.101784);
        d_sig_pilot[441] = gr_complex(-0.074467, -0.023819);
        d_sig_pilot[442] = gr_complex(0.034481, 0.007164);
        d_sig_pilot[443] = gr_complex(-0.014139, -0.078225);
        d_sig_pilot[444] = gr_complex(0.043685, 0.038386);
        d_sig_pilot[445] = gr_complex(-0.000088, 0.119134);
        d_sig_pilot[446] = gr_complex(-0.027655, -0.226222);
        d_sig_pilot[447] = gr_complex(-0.024021, -0.031702);
        d_sig_pilot[448] = gr_complex(0.000000, -0.020203);
        d_sig_pilot[449] = gr_complex(0.097672, 0.071144);
        d_sig_pilot[450] = gr_complex(0.019840, -0.030841);
        d_sig_pilot[451] = gr_complex(0.009679, 0.071967);
        d_sig_pilot[452] = gr_complex(-0.090857, -0.023126);
        d_sig_pilot[453] = gr_complex(0.025508, 0.040340);
        d_sig_pilot[454] = gr_complex(0.043303, 0.111353);
        d_sig_pilot[455] = gr_complex(0.044132, -0.071977);
        d_sig_pilot[456] = gr_complex(0.048930, 0.007399);
        d_sig_pilot[457] = gr_complex(0.002283, -0.048282);
        d_sig_pilot[458] = gr_complex(0.009285, 0.012107);
        d_sig_pilot[459] = gr_complex(0.009089, 0.070968);
        d_sig_pilot[460] = gr_complex(-0.024343, 0.018754);
        d_sig_pilot[461] = gr_complex(-0.010187, -0.043914);
        d_sig_pilot[462] = gr_complex(0.036320, -0.079988);
        d_sig_pilot[463] = gr_complex(0.024966, -0.017825);
        d_sig_pilot[464] = gr_complex(-0.016644, 0.004836);
        d_sig_pilot[465] = gr_complex(-0.061349, -0.113091);
        d_sig_pilot[466] = gr_complex(-0.002406, -0.012651);
        d_sig_pilot[467] = gr_complex(-0.092443, 0.044658);
        d_sig_pilot[468] = gr_complex(-0.035966, -0.057870);
        d_sig_pilot[469] = gr_complex(0.026993, -0.025015);
        d_sig_pilot[470] = gr_complex(-0.076922, 0.010921);
        d_sig_pilot[471] = gr_complex(-0.005580, -0.009283);
        d_sig_pilot[472] = gr_complex(-0.005684, -0.007189);
        d_sig_pilot[473] = gr_complex(-0.029279, 0.053604);
        d_sig_pilot[474] = gr_complex(0.058967, 0.056754);
        d_sig_pilot[475] = gr_complex(0.048875, -0.033311);
        d_sig_pilot[476] = gr_complex(0.061370, 0.006763);
        d_sig_pilot[477] = gr_complex(-0.036133, 0.034443);
        d_sig_pilot[478] = gr_complex(0.036457, 0.015133);
        d_sig_pilot[479] = gr_complex(0.056143, 0.014110);
        d_sig_pilot[480] = gr_complex(-0.089998, 0.001972);
        d_sig_pilot[481] = gr_complex(0.027220, -0.042137);
        d_sig_pilot[482] = gr_complex(0.046770, -0.019674);
        d_sig_pilot[483] = gr_complex(0.009731, 0.010883);
        d_sig_pilot[484] = gr_complex(0.033738, 0.103871);
        d_sig_pilot[485] = gr_complex(0.032109, 0.015406);
        d_sig_pilot[486] = gr_complex(-0.097561, -0.008145);
        d_sig_pilot[487] = gr_complex(-0.084566, -0.009540);
        d_sig_pilot[488] = gr_complex(-0.000182, 0.041839);
        d_sig_pilot[489] = gr_complex(0.070325, -0.019368);
        d_sig_pilot[490] = gr_complex(0.042014, -0.025838);
        d_sig_pilot[491] = gr_complex(-0.008297, -0.007201);
        d_sig_pilot[492] = gr_complex(0.023680, -0.048501);
        d_sig_pilot[493] = gr_complex(-0.054197, 0.083501);
        d_sig_pilot[494] = gr_complex(-0.004478, -0.046444);
        d_sig_pilot[495] = gr_complex(-0.019855, 0.041784);
        d_sig_pilot[496] = gr_complex(0.027719, 0.063685);
        d_sig_pilot[497] = gr_complex(0.044830, 0.027557);
        d_sig_pilot[498] = gr_complex(0.040455, -0.052966);
        d_sig_pilot[499] = gr_complex(-0.001996, 0.018877);
        d_sig_pilot[500] = gr_complex(-0.111876, 0.104163);
        d_sig_pilot[501] = gr_complex(-0.006097, -0.052669);
        d_sig_pilot[502] = gr_complex(-0.057640, -0.028465);
        d_sig_pilot[503] = gr_complex(0.031732, 0.056968);
        d_sig_pilot[504] = gr_complex(-0.021862, 0.030893);
        d_sig_pilot[505] = gr_complex(-0.054635, 0.112473);
        d_sig_pilot[506] = gr_complex(0.038739, 0.031105);
        d_sig_pilot[507] = gr_complex(-0.030816, 0.082688);
        d_sig_pilot[508] = gr_complex(0.096766, -0.071055);
        d_sig_pilot[509] = gr_complex(-0.017665, -0.032652);
        d_sig_pilot[510] = gr_complex(-0.014963, 0.079782);
        d_sig_pilot[511] = gr_complex(0.039790, -0.048247);
        d_sig_pilot[512] = gr_complex(-0.044830, 0.000000);
        d_sig_pilot[513] = gr_complex(0.040344, -0.117638);
        d_sig_pilot[514] = gr_complex(0.017281, 0.015165);
        d_sig_pilot[515] = gr_complex(-0.115405, 0.049897);
        d_sig_pilot[516] = gr_complex(-0.131420, 0.058648);
        d_sig_pilot[517] = gr_complex(0.010338, -0.029019);
        d_sig_pilot[518] = gr_complex(0.036591, 0.017527);
        d_sig_pilot[519] = gr_complex(-0.068350, -0.031204);
        d_sig_pilot[520] = gr_complex(0.169722, 0.014532);
        d_sig_pilot[521] = gr_complex(0.118870, -0.003230);
        d_sig_pilot[522] = gr_complex(-0.063166, -0.013763);
        d_sig_pilot[523] = gr_complex(0.060838, -0.003376);
        d_sig_pilot[524] = gr_complex(-0.047103, -0.107732);
        d_sig_pilot[525] = gr_complex(0.070375, -0.102229);
        d_sig_pilot[526] = gr_complex(0.045635, 0.015452);
        d_sig_pilot[527] = gr_complex(-0.054460, -0.027486);
        d_sig_pilot[528] = gr_complex(0.029685, -0.100571);
        d_sig_pilot[529] = gr_complex(0.032047, -0.008412);
        d_sig_pilot[530] = gr_complex(-0.028420, -0.049466);
        d_sig_pilot[531] = gr_complex(0.002438, 0.044282);
        d_sig_pilot[532] = gr_complex(0.031825, -0.049340);
        d_sig_pilot[533] = gr_complex(-0.026322, 0.003111);
        d_sig_pilot[534] = gr_complex(-0.041349, -0.031904);
        d_sig_pilot[535] = gr_complex(-0.091725, 0.025389);
        d_sig_pilot[536] = gr_complex(-0.038709, -0.014498);
        d_sig_pilot[537] = gr_complex(0.009775, -0.097232);
        d_sig_pilot[538] = gr_complex(0.044382, -0.055806);
        d_sig_pilot[539] = gr_complex(-0.009670, -0.047379);
        d_sig_pilot[540] = gr_complex(-0.017307, -0.017740);
        d_sig_pilot[541] = gr_complex(0.050513, -0.026235);
        d_sig_pilot[542] = gr_complex(0.007184, 0.062341);
        d_sig_pilot[543] = gr_complex(-0.016914, 0.042322);
        d_sig_pilot[544] = gr_complex(-0.015441, -0.030544);
        d_sig_pilot[545] = gr_complex(0.023454, 0.019921);
        d_sig_pilot[546] = gr_complex(0.032053, -0.029849);
        d_sig_pilot[547] = gr_complex(-0.039034, -0.038610);
        d_sig_pilot[548] = gr_complex(0.099329, 0.051077);
        d_sig_pilot[549] = gr_complex(-0.033288, -0.028012);
        d_sig_pilot[550] = gr_complex(0.017257, 0.006862);
        d_sig_pilot[551] = gr_complex(0.124475, 0.031907);
        d_sig_pilot[552] = gr_complex(-0.034772, -0.013219);
        d_sig_pilot[553] = gr_complex(0.020136, 0.052856);
        d_sig_pilot[554] = gr_complex(-0.060309, -0.013017);
        d_sig_pilot[555] = gr_complex(-0.027873, -0.112628);
        d_sig_pilot[556] = gr_complex(-0.071755, 0.076056);
        d_sig_pilot[557] = gr_complex(0.006072, 0.017385);
        d_sig_pilot[558] = gr_complex(0.000221, -0.001064);
        d_sig_pilot[559] = gr_complex(-0.051305, -0.002841);
        d_sig_pilot[560] = gr_complex(-0.019861, -0.002177);
        d_sig_pilot[561] = gr_complex(0.027264, -0.021577);
        d_sig_pilot[562] = gr_complex(0.020808, -0.046434);
        d_sig_pilot[563] = gr_complex(-0.035043, 0.052870);
        d_sig_pilot[564] = gr_complex(0.028610, 0.023931);
        d_sig_pilot[565] = gr_complex(0.044961, 0.019442);
        d_sig_pilot[566] = gr_complex(-0.020900, -0.056648);
        d_sig_pilot[567] = gr_complex(-0.101833, -0.050838);
        d_sig_pilot[568] = gr_complex(0.014269, -0.001896);
        d_sig_pilot[569] = gr_complex(-0.039636, 0.030524);
        d_sig_pilot[570] = gr_complex(0.015859, 0.011837);
        d_sig_pilot[571] = gr_complex(0.044485, 0.016893);
        d_sig_pilot[572] = gr_complex(0.057088, -0.028764);
        d_sig_pilot[573] = gr_complex(0.025546, 0.022265);
        d_sig_pilot[574] = gr_complex(-0.031174, 0.054318);
        d_sig_pilot[575] = gr_complex(-0.021391, 0.048997);
        d_sig_pilot[576] = gr_complex(0.040406, 0.060609);
        d_sig_pilot[577] = gr_complex(-0.133884, -0.004820);
        d_sig_pilot[578] = gr_complex(-0.201962, 0.020896);
        d_sig_pilot[579] = gr_complex(0.143982, 0.031263);
        d_sig_pilot[580] = gr_complex(0.040620, 0.036383);
        d_sig_pilot[581] = gr_complex(0.052793, -0.005394);
        d_sig_pilot[582] = gr_complex(0.026307, 0.013965);
        d_sig_pilot[583] = gr_complex(-0.007893, -0.099068);
        d_sig_pilot[584] = gr_complex(0.025783, -0.048774);
        d_sig_pilot[585] = gr_complex(-0.039561, 0.024610);
        d_sig_pilot[586] = gr_complex(0.041165, 0.057548);
        d_sig_pilot[587] = gr_complex(0.004856, 0.035321);
        d_sig_pilot[588] = gr_complex(-0.004402, -0.082023);
        d_sig_pilot[589] = gr_complex(0.088905, 0.018569);
        d_sig_pilot[590] = gr_complex(-0.027376, 0.026346);
        d_sig_pilot[591] = gr_complex(-0.069822, 0.029407);
        d_sig_pilot[592] = gr_complex(0.064940, -0.001568);
        d_sig_pilot[593] = gr_complex(0.020500, 0.018805);
        d_sig_pilot[594] = gr_complex(-0.014237, 0.070996);
        d_sig_pilot[595] = gr_complex(-0.001405, 0.088381);
        d_sig_pilot[596] = gr_complex(0.008563, -0.057910);
        d_sig_pilot[597] = gr_complex(-0.000118, 0.030549);
        d_sig_pilot[598] = gr_complex(-0.024889, 0.006510);
        d_sig_pilot[599] = gr_complex(-0.012755, -0.018807);
        d_sig_pilot[600] = gr_complex(-0.003316, 0.014938);
        d_sig_pilot[601] = gr_complex(-0.051644, -0.045193);
        d_sig_pilot[602] = gr_complex(0.000706, 0.061681);
        d_sig_pilot[603] = gr_complex(0.014943, -0.014239);
        d_sig_pilot[604] = gr_complex(0.001189, 0.040681);
        d_sig_pilot[605] = gr_complex(0.006336, 0.040932);
        d_sig_pilot[606] = gr_complex(0.019915, 0.068689);
        d_sig_pilot[607] = gr_complex(0.093277, 0.038685);
        d_sig_pilot[608] = gr_complex(-0.004283, 0.011496);
        d_sig_pilot[609] = gr_complex(-0.057756, 0.033024);
        d_sig_pilot[610] = gr_complex(0.039470, -0.024211);
        d_sig_pilot[611] = gr_complex(-0.050999, -0.012346);
        d_sig_pilot[612] = gr_complex(-0.059725, 0.047809);
        d_sig_pilot[613] = gr_complex(0.050409, 0.023301);
        d_sig_pilot[614] = gr_complex(-0.011958, 0.020206);
        d_sig_pilot[615] = gr_complex(0.034603, -0.027731);
        d_sig_pilot[616] = gr_complex(0.031000, -0.087483);
        d_sig_pilot[617] = gr_complex(-0.030065, -0.041756);
        d_sig_pilot[618] = gr_complex(0.060306, -0.083687);
        d_sig_pilot[619] = gr_complex(0.104423, -0.039095);
        d_sig_pilot[620] = gr_complex(0.033828, 0.034169);
        d_sig_pilot[621] = gr_complex(0.064052, 0.005027);
        d_sig_pilot[622] = gr_complex(-0.011716, 0.016074);
        d_sig_pilot[623] = gr_complex(-0.094371, 0.013032);
        d_sig_pilot[624] = gr_complex(-0.049077, -0.036747);
        d_sig_pilot[625] = gr_complex(-0.047277, -0.056654);
        d_sig_pilot[626] = gr_complex(0.033899, 0.050394);
        d_sig_pilot[627] = gr_complex(-0.018856, 0.034317);
        d_sig_pilot[628] = gr_complex(-0.087672, -0.017493);
        d_sig_pilot[629] = gr_complex(-0.017110, 0.025498);
        d_sig_pilot[630] = gr_complex(0.004363, 0.057428);
        d_sig_pilot[631] = gr_complex(-0.078640, 0.048100);
        d_sig_pilot[632] = gr_complex(-0.045850, -0.065748);
        d_sig_pilot[633] = gr_complex(0.021961, 0.038584);
        d_sig_pilot[634] = gr_complex(-0.021962, 0.050281);
        d_sig_pilot[635] = gr_complex(0.000151, -0.062674);
        d_sig_pilot[636] = gr_complex(0.049271, 0.005338);
        d_sig_pilot[637] = gr_complex(0.012328, 0.025628);
        d_sig_pilot[638] = gr_complex(-0.048764, -0.007294);
        d_sig_pilot[639] = gr_complex(-0.031369, -0.010145);

        return;
      }
      std::vector<gr_complex> tx_fft_shifted_1 = std::vector<gr_complex>(d_n_subcarriers);
      std::vector<gr_complex> tx_fft_shifted_2 = std::vector<gr_complex>(d_n_subcarriers);

      // objects to control the FFTs
      fftw_complex *d_fft_in;
      fftw_complex *d_fft_out;
      fftw_plan d_fft_plan;

      std::ofstream outfile;

      // Receiver gain
      const std::vector<gr_complex> rx_gain{
          {4.2629, -3.9491},
          {4.1336, -4.0495},
          {3.9975, -4.1401},
          {3.8652, -4.2310},
          {3.7295, -4.3155},
          {3.5912, -4.3975},
          {3.4533, -4.4750},
          {3.3129, -4.5476},
          {3.1725, -4.6163},
          {3.0307, -4.6805},
          {2.8883, -4.7409},
          {2.7448, -4.7966},
          {2.6006, -4.8483},
          {2.4523, -4.8886},
          {2.3124, -4.9405},
          {2.1672, -4.9797},
          {2.0218, -5.0170},
          {1.8777, -5.0488},
          {1.7309, -5.0759},
          {1.5870, -5.0999},
          {1.4428, -5.1215},
          {1.2972, -5.1378},
          {1.1513, -5.1514},
          {1.0086, -5.1603},
          {0.8637, -5.1657},
          {0.7195, -5.1654},
          {0.5759, -5.1661},
          {0.4352, -5.1599},
          {0.2947, -5.1508},
          {0.1522, -5.1374},
          {0.0128, -5.1213},
          {-0.1268, -5.1004},
          {-0.2649, -5.0799},
          {-0.4021, -5.0506},
          {-0.5378, -5.0205},
          {-0.6736, -4.9875},
          {-0.8066, -4.9523},
          {-0.9392, -4.9093},
          {-1.0701, -4.8669},
          {-1.2009, -4.8203},
          {-1.3285, -4.7709},
          {-1.4553, -4.7189},
          {-1.5802, -4.6605},
          {-1.7042, -4.6021},
          {-1.8255, -4.5387},
          {-1.9461, -4.4735},
          {-2.0630, -4.4040},
          {-2.1799, -4.3343},
          {-2.2944, -4.2575},
          {-2.4060, -4.1808},
          {-2.5155, -4.0985},
          {-2.6227, -4.0141},
      };
      const std::vector<gr_complex> rx_gain_ac{
          {-1.703354537, 1.07687571},
          {-1.868593032, 1.009364244},
          {-1.939997261, 1.539162756},
          {-1.961380583, 1.350257776},
          {-1.905012983, 1.560916064},
          {-2.291566642, 1.6429076},
          {-2.336470985, 1.80017083},
          {-2.027266638, 2.245308845},
          {-1.940152213, 2.144761064},
          {-1.748274691, 2.356080269},
          {-1.913563782, 2.272969287},
          {-2.090600735, 2.589993947},
          {-1.913993852, 3.049792282},
          {-1.830857572, 2.892329828},
          {-1.552058525, 2.972743386},
          {-1.804787755, 2.997361718},
          {-1.804506313, 3.145277255},
          {-1.420656401, 3.182358123},
          {-1.326818974, 2.904808175},
          {-1.122200636, 3.049988343},
          {-0.842965194, 3.188429696},
          {-1.106664365, 2.917669159},
          {-1.093724325, 3.160544732},
          {-0.871858925, 3.147067105},
          {-0.781221722, 3.161120267},
          {-0.427441909, 3.162309283},
          {-0.329265837, 3.186690444},
          {-0.589423258, 3.432684023},
          {-0.484694946, 3.577642831},
          {-0.315214572, 3.266348218},
          {-0.353659647, 3.194216665},
          {0.090040481, 3.385629331},
          {0.293179189, 3.350844277},
          {0.268449545, 3.704671524},
          {0.508244428, 3.332850917},
          {0.256140063, 3.321941059},
          {0.690382134, 3.327253686},
          {0.337924153, 3.333009031},
          {0.448066284, 3.591683344},
          {0.940606841, 3.533054716},
          {0.709384261, 3.309513308},
          {1.157254483, 3.138456223},
          {1.238158195, 3.55269246},
          {1.348047344, 3.436194151},
          {1.548004485, 3.503076324},
          {1.21013409, 3.492957035},
          {1.617255203, 2.99565725},
          {1.660341236, 2.932920824},
          {1.721091752, 2.997241551},
          {2.059306835, 3.305560461},
          {2.162574174, 3.279566539},
          {1.648549103, 3.002022915},
          {1.815839916, 3.065129328},
          {2.132061357, 3.339333586},
          {2.523292025, 2.978780174},
          {2.018532427, 2.943682055},
          {2.42234896, 2.622726684},
          {2.867973965, 2.867430053},
          {2.919677205, 2.728055828},
          {2.807444808, 2.995926044},
          {2.916410572, 2.91839332},
          {2.901623762, 2.316175487},
          {3.110776806, 2.861889743},
          {3.416809389, 2.326199907},
          {3.342400996, 2.648644711},
          {3.014845951, 2.41253325},
          {3.718933397, 2.128216028},
          {3.58014103, 2.419983576},
          {3.866074176, 1.844657752},
          {3.381391879, 2.108398033},
          {3.590829529, 1.483297959},
          {3.544027819, 1.840148344},
          {4.145840881, 1.869000965},
          {4.208106128, 1.802792358},
          {4.330549519, 1.642717863},
          {4.340732053, 1.523990148},
          {3.957590492, 0.888094058},
          {4.0531229, 1.284570944},
          {4.592354486, 0.562309889},
          {4.189606804, 0.489058889},
          {4.265722827, 0.893014562},
          {4.231032641, 0.269719832},
          {4.768746334, -0.055214949},
          {4.322833561, -0.081767962},
          {4.363342338, -0.197630653},
          {4.314200543, -0.352467468},
          {5.016162938, -0.170171331},
          {4.572242401, -0.145279779},
          {4.373050531, -0.847420843},
          {4.618158672, -0.472491717},
          {4.624989192, -0.635813871},
          {4.342249946, -1.247597594},
          {4.75401012, -1.792865969},
          {4.716916603, -1.9707346},
          {4.640484352, -2.075244714},
          {4.197828726, -1.887914548},
          {4.50049032, -2.438403843},
          {4.069250516, -2.245602937},
          {4.021974465, -2.362705241},
          {4.409100496, -2.280356368},
          {4.667458581, -2.854951706},
          {3.762509583, -2.866648971},
          {4.163328276, -2.712424689},
          {3.563001485, -3.160354995},
          {3.97397109, -3.042677157},
          {3.584947692, -3.972073723},
          {3.413900094, -4.035445768},
          {3.766177825, -4.07282389},
          {3.03473984, -3.797800602},
          {3.525971214, -4.336431355},
          {3.23061448, -3.917429565},
          {2.602269909, -4.152481664},
          {3.009976043, -4.72507528},
          {2.830526273, -4.839075389},
          {2.068072669, -4.977108809},
          {2.439741487, -4.492300021},
          {1.734844498, -4.561838507},
          {2.072588401, -4.727541856},
          {1.302497896, -5.204856046},
          {1.547950726, -5.337608463},
          {1.36438367, -5.386497275},
          {0.729474211, -5.459008302},
          {0.536629032, -5.486014153},
          {-0.182002677, -5.210674637},
          {0.37718383, -5.058284477},
          {-0.285276657, -4.741424255},
          {-0.051912583, -5.043706377},
          {-0.94751958, -5.061826228},
          {-0.759360897, -5.363728876},
          {-0.955713041, -5.329418163},
          {-1.04202741, -4.601936188},
          {-0.965171414, -4.906558395},
          {-1.531098948, -5.136202998},
          {-1.530064884, -4.444992347},
          {-1.8299247, -5.034535772},
          {-2.235679709, -4.519590477},
          {-2.429343918, -4.360844139},
          {-1.949461958, -4.497834007},
          {-2.276745047, -3.978461524},
          {-2.248452149, -4.312050195},
          {-2.517033877, -3.778700444},
          {-2.960982876, -4.232202684},
          {-2.607095545, -4.061471313},
          {-2.856406353, -3.510507676},
          {-2.954174492, -3.353247608},
          {-3.415544478, -3.325609301},
          {-3.150169299, -3.178468522},
          {-3.610688632, -3.046452916},
          {-3.755773931, -2.851447902},
          {-3.829423378, -2.721162063},
          {-3.467785305, -2.68215853},
          {-3.614862839, -2.916211349},
          {-3.693793289, -2.779006445},
          {-3.617930248, -2.231790108},
          {-3.842704944, -2.511569461},
          {-4.234194919, -2.227688634},
          {-3.819177599, -1.848629573},
          {-3.789104338, -1.759551373},
          {-3.824079129, -1.591466829},
          {-4.276790799, -1.281282176},
          {-3.934063146, -1.375350449},
          {-4.179614006, -1.522029536},
          {-3.924165217, -1.091943963},
          {-4.318532864, -0.718637085},
          {-4.24867815, -1.096611485},
          {-4.02339749, -0.673527194},
          {-4.473958811, -0.593531056},
          {-4.547133916, -0.412595015},
          {-4.485943843, -0.242235845},
          {-4.528634592, -0.126490474},
          {-4.208801829, -0.250658571},
          {-3.943644847, 0.130000286},
          {-4.252219901, 0.025368329},
          {-4.119815336, 0.626712836},
          {-4.304081255, 0.584705139},
          {-3.950823218, 0.863248043},
          {-4.143975137, 0.548547657},
          {-4.243365524, 0.996506423},
          {-3.886312753, 1.234395085},
          {-4.084429449, 1.285235023},
          {-3.632255366, 1.111157962},
          {-3.999996635, 1.506373099},
          {-3.944214057, 1.625534046},
          {-3.536027257, 1.789200889},
          {-3.385218235, 1.891045203},
          {-3.373106712, 1.686265589},
          {-3.565847535, 1.832154106},
          {-3.232954566, 2.17945125},
          {-3.427276528, 2.342112489},
          {-3.383637096, 2.091615826},
          {-3.190390309, 2.208711805},
          {-2.774228244, 2.550430692},
          {-2.695345228, 2.632213516},
          {-2.750874823, 2.42080893},
          {-2.581032053, 2.540409434},
          {-2.457035983, 2.587916331},
          {-2.519674379, 2.939558445},
          {-2.187047041, 3.004328216},
          {-2.175232772, 3.007787747},
          {-1.9685969, 3.022906597},
          {-1.958189845, 2.952305586},
          {-2.171197706, 3.048328147},
          {-1.732795342, 3.208225555},
          {-1.72710008, 3.346258974},
          {-1.808468647, 3.303916077},
          {-1.419012017, 3.109824961},
          {-1.572347698, 3.300848667},
          {-1.192020564, 3.166483489},
          {-0.906198098, 3.363398519},
          {-0.939999684, 3.226313783},
          {-0.91523905, 3.241935434},
          {-0.837500778, 3.522208103},
          {-0.821186587, 3.451626066},
          {-0.521114898, 3.696228243},
          {-0.384393823, 3.560408418},
          {-0.178316726, 3.354891992},
          {-0.086207484, 3.359287558},
          {-0.190264444, 3.410200229},
          {-0.060131026, 3.393756385},
          {0.05961431, 3.509685484},
          {0.314483454, 3.382814904},
          {0.546144326, 3.156670942},
          {0.581239283, 3.251295776},
          {0.54199858, 3.272641151},
          {0.880681679, 3.201205298},
          {1.101497204, 3.359698654},
          {1.037397836, 3.285922717},
          {1.015347273, 2.808706557},
          {1.25874146, 2.900064759},
          {1.554477667, 2.711959835},
          {1.412567295, 2.896118236},
          {1.352370177, 2.622337723},
          {1.535355374, 2.371028355},
          {1.501696091, 2.408460236},
          {1.707762752, 2.251487935},
          {1.672541304, 2.027152796},
          {1.633822376, 1.851257426},
          {1.444234343, 1.807886787},
          {1.406217441, 1.630122511},
          {1.398374993, 1.200770586},
          {1.322189399, 1.150243714},
          {1.279321563, 1.131624223},
      };

      // message passing values
      pmt::pmt_t d_msg_port_name;
      pmt::pmt_t d_channel_est_request_port_name;

      void debug_out(std::string msg);

      void load_fft_inputs(std::vector<gr_complex> in);
      void compute_fft(std::vector<gr_complex> in, std::vector<gr_complex> &out);
      void send_channel_estimate(std::vector<gr_complex> &channel_estimate);
      void on_request_channel_estimate(pmt::pmt_t msg);

    public:
      channel_estimator_impl(bool debug, uint64_t frame_len, uint64_t estimation_pkt_period, uint64_t occupied_carriers, uint64_t pilot_carriers, uint64_t cyclic_prefix_len, uint64_t fft_len, uint64_t pilot_pos, uint64_t num_packets);
      ~channel_estimator_impl();

      int work(int noutput_items,
               gr_vector_int &ninput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_channel_estimator_IMPL_H */
