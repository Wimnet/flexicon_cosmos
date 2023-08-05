/* -*- c++ -*- */
/*
 * Copyright 2019 <+YOU OR YOUR COMPANY+>.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "digital_sic_single_impl.h"

/* main
*
*/

namespace gr {
  namespace fullduplex {

    digital_sic_single::sptr
    digital_sic_single::make(bool debug,
                            unsigned int delay_tx2rx,
                            unsigned int sig_pilot_pos,
                            unsigned int pad_front,
                            unsigned int frame_len,
                            unsigned int si_chan_k,
                            unsigned int si_chan_dim,
                            double premultiplier)
    {
      return gnuradio::get_initial_sptr
        (new digital_sic_single_impl(debug, delay_tx2rx, sig_pilot_pos, pad_front, frame_len, si_chan_k, si_chan_dim, premultiplier));
    }

    /*
     * The private constructor
     */
    digital_sic_single_impl::digital_sic_single_impl(bool debug,
                                      unsigned int delay_tx2rx,
                                       unsigned int sig_pilot_pos,
                                       unsigned int pad_front,
                                       unsigned int frame_len,
                                       unsigned int si_chan_k,
                                       unsigned int si_chan_dim,
                                       double premultiplier)
      : gr::tagged_stream_block("digital_sic_single",
              gr::io_signature::make2(2, 2, sizeof(gr_complex), sizeof(gr_complex)),
              gr::io_signature::make3(3, 3, sizeof(gr_complex), sizeof(gr_complex), sizeof(gr_complex)),
              "packet_len"),
              d_debug(debug),
              d_delay_tx2rx(delay_tx2rx),
              d_pad_front(pad_front),
              d_sig_pilot_pos(sig_pilot_pos),
              d_frame_len(frame_len),
              d_si_chan_k(si_chan_k),
              d_si_chan_dim(si_chan_dim),
              d_premultiplier(premultiplier),
              d_initial(true),
              d_delay_msg_in(pmt::mp("delay_in")) // MK
    {
      // set_tag_propagation_policy(block::TPP_DONT);

      // d_sig_pilot
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
      d_sig_pilot[320]= gr_complex(0.044885,0.039017);
      d_sig_pilot[321]= gr_complex(-0.035477,0.005077);
      d_sig_pilot[322]= gr_complex(-0.027468,0.053043);
      d_sig_pilot[323]= gr_complex(-0.002450,0.105278);
      d_sig_pilot[324]= gr_complex(0.022608,-0.010450);
      d_sig_pilot[325]= gr_complex(-0.049896,0.085134);
      d_sig_pilot[326]= gr_complex(0.069350,-0.018697);
      d_sig_pilot[327]= gr_complex(0.063929,-0.023199);
      d_sig_pilot[328]= gr_complex(-0.049608,0.001470);
      d_sig_pilot[329]= gr_complex(-0.029457,-0.028104);
      d_sig_pilot[330]= gr_complex(0.036437,-0.087159);
      d_sig_pilot[331]= gr_complex(-0.007760,0.020365);
      d_sig_pilot[332]= gr_complex(-0.010915,-0.091042);
      d_sig_pilot[333]= gr_complex(0.011305,-0.019949);
      d_sig_pilot[334]= gr_complex(-0.083307,-0.024301);
      d_sig_pilot[335]= gr_complex(-0.007308,-0.010636);
      d_sig_pilot[336]= gr_complex(-0.042427,-0.026965);
      d_sig_pilot[337]= gr_complex(0.067833,-0.075261);
      d_sig_pilot[338]= gr_complex(0.098341,-0.011294);
      d_sig_pilot[339]= gr_complex(-0.114264,0.087207);
      d_sig_pilot[340]= gr_complex(-0.031919,-0.036453);
      d_sig_pilot[341]= gr_complex(0.004540,0.032505);
      d_sig_pilot[342]= gr_complex(0.042284,0.028122);
      d_sig_pilot[343]= gr_complex(-0.086105,-0.024985);
      d_sig_pilot[344]= gr_complex(-0.094554,0.021589);
      d_sig_pilot[345]= gr_complex(-0.028750,0.098044);
      d_sig_pilot[346]= gr_complex(0.152547,-0.002044);
      d_sig_pilot[347]= gr_complex(0.044460,0.012573);
      d_sig_pilot[348]= gr_complex(-0.027830,-0.032253);
      d_sig_pilot[349]= gr_complex(0.047628,-0.049329);
      d_sig_pilot[350]= gr_complex(0.020546,0.095219);
      d_sig_pilot[351]= gr_complex(-0.032012,-0.001906);
      d_sig_pilot[352]= gr_complex(0.031476,0.065192);
      d_sig_pilot[353]= gr_complex(-0.039547,-0.010185);
      d_sig_pilot[354]= gr_complex(0.039560,0.013452);
      d_sig_pilot[355]= gr_complex(-0.029019,-0.019756);
      d_sig_pilot[356]= gr_complex(0.040515,-0.020092);
      d_sig_pilot[357]= gr_complex(0.019813,-0.002351);
      d_sig_pilot[358]= gr_complex(-0.072516,-0.051744);
      d_sig_pilot[359]= gr_complex(-0.003651,-0.003138);
      d_sig_pilot[360]= gr_complex(-0.029604,0.018831);
      d_sig_pilot[361]= gr_complex(0.096010,0.038626);
      d_sig_pilot[362]= gr_complex(-0.083199,-0.033600);
      d_sig_pilot[363]= gr_complex(0.018686,-0.007545);
      d_sig_pilot[364]= gr_complex(0.037069,-0.020098);
      d_sig_pilot[365]= gr_complex(0.046747,-0.005872);
      d_sig_pilot[366]= gr_complex(0.037895,-0.093534);
      d_sig_pilot[367]= gr_complex(-0.021946,0.024022);
      d_sig_pilot[368]= gr_complex(-0.038946,-0.003617);
      d_sig_pilot[369]= gr_complex(0.043310,-0.010198);
      d_sig_pilot[370]= gr_complex(0.031723,0.052333);
      d_sig_pilot[371]= gr_complex(-0.090010,-0.025184);
      d_sig_pilot[372]= gr_complex(-0.020542,-0.014555);
      d_sig_pilot[373]= gr_complex(-0.030488,-0.074728);
      d_sig_pilot[374]= gr_complex(-0.022561,0.107434);
      d_sig_pilot[375]= gr_complex(-0.072155,0.072145);
      d_sig_pilot[376]= gr_complex(0.115677,-0.064849);
      d_sig_pilot[377]= gr_complex(-0.021768,-0.023883);
      d_sig_pilot[378]= gr_complex(0.000000,0.000000);
      d_sig_pilot[379]= gr_complex(0.000000,0.000000);
      d_sig_pilot[380]= gr_complex(0.000000,0.000000);
      d_sig_pilot[381]= gr_complex(0.000000,0.000000);
      d_sig_pilot[382]= gr_complex(0.000000,0.000000);
      d_sig_pilot[383]= gr_complex(0.000000,0.000000);
      d_sig_pilot[384]= gr_complex(0.000000,0.000000);
      d_sig_pilot[385]= gr_complex(0.000000,0.000000);
      d_sig_pilot[386]= gr_complex(0.000000,0.000000);
      d_sig_pilot[387]= gr_complex(0.000000,0.000000);
      d_sig_pilot[388]= gr_complex(0.000000,0.000000);
      d_sig_pilot[389]= gr_complex(0.000000,0.000000);
      d_sig_pilot[390]= gr_complex(0.089072,-0.034311);
      d_sig_pilot[391]= gr_complex(-0.037952,-0.085431);
      d_sig_pilot[392]= gr_complex(0.014835,0.000402);
      d_sig_pilot[393]= gr_complex(-0.057210,-0.065170);
      d_sig_pilot[394]= gr_complex(-0.036452,0.073117);
      d_sig_pilot[395]= gr_complex(0.025422,0.010924);
      d_sig_pilot[396]= gr_complex(0.047282,-0.000180);
      d_sig_pilot[397]= gr_complex(0.085829,0.012254);
      d_sig_pilot[398]= gr_complex(-0.078765,0.062856);
      d_sig_pilot[399]= gr_complex(-0.016699,0.112184);
      d_sig_pilot[400]= gr_complex(-0.002659,-0.047490);
      d_sig_pilot[401]= gr_complex(0.062132,0.028358);
      d_sig_pilot[402]= gr_complex(-0.023809,-0.084240);
      d_sig_pilot[403]= gr_complex(-0.058117,0.076824);
      d_sig_pilot[404]= gr_complex(0.020489,-0.110830);
      d_sig_pilot[405]= gr_complex(0.053326,-0.057864);
      d_sig_pilot[406]= gr_complex(-0.013673,0.054181);
      d_sig_pilot[407]= gr_complex(-0.017050,0.072307);
      d_sig_pilot[408]= gr_complex(-0.070716,0.038566);
      d_sig_pilot[409]= gr_complex(0.005098,0.013295);
      d_sig_pilot[410]= gr_complex(-0.071913,-0.070045);
      d_sig_pilot[411]= gr_complex(0.024746,-0.041929);
      d_sig_pilot[412]= gr_complex(0.056566,0.007559);
      d_sig_pilot[413]= gr_complex(-0.081286,0.004450);
      d_sig_pilot[414]= gr_complex(0.068126,0.022789);
      d_sig_pilot[415]= gr_complex(0.003254,-0.044819);
      d_sig_pilot[416]= gr_complex(0.068127,-0.046506);
      d_sig_pilot[417]= gr_complex(-0.055574,-0.006814);
      d_sig_pilot[418]= gr_complex(-0.053625,0.030120);
      d_sig_pilot[419]= gr_complex(0.031228,0.042826);
      d_sig_pilot[420]= gr_complex(-0.052764,-0.032707);
      d_sig_pilot[421]= gr_complex(0.019202,-0.062284);
      d_sig_pilot[422]= gr_complex(0.053817,0.014597);
      d_sig_pilot[423]= gr_complex(0.027363,0.045575);
      d_sig_pilot[424]= gr_complex(-0.034091,-0.013042);
      d_sig_pilot[425]= gr_complex(0.036498,-0.064458);
      d_sig_pilot[426]= gr_complex(-0.011505,-0.039079);
      d_sig_pilot[427]= gr_complex(-0.084919,0.125764);
      d_sig_pilot[428]= gr_complex(-0.036177,0.121087);
      d_sig_pilot[429]= gr_complex(0.052626,-0.094002);
      d_sig_pilot[430]= gr_complex(0.046415,0.050812);
      d_sig_pilot[431]= gr_complex(0.062233,0.050308);
      d_sig_pilot[432]= gr_complex(0.001456,-0.009599);
      d_sig_pilot[433]= gr_complex(-0.017538,-0.006459);
      d_sig_pilot[434]= gr_complex(-0.013714,-0.011872);
      d_sig_pilot[435]= gr_complex(-0.006824,-0.011941);
      d_sig_pilot[436]= gr_complex(0.069932,0.015548);
      d_sig_pilot[437]= gr_complex(-0.004151,-0.065246);
      d_sig_pilot[438]= gr_complex(0.008392,-0.023716);
      d_sig_pilot[439]= gr_complex(-0.026884,-0.006908);
      d_sig_pilot[440]= gr_complex(-0.008039,0.002674);
      d_sig_pilot[441]= gr_complex(-0.022837,0.033623);
      d_sig_pilot[442]= gr_complex(0.051426,-0.053517);
      d_sig_pilot[443]= gr_complex(0.051678,0.010707);
      d_sig_pilot[444]= gr_complex(-0.094268,0.032874);
      d_sig_pilot[445]= gr_complex(0.020338,-0.030002);
      d_sig_pilot[446]= gr_complex(-0.111997,0.002714);
      d_sig_pilot[447]= gr_complex(-0.010968,0.018778);
      d_sig_pilot[448]= gr_complex(0.078675,-0.005931);
      d_sig_pilot[449]= gr_complex(-0.063179,-0.064235);
      d_sig_pilot[450]= gr_complex(-0.063228,0.034462);
      d_sig_pilot[451]= gr_complex(-0.048594,0.054313);
      d_sig_pilot[452]= gr_complex(0.025043,-0.006192);
      d_sig_pilot[453]= gr_complex(0.033183,0.014017);
      d_sig_pilot[454]= gr_complex(0.034788,0.000398);
      d_sig_pilot[455]= gr_complex(-0.062332,-0.064128);
      d_sig_pilot[456]= gr_complex(0.015966,0.077221);
      d_sig_pilot[457]= gr_complex(-0.019747,0.029521);
      d_sig_pilot[458]= gr_complex(0.099617,-0.058244);
      d_sig_pilot[459]= gr_complex(0.043929,-0.081810);
      d_sig_pilot[460]= gr_complex(-0.032249,0.054762);
      d_sig_pilot[461]= gr_complex(0.097312,-0.082826);
      d_sig_pilot[462]= gr_complex(0.027910,0.039715);
      d_sig_pilot[463]= gr_complex(0.002696,0.022711);
      d_sig_pilot[464]= gr_complex(0.002958,-0.049361);
      d_sig_pilot[465]= gr_complex(-0.051784,0.048513);
      d_sig_pilot[466]= gr_complex(0.110545,-0.043485);
      d_sig_pilot[467]= gr_complex(-0.014700,0.046130);
      d_sig_pilot[468]= gr_complex(-0.064829,0.033269);
      d_sig_pilot[469]= gr_complex(0.000407,0.024732);
      d_sig_pilot[470]= gr_complex(-0.031450,-0.057440);
      d_sig_pilot[471]= gr_complex(-0.054899,0.049992);
      d_sig_pilot[472]= gr_complex(-0.009956,0.022405);
      d_sig_pilot[473]= gr_complex(-0.004120,0.071083);
      d_sig_pilot[474]= gr_complex(-0.064148,0.039284);
      d_sig_pilot[475]= gr_complex(0.012757,-0.047743);
      d_sig_pilot[476]= gr_complex(-0.026348,0.006731);
      d_sig_pilot[477]= gr_complex(0.092930,-0.036314);
      d_sig_pilot[478]= gr_complex(0.032904,-0.046267);
      d_sig_pilot[479]= gr_complex(-0.091305,-0.084580);
      d_sig_pilot[480]= gr_complex(0.004374,0.026755);
      d_sig_pilot[481]= gr_complex(-0.013590,-0.015680);
      d_sig_pilot[482]= gr_complex(0.005667,0.118581);
      d_sig_pilot[483]= gr_complex(-0.032587,-0.033819);
      d_sig_pilot[484]= gr_complex(-0.064026,0.022245);
      d_sig_pilot[485]= gr_complex(0.088295,-0.008006);
      d_sig_pilot[486]= gr_complex(0.020438,-0.058267);
      d_sig_pilot[487]= gr_complex(-0.023418,-0.062584);
      d_sig_pilot[488]= gr_complex(0.080616,-0.031514);
      d_sig_pilot[489]= gr_complex(0.114993,-0.087187);
      d_sig_pilot[490]= gr_complex(0.035897,0.044260);
      d_sig_pilot[491]= gr_complex(-0.158021,0.052981);
      d_sig_pilot[492]= gr_complex(0.042779,0.055564);
      d_sig_pilot[493]= gr_complex(-0.026552,0.053966);
      d_sig_pilot[494]= gr_complex(0.027140,-0.019699);
      d_sig_pilot[495]= gr_complex(-0.008848,-0.001313);
      d_sig_pilot[496]= gr_complex(-0.035001,-0.013152);
      d_sig_pilot[497]= gr_complex(0.042508,0.103583);
      d_sig_pilot[498]= gr_complex(0.016168,-0.042484);
      d_sig_pilot[499]= gr_complex(-0.069861,0.064132);
      d_sig_pilot[500]= gr_complex(-0.010340,-0.115932);
      d_sig_pilot[501]= gr_complex(-0.019788,-0.058511);
      d_sig_pilot[502]= gr_complex(0.050894,0.108365);
      d_sig_pilot[503]= gr_complex(-0.047877,-0.059385);
      d_sig_pilot[504]= gr_complex(-0.017383,-0.076909);
      d_sig_pilot[505]= gr_complex(0.009591,0.038773);
      d_sig_pilot[506]= gr_complex(-0.022097,0.035223);
      d_sig_pilot[507]= gr_complex(0.024472,0.023265);
      d_sig_pilot[508]= gr_complex(0.006268,0.077446);
      d_sig_pilot[509]= gr_complex(0.090088,0.007307);
      d_sig_pilot[510]= gr_complex(0.148394,0.018771);
      d_sig_pilot[511]= gr_complex(-0.112817,-0.004808);
      d_sig_pilot[512]= gr_complex(-0.090428,-0.071415);
      d_sig_pilot[513]= gr_complex(-0.004505,0.002012);
      d_sig_pilot[514]= gr_complex(0.017913,-0.054677);
      d_sig_pilot[515]= gr_complex(-0.051446,0.000168);
      d_sig_pilot[516]= gr_complex(-0.026790,0.000327);
      d_sig_pilot[517]= gr_complex(-0.058054,-0.005338);
      d_sig_pilot[518]= gr_complex(0.106066,-0.066423);
      d_sig_pilot[519]= gr_complex(-0.013089,0.028254);
      d_sig_pilot[520]= gr_complex(0.009982,0.003060);
      d_sig_pilot[521]= gr_complex(-0.044794,0.001023);
      d_sig_pilot[522]= gr_complex(0.085604,0.048814);
      d_sig_pilot[523]= gr_complex(-0.001288,0.017352);
      d_sig_pilot[524]= gr_complex(-0.059439,0.053267);
      d_sig_pilot[525]= gr_complex(-0.012185,-0.028986);
      d_sig_pilot[526]= gr_complex(0.020656,-0.065301);
      d_sig_pilot[527]= gr_complex(0.084932,0.022057);
      d_sig_pilot[528]= gr_complex(-0.016729,0.004383);
      d_sig_pilot[529]= gr_complex(0.108634,0.036247);
      d_sig_pilot[530]= gr_complex(-0.091915,0.000133);
      d_sig_pilot[531]= gr_complex(0.015235,0.005464);
      d_sig_pilot[532]= gr_complex(0.032993,-0.043592);
      d_sig_pilot[533]= gr_complex(-0.067297,0.002424);
      d_sig_pilot[534]= gr_complex(-0.098345,0.044122);
      d_sig_pilot[535]= gr_complex(-0.021008,0.000202);
      d_sig_pilot[536]= gr_complex(0.010778,-0.068763);
      d_sig_pilot[537]= gr_complex(0.010873,0.019345);
      d_sig_pilot[538]= gr_complex(-0.039238,-0.011415);
      d_sig_pilot[539]= gr_complex(0.028065,-0.028401);
      d_sig_pilot[540]= gr_complex(0.080471,0.003405);
      d_sig_pilot[541]= gr_complex(-0.026320,0.075958);
      d_sig_pilot[542]= gr_complex(-0.027839,0.013276);
      d_sig_pilot[543]= gr_complex(0.000327,-0.062830);
      d_sig_pilot[544]= gr_complex(0.082393,-0.065352);
      d_sig_pilot[545]= gr_complex(-0.047273,-0.027428);
      d_sig_pilot[546]= gr_complex(0.028773,0.054282);
      d_sig_pilot[547]= gr_complex(-0.008772,0.040894);
      d_sig_pilot[548]= gr_complex(0.007785,0.011922);
      d_sig_pilot[549]= gr_complex(0.009469,0.047515);
      d_sig_pilot[550]= gr_complex(0.042504,0.029230);
      d_sig_pilot[551]= gr_complex(-0.001534,0.024073);
      d_sig_pilot[552]= gr_complex(-0.024072,0.041408);
      d_sig_pilot[553]= gr_complex(0.083499,-0.014739);
      d_sig_pilot[554]= gr_complex(-0.007855,-0.116892);
      d_sig_pilot[555]= gr_complex(-0.046699,0.036556);
      d_sig_pilot[556]= gr_complex(-0.018993,0.062421);
      d_sig_pilot[557]= gr_complex(-0.083622,-0.030668);
      d_sig_pilot[558]= gr_complex(0.009219,0.011081);
      d_sig_pilot[559]= gr_complex(0.040380,-0.078038);
      d_sig_pilot[560]= gr_complex(-0.055311,0.014103);
      d_sig_pilot[561]= gr_complex(-0.047072,-0.072670);
      d_sig_pilot[562]= gr_complex(-0.003294,-0.063509);
      d_sig_pilot[563]= gr_complex(0.028044,-0.019674);
      d_sig_pilot[564]= gr_complex(-0.027705,0.077746);
      d_sig_pilot[565]= gr_complex(0.052413,-0.020162);
      d_sig_pilot[566]= gr_complex(-0.008280,0.004427);
      d_sig_pilot[567]= gr_complex(0.035039,0.065927);
      d_sig_pilot[568]= gr_complex(-0.032549,-0.082436);
      d_sig_pilot[569]= gr_complex(-0.018894,-0.044297);
      d_sig_pilot[570]= gr_complex(0.022527,0.125265);
      d_sig_pilot[571]= gr_complex(-0.016590,-0.002674);
      d_sig_pilot[572]= gr_complex(0.038017,-0.027721);
      d_sig_pilot[573]= gr_complex(-0.071975,-0.009695);
      d_sig_pilot[574]= gr_complex(0.045038,-0.000776);
      d_sig_pilot[575]= gr_complex(0.074191,-0.015702);
      d_sig_pilot[576]= gr_complex(0.044885,0.039017);
      d_sig_pilot[577]= gr_complex(-0.035477,0.005077);
      d_sig_pilot[578]= gr_complex(-0.027468,0.053043);
      d_sig_pilot[579]= gr_complex(-0.002450,0.105278);
      d_sig_pilot[580]= gr_complex(0.022608,-0.010450);
      d_sig_pilot[581]= gr_complex(-0.049896,0.085134);
      d_sig_pilot[582]= gr_complex(0.069350,-0.018697);
      d_sig_pilot[583]= gr_complex(0.063929,-0.023199);
      d_sig_pilot[584]= gr_complex(-0.049608,0.001470);
      d_sig_pilot[585]= gr_complex(-0.029457,-0.028104);
      d_sig_pilot[586]= gr_complex(0.036437,-0.087159);
      d_sig_pilot[587]= gr_complex(-0.007760,0.020365);
      d_sig_pilot[588]= gr_complex(-0.010915,-0.091042);
      d_sig_pilot[589]= gr_complex(0.011305,-0.019949);
      d_sig_pilot[590]= gr_complex(-0.083307,-0.024301);
      d_sig_pilot[591]= gr_complex(-0.007308,-0.010636);
      d_sig_pilot[592]= gr_complex(-0.042427,-0.026965);
      d_sig_pilot[593]= gr_complex(0.067833,-0.075261);
      d_sig_pilot[594]= gr_complex(0.098341,-0.011294);
      d_sig_pilot[595]= gr_complex(-0.114264,0.087207);
      d_sig_pilot[596]= gr_complex(-0.031919,-0.036453);
      d_sig_pilot[597]= gr_complex(0.004540,0.032505);
      d_sig_pilot[598]= gr_complex(0.042284,0.028122);
      d_sig_pilot[599]= gr_complex(-0.086105,-0.024985);
      d_sig_pilot[600]= gr_complex(-0.094554,0.021589);
      d_sig_pilot[601]= gr_complex(-0.028750,0.098044);
      d_sig_pilot[602]= gr_complex(0.152547,-0.002044);
      d_sig_pilot[603]= gr_complex(0.044460,0.012573);
      d_sig_pilot[604]= gr_complex(-0.027830,-0.032253);
      d_sig_pilot[605]= gr_complex(0.047628,-0.049329);
      d_sig_pilot[606]= gr_complex(0.020546,0.095219);
      d_sig_pilot[607]= gr_complex(-0.032012,-0.001906);
      d_sig_pilot[608]= gr_complex(0.031476,0.065192);
      d_sig_pilot[609]= gr_complex(-0.039547,-0.010185);
      d_sig_pilot[610]= gr_complex(0.039560,0.013452);
      d_sig_pilot[611]= gr_complex(-0.029019,-0.019756);
      d_sig_pilot[612]= gr_complex(0.040515,-0.020092);
      d_sig_pilot[613]= gr_complex(0.019813,-0.002351);
      d_sig_pilot[614]= gr_complex(-0.072516,-0.051744);
      d_sig_pilot[615]= gr_complex(-0.003651,-0.003138);
      d_sig_pilot[616]= gr_complex(-0.029604,0.018831);
      d_sig_pilot[617]= gr_complex(0.096010,0.038626);
      d_sig_pilot[618]= gr_complex(-0.083199,-0.033600);
      d_sig_pilot[619]= gr_complex(0.018686,-0.007545);
      d_sig_pilot[620]= gr_complex(0.037069,-0.020098);
      d_sig_pilot[621]= gr_complex(0.046747,-0.005872);
      d_sig_pilot[622]= gr_complex(0.037895,-0.093534);
      d_sig_pilot[623]= gr_complex(-0.021946,0.024022);
      d_sig_pilot[624]= gr_complex(-0.038946,-0.003617);
      d_sig_pilot[625]= gr_complex(0.043310,-0.010198);
      d_sig_pilot[626]= gr_complex(0.031723,0.052333);
      d_sig_pilot[627]= gr_complex(-0.090010,-0.025184);
      d_sig_pilot[628]= gr_complex(-0.020542,-0.014555);
      d_sig_pilot[629]= gr_complex(-0.030488,-0.074728);
      d_sig_pilot[630]= gr_complex(-0.022561,0.107434);
      d_sig_pilot[631]= gr_complex(-0.072155,0.072145);
      d_sig_pilot[632]= gr_complex(0.115677,-0.064849);
      d_sig_pilot[633]= gr_complex(-0.021768,-0.023883);
      d_sig_pilot[634]= gr_complex(0.000000,0.000000);
      d_sig_pilot[635]= gr_complex(0.000000,0.000000);
      d_sig_pilot[636]= gr_complex(0.000000,0.000000);
      d_sig_pilot[637]= gr_complex(0.000000,0.000000);
      d_sig_pilot[638]= gr_complex(0.000000,0.000000);
      d_sig_pilot[639]= gr_complex(0.000000,0.000000);
      // end of d_sig_pilot


      /*for (int idx_sig_pilot = 0; idx_sig_pilot<80; idx_sig_pilot++) {
        d_sig_pilot[idx_sig_pilot+80] = d_sig_pilot[idx_sig_pilot];
      }*/
      for (int idx_sig_pilot = 0; idx_sig_pilot<320*2; idx_sig_pilot++) {
        d_sig_pilot[idx_sig_pilot] *= d_premultiplier;
      }

      set_delay_tx2rx(delay_tx2rx);

      message_port_register_in(d_delay_msg_in);
      set_msg_handler(d_delay_msg_in, boost::bind(&digital_sic_single_impl::delay_handle, this, _1));
    }

    /*
     * Our virtual destructor.
     */
    digital_sic_single_impl::~digital_sic_single_impl()
    {
    }

    void
    digital_sic_single_impl::set_delay_tx2rx(int delay_tx2rx)
    {
      gr::thread::scoped_lock lock(d_mutex);
      d_delay_tx2rx = delay_tx2rx;
    }

    /*
    void
    digital_sic_single_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      // <+forecast+> e.g. ninput_items_required[0] = noutput_items
      ninput_items_required[0] = 2720; // noutput_items;
      ninput_items_required[1] = 2720; // noutput_items;
    }
    */

    /*
    int
    digital_sic_single_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    */

    void
    digital_sic_single_impl::delay_handle(pmt::pmt_t msg)
    {
      d_delay_tx2rx = int(pmt::to_long(pmt::cdr(msg))) + 11;
    }

    int
    digital_sic_single_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)

    {
      const gr_complex *in_tx = (const gr_complex *) input_items[0];
      const gr_complex *in_rx = (const gr_complex *) input_items[1];

      gr_complex *out_rx = (gr_complex *) output_items[0];
      gr_complex *out_rx_res = (gr_complex *) output_items[1];

      /*
      gr_complex *out_rx = (gr_complex *) output_items[0];
      gr_complex *out_rx_res = (gr_complex *) output_items[1];
      */
      gr_complex *h = (gr_complex *) output_items[2];

      noutput_items = d_frame_len; // ninput_items[0];

      //std::memset(out_rx, 0x00, sizeof(gr_complex) * noutput_items);
      std::memset(out_rx_res, 0x00, sizeof(gr_complex) * noutput_items);
      std::memset(h, 0x00, sizeof(gr_complex) * (2*d_si_chan_k+1));

      // init Eigen vectors
      Eigen::VectorXcf pilot_tx(320*2);
      Eigen::VectorXcf pilot_rx(320*2);
      Eigen::VectorXcf tx_sig_buff(d_frame_len + 2*d_si_chan_k + 1);
      Eigen::VectorXcf rx_sig_buff(d_frame_len + 2*d_si_chan_k + 1);

      for (int idx = 0; idx < 320*2; idx++) {
        pilot_tx(idx) = d_sig_pilot[idx];
        pilot_rx(idx) = in_rx[d_pad_front+d_sig_pilot_pos*320*2+ idx +d_delay_tx2rx];
      }

      for (int idx = 0; idx < d_frame_len; idx++) {
        // out_tx[idx] = in_tx[d_pad_front+idx];
        out_rx[idx] = in_rx[d_pad_front+d_delay_tx2rx+idx];
      }

      for (int idx = 0; idx < d_frame_len + 2*d_si_chan_dim + 1; idx++) {
        tx_sig_buff(idx) = in_tx[d_pad_front-d_si_chan_k+idx];
        rx_sig_buff(idx) = in_rx[d_pad_front-d_si_chan_k+d_delay_tx2rx+idx];
      }
      // memcpy(pilot, in, sizeof(gr_complex) * 80)
      // cout << "Vector element type is " << typeid(pilot(1)).name() << endl;

      // Eigen::MatrixXcf pilot_toeplitz;
      // pilot_toeplitz = sig_toeplitz(pilot, 40, 10, 1);
      Eigen::VectorXcf hh( (2*d_si_chan_k+1) * d_si_chan_dim );
      Eigen::VectorXf hh_mag( (2*d_si_chan_k+1) * d_si_chan_dim );
      float hh_mag_max;

      // Eigen::VectorXcf si_chnl_est(Eigen::VectorXcf &tx_sig, Eigen::VectorXcf &rx_sig, int l, int k)
      // if (d_initial) { // MK
      //   d_h = Eigen::VectorXcf( (2*d_si_chan_k+1) * d_si_chan_dim ); // MK
      //   d_h = si_chnl_est(pilot_tx, pilot_rx, 80*2-2*d_si_chan_k-1, d_si_chan_k, d_si_chan_dim); // MK
      //   d_initial = false; // MK
      // }
      // hh = d_h; // MK
    
      hh = si_chnl_est_single(pilot_tx, pilot_rx, 320*2-2*d_si_chan_k-1, d_si_chan_k, d_si_chan_dim);
      for (int idx = 0; idx < (2*d_si_chan_k+1) * d_si_chan_dim; idx++) {
        h[idx] = hh(idx);
      }
      hh_mag = hh.cwiseAbs2();
      // std::cout << "Max |h| = " << hh_mag << std::endl;
      std::ptrdiff_t hh_mag_max_idx;
      hh_mag_max = hh_mag.maxCoeff(&hh_mag_max_idx);
      // std::cout << "Max |h| at " << hh_mag_max_idx << std::endl;

      // perform digital SIC
      Eigen::MatrixXcf tx_sig_toeplitz;
      tx_sig_toeplitz = sig_toeplitz_single(tx_sig_buff, d_frame_len, d_si_chan_k, d_si_chan_dim);
      Eigen::VectorXcf rx_sig_res;
      rx_sig_res = rx_sig_buff.segment(d_si_chan_k-1, d_frame_len) - tx_sig_toeplitz * hh;
      //std::cout<<"Length of Digital DIC operated Rx"<<size()
      for (int idx = 0; idx < d_frame_len-d_si_chan_k; idx++) {
        out_rx_res[idx] = rx_sig_res(idx);
      }



      /* estimate tx pilot
      Eigen::MatrixXcf pilot_tx_toeplitz;
      Eigen::VectorXcf pilot_rx_res;
      pilot_tx_toeplitz = sig_toeplitz(pilot_tx, 80-2*d_si_chan_k-1, d_si_chan_k, d_si_chan_dim);
      pilot_rx_res = pilot_rx.segment(d_si_chan_k-1, 80-2*d_si_chan_k-1) - pilot_tx_toeplitz * hh;
      for (int idx = 0; idx < 80-2*d_si_chan_k-1; idx++) {
        out_rx_pilot_est[idx+d_si_chan_k-1] = pilot_rx_res(idx);
      }
      */





      int produced = noutput_items; // d_frame_len;

      // Tell runtime system how many input items we consumed on
      // each input stream.
      // consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      // return noutput_items;
      return produced;

      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      // consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      // return noutput_items;
    }

  } /* namespace fullduplex */
} /* namespace gr */
