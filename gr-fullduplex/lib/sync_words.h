/* -*- c++ -*- */
/*
 * Copyright 2021 gr-fullduplex author.
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

#ifndef INCLUDED_FULLDUPLEX_SYNC_WORDS_H
#define INCLUDED_FULLDUPLEX_SYNC_WORDS_H

namespace gr {
  namespace fullduplex {

/*
 * DEFINE ALL OF THE PILOT SYMBOLS HERE
 */
    void
    pilot_lts(gr_complex* pilot) {
      pilot[0] = gr_complex(-0.09785022, 0.04082483);
      pilot[1] = gr_complex(0.02180564, -0.1732432);
      pilot[2] = gr_complex(0.1628006, -0.1879265);
      pilot[3] = gr_complex(-0.1631041, -0.2043581);
      pilot[4] = gr_complex(-0.004980664, -0.09545150);
      pilot[5] = gr_complex(0.1332588, 0.1314248);
      pilot[6] = gr_complex(-0.2260059, 0.03639078);
      pilot[7] = gr_complex(-0.2163544, 0.02940571);
      pilot[8] = gr_complex(-0.06219967, 0.2678330);
      pilot[9] = gr_complex(-0.1002102, 0.03870285);
      pilot[10] = gr_complex(-0.1070529, -0.1442862);
      pilot[11] = gr_complex(0.1234663, -0.02506706);
      pilot[12] = gr_complex(0.1459409, -0.1639366);
      pilot[13] = gr_complex(-0.2329965, -0.1157810);
      pilot[14] = gr_complex(-0.1015436, -0.06975659);
      pilot[15] = gr_complex(0.06553086, -0.1745648);
      pilot[16] = gr_complex(0.1109401, 0.1109400);
      pilot[17] = gr_complex(0.2116542, 0.007269842);
      pilot[18] = gr_complex(-0.03990859, -0.2851733);
      pilot[19] = gr_complex(0.1041394, 0.02651732);
      pilot[20] = gr_complex(0.04344563, 0.1038963);
      pilot[21] = gr_complex(-0.2428342, 0.08410110);
      pilot[22] = gr_complex(0.001755476, 0.2041379);
      pilot[23] = gr_complex(0.09467665, -0.007235643);
      pilot[24] = gr_complex(0.1731397, 0.04595287);
      pilot[25] = gr_complex(-0.06801241, 0.1884577);
      pilot[26] = gr_complex(-0.2043626, 0.09794762);
      pilot[27] = gr_complex(0.1061898, 0.1556831);
      pilot[28] = gr_complex(0.03747426, -0.04949864);
      pilot[29] = gr_complex(0.1718805, -0.1469697);
      pilot[30] = gr_complex(0.07055733, 0.1973099);
      pilot[31] = gr_complex(-0.009090421, 0.2135820);
      pilot[32] = gr_complex(0.2773501, 0.000000);
      pilot[33] = gr_complex(-0.009090421, -0.2135820);
      pilot[34] = gr_complex(0.07055733, -0.1973099);
      pilot[35] = gr_complex(0.1718805, 0.1469697);
      pilot[36] = gr_complex(0.03747426, 0.04949864);
      pilot[37] = gr_complex(0.1061898, -0.1556831);
      pilot[38] = gr_complex(-0.2043626, -0.09794762);
      pilot[39] = gr_complex(-0.06801241, -0.1884577);
      pilot[40] = gr_complex(0.1731397, -0.04595287);
      pilot[41] = gr_complex(0.09467665, 0.007235643);
      pilot[42] = gr_complex(0.001755476, -0.2041379);
      pilot[43] = gr_complex(-0.2428342, -0.08410110);
      pilot[44] = gr_complex(0.04344563, -0.1038963);
      pilot[45] = gr_complex(0.1041394, -0.02651732);
      pilot[46] = gr_complex(-0.03990859, 0.2851733);
      pilot[47] = gr_complex(0.2116542, -0.007269842);
      pilot[48] = gr_complex(0.1109401, -0.1109400);
      pilot[49] = gr_complex(0.06553086, 0.1745648);
      pilot[50] = gr_complex(-0.1015436, 0.06975659);
      pilot[51] = gr_complex(-0.2329965, 0.1157810);
      pilot[52] = gr_complex(0.1459409, 0.1639366);
      pilot[53] = gr_complex(0.1234663, 0.02506706);
      pilot[54] = gr_complex(-0.1070529, 0.1442862);
      pilot[55] = gr_complex(-0.1002102, -0.03870285);
      pilot[56] = gr_complex(-0.06219967, -0.2678330);
      pilot[57] = gr_complex(-0.2163544, -0.02940571);
      pilot[58] = gr_complex(-0.2260059, -0.03639078);
      pilot[59] = gr_complex(0.1332588, -0.1314248);
      pilot[60] = gr_complex(-0.004980664, 0.09545150);
      pilot[61] = gr_complex(-0.1631041, 0.2043581);
      pilot[62] = gr_complex(0.1628006, 0.1879265);
      pilot[63] = gr_complex(0.02180564, 0.1732432);
      pilot[64] = gr_complex(-0.2773501, 0.000000);
      pilot[65] = gr_complex(0.02180564, -0.1732432);
      pilot[66] = gr_complex(0.1628006, -0.1879265);
      pilot[67] = gr_complex(-0.1631041, -0.2043581);
      pilot[68] = gr_complex(-0.004980664, -0.09545150);
      pilot[69] = gr_complex(0.1332588, 0.1314248);
      pilot[70] = gr_complex(-0.2260059, 0.03639078);
      pilot[71] = gr_complex(-0.2163544, 0.02940571);
      pilot[72] = gr_complex(-0.06219967, 0.2678330);
      pilot[73] = gr_complex(-0.1002102, 0.03870285);
      pilot[74] = gr_complex(-0.1070529, -0.1442862);
      pilot[75] = gr_complex(0.1234663, -0.02506706);
      pilot[76] = gr_complex(0.1459409, -0.1639366);
      pilot[77] = gr_complex(-0.2329965, -0.1157810);
      pilot[78] = gr_complex(-0.1015436, -0.06975659);
      pilot[79] = gr_complex(0.06553086, -0.1745648);
      pilot[80] = gr_complex(0.1109401, 0.1109400);
      pilot[81] = gr_complex(0.2116542, 0.007269842);
      pilot[82] = gr_complex(-0.03990859, -0.2851733);
      pilot[83] = gr_complex(0.1041394, 0.02651732);
      pilot[84] = gr_complex(0.04344563, 0.1038963);
      pilot[85] = gr_complex(-0.2428342, 0.08410110);
      pilot[86] = gr_complex(0.001755476, 0.2041379);
      pilot[87] = gr_complex(0.09467665, -0.007235643);
      pilot[88] = gr_complex(0.1731397, 0.04595287);
      pilot[89] = gr_complex(-0.06801241, 0.1884577);
      pilot[90] = gr_complex(-0.2043626, 0.09794762);
      pilot[91] = gr_complex(0.1061898, 0.1556831);
      pilot[92] = gr_complex(0.03747426, -0.04949864);
      pilot[93] = gr_complex(0.1718805, -0.1469697);
      pilot[94] = gr_complex(0.07055733, 0.1973099);
      pilot[95] = gr_complex(-0.009090421, 0.2135820);
      pilot[96] = gr_complex(0.2773501, 0.000000);
      pilot[97] = gr_complex(-0.009090421, -0.2135820);
      pilot[98] = gr_complex(0.07055733, -0.1973099);
      pilot[99] = gr_complex(0.1718805, 0.1469697);
      pilot[100] = gr_complex(0.03747426, 0.04949864);
      pilot[101] = gr_complex(0.1061898, -0.1556831);
      pilot[102] = gr_complex(-0.2043626, -0.09794762);
      pilot[103] = gr_complex(-0.06801241, -0.1884577);
      pilot[104] = gr_complex(0.1731397, -0.04595287);
      pilot[105] = gr_complex(0.09467665, 0.007235643);
      pilot[106] = gr_complex(0.001755476, -0.2041379);
      pilot[107] = gr_complex(-0.2428342, -0.08410110);
      pilot[108] = gr_complex(0.04344563, -0.1038963);
      pilot[109] = gr_complex(0.1041394, -0.02651732);
      pilot[110] = gr_complex(-0.03990859, 0.2851733);
      pilot[111] = gr_complex(0.2116542, -0.007269842);
      pilot[112] = gr_complex(0.1109401, -0.1109400);
      pilot[113] = gr_complex(0.06553086, 0.1745648);
      pilot[114] = gr_complex(-0.1015436, 0.06975659);
      pilot[115] = gr_complex(-0.2329965, 0.1157810);
      pilot[116] = gr_complex(0.1459409, 0.1639366);
      pilot[117] = gr_complex(0.1234663, 0.02506706);
      pilot[118] = gr_complex(-0.1070529, 0.1442862);
      pilot[119] = gr_complex(-0.1002102, -0.03870285);
      pilot[120] = gr_complex(-0.06219967, -0.2678330);
      pilot[121] = gr_complex(-0.2163544, -0.02940571);
      pilot[122] = gr_complex(-0.2260059, -0.03639078);
      pilot[123] = gr_complex(0.1332588, -0.1314248);
      pilot[124] = gr_complex(-0.004980664, 0.09545150);
      pilot[125] = gr_complex(-0.1631041, 0.2043581);
      pilot[126] = gr_complex(0.1628006, 0.1879265);
      pilot[127] = gr_complex(0.02180564, 0.1732432);
      pilot[128] = gr_complex(-0.2773501, 0.000000);
      pilot[129] = gr_complex(0.02180564, -0.1732432);
      pilot[130] = gr_complex(0.1628006, -0.1879265);
      pilot[131] = gr_complex(-0.1631041, -0.2043581);
      pilot[132] = gr_complex(-0.004980664, -0.09545150);
      pilot[133] = gr_complex(0.1332588, 0.1314248);
      pilot[134] = gr_complex(-0.2260059, 0.03639078);
      pilot[135] = gr_complex(-0.2163544, 0.02940571);
      pilot[136] = gr_complex(-0.06219967, 0.2678330);
      pilot[137] = gr_complex(-0.1002102, 0.03870285);
      pilot[138] = gr_complex(-0.1070529, -0.1442862);
      pilot[139] = gr_complex(0.1234663, -0.02506706);
      pilot[140] = gr_complex(0.1459409, -0.1639366);
      pilot[141] = gr_complex(-0.2329965, -0.1157810);
      pilot[142] = gr_complex(-0.1015436, -0.06975659);
      pilot[143] = gr_complex(0.06553086, -0.1745648);
      pilot[144] = gr_complex(0.1109401, 0.1109400);
      pilot[145] = gr_complex(0.2116542, 0.007269842);
      pilot[146] = gr_complex(-0.03990859, -0.2851733);
      pilot[147] = gr_complex(0.1041394, 0.02651732);
      pilot[148] = gr_complex(0.04344563, 0.1038963);
      pilot[149] = gr_complex(-0.2428342, 0.08410110);
      pilot[150] = gr_complex(0.001755476, 0.2041379);
      pilot[151] = gr_complex(0.09467665, -0.007235643);
      pilot[152] = gr_complex(0.1731397, 0.04595287);
      pilot[153] = gr_complex(-0.06801241, 0.1884577);
      pilot[154] = gr_complex(-0.2043626, 0.09794762);
      pilot[155] = gr_complex(0.1061898, 0.1556831);
      pilot[156] = gr_complex(0.03747426, -0.04949864);
      pilot[157] = gr_complex(0.1718805, -0.1469697);
      pilot[158] = gr_complex(0.07055733, 0.1973099);
      pilot[159] = gr_complex(-0.009090421, 0.2135820);
      // scale appropriately
      //for (int idx = 0; idx < 160; idx++) {
      //  pilot[idx] *= 2.25;
      //}
    }

    void
    pilot_qpsk(gr_complex* pilot)
    {
      pilot[0] = gr_complex(0.2415301, 0.2260483);
      pilot[1] = gr_complex(0.2362651, 0.2448738);
      pilot[2] = gr_complex(0.2294977, 0.2328913);
      pilot[3] = gr_complex(0.2469668, 0.2359375);
      pilot[4] = gr_complex(0.2266778, 0.2383683);
      pilot[5] = gr_complex(0.2061197, 0.2414872);
      pilot[6] = gr_complex(0.2997628, 0.2237712);
      pilot[7] = gr_complex(0.2841261, 0.2129913);
      pilot[8] = gr_complex(-0.06339657, 0.2949414);
      pilot[9] = gr_complex(-0.2966726, 0.2741778);
      pilot[10] = gr_complex(-0.002312267, -0.04418950);
      pilot[11] = gr_complex(0.2996568, -0.2846775);
      pilot[12] = gr_complex(0.06442316, -0.05345832);
      pilot[13] = gr_complex(-0.2925742, 0.2873699);
      pilot[14] = gr_complex(-0.2882507, 0.2860050);
      pilot[15] = gr_complex(-0.1892664, 0.1848816);
      pilot[16] = gr_complex(-0.2752704, 0.2856798);
      pilot[17] = gr_complex(-0.2717815, 0.2861970);
      pilot[18] = gr_complex(0.006807338, -0.05156344);
      pilot[19] = gr_complex(0.2665204, -0.2855588);
      pilot[20] = gr_complex(0.2675541, -0.04496950);
      pilot[21] = gr_complex(0.2081669, 0.2820880);
      pilot[22] = gr_complex(0.2716217, 0.2837466);
      pilot[23] = gr_complex(0.2605682, 0.1966480);
      pilot[24] = gr_complex(0.01265812, 0.2718523);
      pilot[25] = gr_complex(-0.2487035, 0.2683555);
      pilot[26] = gr_complex(-0.3282138, 0.0005040909);
      pilot[27] = gr_complex(-0.2421811, -0.2604103);
      pilot[28] = gr_complex(0.005069729, -0.2852476);
      pilot[29] = gr_complex(0.2653926, -0.2160861);
      pilot[30] = gr_complex(0.2735989, -0.2267712);
      pilot[31] = gr_complex(0.1907513, -0.2531302);
      pilot[32] = gr_complex(0.2886056, -0.2191226);
      pilot[33] = gr_complex(0.2846102, -0.2205545);
      pilot[34] = gr_complex(-0.05311486, -0.2880961);
      pilot[35] = gr_complex(-0.2726818, -0.2470663);
      pilot[36] = gr_complex(-0.06083876, -0.01463879);
      pilot[37] = gr_complex(0.2497119, 0.2417358);
      pilot[38] = gr_complex(0.3604469, 0.3383162);
      pilot[39] = gr_complex(0.2476456, 0.2339509);
      pilot[40] = gr_complex(-0.05856207, -0.002824778);
      pilot[41] = gr_complex(-0.2698561, -0.2309523);
      pilot[42] = gr_complex(-0.05873690, -0.3357959);
      pilot[43] = gr_complex(0.2854967, -0.2470277);
      pilot[44] = gr_complex(0.2933628, 0.01821897);
      pilot[45] = gr_complex(0.1791395, 0.2554515);
      pilot[46] = gr_complex(0.2849427, 0.2719296);
      pilot[47] = gr_complex(0.2922719, 0.2106134);
      pilot[48] = gr_complex(-0.05788809, 0.2645695);
      pilot[49] = gr_complex(-0.2855059, 0.2676133);
      pilot[50] = gr_complex(-0.03860194, 0.008550827);
      pilot[51] = gr_complex(0.2692111, -0.2739787);
      pilot[52] = gr_complex(0.2947738, -0.2746450);
      pilot[53] = gr_complex(0.2248881, -0.1889465);
      pilot[54] = gr_complex(0.2075375, -0.2892225);
      pilot[55] = gr_complex(0.2236260, -0.2924972);
      pilot[56] = gr_complex(0.2960655, 0.06529105);
      pilot[57] = gr_complex(0.2706344, 0.2988375);
      pilot[58] = gr_complex(-0.04156519, -0.002312247);
      pilot[59] = gr_complex(-0.2855397, -0.2974918);
      pilot[60] = gr_complex(-0.05456954, -0.06252863);
      pilot[61] = gr_complex(0.2885890, 0.2842031);
      pilot[62] = gr_complex(0.2859116, 0.2987909);
      pilot[63] = gr_complex(0.1893616, 0.2072588);
      pilot[64] = gr_complex(0.2776349, 0.2264353);
      pilot[65] = gr_complex(0.2698198, 0.2455117);
      pilot[66] = gr_complex(-0.007173449, 0.2313452);
      pilot[67] = gr_complex(-0.2564558, 0.2353229);
      pilot[68] = gr_complex(-0.2818089, 0.2406278);
      pilot[69] = gr_complex(-0.2293922, 0.2347062);
      pilot[70] = gr_complex(-0.2122173, 0.2324805);
      pilot[71] = gr_complex(-0.2275259, 0.2367725);
      pilot[72] = gr_complex(-0.2839042, 0.2383511);
      pilot[73] = gr_complex(-0.2584940, 0.2324972);
      pilot[74] = gr_complex(-0.002667191, 0.2369672);
      pilot[75] = gr_complex(0.2687763, 0.2446251);
      pilot[76] = gr_complex(0.2744262, 0.2216781);
      pilot[77] = gr_complex(0.2000531, 0.2188707);
      pilot[78] = gr_complex(0.2742395, 0.2874470);
      pilot[79] = gr_complex(0.2675741, 0.2556854);
      for (int idx_sig_pilot = 0; idx_sig_pilot < 80; idx_sig_pilot++) 
      {
              pilot[idx_sig_pilot+80] = pilot[idx_sig_pilot];
      }
    }

    void
    pilot_symmetric(gr_complex* pilot)
    {
      pilot[0] = gr_complex(-0.125000000000000, 0.0625000000000000);
      pilot[1] = gr_complex(0.105394756677054, 0.0878555783066188);
      pilot[2] = gr_complex(-0.00737027342239904, -0.0966425138032576);
      pilot[3] = gr_complex(-0.348206785428921, 0.0432568283664788);
      pilot[4] = gr_complex(-0.306726427265922, 0.309813334387990);
      pilot[5] = gr_complex(-0.0488555723557256, -0.105744015744401);
      pilot[6] = gr_complex(-0.0941327917604179, -0.101820914942547);
      pilot[7] = gr_complex(-0.177351248468494, 0.170844567046037);
      pilot[8] = gr_complex(0.0388325214724777, 0.0129441738241592);
      pilot[9] = gr_complex(-0.0350906596181919, 0.0425799529170058);
      pilot[10] = gr_complex(0.218936491841036, -0.0816331390336931);
      pilot[11] = gr_complex(0.277179654820419, -0.178890283239925);
      pilot[12] = gr_complex(-0.356724011776221, 0.0748317924806947);
      pilot[13] = gr_complex(-0.121019298472119, -0.0142235362871961);
      pilot[14] = gr_complex(0.0739162194461587, -0.230699357452330);
      pilot[15] = gr_complex(-0.0501506182661379, -0.214717908809369);
      pilot[16] = gr_complex(0.125000000000000, -0.187500000000000);
      pilot[17] = gr_complex(-0.0501506182661380, -0.214717908809369);
      pilot[18] = gr_complex(0.0739162194461587, -0.230699357452330);
      pilot[19] = gr_complex(-0.121019298472119, -0.0142235362871961);
      pilot[20] = gr_complex(-0.356724011776221, 0.0748317924806947);
      pilot[21] = gr_complex(0.277179654820419, -0.178890283239925);
      pilot[22] = gr_complex(0.218936491841036, -0.0816331390336930);
      pilot[23] = gr_complex(-0.0350906596181919, 0.0425799529170058);
      pilot[24] = gr_complex(0.0388325214724777, 0.0129441738241592);
      pilot[25] = gr_complex(-0.177351248468494, 0.170844567046037);
      pilot[26] = gr_complex(-0.0941327917604179, -0.101820914942547);
      pilot[27] = gr_complex(-0.0488555723557256, -0.105744015744401);
      pilot[28] = gr_complex(-0.306726427265922, 0.309813334387990);
      pilot[29] = gr_complex(-0.348206785428921, 0.0432568283664788);
      pilot[30] = gr_complex(-0.00737027342239904, -0.0966425138032576);
      pilot[31] = gr_complex(0.105394756677054, 0.0878555783066188);
      pilot[32] = gr_complex(-0.125000000000000, 0.0625000000000000);
      pilot[33] = gr_complex(0.0574520747571235, -0.0108560828136444);
      pilot[34] = gr_complex(0.141623416990853, -0.0191043426282881);
      pilot[35] = gr_complex(0.126529190000493, 0.227803787646502);
      pilot[36] = gr_complex(0.163420601090081, 0.0986575347328064);
      pilot[37] = gr_complex(-0.173155377467864, -0.293335891969060);
      pilot[38] = gr_complex(-0.0165946205854483, -0.258906497403319);
      pilot[39] = gr_complex(0.114615531315137, -0.0793133234540199);
      pilot[40] = gr_complex(-0.226332521472478, -0.0754441738241592);
      pilot[41] = gr_complex(-0.0271736232284508, -0.244546175623114);
      pilot[42] = gr_complex(0.0167909205048302, 0.0673605513795593);
      pilot[43] = gr_complex(-0.180168704996829, 0.167807119994380);
      pilot[44] = gr_complex(0.125029837952061, -0.108302661601491);
      pilot[45] = gr_complex(0.217696893900546, 0.256879381826496);
      pilot[46] = gr_complex(-0.0831693630146130, -0.0285537861161242);
      pilot[47] = gr_complex(-0.237696213168040, -0.355399998162787);
      pilot[48] = gr_complex(-0.250000000000000, -0.0625000000000000);
      pilot[49] = gr_complex(-0.237696213168040, -0.355399998162788);
      pilot[50] = gr_complex(-0.0831693630146130, -0.0285537861161242);
      pilot[51] = gr_complex(0.217696893900546, 0.256879381826496);
      pilot[52] = gr_complex(0.125029837952061, -0.108302661601491);
      pilot[53] = gr_complex(-0.180168704996829, 0.167807119994380);
      pilot[54] = gr_complex(0.0167909205048302, 0.0673605513795593);
      pilot[55] = gr_complex(-0.0271736232284508, -0.244546175623114);
      pilot[56] = gr_complex(-0.226332521472478, -0.0754441738241592);
      pilot[57] = gr_complex(0.114615531315137, -0.0793133234540199);
      pilot[58] = gr_complex(-0.0165946205854483, -0.258906497403319);
      pilot[59] = gr_complex(-0.173155377467864, -0.293335891969060);
      pilot[60] = gr_complex(0.163420601090081, 0.0986575347328064);
      pilot[61] = gr_complex(0.126529190000493, 0.227803787646502);
      pilot[62] = gr_complex(0.141623416990853, -0.0191043426282880);
      pilot[63] = gr_complex(0.0574520747571235, -0.0108560828136444);
      pilot[64] = gr_complex(-0.125000000000000, 0.0625000000000000);
      pilot[65] = gr_complex(0.105394756677054, 0.0878555783066188);
      pilot[66] = gr_complex(-0.00737027342239904, -0.0966425138032576);
      pilot[67] = gr_complex(-0.348206785428921, 0.0432568283664788);
      pilot[68] = gr_complex(-0.306726427265922, 0.309813334387990);
      pilot[69] = gr_complex(-0.0488555723557256, -0.105744015744401);
      pilot[70] = gr_complex(-0.0941327917604179, -0.101820914942547);
      pilot[71] = gr_complex(-0.177351248468494, 0.170844567046037);
      pilot[72] = gr_complex(0.0388325214724777, 0.0129441738241592);
      pilot[73] = gr_complex(-0.0350906596181919, 0.0425799529170058);
      pilot[74] = gr_complex(0.218936491841036, -0.0816331390336931);
      pilot[75] = gr_complex(0.277179654820419, -0.178890283239925);
      pilot[76] = gr_complex(-0.356724011776221, 0.0748317924806947);
      pilot[77] = gr_complex(-0.121019298472119, -0.0142235362871961);
      pilot[78] = gr_complex(0.0739162194461587, -0.230699357452330);
      pilot[79] = gr_complex(-0.0501506182661379, -0.214717908809369);
      for (int idx_sig_pilot = 0; idx_sig_pilot < 80; idx_sig_pilot++) 
      {
        pilot[idx_sig_pilot+80] = pilot[idx_sig_pilot];
      }
    }

    void
    pilot_sync2(gr_complex* pilot)
    {
      pilot[0] = gr_complex(0.186025000000000, 0.0930125000000000);
      pilot[1] = gr_complex(0.321146430470934, 0.245711291356193);
      pilot[2] = gr_complex(0.00541891602060207, 0.0993353200279574); 
      pilot[3] = gr_complex(-0.285318228340574, 0.229518079506709);
      pilot[4] = gr_complex(0.204526336570900, 0.165862424467408);
      pilot[5] = gr_complex(-0.0110874127106207, -0.126700474986762);
      pilot[6] = gr_complex(-0.106094920550298, -0.114687722577076);
      pilot[7] = gr_complex(0.422432004720176, -0.0542106075797569);
      pilot[8] = gr_complex(-0.449104077940455, -0.104296808455341);
      pilot[9] = gr_complex(-0.431334563663399, 0.0343342477095727);
      pilot[10] = gr_complex(0.237634459520526, 0.172572873877521);
      pilot[11] = gr_complex(0.194773625816765, 0.0715132110883542);
      pilot[12] = gr_complex(0.325228043499924, -0.155849573267744);
      pilot[13] = gr_complex(-0.425724045609008, -0.383327169591475);
      pilot[14] = gr_complex(-0.136958454990830, -0.387026621223704);
      pilot[15] = gr_complex(0.345087674600714, -0.162027241244462);
      pilot[16] = gr_complex(-0.186025000000000, 0.00000000000000);
      pilot[17] = gr_complex(0.345087674600714, 0.162027241244462);
      pilot[18] = gr_complex(-0.136958454990830, 0.387026621223704);
      pilot[19] = gr_complex(-0.425724045609008, 0.383327169591475);
      pilot[20] = gr_complex(0.325228043499924, 0.155849573267744);
      pilot[21] = gr_complex(0.194773625816765, -0.0715132110883542);
      pilot[22] = gr_complex(0.237634459520526, -0.172572873877521);
      pilot[23] = gr_complex(-0.431334563663399, -0.0343342477095727);
      pilot[24] = gr_complex(-0.449104077940455, 0.104296808455341);
      pilot[25] = gr_complex(0.422432004720176, 0.0542106075797570);
      pilot[26] = gr_complex(-0.106094920550298, 0.114687722577076);
      pilot[27] = gr_complex(-0.0110874127106206, 0.126700474986762);
      pilot[28] = gr_complex(0.204526336570900, -0.165862424467408);
      pilot[29] = gr_complex(-0.285318228340574, -0.229518079506709);
      pilot[30] = gr_complex(0.00541891602060209, -0.0993353200279574);
      pilot[31] = gr_complex(0.321146430470934, -0.245711291356193);
      pilot[32] = gr_complex(0.186025000000000, -0.0930125000000000);
      pilot[33] = gr_complex(0.0156983448938515, 0.351198300308213);
      pilot[34] = gr_complex(0.00541891602060207, 0.157878365477758);
      pilot[35] = gr_complex(-0.176287259044145, -0.0799775834093137);
      pilot[36] = gr_complex(-0.281580414511355, 0.337727114502820);
      pilot[37] = gr_complex(-0.185352351175256, 0.436536075174812);
      pilot[38] = gr_complex(-0.106094920550298, 0.100527412612488);
      pilot[39] = gr_complex(0.312168281575520, 0.162150025937668);
      pilot[40] = gr_complex(0.0770540779404550, 0.290321808455341);
      pilot[41] = gr_complex(-0.182564015703273, 0.222627497506538);
      pilot[42] = gr_complex(0.237634459520526, 0.185316816157891);
      pilot[43] = gr_complex(0.0823164401994809, -0.0720005420192420);
      pilot[44] = gr_complex(0.123876034440531, -0.0846608877620284);
      pilot[45] = gr_complex(0.280521074982447, 0.440486578689023);
      pilot[46] = gr_complex(-0.136958454990830, 0.272190306729420);
      pilot[47] = gr_complex(-0.276476001013613, -0.362298996113843);
      pilot[48] = gr_complex(-0.186025000000000, 0.00000000000000);
      pilot[49] = gr_complex(-0.276476001013613, 0.362298996113843);
      pilot[50] = gr_complex(-0.136958454990830, -0.272190306729420);
      pilot[51] = gr_complex(0.280521074982447, -0.440486578689023);
      pilot[52] = gr_complex(0.123876034440531, 0.0846608877620283);
      pilot[53] = gr_complex(0.0823164401994809, 0.0720005420192421);
      pilot[54] = gr_complex(0.237634459520526, -0.185316816157891);
      pilot[55] = gr_complex(-0.182564015703273, -0.222627497506538);
      pilot[56] = gr_complex(0.0770540779404550, -0.290321808455341);
      pilot[57] = gr_complex(0.312168281575520, -0.162150025937668);
      pilot[58] = gr_complex(-0.106094920550298, -0.100527412612488);
      pilot[59] = gr_complex(-0.185352351175256, -0.436536075174812);
      pilot[60] = gr_complex(-0.281580414511355, -0.337727114502820);
      pilot[61] = gr_complex(-0.176287259044145, 0.0799775834093136);
      pilot[62] = gr_complex(0.00541891602060209, -0.157878365477758);
      pilot[63] = gr_complex(0.0156983448938515, -0.351198300308213);
      pilot[64] = gr_complex(0.186025000000000, 0.0930125000000000);
      pilot[65] = gr_complex(0.321146430470934, 0.245711291356193);
      pilot[66] = gr_complex(0.00541891602060207, 0.0993353200279574);
      pilot[67] = gr_complex(-0.285318228340574, 0.229518079506709);
      pilot[68] = gr_complex(0.204526336570900, 0.165862424467408);
      pilot[69] = gr_complex(-0.0110874127106207, -0.126700474986762);
      pilot[70] = gr_complex(-0.106094920550298, -0.114687722577076);
      pilot[71] = gr_complex(0.422432004720176, -0.0542106075797569);
      pilot[72] = gr_complex(-0.449104077940455, -0.104296808455341);
      pilot[73] = gr_complex(-0.431334563663399, 0.0343342477095727);
      pilot[74] = gr_complex(0.237634459520526, 0.172572873877521);
      pilot[75] = gr_complex(0.194773625816765, 0.0715132110883542);
      pilot[76] = gr_complex(0.325228043499924, -0.155849573267744);
      pilot[77] = gr_complex(-0.425724045609008, -0.383327169591475);
      pilot[78] = gr_complex(-0.136958454990830, -0.387026621223704);
      pilot[79] = gr_complex(0.345087674600714, -0.162027241244462);
      for (int idx_sig_pilot = 0; idx_sig_pilot < 80; idx_sig_pilot++) 
      {
        pilot[idx_sig_pilot+80] = pilot[idx_sig_pilot];
      }
    }

    void
    pilot_sync1(gr_complex* pilot)
    {
      pilot[ 0] = gr_complex( 0.0000,  0.0000);
      pilot[ 1] = gr_complex( 0.5000, -0.1435);
      pilot[ 2] = gr_complex( 0.0517, -0.1429);
      pilot[ 3] = gr_complex( 0.0615, -0.1139);
      pilot[ 4] = gr_complex( 0.0147, -0.2222);
      pilot[ 5] = gr_complex(-0.1490,  0.0156);
      pilot[ 6] = gr_complex( 0.0912,  0.0139);
      pilot[ 7] = gr_complex( 0.2461, -0.1844);
      pilot[ 8] = gr_complex( 0.1973,  0.1315);
      pilot[ 9] = gr_complex(-0.0908,  0.2135);
      pilot[10] = gr_complex( 0.0181, -0.0698);
      pilot[11] = gr_complex( 0.1200, -0.1509);
      pilot[12] = gr_complex(-0.2075, -0.1927);
      pilot[13] = gr_complex(-0.0559, -0.0674);
      pilot[14] = gr_complex(-0.0773, -0.0955);
      pilot[15] = gr_complex(-0.1356, -0.3690);
      pilot[16] = gr_complex( 0.0930,  0.0000);
      pilot[17] = gr_complex(-0.1356,  0.3690);
      pilot[18] = gr_complex(-0.0773,  0.0955);
      pilot[19] = gr_complex(-0.0559,  0.0674);
      pilot[20] = gr_complex(-0.2075,  0.1927);
      pilot[21] = gr_complex( 0.1200,  0.1509);
      pilot[22] = gr_complex( 0.0181,  0.0698);
      pilot[23] = gr_complex(-0.0908, -0.2135);
      pilot[24] = gr_complex( 0.1973, -0.1315);
      pilot[25] = gr_complex( 0.2461,  0.1844);
      pilot[26] = gr_complex( 0.0912, -0.0139);
      pilot[27] = gr_complex(-0.1490, -0.0156);
      pilot[28] = gr_complex( 0.0147,  0.2222);
      pilot[29] = gr_complex( 0.0615,  0.1139);
      pilot[30] = gr_complex( 0.0517,  0.1429);
      pilot[31] = gr_complex( 0.5000,  0.1435);
      pilot[32] = gr_complex( 0.0000,  0.0000);
      pilot[33] = gr_complex(-0.5000,  0.1435);
      pilot[34] = gr_complex(-0.0517,  0.1429);
      pilot[35] = gr_complex(-0.0615,  0.1139);
      pilot[36] = gr_complex(-0.0147,  0.2222);
      pilot[37] = gr_complex( 0.1490, -0.0156);
      pilot[38] = gr_complex(-0.0912, -0.0139);
      pilot[39] = gr_complex(-0.2461,  0.1844);
      pilot[40] = gr_complex(-0.1973, -0.1315);
      pilot[41] = gr_complex( 0.0908, -0.2135);
      pilot[42] = gr_complex(-0.0181,  0.0698);
      pilot[43] = gr_complex(-0.1200,  0.1509);
      pilot[44] = gr_complex( 0.2075,  0.1927);
      pilot[45] = gr_complex( 0.0559,  0.0674);
      pilot[46] = gr_complex( 0.0773,  0.0955);
      pilot[47] = gr_complex( 0.1356,  0.3690);
      pilot[48] = gr_complex(-0.0930,  0.0000);
      pilot[49] = gr_complex( 0.1356, -0.3690);
      pilot[50] = gr_complex( 0.0773, -0.0955);
      pilot[51] = gr_complex( 0.0559, -0.0674);
      pilot[52] = gr_complex( 0.2075, -0.1927);
      pilot[53] = gr_complex(-0.1200, -0.1509);
      pilot[54] = gr_complex(-0.0181, -0.0698);
      pilot[55] = gr_complex( 0.0908,  0.2135);
      pilot[56] = gr_complex(-0.1973,  0.1315);
      pilot[57] = gr_complex(-0.2461, -0.1844);
      pilot[58] = gr_complex(-0.0912,  0.0139);
      pilot[59] = gr_complex( 0.1490,  0.0156);
      pilot[60] = gr_complex(-0.0147, -0.2222);
      pilot[61] = gr_complex(-0.0615, -0.1139);
      pilot[62] = gr_complex(-0.0517, -0.1429);
      pilot[63] = gr_complex(-0.5000, -0.1435);
      pilot[64] = gr_complex( 0.0000,  0.0000);
      pilot[65] = gr_complex( 0.5000, -0.1435);
      pilot[66] = gr_complex( 0.0517, -0.1429);
      pilot[67] = gr_complex( 0.0615, -0.1139);
      pilot[68] = gr_complex( 0.0147, -0.2222);
      pilot[69] = gr_complex(-0.1490,  0.0156);
      pilot[70] = gr_complex( 0.0912,  0.0139);
      pilot[71] = gr_complex( 0.2461, -0.1844);
      pilot[72] = gr_complex( 0.1973,  0.1315);
      pilot[73] = gr_complex(-0.0908,  0.2135);
      pilot[74] = gr_complex( 0.0181, -0.0698);
      pilot[75] = gr_complex( 0.1200, -0.1509);
      pilot[76] = gr_complex(-0.2075, -0.1927);
      pilot[77] = gr_complex(-0.0559, -0.0674);
      pilot[78] = gr_complex(-0.0773, -0.0955);
      pilot[79] = gr_complex(-0.1356, -0.3690);
      for (int idx_sig_pilot = 0; idx_sig_pilot < 80; idx_sig_pilot++) 
      {
        pilot[idx_sig_pilot+80] = pilot[idx_sig_pilot];
      }
    }

    void
    pilot_zero(gr_complex* pilot)
    {
      for (int idx = 0; idx < 160; idx++) {
        pilot[idx] = gr_complex(0.0, 0.0);
      }
    }

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_SYNC_WORDS_H */

