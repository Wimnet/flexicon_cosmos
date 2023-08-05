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

#ifndef INCLUDED_FULLDUPLEX_TX_RX_DELAY_CALC_IMPL_H
#define INCLUDED_FULLDUPLEX_TX_RX_DELAY_CALC_IMPL_H

#include <fullduplex/tx_rx_delay_calc.h>
#include <chrono>
#include <iostream>
#include <sstream>
#include <limits>
#include <Eigen/Dense>

void fill_sts_vector(gr_complex* sts)
{
  for (uint64_t offset = 0; offset < 160; offset = offset + 16)
  {
  	sts[offset+0] = gr_complex(0.0306, 0.0306); 
  	sts[offset+1] = gr_complex(-0.1763, 0.0031); 
  	sts[offset+2] = gr_complex(-0.0179, -0.1045); 
  	sts[offset+3] = gr_complex(0.1900, -0.0168); 
  	sts[offset+4] = gr_complex(0.1225, 0); 
  	sts[offset+5] = gr_complex(0.1900, -0.0168); 
  	sts[offset+6] = gr_complex(-0.0179, -0.1045); 
  	sts[offset+7] = gr_complex(-0.1763, 0.0031); 
  	sts[offset+8] = gr_complex(0.0612, 0.0612); 
  	sts[offset+9] = gr_complex(0.0031, -0.1763); 
  	sts[offset+10] = gr_complex(-0.1045, -0.0179); 
  	sts[offset+11] = gr_complex(-0.0168, 0.1900); 
  	sts[offset+12] = gr_complex(0, 0.1225); 
  	sts[offset+13] = gr_complex(-0.0168, 0.1900); 
  	sts[offset+14] = gr_complex(-0.1045, -0.0179); 
  	sts[offset+15] = gr_complex(0.0031, -0.1763); 
  }
}


namespace gr {
  namespace fullduplex {

    enum class txrx_delay_state { WAIT, COMPUTE, HOLDOFF, DONE };

    class tx_rx_delay_calc_impl : public tx_rx_delay_calc
    {
     private:
      bool d_debug;
      uint64_t d_radio;
      float d_startup_delay;
      float d_measure_delay;
      uint64_t d_startup_time;
      uint64_t d_pad_front;
      uint64_t d_pad_tail;
      uint64_t d_frame_len;
      uint64_t d_sts_length;
      uint64_t d_pilot_pos;
      uint64_t d_algorithm;
      bool d_multi_usrp;
      txrx_delay_state d_state;
      
      // output message ports
      const pmt::pmt_t d_mute_ctrl_port;
      const pmt::pmt_t d_txrx_delay_port;
      
      // STS
      // We need to define this by hand here so we can use it for
      // correlation with the Rx and Tx streams.
      gr_complex* d_sts;

     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);
      void debug_print(std::string msg);
      void send_delay_msg(uint64_t delay);
      void send_mute_msg(bool mute);
      Eigen::MatrixXcf toeplitz_transform(const gr_complex* rx_pkt, uint64_t rx_length, uint64_t kernel_length);
      uint64_t correlate(const gr_complex* in_rx);
      uint64_t energy_detection(const gr_complex* in_rx, float threshold);
      uint64_t correlate_approx(const gr_complex* in_rx);

     public:
      tx_rx_delay_calc_impl(bool debug, uint64_t radio, float measure_delay, float startup_delay, uint64_t pad_front, uint64_t pad_tail, uint64_t frame_len, uint64_t pilot_pos, uint64_t algorithm, bool multi_usrp);
      ~tx_rx_delay_calc_impl();

      // Where all the action really happens
      int work(
              int noutput_items,
              gr_vector_int &ninput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_TX_RX_DELAY_CALC_IMPL_H */

