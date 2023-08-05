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

#ifndef INCLUDED_FULLDUPLEX_RADIO_SYNC_IMPL_H
#define INCLUDED_FULLDUPLEX_RADIO_SYNC_IMPL_H

#include <fullduplex/radio_sync.h>
#include <chrono>
#include <iostream>
#include <sstream>
#include <limits>
#include <Eigen/Dense>
#include <vector>

namespace gr {
  namespace fullduplex {

    enum class radio_sync_state { WAIT, COMPUTE, RADIO_WAIT, SEND_DELAYS, WAIT_TO_START_EXP, DONE };

    class radio_sync_impl : public radio_sync
    {
     private:
      // Nothing to declare in this block.
      bool d_debug;
      float d_startup_delay;
      uint64_t d_num_radios;
      float d_radio_delay;
      float d_exp_start_delay;
      uint64_t d_pad_front;
      uint64_t d_pad_tail;
      uint64_t d_frame_len;
      int* d_pilot_pos;
      radio_sync_state d_state;
      uint64_t d_startup_time;
      uint64_t d_current_radio;
      uint64_t d_symbol_length;
      uint64_t d_txrx_delay;
      std::vector<int64_t> d_tx_offsets;

      // output message ports
      const pmt::pmt_t d_mute_ctrl_port;
      const pmt::pmt_t d_txtx_delay_port;
      const pmt::pmt_t d_exp_start_port;
      const pmt::pmt_t d_txrx_delay_port;

     protected:
      void debug_print(std::string msg);
      int calculate_output_stream_length(const gr_vector_int &ninput_items);
      void send_delay_msg(uint64_t delay, uint64_t radio);
      void send_mute_msg(bool mute, uint64_t radio);
      void send_exp_start_msg();
      void control_handle(pmt::pmt_t msg);
      int64_t energy_detection(const gr_complex* in_rx, float threshold);
      void wait_state();
      void compute_state(const gr_complex* in_rx);
      void radio_wait_state();
      void send_delays_state();
      void experiment_start_wait_state();

     public:
      radio_sync_impl(bool debug, float startup_delay, uint64_t num_radios, float radio_delay, float exp_start_delay, uint64_t pad_front, uint64_t pad_tail, uint64_t frame_len, std::vector<int> pilot_pos);
      ~radio_sync_impl();

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

#endif /* INCLUDED_FULLDUPLEX_RADIO_SYNC_IMPL_H */

