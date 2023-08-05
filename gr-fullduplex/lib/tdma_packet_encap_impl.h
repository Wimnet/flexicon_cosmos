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

#ifndef INCLUDED_FULLDUPLEX_TDMA_PACKET_ENCAP_IMPL_H
#define INCLUDED_FULLDUPLEX_TDMA_PACKET_ENCAP_IMPL_H

#include <fullduplex/tdma_packet_encap.h>
#include <vector>

namespace gr {
  namespace fullduplex {

    class tdma_packet_encap_impl : public tdma_packet_encap
    {
     private:
      bool d_debug;
      uint64_t d_pad_front;
      uint64_t d_pad_tail;
      uint64_t d_sig_pilot_pos;
      double d_premultiplier;
      int d_sync_word;
      uint64_t d_radio;
      bool d_transmit;
      bool d_started;
      bool d_mode;
      uint64_t d_slot;
      uint64_t d_packets_sent;
      uint64_t d_sig_pilot_len;

      std::ofstream d_fstream;

      std::vector<int> d_schedule;

      pmt::pmt_t d_schedule_port;
      pmt::pmt_t d_full_schedule_port;
      pmt::pmt_t d_start_port;
      pmt::pmt_t d_padding_adjustment_port;
      void schedule_handle(pmt::pmt_t msg);
      void full_schedule_handle(pmt::pmt_t msg);
      void start_handle(pmt::pmt_t msg);
      void padding_adj_handle(pmt::pmt_t msg);


     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:
      tdma_packet_encap_impl(bool debug, uint64_t pad_front, uint64_t pad_tail, uint64_t sig_pilot_pos, double premultiplier, int sync_word, uint64_t radio);
      ~tdma_packet_encap_impl();

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

#endif /* INCLUDED_FULLDUPLEX_TDMA_PACKET_ENCAP_IMPL_H */

