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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "tdma_packet_encap_impl.h"
#include "sync_words.h"
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <chrono>
#include <string>
#include <fstream>
#include <streambuf>
#include <volk/volk.h>

namespace gr {
  namespace fullduplex {

    void pilot_lts(gr_complex* pilot);
    void pilot_qpsk(gr_complex* pilot);
    void pilot_symmetric(gr_complex* pilot);
    void pilot_sync1(gr_complex* pilot);
    void pilot_sync2(gr_complex* pilot);
    void pilot_zero(gr_complex* pilot);

    tdma_packet_encap::sptr
    tdma_packet_encap::make(bool debug, uint64_t pad_front, uint64_t pad_tail, uint64_t sig_pilot_pos, double premultiplier, int sync_word, uint64_t radio)
    {
      return gnuradio::get_initial_sptr
        (new tdma_packet_encap_impl(debug, pad_front, pad_tail, sig_pilot_pos, premultiplier, sync_word, radio));
    }


    /*
     * The private constructor
     */
    tdma_packet_encap_impl::tdma_packet_encap_impl(bool debug, uint64_t pad_front, uint64_t pad_tail, uint64_t sig_pilot_pos, double premultiplier, int sync_word, uint64_t radio)
      : gr::tagged_stream_block("tdma_packet_encap",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)), "packet_len"),
        d_debug(debug),
        d_pad_front(pad_front),
        d_pad_tail(pad_tail),
        d_sig_pilot_pos(sig_pilot_pos),
        d_premultiplier(premultiplier),
        d_sync_word(sync_word),
        d_radio(radio),
        d_transmit(true),
        d_schedule_port(pmt::mp("slot")),
        d_full_schedule_port(pmt::mp("schedule")),
        d_start_port(pmt::mp("start")),
        d_padding_adjustment_port(pmt::mp("pad_adj")),
        d_started(false),
        d_sig_pilot_len(160),
        d_packets_sent(0),
        d_mode(true), // true = real time mode, false = pregenerated schedule provided. Set to false when pregen schedule received
        d_slot(0) // used only for the pregenerated schedule
    {
      set_tag_propagation_policy(block::TPP_DONT);
      switch (d_sync_word) {
        case 0: {
          pilot_lts(d_sig_pilot);
          std::cout << "WiFi LTS pilot" << std::endl;
          break;
        }
        case 4: {
          pilot_qpsk(d_sig_pilot);
          std::cout << "QPSK pilot" << std::endl;
          break;
        }
        case 3: {
          pilot_symmetric(d_sig_pilot);
          std::cout << "Symmetric OFDM pilot" << std::endl;
          break;
        }
        case 2: {
          pilot_sync2(d_sig_pilot);
          std::cout << "Sync word 2 OFDM pilot" << std::endl;
          break;
        }
        case 1 : {
          pilot_sync1(d_sig_pilot);
          std::cout << "Sync word 1 OFDM pilot" << std::endl;
          break;
        }
        case 5: {
          pilot_zero(d_sig_pilot);
          std::cout << "No pilot";
          break;
        }
      }
      // end of d_sig_pilot

      for (int idx_sig_pilot = 0; idx_sig_pilot<80*2; idx_sig_pilot++) {
        d_sig_pilot[idx_sig_pilot] *= d_premultiplier;
      }

      std::string fn = "radio" + std::to_string(d_radio) + "_tx.out";
      d_fstream.open(fn);
      //std::cout.rdbuf(d_fstream.rdbuf());

      message_port_register_in(d_schedule_port);
      set_msg_handler(d_schedule_port, boost::bind(&tdma_packet_encap_impl::schedule_handle, this, _1));

      message_port_register_in(d_full_schedule_port);
      set_msg_handler(d_full_schedule_port, boost::bind(&tdma_packet_encap_impl::full_schedule_handle, this, _1));

      message_port_register_in(d_start_port);
      set_msg_handler(d_start_port, boost::bind(&tdma_packet_encap_impl::start_handle, this, _1));

      message_port_register_in(d_padding_adjustment_port);
      set_msg_handler(d_padding_adjustment_port, boost::bind(&tdma_packet_encap_impl::padding_adj_handle, this, _1));
    }

    /*
     * Our virtual destructor.
     */
    tdma_packet_encap_impl::~tdma_packet_encap_impl()
    {
      d_fstream.close();
    }

    void 
    tdma_packet_encap_impl::schedule_handle(pmt::pmt_t msg)
    {
      int radio = int(pmt::to_long(pmt::car(msg)));
      int schedule = int(pmt::to_long(pmt::cdr(msg)));
      if (schedule == 1 && radio == d_radio)
      {
        d_transmit = true;
      }
    }

    void 
    tdma_packet_encap_impl::full_schedule_handle(pmt::pmt_t msg)
    {
      int radio = int(pmt::to_long(pmt::car(msg)));
      if (radio == d_radio)
      {
        int num_slots = int(pmt::to_long(pmt::car(pmt::cdr(msg))));
        pmt::pmt_t schedule = pmt::cdr(pmt::cdr(msg));
        d_schedule = std::vector<int>(num_slots);
        for (int idx = 0; idx < num_slots; idx++)
        {
          d_schedule[idx] = int(pmt::to_long(pmt::vector_ref(schedule, idx)));
        }
        d_mode = false; // set to pregenerated schedule mode.
      }
      else { return; }
    }

    void 
    tdma_packet_encap_impl::start_handle(pmt::pmt_t msg)
    {
      d_transmit = false;
      d_started = true;
      // Logic when starting
    }

    void 
    tdma_packet_encap_impl::padding_adj_handle(pmt::pmt_t msg)
    {
      int radio = int(pmt::to_long(pmt::car(msg)));
      int adjustment = int(pmt::to_long(pmt::cdr(msg)));
      if (radio == d_radio)
      {
        d_pad_front = d_pad_front - adjustment;
        d_pad_tail = d_pad_tail + adjustment;
        std::cout << "Radio: " << radio << " Adjustment: " << adjustment << std::endl;
      }
    }

    int
    tdma_packet_encap_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      return ninput_items[0] + d_pad_front + d_pad_tail;
    }

    int
    tdma_packet_encap_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex* in = (const gr_complex*) input_items[0];
      gr_complex* out = (gr_complex*) output_items[0];

      int ninput = ninput_items[0];
      int noutput = noutput_items;
      int memset_size = ninput + 4*d_sig_pilot_len + d_pad_front + d_pad_tail;

      // if requested memset size is incorrect, return 0
      if (memset_size < 0 || memset_size > noutput_items)
      {
        if (d_debug)
        {
            std::cout << "REQUESTED MEMSET SIZE INCORRECT: " << memset_size << ", NOUTPUT ITEMS: " << noutput_items << std::endl;
        }
        out[0] = gr_complex(0.0, 0.0);
        return 1;
      }

      if (d_debug)
      {
          std::cout << "MEMSET: " << memset_size << ", NOUTPUT ITEMS: " << noutput_items << std::endl;
      }

      std::memset(out, 0x00, sizeof(gr_complex)*memset_size);

      if (!d_started)
      { // if not started (i.e. we are in the synch stage) then only copy the pilots
        // boost the Tx power of the pilots by 10dB
        gr_complex* tmp = (gr_complex*)malloc(sizeof(gr_complex)*d_sig_pilot_len);
        volk_32fc_s32fc_multiply_32fc(tmp, d_sig_pilot, 3.16227766, d_sig_pilot_len);
        std::memcpy(out + d_pad_front + d_sig_pilot_pos*d_sig_pilot_len, tmp, sizeof(gr_complex)*d_sig_pilot_len);
        free(tmp);
      }
      else 
      { // compute the logic to transmit. If we are in realtime mode AND d_transmit is true, OR if we are not in realtime mode and 
        // the current schedule slot is equal to 1.
        bool time_to_transmit = (d_mode && d_transmit) || (!d_mode && d_schedule[d_slot] == 1);
        if (time_to_transmit)
        {
          if (d_debug)
          {
            unsigned long long now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::cout << "PACKET SENT AT: " << now << std::endl;
          }
          std::memcpy(out + d_pad_front + d_sig_pilot_pos*d_sig_pilot_len, d_sig_pilot, sizeof(gr_complex)*d_sig_pilot_len);
          std::memcpy(out + d_pad_front +  4*d_sig_pilot_len, in, sizeof(gr_complex)*ninput_items[0]);
          d_transmit = false; // reset to false.
          d_fstream << "Sending packet " << d_packets_sent << std::endl;
        }
        d_slot = (d_slot + 1) % d_schedule.size(); // increment d_slot
      }

      int produced = ninput_items[0] + 4*d_sig_pilot_len + d_pad_front + d_pad_tail;
      const pmt::pmt_t src = pmt::string_to_symbol(alias());

      std::vector<gr::tag_t> tags;
      get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + ninput_items[0]);
      for (size_t i = 0; i < tags.size(); i++) {
        add_item_tag(0, nitems_written(0),
        tags[i].key,
        tags[i].value);
      }

      d_packets_sent++;

      return produced;
    }

  } /* namespace fullduplex */
} /* namespace gr */

