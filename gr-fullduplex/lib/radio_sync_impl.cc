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
#include "radio_sync_impl.h"

namespace gr {
  namespace fullduplex {

    radio_sync::sptr
    radio_sync::make(bool debug, float startup_delay, uint64_t num_radios, float radio_delay, float exp_start_delay, uint64_t pad_front, uint64_t pad_tail, uint64_t frame_len, std::vector<int> pilot_pos)
    {
      return gnuradio::get_initial_sptr
        (new radio_sync_impl(debug, startup_delay, num_radios, radio_delay, exp_start_delay, pad_front, pad_tail, frame_len, pilot_pos));
    }


    /*
     * The private constructor
     */
    radio_sync_impl::radio_sync_impl(bool debug, float startup_delay, uint64_t num_radios, float radio_delay, float exp_start_delay, uint64_t pad_front, uint64_t pad_tail, uint64_t frame_len, std::vector<int> pilot_pos)
      : gr::tagged_stream_block("radio_sync",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0), "packet_len"),
              d_debug(debug),
              d_startup_delay(startup_delay),
              d_num_radios(num_radios),
              d_radio_delay(radio_delay),
              d_pad_front(pad_front),
              d_pad_tail(pad_tail),
              d_frame_len(frame_len),
              d_pilot_pos(pilot_pos.data()),
              d_symbol_length(80),
              d_state(radio_sync_state::WAIT),
              d_exp_start_delay(exp_start_delay),
              d_mute_ctrl_port(pmt::mp("mute_out")),
              d_txtx_delay_port(pmt::mp("delay_out")),
              d_exp_start_port(pmt::mp("begin")),
              d_txrx_delay_port(pmt::mp("delay_in"))
    {
      d_startup_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
      ).count();
      d_current_radio = 2;
      d_tx_offsets = std::vector<int64_t>(d_num_radios);
      d_tx_offsets[0] = 0; // first radio is the reference, so it has an offset of 0
      message_port_register_out(d_mute_ctrl_port);
      message_port_register_out(d_txtx_delay_port);
      message_port_register_out(d_exp_start_port);
      message_port_register_in(d_txrx_delay_port);
      set_msg_handler(d_txrx_delay_port, boost::bind(&radio_sync_impl::control_handle, this, _1));
    }

    /*
     * Our virtual destructor.
     */
    radio_sync_impl::~radio_sync_impl()
    {
    }

    void 
    radio_sync_impl::debug_print(std::string msg)
    {
      if (d_debug)
      {
        std::cout << "[RADIO SYNC] " << msg << std::endl;
      }
    }

    void radio_sync_impl::control_handle(pmt::pmt_t msg)
    {
      d_txrx_delay = uint64_t(pmt::to_long(pmt::cdr(msg)));
    }

    void 
    radio_sync_impl::send_delay_msg(uint64_t delay, uint64_t radio)
    {
      pmt::pmt_t msg = pmt::cons(pmt::from_long(radio), pmt::from_long(delay));
      message_port_pub(d_txtx_delay_port, msg);
    }

    void 
    radio_sync_impl::send_mute_msg(bool mute, uint64_t radio)
    {
      pmt::pmt_t msg = pmt::cons(pmt::PMT_NIL, pmt::cons(pmt::from_bool(mute), pmt::from_long(radio)));
      message_port_pub(d_mute_ctrl_port, msg);
    }

    void
    radio_sync_impl::send_exp_start_msg()
    {
      pmt::pmt_t msg = pmt::cons(pmt::PMT_NIL, pmt::PMT_NIL);
      message_port_pub(d_exp_start_port, msg);
    }

    int64_t
    radio_sync_impl::energy_detection(const gr_complex* in_rx, float threshold)
    {
      uint64_t len = d_pad_front + d_frame_len + d_pad_tail;
      Eigen::VectorXcf rx_(len);
      for (uint64_t idx = 0; idx < len; idx++)
      {
        rx_(idx) = in_rx[idx];
      }
      Eigen::VectorXf energy = rx_.cwiseAbs2();
      float max = -std::numeric_limits<float>::infinity();
      // uint64_t max_idx = 0;
      // for (uint64_t idx = 0; idx < len; idx++) {
      //   if (energy[idx] > max) {
      //     max_idx = idx;
      //     max = energy[idx];
      //   }
      // }
      // std::cout << "Maximum energy is " << max << " at index " << max_idx << std::endl;
      // return max_idx;
      for (int64_t idx = 0; idx < len; idx++) {
        if (energy[idx] > threshold) {
          std::stringstream ss;
          ss << "Energy exceeded threshold at " << idx;
          debug_print(ss.str());
          return idx;
        }
      }
      return 0;
    }

    int
    radio_sync_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items = 0;
      return noutput_items ;
    }

    void
    radio_sync_impl::wait_state()
    {
      uint64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
      ).count();
      if (current_time - d_startup_time > d_startup_delay * 1000) 
      {
        d_state = radio_sync_state::COMPUTE;
        send_mute_msg(false, d_current_radio);
      }
    }

    void 
    radio_sync_impl::compute_state(const gr_complex* in_rx)
    {
      int64_t measured_delay = energy_detection(in_rx, 0.000011);
      if (measured_delay != 0)
      {
        uint64_t pilot_pos = d_current_radio - 1;
        measured_delay = measured_delay - d_pad_front - pilot_pos * (2 * d_symbol_length) - d_txrx_delay;
        std::cout << measured_delay << std::endl;
        d_tx_offsets[pilot_pos] = measured_delay; // store the delay in our list
        send_mute_msg(true, d_current_radio);
        // setup next state
        if (d_current_radio == d_num_radios) {
          d_state = radio_sync_state::SEND_DELAYS;
        }
        else
        {
          d_state = radio_sync_state::RADIO_WAIT;
          d_startup_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
          ).count();
        }
      }
      else
      {
        std::cout << d_current_radio << " No signal Rx" << std::endl;
      }
    }

    void 
    radio_sync_impl::radio_wait_state()
    {
      uint64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
      ).count();
      if (current_time - d_startup_time > d_radio_delay * 1000) 
      {
        d_current_radio = d_current_radio + 1;
        d_state = radio_sync_state::COMPUTE;
        send_mute_msg(false, d_current_radio);
      }
    }

    void
    radio_sync_impl::send_delays_state()
    {
      // no idea how long this will take, so set the state to done so if the 
      // work function is triggered again this wont run twice (or more)
      d_state = radio_sync_state::WAIT_TO_START_EXP;
      // find the most positive delay 
      int64_t max_delay = -9999999;
      for (uint64_t r = 0; r < d_num_radios; r++) 
      {
        if (d_tx_offsets[r] > max_delay)
        {
          max_delay = d_tx_offsets[r];
        }
      }
      // if max_delay is less than the front padding, we can just use the delays as they are
      // if not, then we need to adjust the delays to make sure we dont try use negative padding length
      if (max_delay > d_pad_front)
      {
        int64_t offset_adjustment = max_delay - d_pad_front;
        for (uint64_t r = 0; r < d_num_radios; r++) 
        {
          d_tx_offsets[r] = d_tx_offsets[r] - offset_adjustment;
        }
      }
      // sent the delay message and turn on radio transmission
      for (uint64_t r = 0; r < d_num_radios; r++) 
      {
        send_delay_msg(d_tx_offsets[r] + (2 * d_symbol_length * 0), r + 1);
        send_mute_msg(false, r + 1);
      }
      // reset the startup time counter so we can count down until we signal to start experiment
      d_startup_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
      ).count();
    }

    void 
    radio_sync_impl::experiment_start_wait_state()
    {
      uint64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
      ).count();
      if (current_time - d_startup_time > d_exp_start_delay * 1000) 
      {
        d_state = radio_sync_state::DONE;
        send_exp_start_msg();
      }
    }

    int
    radio_sync_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex* in_rx = (const gr_complex*) input_items[0];

      switch (d_state) {
        case (radio_sync_state::WAIT) : {
          wait_state();
          break;
        }
        case (radio_sync_state::COMPUTE) : {
          compute_state(in_rx);
          break;
        }
        case (radio_sync_state::RADIO_WAIT) : {
          radio_wait_state();
          break;
        }
        case (radio_sync_state::SEND_DELAYS) : { // compute the set of delays to align all the radio transmissions
          send_delays_state();
          break;
        }
        case (radio_sync_state::WAIT_TO_START_EXP) : { 
          experiment_start_wait_state();
          break;
        }
        default : {
          break;
        }
      }
      // Tell runtime system how many output items we produced.
      noutput_items = 0;
      return noutput_items;
    }

  } /* namespace fullduplex */
} /* namespace gr */

