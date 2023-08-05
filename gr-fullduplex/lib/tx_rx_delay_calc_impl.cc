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
#include "tx_rx_delay_calc_impl.h"

namespace gr {
  namespace fullduplex {

    tx_rx_delay_calc::sptr
    tx_rx_delay_calc::make(bool debug, uint64_t radio, float measure_delay, float startup_delay, uint64_t pad_front, uint64_t pad_tail, uint64_t frame_len, uint64_t pilot_pos, uint64_t algorithm, bool multi_usrp)
    {
      return gnuradio::get_initial_sptr
        (new tx_rx_delay_calc_impl(debug, radio, measure_delay, startup_delay, pad_front, pad_tail, frame_len, pilot_pos, algorithm, multi_usrp));
    }


    /*
     * The private constructor
     */
    tx_rx_delay_calc_impl::tx_rx_delay_calc_impl(bool debug, uint64_t radio, float measure_delay, float startup_delay, uint64_t pad_front, uint64_t pad_tail, uint64_t frame_len, uint64_t pilot_pos, uint64_t algorithm, bool multi_usrp)
      : gr::tagged_stream_block("tx_rx_delay_calc",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0), "packet_len"),
              d_debug(debug),
              d_radio(radio),
              d_startup_delay(startup_delay),
              d_measure_delay(measure_delay),
              d_pad_front(pad_front),
              d_pad_tail(pad_tail),
              d_frame_len(frame_len),
              d_state(txrx_delay_state::WAIT),
              d_sts_length(160),
              d_pilot_pos(pilot_pos),
              d_algorithm(algorithm),
              d_multi_usrp(multi_usrp),
              d_mute_ctrl_port(pmt::mp("mute_out")),
              d_txrx_delay_port(pmt::mp("delay_out"))
    {
    	d_startup_time = std::chrono::duration_cast<std::chrono::milliseconds>(
    		std::chrono::system_clock::now().time_since_epoch()
			).count();
      d_sts = (gr_complex*)malloc(d_sts_length*sizeof(gr_complex));
      fill_sts_vector(d_sts);
			message_port_register_out(d_mute_ctrl_port);
			message_port_register_out(d_txrx_delay_port);
    }

    /*
     * Our virtual destructor.
     */
    tx_rx_delay_calc_impl::~tx_rx_delay_calc_impl()
    {
      free(d_sts);
    }

    int
    tx_rx_delay_calc_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items = 0;
      return noutput_items;
    }

    void 
    tx_rx_delay_calc_impl::debug_print(std::string msg)
    {
      if (d_debug)
      {
        std::cout << "[TX RX DELAY CALCULATOR] " << msg << std::endl;
      }
    }

    void 
    tx_rx_delay_calc_impl::send_delay_msg(uint64_t delay)
    {
      std::stringstream ss;
      ss << "Radio: " << d_radio << " Delay: " << delay;
      debug_print(ss.str());
      pmt::pmt_t msg = pmt::cons(pmt::PMT_NIL, pmt::from_long(delay));
      message_port_pub(d_txrx_delay_port, msg);
    }

    void 
    tx_rx_delay_calc_impl::send_mute_msg(bool mute)
    {
      pmt::pmt_t msg = pmt::cons(pmt::PMT_NIL, pmt::cons(pmt::from_bool(mute), pmt::from_long(d_radio)));
      message_port_pub(d_mute_ctrl_port, msg);
    }

    Eigen::MatrixXcf 
    tx_rx_delay_calc_impl::toeplitz_transform(const gr_complex* rx_pkt, uint64_t rx_length, uint64_t kernel_length)
    {
      // copy the rx_pkt into a NxM toeplitz matrix
      // the dimension of this is huge, ~ 41000 x 160.

      // Toeplitz matrix must be defined such that it wraps around to the start 
      Eigen::MatrixXcf T(rx_length, kernel_length);
      for (uint64_t i = 0; i < rx_length; i++)
      {
        for (uint64_t j = 0; j < kernel_length; j++)
        {
          // compute index for rx_pkt
          uint64_t index = (i + j) % rx_length;
          T(i, j) = rx_pkt[index];
        }
      }
      return T;
    }

    uint64_t 
    tx_rx_delay_calc_impl::correlate(const gr_complex* in_rx)
    {
      Eigen::VectorXcf STS(160); // need to copy d_sts into an Eigen vector
      for (uint64_t idx = 0; idx < d_sts_length; idx++)
      {
        STS(idx) = d_sts[idx];
      }
      uint64_t len = d_pad_front + d_frame_len + d_pad_tail;
      // perform the correlation as a matrix multiplication
      Eigen::VectorXcf corr_out = toeplitz_transform(in_rx, len, d_sts_length) * STS;
      // output the index of maximum power
      Eigen::VectorXf corr_magnitudes = corr_out.cwiseAbs2();
      float max = -std::numeric_limits<float>::infinity();
      uint64_t max_idx = 0;
      for (uint64_t idx = 0; idx < len; idx++) {
        if (corr_magnitudes[idx] > max) {
          max_idx = idx;
          max = corr_magnitudes[idx];
        }
      }
      std::cout << "Maximum found at index " << max_idx << std::endl;
      return max_idx;
    }

    uint64_t
    tx_rx_delay_calc_impl::correlate_approx(const gr_complex* in_rx)
    {
      Eigen::VectorXcf STS(16); // need to copy d_sts into an Eigen vector
      for (uint64_t idx = 0; idx < 16; idx++)
      {
        STS(idx) = d_sts[idx];
      }
      uint64_t len = d_pad_front + d_frame_len + d_pad_tail;
      // perform the correlation as a matrix multiplication
      Eigen::VectorXcf corr_out = toeplitz_transform(in_rx, len, d_sts_length) * STS;
      // output the index of maximum power
      Eigen::VectorXf corr_magnitudes = corr_out.cwiseAbs2();
      float max = -std::numeric_limits<float>::infinity();
      uint64_t max_idx = 0;
      for (uint64_t idx = 0; idx < len; idx++) {
        if (corr_magnitudes[idx] > max) {
          max_idx = idx;
          max = corr_magnitudes[idx];
        }
      }
      std::cout << "Maximum " << max << " found at index " << max_idx << std::endl;
      return max_idx;
    }

    uint64_t
    tx_rx_delay_calc_impl::energy_detection(const gr_complex* in_rx, float threshold)
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
      for (uint64_t idx = 0; idx < len; idx++) {
        if (energy[idx] > threshold) {
          return idx;
        }
      }
      return 0;
    }

    int
    tx_rx_delay_calc_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex* in_rx = (const gr_complex*) input_items[0];

      switch (d_state) 
      {
        case (txrx_delay_state::WAIT) :
        {
          uint64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
          ).count();
          // std::stringstream ss;
          // ss << "Time offset: " << current_time - d_startup_time;
          // debug_print(ss.str());
          if (current_time - d_startup_time > d_measure_delay * 1000) 
          {
            d_state = txrx_delay_state::COMPUTE;
            send_mute_msg(false);
          }
          break;
        }
        case (txrx_delay_state::COMPUTE) :
        {
          uint64_t measured_delay;
          switch (d_algorithm)
          {
            case 0: {
              measured_delay = energy_detection(in_rx, 0.000011);
              break;
            }
            case 1: {
              measured_delay = correlate(in_rx);
              break;
            }
            case 2: {
              measured_delay = correlate_approx(in_rx);
              break;
            }
          }
          if (measured_delay != 0)
          {
            measured_delay = measured_delay - d_pad_front - d_pilot_pos * d_sts_length;
            send_delay_msg(measured_delay);
            d_state = txrx_delay_state::HOLDOFF;
            send_mute_msg(true);
          }
          break;
        }
        case (txrx_delay_state::HOLDOFF) :
        {
          uint64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
          ).count();
          // std::stringstream ss;
          // ss << "Time offset: " << current_time - d_startup_time;
          // debug_print(ss.str());
          if (current_time - d_startup_time > d_startup_delay * 1000) 
          {
            /* TODO MK this wont work with more than two radios */
            d_state = txrx_delay_state::DONE;
            send_mute_msg(d_multi_usrp);
          }
          break;
        }
        case (txrx_delay_state::DONE) :
        {
          break;
        }
      }

      noutput_items = 0;
      return noutput_items;
    }

  } /* namespace fullduplex */
} /* namespace gr */

