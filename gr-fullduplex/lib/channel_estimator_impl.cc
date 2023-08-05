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
#include "channel_estimator_impl.h"

#include <volk/volk.h>

namespace gr
{
  namespace fullduplex
  {

    channel_estimator::sptr
    channel_estimator::make(bool debug, uint64_t frame_len, uint64_t estimation_pkt_period, uint64_t occupied_carriers, uint64_t pilot_carriers, uint64_t cyclic_prefix_len, uint64_t fft_len, uint64_t pilot_pos, uint64_t num_packets)
    {
      return gnuradio::get_initial_sptr(new channel_estimator_impl(debug, frame_len, estimation_pkt_period, occupied_carriers, pilot_carriers, cyclic_prefix_len, fft_len, pilot_pos, num_packets));
    }

    /*
     * The private constructor
     */
    channel_estimator_impl::channel_estimator_impl(bool debug, uint64_t frame_len, uint64_t estimation_pkt_period, uint64_t occupied_carriers, uint64_t pilot_carriers, uint64_t cyclic_prefix_len, uint64_t fft_len, uint64_t pilot_pos, uint64_t num_packets)
        : gr::tagged_stream_block("channel_estimator",
                                  gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                  gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                  "packet_len"),
          d_debug(debug),
          d_frame_len(frame_len),
          d_estim_pkt_prd(estimation_pkt_period),
          d_pkt_counter(0),
          d_num_packet(num_packets),
          d_ofdm_sym_len(fft_len + cyclic_prefix_len),
          d_cp_len(cyclic_prefix_len),
          d_n_subcarriers(fft_len),
          d_active_subcarriers(occupied_carriers + pilot_carriers),
          d_pilot_pos(pilot_pos),
          d_packet_count(0),
          d_msg_port_name(pmt::mp("est_msg_out")),
          d_channel_est_request_port_name(pmt::mp("channel_request_in")),
          d_requesting_estimate(true)
    {
      // Prepare message ports
      message_port_register_in(d_channel_est_request_port_name);
      set_msg_handler(d_channel_est_request_port_name, boost::bind(&channel_estimator_impl::on_request_channel_estimate, this, _1));
      message_port_register_out(d_msg_port_name);

      // Set up buffers and plan for FFT (we will reuse these for all FFTs)
      d_fft_in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * d_n_subcarriers);
      d_fft_out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * d_n_subcarriers);
      d_fft_plan = fftw_plan_dft_1d(d_n_subcarriers, d_fft_in, d_fft_out, FFTW_FORWARD, FFTW_MEASURE);

      // Prepare file for recording channel estimates
      outfile.open("/home/wimnet-gen3/Documents/Gen-3 Canceller/channel_Est.csv");

      // Pre-process Tx pilots
      load_pilots();
      std::vector<std::vector<gr_complex>> tx_ofdm_symbols;
      unsigned int sym_start;
      for (uint64_t sym_idx = 0; sym_idx < 2; sym_idx++)
      {
        std::vector<gr_complex> tx_sym(d_n_subcarriers);
        sym_start = (sym_idx * d_ofdm_sym_len) + d_cp_len; // offset by d_cp_len to remove cyclic prefix
        for (uint64_t sym_ptr = 0; sym_ptr < d_n_subcarriers; sym_ptr++)
          tx_sym[sym_ptr] = d_sig_pilot[sym_start + sym_ptr];

        tx_ofdm_symbols.push_back(tx_sym);
      }
      compute_fft(tx_ofdm_symbols[0], tx_fft_shifted_1);
      compute_fft(tx_ofdm_symbols[1], tx_fft_shifted_2);

      // Print debug information
      if (d_debug)
      {
        std::cout << "Debug mode on" << std::endl;
        std::cout << "Packet length set to " << d_frame_len << std::endl;
        std::cout << "Estimating channel every " << d_estim_pkt_prd << " packets" << std::endl;
      }
    }

    /*
     * Our virtual destructor.
     */
    channel_estimator_impl::~channel_estimator_impl()
    {
      outfile.close();
    }

    void
    channel_estimator_impl::on_request_channel_estimate(pmt::pmt_t msg)
    {
      d_requesting_estimate = true;
      debug_out("Channel estimate requested");
    }

    void
    channel_estimator_impl::debug_out(std::string msg)
    {
      if (d_debug)
        std::cout << "[OFDM CHANNEL ESTIMATOR] " << msg << std::endl;
    }

    /*void
    set_premultiplier(float premultiplier)
    {
      gr::thread::scoped_lock lock(d_mutex);
    }*/

    void
    channel_estimator_impl::load_fft_inputs(std::vector<gr_complex> in)
    {
      // need to convert gr_complex into fftw_complex
      // fftw_complex is a wrapper for a 2-element array, index 0 = real, index 1 = imag
      // gr_complex is an object
      for (int idx = 0; idx < d_n_subcarriers; idx++)
      {
        d_fft_in[idx][0] = in[idx].real();
        d_fft_in[idx][1] = in[idx].imag();
      }
    }

    void
    channel_estimator_impl::compute_fft(std::vector<gr_complex> in, std::vector<gr_complex> &out)
    {
      load_fft_inputs(in);
      fftw_execute(d_fft_plan);

      // copy the FFT into the gr_complex vector so that it is shifted and the result
      // is equivalent to MATLAB fftshift(fft(x))
      int fft_halved = d_n_subcarriers / 2;
      for (int idx = 0; idx < d_n_subcarriers; idx++)
      {
        int idx2 = (idx + fft_halved) % d_n_subcarriers;
        fftw_complex cplx = {d_fft_out[idx2][0], d_fft_out[idx2][1]};
        out[idx] = gr_complex(cplx[0], cplx[1]);
      }
    }

    void
    channel_estimator_impl::send_channel_estimate(std::vector<gr_complex> &channel_estimate)
    {
      pmt::pmt_t ch_c32_vector = pmt::make_c32vector(d_active_subcarriers, std::complex<float>(0.0, 0.0));

      // copy gr_complex channel estimate into the PMT vector
      for (uint64_t idx = 0; idx < d_active_subcarriers; idx++)
      {
        // these are theoretically 16-bit floats but such a data type doesn't exist in C++
        // so I think these are typed as "float" but I don't know. Will let the compiler
        // figure it out :^)
        // auto re = channel_estimate[idx].real();
        // auto im = channel_estimate[idx].imag();
        // pmt::c32vector_set(ch_c32_vector, idx, std::complex<float>(re, im));
        pmt::c32vector_set(ch_c32_vector, idx, channel_estimate[idx]);
      }

      message_port_pub(d_msg_port_name, pmt::cons(pmt::PMT_NIL, ch_c32_vector));
    }

    int
    channel_estimator_impl::work(int noutput_items,
                                       gr_vector_int &ninput_items,
                                       gr_vector_const_void_star &input_items,
                                       gr_vector_void_star &output_items)
    {

      // Take in an Rx frame
      const gr_complex *in_rx = (const gr_complex *)input_items[0];
      d_packet_count++;

      // Prepare outputs
      gr_complex *out_estimate = (gr_complex *)output_items[0];
      std::vector<gr_complex> channel_estimate(d_active_subcarriers);

      // Check if we received an all-zero Rx frame
      bool check_if_empty = true;
      for (int idx = 0; idx < (d_frame_len); idx++)
        check_if_empty &= (in_rx[idx] == gr_complex(0, 0));

      if (check_if_empty)
      {
        // Report to console
        std::stringstream d_str;
        d_str << "Empty timeslot. Discarded." << std::endl;
        debug_out(d_str.str());

        // Decrease packet count (undo previous addition)
        d_packet_count--;

        // Keep previous visual output, and return.
        for (uint64_t subcarrier_idx = 0; subcarrier_idx < d_active_subcarriers; subcarrier_idx++)
          out_estimate[subcarrier_idx] = d_old_est[subcarrier_idx];
        return d_active_subcarriers;
      }

      // Check if the frame length contains a whole number of OFDM symbols
      if (d_frame_len % d_ofdm_sym_len != 0)
      {
        std::stringstream d_str;
        d_str << "Packet does not contain a whole number of OFDM symbols. Packet length: "
              << d_frame_len << ", OFDM symbol size: " << d_ofdm_sym_len << ". Not performing channel estimation.";
        debug_out(d_str.str());
      }
      else
      {
        // If we've not yet hit the number of frames we're averaging...
        if (d_packet_count != d_num_packet + 1)
        {
          // Add the current frame to the queue
          std::vector<gr_complex> curr_rx_packet(d_frame_len);
          for (int idx = 0; idx < (d_frame_len); idx++)
            curr_rx_packet[idx] = in_rx[idx];
          packet_queue.push(curr_rx_packet);

          // Output the old channel estimate
          for (uint64_t subcarrier_idx = 0; subcarrier_idx < d_active_subcarriers; subcarrier_idx++)
            out_estimate[subcarrier_idx] = d_old_est[subcarrier_idx];
        }
        else
        {
          // find number of OFDM symbols in frame
          uint64_t num_ofdm_symbols = d_frame_len / d_ofdm_sym_len;

          // std::cout<<"number of packets stored in the queue: "<<packet_queue.size()<<std::endl;
          // RX
          std::vector<std::vector<gr_complex>> rx_ofdm_symbols;
          uint64_t sym_start = 0; // used to copy data from packet
          std::vector<gr_complex> rx_data(d_frame_len);
          std::vector<gr_complex> rx_sym(d_n_subcarriers);
          for (uint64_t packet_idx = 0; packet_idx < d_num_packet; packet_idx++)
          {
            rx_data = packet_queue.front();
            packet_queue.pop();
            for (uint64_t sym_idx = 0; sym_idx < (num_ofdm_symbols); sym_idx++)
            {
              sym_start = sym_idx * d_ofdm_sym_len + d_cp_len; // offset by d_cp_len to remove cyclic prefix
              for (uint64_t sym_ptr = 0; sym_ptr < d_n_subcarriers; sym_ptr++)
                rx_sym[sym_ptr] = rx_data[sym_start + sym_ptr];
              rx_ofdm_symbols.push_back(rx_sym);
            }
          }
          // std::cout<<"2nd Rx ofdm symbol of 1st packet stored in the queue: "<<rx_ofdm_symbols[1][0]<<std::endl;
          // std::cout<<"2nd Rx ofdm symbol of 2nd packet stored in the queue: "<<rx_ofdm_symbols[10][0]<<std::endl;

          // Compute channel for each OFDM symbol
          std::vector<std::vector<gr_complex>> channel_estimates;
          std::vector<gr_complex> rx_fft_shifted1(d_n_subcarriers);
          std::vector<gr_complex> rx_fft_shifted2(d_n_subcarriers);
          // int packet_idx;
          for (uint64_t idx = 0; idx < d_num_packet; idx++)
          {
            // FFTW produces a result very similar to MATLAB fft() so we need to shift the data
            // so that the subcarrier indexing still makes sense.
            // packet_idx = idx*(d_n_subcarriers*(num_ofdm_symbols/d_num_packet));

            // compute FFTs. compute_fft() takes care of the shifting and loading/unloading of FFT input and output

            compute_fft(rx_ofdm_symbols[(idx * num_ofdm_symbols) + 0 + (2 * d_pilot_pos)], rx_fft_shifted1);
            compute_fft(rx_ofdm_symbols[(idx * num_ofdm_symbols) + 1 + (2 * d_pilot_pos)], rx_fft_shifted2);
            // std::cout<<"RX_INDEX, "<<idx*(num_ofdm_symbols/d_num_packet)<<std::endl;
            //  estimate the channel
            std::vector<gr_complex> ch_estimate1(d_n_subcarriers);
            std::vector<gr_complex> ch_estimate2(d_n_subcarriers);
            volk_32fc_x2_divide_32fc(ch_estimate1.data(), rx_fft_shifted1.data(), tx_fft_shifted_1.data(), d_n_subcarriers);
            volk_32fc_x2_divide_32fc(ch_estimate2.data(), rx_fft_shifted2.data(), tx_fft_shifted_2.data(), d_n_subcarriers);

            // strip the guard and null subcarriers
            // MK TODO: this is totally specific to 802.11a / the OFDM transmitter block as configured in the
            // FD experiments. at some point we should generalize this
            std::vector<gr_complex> ch_estimate_noguard1(d_active_subcarriers);
            std::vector<gr_complex> ch_estimate_noguard2(d_active_subcarriers);
            std::vector<gr_complex> ch_estimate_corrected1(d_active_subcarriers);
            std::vector<gr_complex> ch_estimate_corrected2(d_active_subcarriers);
            if (d_active_subcarriers == 52)
            {
              uint64_t guard_offset_left = 6;

              for (uint64_t idx = 0; idx < d_active_subcarriers; idx++)
              {
                if (idx == 26) // skip the null subcarrier by incrementing the offset by 1
                  guard_offset_left++;

                ch_estimate_noguard1[idx] = ch_estimate1[guard_offset_left + idx];
                ch_estimate_noguard2[idx] = ch_estimate2[guard_offset_left + idx];
              }
              // Divide by receiver gain

              volk_32fc_x2_divide_32fc(ch_estimate_corrected1.data(), ch_estimate_noguard1.data(), rx_gain.data(), d_active_subcarriers);
              volk_32fc_x2_divide_32fc(ch_estimate_corrected2.data(), ch_estimate_noguard2.data(), rx_gain.data(), d_active_subcarriers);
            }
            else if (d_active_subcarriers == 242)
            {
              uint64_t guard_offset_left = 6;
              for (uint64_t idx = 0; idx < d_active_subcarriers; idx++)
              {
                if (idx == 121)
                  guard_offset_left += 3;

                // ch_estimate_noguard[idx] = ch_estimate[guard_offset_left + idx];
                ch_estimate_noguard1[idx] = ch_estimate1[guard_offset_left + idx];
                ch_estimate_noguard2[idx] = ch_estimate2[guard_offset_left + idx];
              }
              // Divide by receiver gain
              volk_32fc_x2_divide_32fc(ch_estimate_corrected1.data(), ch_estimate_noguard1.data(), rx_gain_ac.data(), d_active_subcarriers);
              volk_32fc_x2_divide_32fc(ch_estimate_corrected2.data(), ch_estimate_noguard2.data(), rx_gain_ac.data(), d_active_subcarriers);
            }
            // channel_estimates.push_back(ch_estimate_noguard);

            channel_estimates.push_back(ch_estimate_noguard1);
            channel_estimates.push_back(ch_estimate_noguard2);
          }

          // std::cout<<"the length of  ch_estimate vector is: "<<channel_estimates.size()<<std::endl;
          /*for (uint64_t i=0; i<d_active_subcarriers; i++)
          {
            std::cout<<"the 1st ch_estimate is: "<<channel_estimates[0][i]<<std::endl;
            std::cout<<"the 3rd ch_estimate is: "<<channel_estimates[2][i]<<std::endl;
          }*/

          // Average the channel estimate over the two pilot symbols
          debug_out("Computing Average");
          for (uint64_t subcarrier_idx = 0; subcarrier_idx < d_active_subcarriers; subcarrier_idx++)
          {
            float tally_real = 0.0;
            float tally_imag = 0.0;
            for (uint64_t sym_idx = 0; sym_idx < (2 * (d_num_packet)); sym_idx++)
            {
              tally_real += channel_estimates[sym_idx][subcarrier_idx].real();
              tally_imag += channel_estimates[sym_idx][subcarrier_idx].imag();
            }
            gr_complex avg = gr_complex(tally_real / (2 * d_num_packet), tally_imag / (2 * d_num_packet));
            channel_estimate[subcarrier_idx] = avg;
          }
          debug_out("Average computed.");

          // Copy the channel estimate into the outputs
          for (uint64_t subcarrier_idx = 0; subcarrier_idx < d_active_subcarriers; subcarrier_idx++)
          {
            /*if (subcarrier_idx == 10)
              std::cout << "Subcarrier idx 10 estimation: " << channel_estimate[subcarrier_idx] << std::endl;
            */
            out_estimate[subcarrier_idx] = channel_estimate[subcarrier_idx];
            d_old_est[subcarrier_idx] = channel_estimate[subcarrier_idx];

            //std::cout << "{" << out_estimate[subcarrier_idx].real() << ", " << out_estimate[subcarrier_idx].imag() << "}, " << std::endl;

            // Write magnitude response to csv file
            outfile << sqrt(pow(out_estimate[subcarrier_idx].real(), 2) + pow(out_estimate[subcarrier_idx].imag(), 2)) << ',';
          }
          outfile << std::endl;
          //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

          // Reset the local packet count
          d_packet_count = 0;
        }

        // Increase the global packet count...?
        d_pkt_counter++;
      }

      // Conditions to send a channel estimate to the controller:
      //  1) We received a request from the controller;
      //  2) We've reached the end of the estimation period (?)
      //  3) We've counted sufficient packets for an average
      if (d_requesting_estimate && ((d_pkt_counter % d_estim_pkt_prd) == 0) && d_packet_count == 0)
      {
        // print debug msg
        std::stringstream d_str;
        std::cout << "Channel estimation triggered, packet count " << d_pkt_counter << std::endl;
        debug_out(d_str.str());

        send_channel_estimate(channel_estimate);

        // disable channel estimation
        d_requesting_estimate = false;
      }

      // Tell runtime system how many output items we produced.
      noutput_items = d_active_subcarriers;
      return noutput_items;
    }

  } /* namespace fullduplex */
} /* namespace gr */
