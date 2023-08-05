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
#include "sync_rx_stream_impl.h"
#include <unistd.h> // write(), read(), close()

namespace gr
{
  namespace fullduplex
  {
    sync_rx_stream::sptr
    sync_rx_stream::make(
        bool debug,
        unsigned int pad_front_len,
        unsigned int frame_len,
        unsigned int timeslot_len,
        unsigned int symbol_len,
        unsigned int sig_pilot_pos,
        float thresh)
    {
      return gnuradio::get_initial_sptr(new sync_rx_stream_impl(debug, pad_front_len, frame_len, timeslot_len, symbol_len, sig_pilot_pos, thresh));
    }

    /*
     * The private constructor
     */
    sync_rx_stream_impl::sync_rx_stream_impl(
        bool debug,
        unsigned int pad_front_len,
        unsigned int frame_len,
        unsigned int timeslot_len,
        unsigned int symbol_len,
        unsigned int sig_pilot_pos,
        float thresh)
        : gr::tagged_stream_block("sync_rx_stream",
                                  gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                  gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                  "packet_len"),
          d_debug(debug),
          d_pad_front(pad_front_len),
          d_frame_len(frame_len),
          d_timeslot_len(timeslot_len),
          d_symbol_len(symbol_len),
          d_sig_pilot_pos(sig_pilot_pos),
          d_thresh(thresh),
          d_pilot_len(2 * d_symbol_len)
    {
      // Set initial delay
      set_delay(0);
      std::cout << "Initial delay: " << 0 << std::endl;
      currState = SystemState::UNSYNCED_ENERGY;
      // std::cout << "TIMESLOT [" << d_pkt_counter++ << "]\t:: INIT" << std::endl;

      // Prepare file
      outfile.open("/home/wimnet-gen3/Documents/Gen-3 Canceller/XCORR_val.csv");

      // Set up buffers and plan for FFT (we will reuse this for all FFTs)
      d_fft_in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * num_fft_pts);
      d_fft_out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * num_fft_pts);
      d_fft_plan = fftw_plan_dft_1d(num_fft_pts, d_fft_in, d_fft_out, FFTW_FORWARD, FFTW_MEASURE);
      d_ifft_plan = fftw_plan_dft_1d(num_fft_pts, d_fft_in, d_fft_out, FFTW_BACKWARD, FFTW_MEASURE);

      // Prepare the comparison signal (time-domain) by flipping and padding
      // pilot_vec.reverseInPlace();
      for (int ticker = 0; ticker < len_comp_sig; ticker++)
        pilot_vec_flipped_padded[ticker] = pilot_vec[ticker];
      for (int ticker = len_comp_sig; ticker < num_fft_pts; ticker++)
        pilot_vec_flipped_padded[ticker] = 0;

      // Compute the FFT of the comparison signal
      compute_fft(pilot_vec_flipped_padded, comp_sig_FFT);
      comp_sig_FFT_vec = Eigen::Map<Eigen::VectorXcf, Eigen::Unaligned>(comp_sig_FFT.data(), num_fft_pts);
    }

    /*
     * Our virtual destructor.
     */
    sync_rx_stream_impl::~sync_rx_stream_impl()
    {
      // Close the file
      outfile.close();
    }

    /* Set Delay */
    void sync_rx_stream_impl::set_delay(int delay_tx2rx)
    {
      gr::thread::scoped_lock lock(d_mutex);
      d_delay_tx2rx = delay_tx2rx;
      std::cout << "d_delay_tx2rx set to " << int(delay_tx2rx) << std::endl;
    }

    /* FFT Functions */
    void sync_rx_stream_impl::load_fft_inputs(std::vector<gr_complex> in)
    {
      // need to convert gr_complex into fftw_complex
      // fftw_complex is a wrapper for a 2-element array, index 0 = real, index 1 = imag
      // gr_complex is an object
      for (int idx = 0; idx < num_fft_pts; idx++)
      {
        d_fft_in[idx][0] = in[idx].real();
        d_fft_in[idx][1] = in[idx].imag();
      }
    }
    void sync_rx_stream_impl::compute_fft(std::vector<gr_complex> in, std::vector<gr_complex> &out)
    {
      load_fft_inputs(in);
      fftw_execute(d_fft_plan);

      // copy the FFT into the gr_complex vector so that it is shifted and the result
      // is equivalent to MATLAB fftshift(fft(x))
      /*
      int fft_halved = num_fft_pts / 2;
      for (int idx = 0; idx < num_fft_pts; idx++)
      {
        int idx2 = (idx + fft_halved) % num_fft_pts;
        fftw_complex cplx = {d_fft_out[idx2][0], d_fft_out[idx2][1]};
        out[idx] = gr_complex(cplx[0], cplx[1]);
      }
      */
      for (int idx = 0; idx < num_fft_pts; idx++)
      {
        out[idx] = gr_complex(d_fft_out[idx][0], d_fft_out[idx][1]);
      }
    }
    void sync_rx_stream_impl::compute_ifft(std::vector<gr_complex> in, std::vector<gr_complex> &out)
    {
      load_fft_inputs(in);
      fftw_execute(d_ifft_plan);

      // copy the FFT into the gr_complex vector so that it is shifted and the result
      // is equivalent to MATLAB fftshift(fft(x))
      /*
      int fft_halved = num_fft_pts / 2;
      for (int idx = 0; idx < num_fft_pts; idx++)
      {
        int idx2 = (idx + fft_halved) % num_fft_pts;
        fftw_complex cplx = {d_fft_out[idx2][0], d_fft_out[idx2][1]};
        out[idx] = gr_complex(cplx[0], cplx[1]);
      }
      */
      for (int idx = 0; idx < num_fft_pts; idx++)
      {
        out[idx] = gr_complex(d_fft_out[idx][0], d_fft_out[idx][1]);
      }
    }

    /* Cross Correlation */
    int sync_rx_stream_impl::xcorr(std::vector<gr_complex> &rx_timeslot, int start_idx, int num_delays_prev, int num_delays_next)
    {
      // Correct indices if need be
      if (start_idx - num_delays_prev < 0)
      {
        // std::cout << "XCORR ERROR: Out of bounds (timeslot start)." << std::endl;
        num_delays_prev = start_idx;
      }
      if (start_idx + num_delays_next >= d_timeslot_len)
      {
        // std::cout << "XCORR ERROR: Out of bounds (timeslot end)." << std::endl;
        num_delays_next = (d_timeslot_len - start_idx) - 1;
      }

      // Initialize xcorr vector
      int num_delays = num_delays_prev + 1 + num_delays_next;
      float xcorr[num_delays] = {};

      // Convert to Eigen vector and (don't) normalize the input time slot
      gr_complex *rx_timeslot_ptr = &rx_timeslot[0];
      Eigen::Map<Eigen::VectorXcf> rx_vec(rx_timeslot_ptr, d_timeslot_len);

      // Iterate through all delays
      int currDelay, idx_beg_slice;
      Eigen::VectorXcf rx_vec_slice(len_comp_sig);
      for (int ticker = 0; ticker < num_delays; ticker++)
      {
        // Determine current delay
        currDelay = -num_delays_prev + ticker;

        // Determine start index for rx_timeslot, and take the proper slice. Pad with zeros if necessary.
        idx_beg_slice = start_idx + currDelay;
        if (idx_beg_slice < 0)
        {
          // std::cout << "XCORR ERROR: Front padding required." << std::endl;

          // Front padding
          Eigen::VectorXcf rx_vec_front(-idx_beg_slice);
          for (int ticker = 0; ticker < -idx_beg_slice; ticker++)
            rx_vec_front << 0;

          // Timeslot slice
          Eigen::VectorXcf rx_vec_middle = rx_vec(Eigen::seqN(0, len_comp_sig + idx_beg_slice, 1));

          // Combine
          rx_vec_slice << rx_vec_front, rx_vec_middle;
        }
        else if (idx_beg_slice + len_comp_sig > d_timeslot_len)
        {
          // std::cout << "XCORR ERROR: Back padding required." << std::endl;

          // Back padding
          Eigen::VectorXcf rx_vec_back(len_comp_sig + idx_beg_slice - d_timeslot_len);
          for (int ticker = 0; ticker < len_comp_sig + idx_beg_slice - d_timeslot_len; ticker++)
            rx_vec_back << 0;

          // Timeslot slice
          Eigen::VectorXcf rx_vec_middle = rx_vec(Eigen::seq(idx_beg_slice, d_timeslot_len - 1, 1));

          // Combine
          rx_vec_slice << rx_vec_middle, rx_vec_back;
        }
        else
        {
          // Timeslot slice
          rx_vec_slice = rx_vec(Eigen::seqN(idx_beg_slice, len_comp_sig, 1));
        }

        // Normalize the slice
        // for (int ticker = 0; ticker < len_comp_sig; ticker++)
        // std::cout << "orig [" << ticker << "] = " << rx_vec_slice(ticker) << std::endl;
        rx_vec_slice.normalize();
        // for (int ticker = 0; ticker < len_comp_sig; ticker++)
        // std::cout << "norm [" << ticker << "] = " << rx_vec_slice(ticker) << std::endl;

        // for (int ticker = 0; ticker < len_comp_sig; ticker++)
        // std::cout << "pilot [" << ticker << "] = " << pilot_vec(ticker) << std::endl;

        // Take the dot product of the current slice with the pilot signal
        xcorr[ticker] = std::abs(rx_vec_slice.dot(pilot_vec));
        // std::cout << "xcorr[" << currDelay << "] = " << xcorr[ticker] << std::endl;
        // outfile << xcorr[ticker] << ',';
      }
      // outfile << std::endl;

      // Find the best xcorr
      float *best_xcorr_ptr = std::max_element(xcorr, xcorr + num_delays);
      float best_xcorr = *best_xcorr_ptr;

      /*if (currState == SystemState::SYNCED) {
        std::cout << "\t\tbest_xcorr: " << best_xcorr << std::endl;
        std::cout << "\t\tcomparison: " << xcorr_thresh_max * xcorr_thresh_frac << std::endl;
      }*/

      // If does not meet minimum threshold, return -1
      if (best_xcorr <= xcorr_thresh_max * xcorr_thresh_frac) {
        return -1;
      }

      // Else, determine optimal delay
      int opt_delay = std::distance(xcorr, best_xcorr_ptr) - num_delays_prev;
      // std::cout << "Max xcorr @ delay " << opt_delay << std::endl;

      // Return optimal
      return start_idx + opt_delay;
    }

    /* Fast Cross Correlation */
    int sync_rx_stream_impl::fast_xcorr(std::vector<gr_complex> &rx_timeslot, int start_idx, int num_delays_prev, int num_delays_next)
    {
      // Correct indices if need be
      if (start_idx - num_delays_prev < 0)
      {
        // std::cout << "XCORR ERROR: Out of bounds (timeslot start)." << std::endl;
        num_delays_prev = start_idx;
      }
      if (start_idx + num_delays_next >= d_timeslot_len)
      {
        // std::cout << "XCORR ERROR: Out of bounds (timeslot end)." << std::endl;
        num_delays_next = (d_timeslot_len - start_idx) - 1;
      }

      // Begin counting at start_idx - num_delays_prev
      int curr_idx = start_idx - num_delays_prev;
      int num_samples_left = num_delays_prev + 1 + num_delays_next;
      // std::cout << "Sampling from " << curr_idx << " to " << curr_idx + num_samples_left - 1 << std::endl;

      // Prepare slice vector
      // num_fft_pts = slice_len + cp_len - 1
      int slice_len = (num_fft_pts + 1) - len_comp_sig;
      std::vector<gr_complex> slice_padded(num_fft_pts);
      std::vector<gr_complex> slice_FFT(num_fft_pts);
      Eigen::VectorXcf slice_FFT_vec;

      // Prepare output
      std::vector<gr_complex> xcorr(num_samples_left, 0);

      // Process one slice at a time
      Eigen::VectorXcf curr_xcorr_FFT_vec(num_fft_pts);
      std::vector<gr_complex> curr_xcorr_FFT(num_fft_pts);
      std::vector<gr_complex> curr_xcorr(num_fft_pts);
      do
      {
        // If we have a full slice, use it; otherwise, use however many samples are left
        if (num_samples_left >= slice_len)
        {
          // std::cout << "Full slice!" << std::endl;
          for (int ticker = 0; ticker < slice_len; ticker++)
            slice_padded[ticker] = rx_timeslot[curr_idx + ticker];
          for (int ticker = slice_len; ticker < num_fft_pts; ticker++)
            slice_padded[ticker] = 0;
        }
        else
        {
          // std::cout << "Non-full slice!" << std::endl;
          for (int ticker = 0; ticker < num_samples_left; ticker++)
            slice_padded[ticker] = rx_timeslot[curr_idx + ticker];
          for (int ticker = num_samples_left; ticker < num_fft_pts; ticker++)
            slice_padded[ticker] = 0;
        }

        // Perform an FFT on the slice
        compute_fft(slice_padded, slice_FFT);
        slice_FFT_vec = Eigen::Map<Eigen::VectorXcf, Eigen::Unaligned>(slice_FFT.data(), num_fft_pts);

        // for (int ticker = 0; ticker < num_fft_pts; ticker++)
        //   std::cout << "slice_FFT[" << ticker << "] = " << slice_FFT[ticker] << std::endl;

        // Point-wise multiplication
        curr_xcorr_FFT_vec = slice_FFT_vec.array() * comp_sig_FFT_vec.array();

        // Convert xcorr_FFT_vec to gr_complex type
        for (int ticker = 0; ticker < num_fft_pts; ticker++)
          curr_xcorr_FFT[ticker] = curr_xcorr_FFT_vec[ticker];

        // Perform an IFFT
        compute_ifft(curr_xcorr_FFT, curr_xcorr);

        // Add to the full xcorr array
        if (num_samples_left >= num_fft_pts)
          for (int ticker = 0; ticker < num_fft_pts; ticker++)
            xcorr[(curr_idx - (start_idx - num_delays_prev)) + ticker] += curr_xcorr[ticker];
        else
          for (int ticker = 0; ticker < num_samples_left; ticker++)
            xcorr[(curr_idx - (start_idx - num_delays_prev)) + ticker] += curr_xcorr[ticker];

        // Update number of samples left
        num_samples_left -= slice_len;
        curr_idx += slice_len;

      } while (num_samples_left > 0);

      // Convert xcorr to a vector, obtain its magnitude, convert back to float array
      Eigen::VectorXcf xcorr_vec = Eigen::Map<Eigen::VectorXcf, Eigen::Unaligned>(xcorr.data(), xcorr.size());
      Eigen::VectorXf xcorr_mag_vec = xcorr_vec.cwiseAbs2();
      float xcorr_mag[xcorr.size()] = {};
      for (int ticker = 0; ticker < xcorr.size(); ticker++)
      {
        // std::cout << "xcorr_mag[" << ticker - num_delays_prev << "] = " << xcorr_mag_vec[ticker] << std::endl;
        xcorr_mag[ticker] = xcorr_mag_vec[ticker];
      }

      // Find the best xcorr
      float *best_xcorr_ptr = std::max_element(xcorr_mag, xcorr_mag + xcorr.size());
      float best_xcorr = *best_xcorr_ptr;

      // std::cout << "best_xcorr: " << best_xcorr << std::endl;
      // std::cout << "comparison: " << xcorr_thresh_max * xcorr_thresh_frac << std::endl;

      // If does not meet minimum threshold, return -1
      /*
      if (best_xcorr <= xcorr_thresh_max * xcorr_thresh_frac)
      {
        return -1;
      }
      */

      // Else, determine optimal delay
      int opt_delay = std::distance(xcorr_mag, best_xcorr_ptr) - num_delays_prev;
      // std::cout << "Max xcorr @ delay " << opt_delay << std::endl;

      // Return optimal
      return start_idx + opt_delay;
    }

    /* Energy Detection */
    int sync_rx_stream_impl::energy_detection(std::vector<gr_complex> &rx_timeslot, float threshold, int start_idx)
    {
      // Convert to Eigen vector and compute energy at each index
      gr_complex *ptr = &rx_timeslot[start_idx];
      Eigen::Map<Eigen::VectorXcf> rx_(ptr, d_timeslot_len - start_idx);
      Eigen::VectorXf energy = rx_.cwiseAbs2();

      // Find the first instance where energy is above the threshold
      // for (uint64_t idx = 0; idx < d_timeslot_len-start_idx; idx++)
      // for (uint64_t idx = 0; idx < energy.size(); idx++)
      // for (uint64_t idx = 0; idx < d_timeslot_len; idx++)
      for (uint64_t idx = 0; idx < energy.size(); idx++)
      {
        //std::cout << "\t\trx_timeslot[" << idx << "] = " << rx_timeslot[idx] << std::endl;
        if (energy[idx] > threshold) 
          return idx + start_idx;
      }
      return -1;
    }

    int
    sync_rx_stream_impl::work(int noutput_items,
                              gr_vector_int &ninput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items)
    {
      // Thread lock
      gr::thread::scoped_lock lock(d_mutex);

      // Input and output streams
      const gr_complex *in_rx = (const gr_complex *)input_items[0];
      gr_complex *out_rx = (gr_complex *)output_items[0];

      // Number of output items = length of frame
      noutput_items = d_frame_len;

      // Set the memory size for output streams
      std::memset(out_rx, 0x00, sizeof(gr_complex) * noutput_items);

      // Store Rx timeslot as vector, record its count
      std::vector<gr_complex> rx_timeslot{in_rx, in_rx + d_timeslot_len};

      // State case system
      int energy_search_idx = 0;
      int num_delays_prev = 10;
      int num_delays_next = 10;
      int num_delays_glob = 30;
      while (1)
      {
        switch (currState)
        {
        // Synced state
        // We should confirm the current delay is correct with a local xcorr
        case SystemState::SYNCED:
        {

          // Check if still synchronized
          check_idx = xcorr(rx_timeslot, d_delay_tx2rx+d_pad_front, num_delays_prev, num_delays_next);

          // If they do not match, we are no longer synchronized, and we should try energy detection.
          if (d_delay_tx2rx + d_pad_front != check_idx)
          {
            std::cout << "\tCorrelation check failed." << std::endl;
            currState = SystemState::UNSYNCED_ENERGY;
            std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: UNSYNCED_ENERGY" << std::endl;
            continue;
          }

          // Otherwise, all is well and we break to next time slot
          break;
        }

        // Unsynced states
        // First pass: Energy detection
        case SystemState::UNSYNCED_ENERGY:
        {
          // Try to detect energy
          std::cout << "\tEnergy search index = " << energy_search_idx << std::endl;
          energy_detection_idx = energy_detection(rx_timeslot, d_thresh, energy_search_idx);
          std::cout << "\tEnergy detected @ idx[" << energy_detection_idx << "]" << std::endl;

          // If energy detected, continue to fine-tuning correlation; otherwise, break to next timeslot
          if (energy_detection_idx >= 0 && energy_detection_idx < d_timeslot_len)
          {
            std::cout << "\tEnergy detected @ idx[" << energy_detection_idx << "]" << std::endl;

            // Run cross-correlation
            int idx_correlated = xcorr(rx_timeslot, energy_detection_idx, num_delays_prev, num_delays_next);

            // If xcorr is valid, set the new delay
            if (idx_correlated >= 0)
            {
              // Note that due to changes in sync block functionality, we no longer need to accout for front_padding. This is legacy and should eventually be removed.
              std::cout << "\tCorrelated @ idx[" << idx_correlated << "]" << std::endl;
              d_delay_tx2rx = idx_correlated - d_pad_front;

              // The current state is now synced; break to the next timeslot
              currState = SystemState::SYNCED;
              std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: SYNCED" << std::endl;
              break;
            }

            // Otherwise it fails, so we jump forward a frame length (if possible) and try again
            std::cout << "\tCorrelation failed. Energy detection index jumped";
            energy_search_idx += energy_detection_idx + d_frame_len;
            if (energy_search_idx > d_timeslot_len)
            {
              std::cout << " out of bounds. Timeslot discarded." << std::endl;
              currState = SystemState::UNSYNCED_XCORR;
              std::cout << "TIMESLOT [" << d_pkt_counter + 1 << "]\t:: UNSYNCED_XCORR" << std::endl;
              //currState = SystemState::UNSYNCED_ENERGY;
              //std::cout << "TIMESLOT [" << d_pkt_counter + 1 << "]\t:: UNSYNCED_ENERGY" << std::endl;
              break;
            }
            std::cout << "." << std::endl;
            continue;
          }

          // If no energy detected, go to cross-correlation.
          std::cout << "\tEnergy detection failed." << std::endl;
          currState = SystemState::UNSYNCED_XCORR;
          std::cout << "TIMESLOT [" << d_pkt_counter + 1 << "]\t:: UNSYNCED_XCORR" << std::endl;
          //urrState = SystemState::UNSYNCED_ENERGY;
          //std::cout << "TIMESLOT [" << d_pkt_counter + 1 << "]\t:: UNSYNCED_ENERGY" << std::endl; 
        }

        // Further attempts: Sliding cross-correlation
        case SystemState::UNSYNCED_XCORR:
        {
          // Start from the current start index
          xcorr_test_idx = xcorr(rx_timeslot, glob_xcorr_start_idx, 0, num_delays_glob);

          // If xcorr is valid, set the new delay
          if (xcorr_test_idx >= 0)
          {
            // Note that due to changes in sync block functionality, we no longer need to accout for front_padding. This is legacy and should eventually be removed.
            std::cout << "\tCorrelated @ idx[" << xcorr_test_idx << "]" << std::endl;
            d_delay_tx2rx = xcorr_test_idx - d_pad_front;

            // Reset the global xcorr start index
            glob_xcorr_start_idx = 0;

            // The current state is now synced; break to the next timeslot
            currState = SystemState::SYNCED;
            std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: SYNCED" << std::endl;
            break;
          }

          // Otherwise it fails, so we jump forward by num_delays+1 and try energy detection again on the next timeslot
          glob_xcorr_start_idx += num_delays_glob + 1;
          //std::cout << "\tGlobal xcorr index jumped";
          if (glob_xcorr_start_idx > d_timeslot_len)
          {
            std::cout << "\tGlobal xcorr index jumped out of bounds. Timeslot discarded." << std::endl;
            glob_xcorr_start_idx = 0;
          }
          currState = SystemState::UNSYNCED_ENERGY;
          std::cout << "TIMESLOT [" << d_pkt_counter + 1 << "]\t:: UNSYNCED_ENERGY" << std::endl;
          break;
        }

        }

        // Break to the next timeslot
        d_pkt_counter++;
        break;
      }

      /*
      while (1)
      {
        switch (currState)
        {
        // Initial state, where we haven't found the first Rx frame yet
        case SystemState::INIT:
        {
          // std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: INIT" << std::endl;
          //  Try to detect energy
          std::cout << "\tEnergy detection index = " << energy_search_idx << std::endl;
          energy_detection_idx = energy_detection(rx_timeslot, d_thresh, energy_search_idx);

          // If energy detected, continue to fine-tuning correlation; otherwise, break to next timeslot
          if (energy_detection_idx >= 0 && energy_detection_idx < d_timeslot_len) // <-- !!!
          {
            std::cout << "\tEnergy detected @ idx[" << energy_detection_idx << "]" << std::endl;
            currState = SystemState::TUNE_CORR;
            std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: TUNE_CORR" << std::endl;

            continue;
          }

          std::cout << "\tEnergy detection failed." << std::endl;
          std::cout << "TIMESLOT [" << d_pkt_counter + 1 << "]\t:: INIT" << std::endl;
          break;
        }

        // Fine-tuning after energy detection
        case SystemState::TUNE_CORR:
        {
          // Run cross-correlation
          int idx_correlated = xcorr(rx_timeslot, energy_detection_idx, num_delays_prev, num_delays_next);
          // int idx_correlated = xcorr(rx_timeslot, 0, 0, d_timeslot_len - 1);

          // Is the xcorr valid? If not, go to SIGNAL_LOST
          if (idx_correlated < 0)
          {
            std::cout << "\tCorrelation failed." << std::endl;
            // std::cout << "\tCorrelated Index" << idx_correlated <<std::endl;
            // currState = SystemState::SIGNAL_LOST;
            // std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: SIGNAL_LOST" << std::endl;
            // continue;

            currState = SystemState::INIT;

            // Jump forward a frame length to try again
            energy_search_idx += energy_detection_idx + d_frame_len;
            std::cout << "\tEnergy detection index jumped";
            if (energy_search_idx > d_timeslot_len)
            {
              std::cout << " out of bounds." << std::endl;
              energy_search_idx = 0;
              std::cout << "\tReturning to INIT..." << std::endl;
              std::cout << "TIMESLOT [" << d_pkt_counter + 1 << "]\t:: INIT" << std::endl;
              break;
            }
            std::cout << "." << std::endl;
            std::cout << "\tReturning to INIT..." << std::endl;
            std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: INIT" << std::endl;
            continue;
          }

          // Set the new delay, accounting for front padding
          // Note that due to changes in sync block functionality, we no longer need to accout for front_padding. This is legacy and should eventually be removed.
          std::cout << "\tCorrelated @ idx[" << idx_correlated << "]" << std::endl;
          d_delay_tx2rx = idx_correlated - d_pad_front;

          // The current state is now synced; break to the next timeslot
          currState = SystemState::SYNCED;
          std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: SYNCED" << std::endl;
          break;
        }

        // Synchronized state, when the previous time slot was aligned correctly
        case SystemState::SYNCED:
        {
          // NOTE: THIS REMOVES ALL FURTHER PROCESSING AFTER INITIAL ENERGY DETECTION

          // Check if still synchronized
          check_idx = xcorr(rx_timeslot, energy_detection_idx, num_delays_prev, num_delays_next);
          // check_idx = fast_xcorr(rx_timeslot, 0, 0, d_timeslot_len - 1);
          // int idx_correlated_test = xcorr(rx_timeslot, energy_detection_idx, num_delays_prev, num_delays_next);
          // std::cout << "Stan xcorr ==> " << idx_correlated_test << std::endl;
          // std::cout << "Fast xcorr ==> " << check_idx << std::endl;

          // If they do not match, we are no longer synchronized; otherwise, all is well and break to next timeslot
          if (d_delay_tx2rx + d_pad_front != check_idx)
          {
            currState = SystemState::UNSYNCED;
            std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: UNSYNCED" << std::endl;
            continue;
          }

          break;
        }

        // A synchronization error has been detected in the current time slot
        case SystemState::UNSYNCED:
        {
          // Try energy detection again
          energy_detection_idx = energy_detection(rx_timeslot, d_thresh, 0);
          if (energy_detection_idx == 0)
          {
            energy_detection_idx = energy_detection(rx_timeslot, d_thresh, d_frame_len - 1280);
          }

          // If energy detected, continue to fine-tuning correlation; otherwise, the signal is lost
          if (energy_detection_idx > 0)
          {
            // std::cout << "\tEnergy detected @ idx[" << energy_detection_idx << "]" << std::endl;
            currState = SystemState::TUNE_CORR;
            // std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: TUNE_CORR" << std::endl;
          }
          else
          {
            currState = SystemState::INIT;
            // std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: INIT" << std::endl;
            currState = SystemState::SIGNAL_LOST;
            std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: SIGNAL_LOST" << std::endl;
          }
          continue;
        }

        // In case the signal has been lost
        case SystemState::SIGNAL_LOST:
        {
          // usleep(delay);
          //  As energy detection failed to find the signal, we must cross-correlate across the whole timeslot
          int idx_correlated = xcorr(rx_timeslot, 0, 0, d_timeslot_len - 1);

          // If correlation not found, break to the next timeslot
          if (idx_correlated < 0)
            break;

          // Otherwise, set the new delay, accounting for front padding
          // Note that due to changes in sync block functionality, we no longer need to accout for front_padding. This is legacy and should eventually be removed.
          // set_delay(idx_correlated - d_pad_front); //   <-- this does not work for some reason... check with Manav
          // std::cout << "\tCorrelated @ idx[" << idx_correlated << "]" << std::endl;
          d_delay_tx2rx = idx_correlated - d_pad_front;

          // The current state is now synced; break to the next timeslot
          currState = SystemState::SYNCED;
          // std::cout << "TIMESLOT [" << d_pkt_counter << "]\t:: SYNCED" << std::endl;
          break;
        }
        }

        // Break to the next timeslot
        d_pkt_counter++;
        break;
      }
      */

      // Route to output
      switch (currState)
      {
      case SystemState::SYNCED:
      {

        // std::cout << "Index [" << d_pad_front + d_delay_tx2rx - d_sig_pilot_pos * d_pilot_len - 1 << "] --> [" << d_pad_front + (d_frame_len-1) + d_delay_tx2rx - d_sig_pilot_pos * d_pilot_len - 1 << "]... " << std::endl;
        for (int idx = 0; idx < d_frame_len; idx++)
        {
          /*if (d_pad_front + idx + d_delay_tx2rx - (d_sig_pilot_pos * d_pilot_len) - 1 == d_timeslot_len) {
            std::cout << "\tidx = " << d_pad_front + idx + d_delay_tx2rx - (d_sig_pilot_pos * d_pilot_len) - 1 << std::endl;
            std::cout << "\tWraparound!!!" << std::endl;
            d_delay_tx2rx = d_timeslot_len - (d_pad_front + idx - (d_sig_pilot_pos * d_pilot_len) - 1);
            std::cout << "\tidx = " << d_pad_front + idx + d_delay_tx2rx - (d_sig_pilot_pos * d_pilot_len) - 1 << std::endl;
          }*/
          out_rx[idx] = rx_timeslot[(d_pad_front + idx + d_delay_tx2rx - (d_sig_pilot_pos * d_pilot_len)) % d_timeslot_len];
        }
        // std::cout << "Success!" << std::endl;
        break;
      }

      default:
      {
        for (int idx = 0; idx < d_frame_len; idx++)
          out_rx[idx] = 0;
        break;
      }

      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace fullduplex */
} /* namespace gr */
