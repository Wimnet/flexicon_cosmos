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

#ifndef INCLUDED_FULLDUPLEX_SYNC_RX_STREAM_IMPL_H
#define INCLUDED_FULLDUPLEX_SYNC_RX_STREAM_IMPL_H

#include <fullduplex/sync_rx_stream.h>
#include <queue>
#include <math.h>
#include <complex.h>
#include <numeric>
#include <Eigen/Dense>
#include <fftw3.h>
#include <sstream>
#include <fstream>

namespace gr
{
  namespace fullduplex
  {

    class sync_rx_stream_impl : public sync_rx_stream
    {
    private:
      // Load from external call
      bool d_debug;
      int d_pad_front;
      int d_frame_len;
      int d_timeslot_len;
      int d_symbol_len;
      int d_sig_pilot_pos;
      float d_thresh;
      std::ofstream outfile;
      const int delay = 1000000 * 2;

      // Typedef <--- TO BE REMOVED!!!!!
      // typedef std::complex<float> gr_complex;

      // Internal variables
      int d_delay_tx2rx;
      enum class SystemState
      {
        INIT,
        SYNCED,
        UNSYNCED,
        TUNE_CORR,
        SIGNAL_LOST,
        UNSYNCED_ENERGY,
        UNSYNCED_XCORR
      };
      SystemState currState;

      // Thread locking
      gr::thread::mutex d_mutex;

      // Packet counting
      unsigned int d_pkt_counter = 0;

      // Energy detection
      int energy_detection_idx = -1;
      int energy_detection(std::vector<gr_complex> &rx_timeslot, float threshold, int start_idx);

      // Prepare FFT functions
      int num_fft_pts = 256;
      fftw_complex *d_fft_in;
      fftw_complex *d_fft_out;
      fftw_plan d_fft_plan;
      fftw_plan d_ifft_plan;
      void load_fft_inputs(std::vector<gr_complex> in);
      void compute_fft(std::vector<gr_complex> in, std::vector<gr_complex> &out);
      void compute_ifft(std::vector<gr_complex> in, std::vector<gr_complex> &out);

      // Pilot symbols - important for cross-correlation
      float xcorr_thresh_max = 0.873109;
      float xcorr_thresh_frac = 0.5;
      int d_pilot_len;

      std::vector<gr_complex> d_sig_pilot_CP = {
          gr_complex(0.040406, 0.060609),
          gr_complex(-0.133884, -0.004820),
          gr_complex(-0.201962, 0.020896),
          gr_complex(0.143982, 0.031263),
          gr_complex(0.040620, 0.036383),
          gr_complex(0.052793, -0.005394),
          gr_complex(0.026307, 0.013965),
          gr_complex(-0.007893, -0.099068),
          gr_complex(0.025783, -0.048774),
          gr_complex(-0.039561, 0.024610),
          gr_complex(0.041165, 0.057548),
          gr_complex(0.004856, 0.035321),
          gr_complex(-0.004402, -0.082023),
          gr_complex(0.088905, 0.018569),
          gr_complex(-0.027376, 0.026346),
          gr_complex(-0.069822, 0.029407),
          gr_complex(0.064940, -0.001568),
          gr_complex(0.020500, 0.018805),
          gr_complex(-0.014237, 0.070996),
          gr_complex(-0.001405, 0.088381),
          gr_complex(0.008563, -0.057910),
          gr_complex(-0.000118, 0.030549),
          gr_complex(-0.024889, 0.006510),
          gr_complex(-0.012755, -0.018807),
          gr_complex(-0.003316, 0.014938),
          gr_complex(-0.051644, -0.045193),
          gr_complex(0.000706, 0.061681),
          gr_complex(0.014943, -0.014239),
          gr_complex(0.001189, 0.040681),
          gr_complex(0.006336, 0.040932),
          gr_complex(0.019915, 0.068689),
          gr_complex(0.093277, 0.038685),
          gr_complex(-0.004283, 0.011496),
          gr_complex(-0.057756, 0.033024),
          gr_complex(0.039470, -0.024211),
          gr_complex(-0.050999, -0.012346),
          gr_complex(-0.059725, 0.047809),
          gr_complex(0.050409, 0.023301),
          gr_complex(-0.011958, 0.020206),
          gr_complex(0.034603, -0.027731),
          gr_complex(0.031000, -0.087483),
          gr_complex(-0.030065, -0.041756),
          gr_complex(0.060306, -0.083687),
          gr_complex(0.104423, -0.039095),
          gr_complex(0.033828, 0.034169),
          gr_complex(0.064052, 0.005027),
          gr_complex(-0.011716, 0.016074),
          gr_complex(-0.094371, 0.013032),
          gr_complex(-0.049077, -0.036747),
          gr_complex(-0.047277, -0.056654),
          gr_complex(0.033899, 0.050394),
          gr_complex(-0.018856, 0.034317),
          gr_complex(-0.087672, -0.017493),
          gr_complex(-0.017110, 0.025498),
          gr_complex(0.004363, 0.057428),
          gr_complex(-0.078640, 0.048100),
          gr_complex(-0.045850, -0.065748),
          gr_complex(0.021961, 0.038584),
          gr_complex(-0.021962, 0.050281),
          gr_complex(0.000151, -0.062674),
          gr_complex(0.049271, 0.005338),
          gr_complex(0.012328, 0.025628),
          gr_complex(-0.048764, -0.007294),
          gr_complex(-0.031369, -0.010145),
          gr_complex(-0.130264, -0.202031),
          gr_complex(-0.028743, -0.038895),
          gr_complex(0.062022, -0.061130),
          gr_complex(-0.050846, 0.019249),
          gr_complex(0.061210, -0.014848),
          gr_complex(0.037897, -0.086170),
          gr_complex(0.043598, 0.016134),
          gr_complex(0.048883, -0.069215),
          gr_complex(0.004538, -0.000094),
          gr_complex(-0.042622, -0.050426),
          gr_complex(0.050437, 0.048491),
          gr_complex(-0.013121, -0.020034),
          gr_complex(-0.055236, 0.073672),
          gr_complex(0.091193, -0.071442),
          gr_complex(-0.035037, -0.006278),
          gr_complex(0.048797, 0.036206),
          gr_complex(0.056707, -0.010447),
          gr_complex(0.094041, 0.007664),
          gr_complex(-0.022433, 0.015645),
          gr_complex(0.077089, 0.069136),
          gr_complex(0.094844, 0.058516),
          gr_complex(-0.021084, -0.007160),
          gr_complex(0.016825, -0.016351),
          gr_complex(-0.111656, 0.070613),
          gr_complex(0.058866, -0.004409),
          gr_complex(-0.018961, -0.013593),
          gr_complex(-0.008811, -0.095763),
          gr_complex(0.000615, -0.038290),
          gr_complex(-0.009615, -0.027485),
          gr_complex(-0.001512, -0.000066),
          gr_complex(-0.027170, -0.055685),
          gr_complex(0.048042, 0.004387),
          gr_complex(-0.024965, 0.017075),
          gr_complex(-0.063603, 0.051473),
          gr_complex(0.029463, 0.021157),
          gr_complex(0.041957, -0.020252),
          gr_complex(-0.006771, -0.021998),
          gr_complex(0.001506, -0.025735),
          gr_complex(-0.120350, -0.069415),
          gr_complex(-0.057970, 0.068349),
          gr_complex(0.024356, 0.031926),
          gr_complex(-0.082850, -0.112458),
          gr_complex(-0.058152, 0.027099),
          gr_complex(-0.058055, 0.053410),
          gr_complex(-0.029366, -0.018271),
          gr_complex(0.082018, -0.022356),
          gr_complex(0.051042, 0.008649),
          gr_complex(0.070567, -0.023733),
          gr_complex(-0.039593, 0.029115),
          gr_complex(-0.036036, -0.051537),
          gr_complex(-0.041044, 0.056777),
          gr_complex(-0.068268, 0.100477),
          gr_complex(0.017798, 0.081437),
          gr_complex(0.116592, -0.029581),
          gr_complex(0.079385, 0.026213),
          gr_complex(-0.028811, 0.042502),
          gr_complex(-0.011590, 0.101784),
          gr_complex(-0.074467, -0.023819),
          gr_complex(0.034481, 0.007164),
          gr_complex(-0.014139, -0.078225),
          gr_complex(0.043685, 0.038386),
          gr_complex(-0.000088, 0.119134),
          gr_complex(-0.027655, -0.226222),
          gr_complex(-0.024021, -0.031702)
      };
      int len_comp_sig = d_sig_pilot_CP.size();
      Eigen::VectorXcf pilot_vec = Eigen::Map<Eigen::VectorXcf, Eigen::Unaligned>(d_sig_pilot_CP.data(), len_comp_sig);

      std::vector<gr_complex> pilot_vec_flipped_padded = std::vector<gr_complex>(num_fft_pts);
      std::vector<gr_complex> comp_sig_FFT = std::vector<gr_complex>(num_fft_pts);
      Eigen::VectorXcf comp_sig_FFT_vec;

      // Cross-correlation
      int xcorr_idx = -1;
      int check_idx = -1;
      int glob_xcorr_start_idx = 0;
      int xcorr_test_idx = 0;
      int xcorr(std::vector<gr_complex> &rx_timeslot, int start_idx, int num_delays_prev, int num_delays_next);
      int fast_xcorr(std::vector<gr_complex> &rx_timeslot, int start_idx, int num_delays_prev, int num_delays_next);

    public:
      // Constructor and destructor
      sync_rx_stream_impl(
          bool debug,
          unsigned int pad_front_len,
          unsigned int frame_len,
          unsigned int timeslot_len,
          unsigned int symbol_len,
          unsigned int sig_pilot_pos,
          float thresh);
      ~sync_rx_stream_impl();

      // Set delay
      void set_delay(int delay_tx2rx);

      // Where all the action really happens
      int work(int noutput_items,
               gr_vector_int &ninput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_SYNC_RX_STREAM_IMPL_H */
