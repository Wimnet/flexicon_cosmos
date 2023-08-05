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
#include "digital_sic_135_impl.h"

namespace gr {
  namespace fullduplex {

    digital_sic_135::sptr
    digital_sic_135::make(bool debug,
                            unsigned int delay_tx2rx,
                            unsigned int pad_front,
                            unsigned int sig_pilot_pos,
                            unsigned int frame_len,
                            unsigned int si_chan_k,
                            unsigned int si_chan_dim,
                            double premultiplier)
    {
      return gnuradio::get_initial_sptr
        (new digital_sic_135_impl(debug, delay_tx2rx, pad_front, sig_pilot_pos, frame_len, si_chan_k, si_chan_dim, premultiplier));
    }


    /*
     * The private constructor
     */
    digital_sic_135_impl::digital_sic_135_impl(bool debug,
                                       unsigned int delay_tx2rx,
                                       unsigned int pad_front,
                                       unsigned int sig_pilot_pos,
                                       unsigned int frame_len,
                                       unsigned int si_chan_k,
                                       unsigned int si_chan_dim,
                                       double premultiplier)
      : gr::tagged_stream_block("digital_sic_135",
              gr::io_signature::make2(2, 2, sizeof(gr_complex), sizeof(gr_complex)),
              gr::io_signature::make3(3, 3, sizeof(gr_complex), sizeof(gr_complex), sizeof(gr_complex)),
              "packet_len"),
              d_debug(debug),
              d_delay_tx2rx(delay_tx2rx),
              d_pad_front(pad_front),
              d_sig_pilot_pos(sig_pilot_pos),
              d_frame_len(frame_len),
              d_si_chan_k(si_chan_k),
              d_si_chan_dim(si_chan_dim),
              d_sig_pilot_scale(0.75),
              d_premultiplier(premultiplier),
              d_initial(true),
              d_delay_msg_in(pmt::mp("delay_in")) // MK // MK
    {
      // set_tag_propagation_policy(block::TPP_DONT);

      set_delay_tx2rx(delay_tx2rx);
      message_port_register_in(d_delay_msg_in);
      set_msg_handler(d_delay_msg_in, boost::bind(&digital_sic_135_impl::delay_handle, this, _1));
    }

    /*
     * Our virtual destructor.
     */
    digital_sic_135_impl::~digital_sic_135_impl()
    {
    }
    void
    digital_sic_135_impl::set_delay_tx2rx(int delay_tx2rx)
    {
      gr::thread::scoped_lock lock(d_mutex);
      d_delay_tx2rx = delay_tx2rx;
    }
  /*
    void
    digital_sic_135_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
       <+forecast+> e.g. ninput_items_required[0] = noutput_items 
    }
    

    int
    digital_sic_135_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    */
    
    void
    digital_sic_135_impl::delay_handle(pmt::pmt_t msg)
    {
      d_delay_tx2rx = int(pmt::to_long(pmt::cdr(msg))) + d_si_chan_k + 1;
    }

    int 
    digital_sic_135_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in_tx = (const gr_complex *) input_items[0];
      const gr_complex *in_rx = (const gr_complex *) input_items[1];

      gr_complex *out_rx = (gr_complex *) output_items[0];
      gr_complex *out_rx_res = (gr_complex *) output_items[1];

      /*
      gr_complex *out_rx = (gr_complex *) output_items[0];
      gr_complex *out_rx_res = (gr_complex *) output_items[1];
      */
      gr_complex *h = (gr_complex *) output_items[2];

      noutput_items = d_frame_len; // ninput_items[0];

      std::memset(out_rx, 0x00, sizeof(gr_complex) * noutput_items);
      std::memset(out_rx_res, 0x00, sizeof(gr_complex) * noutput_items);
      std::memset(h, 0x00, sizeof(gr_complex) * (2*d_si_chan_k+1));

      // init Eigen vectors
      Eigen::VectorXcf pilot_tx(80*2);
      Eigen::VectorXcf pilot_rx(80*2);
      Eigen::VectorXcf tx_sig_buff(d_frame_len + 2*d_si_chan_k + 1);
      Eigen::VectorXcf rx_sig_buff(d_frame_len + 2*d_si_chan_k + 1);

      for (int idx = 0; idx < 80*2; idx++) {
        pilot_tx(idx) = in_tx[d_pad_front+d_sig_pilot_pos*80*2 + idx];
        pilot_rx(idx) = in_rx[d_pad_front+d_sig_pilot_pos*80*2+d_delay_tx2rx + idx];
      }

      for (int idx = 0; idx < d_frame_len; idx++) {
        // out_tx[idx] = in_tx[d_pad_front+idx];
        out_rx[idx] = in_rx[d_pad_front+d_delay_tx2rx+idx];
      }

      for (int idx = 0; idx < d_frame_len + 2*d_si_chan_dim + 1; idx++) {
        tx_sig_buff(idx) = in_tx[d_pad_front-d_si_chan_k+idx];
        rx_sig_buff(idx) = in_rx[d_pad_front-d_si_chan_k+d_delay_tx2rx+idx];
      }
      // memcpy(pilot, in, sizeof(gr_complex) * 80)
      // cout << "Vector element type is " << typeid(pilot(1)).name() << endl;

      // Eigen::MatrixXcf pilot_toeplitz;
      // pilot_toeplitz = sig_toeplitz(pilot, 40, 10, 1);
      Eigen::VectorXcf hh( (2*d_si_chan_k+1) * d_si_chan_dim );
      Eigen::VectorXcf h3( (2*d_si_chan_k+1) * d_si_chan_dim );
      Eigen::VectorXcf h5( (2*d_si_chan_k+1) * d_si_chan_dim );
      Eigen::VectorXf hh_mag( (2*d_si_chan_k+1) * d_si_chan_dim );
      float hh_mag_max;

      // Eigen::VectorXcf si_chnl_est(Eigen::VectorXcf &tx_sig, Eigen::VectorXcf &rx_sig, int l, int k)
      // if (d_initial) { // MK
      //   d_h = Eigen::VectorXcf( (2*d_si_chan_k+1) * d_si_chan_dim ); // MK
      //   d_h = si_chnl_est(pilot_tx, pilot_rx, 80*2-2*d_si_chan_k-1, d_si_chan_k, d_si_chan_dim); // MK
      //   d_initial = false; // MK
      // }
      // hh = d_h; // MK
    
    //LL here compute h filter for 1,3,5 orders
      hh = si_chnl_est(pilot_tx, pilot_rx, 80*2-2*d_si_chan_k-1, d_si_chan_k, d_si_chan_dim); 
      // linear estimation ended
      
      Eigen::VectorXcf pilot_rx_3(80*2);
      Eigen::MatrixXcf tx_pilot_toeplitz;
      tx_pilot_toeplitz = sig_toeplitz(pilot_tx, 80*2-2*d_si_chan_k-1, d_si_chan_k, d_si_chan_dim);
      pilot_rx_3 = pilot_rx - tx_pilot_toeplitz * hh;
      h3 = si_chnl_est_3(pilot_tx, pilot_rx_3, 80*2-2*d_si_chan_k-1, d_si_chan_k, d_si_chan_dim);
      // third order estimation ended
      /*
      Eigen::VectorXcf pilot_rx_5(80*2);
      Eigen::MatrixXcf tx_pilot_toeplitz_3;
      tx_pilot_toeplitz_3 = sig_toeplitz_3(pilot_tx, 80*2-2*d_si_chan_k-1, d_si_chan_k, d_si_chan_dim);
      pilot_rx_5 = pilot_rx - tx_pilot_toeplitz * hh - tx_pilot_toeplitz_3 * h3;
      h5 = si_chnl_est_5(pilot_tx,pilot_rx_5, 80*2-2*d_si_chan_k-1, d_si_chan_k, d_si_chan_dim);
      */ // comment out the fifth order filter
    // filter calculations end
      
      // LL what is this part doing? 
      for (int idx = 0; idx < (2*d_si_chan_k+1) * d_si_chan_dim; idx++) {
        h[idx] = h3(idx);
      }
      hh_mag = hh.cwiseAbs2();
      // std::cout << "Max |h| = " << hh_mag << std::endl;
      std::ptrdiff_t hh_mag_max_idx;
      hh_mag_max = hh_mag.maxCoeff(&hh_mag_max_idx);
      // std::cout << "Max |h| at " << hh_mag_max_idx << std::endl;

      // perform digital SIC with 3 filters
      Eigen::MatrixXcf tx_sig_toeplitz;
      tx_sig_toeplitz = sig_toeplitz(tx_sig_buff, d_frame_len, d_si_chan_k, d_si_chan_dim);
      Eigen::MatrixXcf tx_sig_toeplitz_3;
      tx_sig_toeplitz_3 = sig_toeplitz_3(tx_sig_buff, d_frame_len, d_si_chan_k, d_si_chan_dim);
      //Eigen::MatrixXcf tx_sig_toeplitz_5;
      //tx_sig_toeplitz_5 = sig_toeplitz_5(tx_sig_buff, d_frame_len, d_si_chan_k, d_si_chan_dim);
      Eigen::VectorXcf rx_sig_res;
      rx_sig_res = rx_sig_buff.segment(d_si_chan_k-1, d_frame_len) - tx_sig_toeplitz * hh - tx_sig_toeplitz_3 * h3;
      for (int idx = 0; idx < d_frame_len - 80*2; idx++) {
        out_rx_res[idx] = rx_sig_res(idx+d_si_chan_k-1);
      }


      /* estimate tx pilot
      Eigen::MatrixXcf pilot_tx_toeplitz;
      Eigen::VectorXcf pilot_rx_res;
      pilot_tx_toeplitz = sig_toeplitz(pilot_tx, 80-2*d_si_chan_k-1, d_si_chan_k, d_si_chan_dim);
      pilot_rx_res = pilot_rx.segment(d_si_chan_k-1, 80-2*d_si_chan_k-1) - pilot_tx_toeplitz * hh;
      for (int idx = 0; idx < 80-2*d_si_chan_k-1; idx++) {
        out_rx_pilot_est[idx+d_si_chan_k-1] = pilot_rx_res(idx);
      }
      */

      int produced = noutput_items; // d_frame_len;

      // Tell runtime system how many input items we consumed on
      // each input stream.
      // consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      // return noutput_items;
      return produced;

      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      // consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      // return noutput_items;
    }

  } /* namespace fullduplex */
} /* namespace gr */