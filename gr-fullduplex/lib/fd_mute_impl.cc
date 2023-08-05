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
#include "fd_mute_impl.h"

namespace gr {
  namespace fullduplex {

    fd_mute::sptr
    fd_mute::make(bool debug, uint64_t radio, bool initial_state)
    {
      return gnuradio::get_initial_sptr
        (new fd_mute_impl(debug, radio, initial_state));
    }


    /*
     * The private constructor
     */
    fd_mute_impl::fd_mute_impl(bool debug, uint64_t radio, bool initial_state)
      : gr::tagged_stream_block("fd_mute",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)), "packet_len"),
        d_debug(debug),
        d_radio(radio),
        d_muted(initial_state),
        d_muted_ctrl_in(pmt::mp("muted_ctrl"))
    {
      message_port_register_in(d_muted_ctrl_in);
      set_msg_handler(d_muted_ctrl_in, boost::bind(&fd_mute_impl::control_handle, this, _1));
    }

    /*
     * Our virtual destructor.
     */
    fd_mute_impl::~fd_mute_impl()
    {
    }

    void fd_mute_impl::control_handle(pmt::pmt_t msg)
    {
      pmt::pmt_t msgpair = pmt::cdr(msg);
      uint64_t radio = uint64_t(pmt::to_long(pmt::cdr(msgpair)));
      if (radio == d_radio)
      {
        d_muted = bool(pmt::to_bool(pmt::car(msgpair)));
      }
    }

    int
    fd_mute_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int noutput_items = ninput_items[0];
      return noutput_items ;
    }

    int
    fd_mute_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      // const gr_complex* in = (const gr_complex*) input_items[0];
      gr_complex* in = (gr_complex*) input_items[0];
      gr_complex* out = (gr_complex*) output_items[0];

      noutput_items = ninput_items[0];

      if (d_muted)
      {
        volk_32fc_s32fc_multiply_32fc(out, in, 0.0, noutput_items);
        //for (uint64_t idx = 0; idx < noutput_items; idx++)
        //{
        //  out[idx] = in[idx];
        //}
      }
      else
      {
        volk_32fc_s32fc_multiply_32fc(out, in, 1.0, noutput_items);
        //for (uint64_t idx = 0; idx < noutput_items; idx++)
        //{
        //  out[idx] = in[idx];
        //}
        
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace fullduplex */
} /* namespace gr */

