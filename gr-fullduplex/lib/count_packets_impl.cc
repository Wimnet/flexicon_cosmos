/* -*- c++ -*- */
/* 
 * Copyright 2020 gr-fullduplex author.
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
#include "count_packets_impl.h"

namespace gr {
  namespace fullduplex {

    count_packets::sptr
    count_packets::make(bool debug)
    {
      return gnuradio::get_initial_sptr
        (new count_packets_impl(debug));
    }

    /*
     * The private constructor
     */
    count_packets_impl::count_packets_impl(bool debug)
      : gr::tagged_stream_block("count_packets",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0), "packet_len"),
              d_debug(debug),
              d_count(0),
              d_output_port(pmt::mp("out"))
              
    {
      message_port_register_out(d_output_port);
    }

    /*
     * Our virtual destructor.
     */
    count_packets_impl::~count_packets_impl()
    {
    }

    int
    count_packets_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      return 0;
    }

    int
    count_packets_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex* in = (const gr_complex*) input_items[0];
      gr_complex* out = (gr_complex*) output_items[0];

      noutput_items = calculate_output_stream_length(ninput_items);

      // Do <+signal processing+>
      d_count += 1;

      if (d_debug)
      {
        std::cout << d_count << std::endl;
      }

      // make message
      pmt::pmt_t count = pmt::from_long(d_count);
      message_port_pub(d_output_port, pmt::cons(pmt::PMT_NIL, count));

      return 0;
    }

  } /* namespace fullduplex */
} /* namespace gr */

