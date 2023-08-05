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
#include <volk/volk.h>
#include "add_padding_impl.h"

namespace gr {
  namespace fullduplex {

    add_padding::sptr
    add_padding::make(bool debug, unsigned int pad_front, unsigned int pad_tail)
    {
      return gnuradio::get_initial_sptr
        (new add_padding_impl(debug, pad_front, pad_tail));
    }

    /*
     * The private constructor
     */
    add_padding_impl::add_padding_impl(bool debug, unsigned int pad_front, unsigned int pad_tail)
      : gr::tagged_stream_block("add_padding",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)),"packet_len"),
      d_debug(debug),
      d_pad_front(pad_front),
      d_pad_tail(pad_tail)
    {
      set_tag_propagation_policy(block::TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    add_padding_impl::~add_padding_impl()
    {
    }

    void
    add_padding_impl::set_premultiplier(float premultiplier)
    {
      gr::thread::scoped_lock lock(d_mutex);
      // // undo old premultiplier
      // for (int idx_sig_pilot = 0; idx_sig_pilot<80*2; idx_sig_pilot++) {
      //   d_sig_pilot[idx_sig_pilot] /= d_premultiplier;
      // }
      // d_premultiplier = premultiplier;
      // // apply new premultiplier
      // for (int idx_sig_pilot = 0; idx_sig_pilot<80*2; idx_sig_pilot++) {
      //   d_sig_pilot[idx_sig_pilot] *= d_premultiplier;
      // }
    }

    int add_padding_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      return ninput_items[0] + d_pad_front + d_pad_tail;
    }

    int
    add_padding_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex*)input_items[0];
      gr_complex *out = (gr_complex*)output_items[0];

      int ninput = ninput_items[0];
      int noutput = noutput_items;

      std::memset(out, 0x00, sizeof(gr_complex)*(ninput_items[0] + d_pad_front + d_pad_tail));
      //std::memcpy(out + d_pad_front + d_sig_pilot_pos*d_sig_pilot_len, d_sig_pilot, sizeof(gr_complex)*d_sig_pilot_len);
      std::memcpy(out + d_pad_front, in, sizeof(gr_complex)*ninput_items[0]);

      // std::cout << "Pad position: " << d_sig_pilot_pos*d_sig_pilot_len << std::endl;

      int produced = ninput_items[0] + d_pad_front + d_pad_tail;
      const pmt::pmt_t src = pmt::string_to_symbol(alias());

      std::vector<gr::tag_t> tags;
      get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + ninput_items[0]);
      for (size_t i = 0; i < tags.size(); i++) {
        add_item_tag(0, nitems_written(0),
        tags[i].key,
        tags[i].value);
      }

      return produced;
    }

  } /* namespace fullduplex */
} /* namespace gr */

