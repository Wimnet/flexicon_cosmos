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

#ifndef INCLUDED_FULLDUPLEX_FD_MUTE_IMPL_H
#define INCLUDED_FULLDUPLEX_FD_MUTE_IMPL_H

#include <fullduplex/fd_mute.h>
#include <volk/volk.h>
#include <iostream>

namespace gr {
  namespace fullduplex {

    class fd_mute_impl : public fd_mute
    {
     private:
      // Nothing to declare in this block.
      bool d_debug;
      bool d_muted;
      uint64_t d_radio;
      pmt::pmt_t d_muted_ctrl_in;

     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);
      void control_handle(pmt::pmt_t msg);

     public:
      fd_mute_impl(bool debug, uint64_t radio, bool initial_state);
      ~fd_mute_impl();

      // Where all the action really happens
      int work(
              int noutput_items,
              gr_vector_int &ninput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_FD_MUTE_IMPL_H */

