/* -*- c++ -*- */
/*
 * Copyright 2019 <+YOU OR YOUR COMPANY+>.
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


#ifndef INCLUDED_FULLDUPLEX_DIGITAL_SIC_SINGLE_H
#define INCLUDED_FULLDUPLEX_DIGITAL_SIC_SINGLE_H

#include <fullduplex/api.h>
// #include <gnuradio/block.h>
// #include <gnuradio/sync_block.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
  namespace fullduplex {

    /*!
     * \brief <+description of block+>
     * \ingroup fullduplex
     *
     */
    class FULLDUPLEX_API digital_sic_single : virtual public gr::tagged_stream_block
    {
     public:
      typedef boost::shared_ptr<digital_sic_single> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of fullduplex::digital_sic_single.
       *
       * To avoid accidental use of raw pointers, fullduplex::digital_sic_single's
       * constructor is in a private implementation
       * class. fullduplex::digital_sic_single::make is the public interface for
       * creating new instances.
       */
      static sptr make(bool debug=false,
                       unsigned int delay_tx2rx=0,
                       unsigned int sig_pilot_pos=0,
                       unsigned int pad_front=0,
                       unsigned int frame_len=0,
                       unsigned int si_chan_k=0,
                       unsigned int si_chan_dim=1,
                       double premultiplier=1);

      virtual void set_delay_tx2rx(int delay_tx2rx) = 0;

      virtual void set_pad_front(int pad_front) = 0;

     protected:
       gr_complex d_sig_pilot[640];
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_DIGITAL_SIC_SINGLE_H */
