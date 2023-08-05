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

#ifndef INCLUDED_FULLDUPLEX_TDMA_PACKET_ENCAP_H
#define INCLUDED_FULLDUPLEX_TDMA_PACKET_ENCAP_H

#include <fullduplex/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
  namespace fullduplex {

    /*!
     * \brief <+description of block+>
     * \ingroup fullduplex
     *
     */
    class FULLDUPLEX_API tdma_packet_encap : virtual public gr::tagged_stream_block
    {
     public:
      typedef boost::shared_ptr<tdma_packet_encap> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of fullduplex::tdma_packet_encap.
       *
       * To avoid accidental use of raw pointers, fullduplex::tdma_packet_encap's
       * constructor is in a private implementation
       * class. fullduplex::tdma_packet_encap::make is the public interface for
       * creating new instances.
       */

      static sptr make(bool debug, uint64_t pad_front, uint64_t pad_tail, uint64_t sig_pilot_pos, double premultiplier, int sync_word, uint64_t radio);
    
      protected:
        gr_complex d_sig_pilot[2*80];

    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_TDMA_PACKET_ENCAP_H */

