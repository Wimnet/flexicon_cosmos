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

#ifndef INCLUDED_FULLDUPLEX_RADIO_SYNC_H
#define INCLUDED_FULLDUPLEX_RADIO_SYNC_H

#include <fullduplex/api.h>
#include <gnuradio/tagged_stream_block.h>
#include <vector>

namespace gr {
  namespace fullduplex {

    /*!
     * \brief <+description of block+>
     * \ingroup fullduplex
     *
     */
    class FULLDUPLEX_API radio_sync : virtual public gr::tagged_stream_block
    {
     public:
      typedef boost::shared_ptr<radio_sync> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of fullduplex::radio_sync.
       *
       * To avoid accidental use of raw pointers, fullduplex::radio_sync's
       * constructor is in a private implementation
       * class. fullduplex::radio_sync::make is the public interface for
       * creating new instances.
       */
      static sptr make(bool debug, float startup_delay, uint64_t num_radios, float radio_delay, float exp_start_delay, uint64_t pad_front, uint64_t pad_tail, uint64_t frame_len, std::vector<int> pilot_pos);
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_RADIO_SYNC_H */

