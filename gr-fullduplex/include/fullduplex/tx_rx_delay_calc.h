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

#ifndef INCLUDED_FULLDUPLEX_TX_RX_DELAY_CALC_H
#define INCLUDED_FULLDUPLEX_TX_RX_DELAY_CALC_H

#include <fullduplex/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
  namespace fullduplex {

    /*!
     * \brief <+description of block+>
     * \ingroup fullduplex
     *
     */
    class FULLDUPLEX_API tx_rx_delay_calc : virtual public gr::tagged_stream_block
    {
     public:
      typedef boost::shared_ptr<tx_rx_delay_calc> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of fullduplex::tx_rx_delay_calc.
       *
       * To avoid accidental use of raw pointers, fullduplex::tx_rx_delay_calc's
       * constructor is in a private implementation
       * class. fullduplex::tx_rx_delay_calc::make is the public interface for
       * creating new instances.
       */
      static sptr make(bool debug, uint64_t radio, float measure_delay, float startup_delay, uint64_t pad_front, uint64_t pad_tail, uint64_t frame_len, uint64_t pilot_pos, uint64_t algorithm, bool multi_usrp);
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_TX_RX_DELAY_CALC_H */

