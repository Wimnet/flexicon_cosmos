/* -*- c++ -*- */
/*
 * Copyright 2021 gr-fullduplex author.
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

#ifndef INCLUDED_FULLDUPLEX_CARRIER_SENSE_H
#define INCLUDED_FULLDUPLEX_CARRIER_SENSE_H

#include <fullduplex/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace fullduplex {

    /*!
     * \brief <+description of block+>
     * \ingroup fullduplex
     *
     */
    class FULLDUPLEX_API carrier_sense : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<carrier_sense> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of fullduplex::carrier_sense.
       *
       * To avoid accidental use of raw pointers, fullduplex::carrier_sense's
       * constructor is in a private implementation
       * class. fullduplex::carrier_sense::make is the public interface for
       * creating new instances.
       */
      static sptr make(bool debug, float threshold);
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_CARRIER_SENSE_H */

