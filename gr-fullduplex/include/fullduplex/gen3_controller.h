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


#ifndef INCLUDED_FULLDUPLEX_GEN3_CONTROLLER_H
#define INCLUDED_FULLDUPLEX_GEN3_CONTROLLER_H

#include <fullduplex/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace fullduplex {

    /*!
     * \brief <+description of block+>
     * \ingroup fullduplex
     *
     */
    class FULLDUPLEX_API gen3_controller : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<gen3_controller> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of fullduplex::gen3_controller.
       *
       * To avoid accidental use of raw pointers, fullduplex::gen3_controller's
       * constructor is in a private implementation
       * class. fullduplex::gen3_controller::make is the public interface for
       * creating new instances.
       */
      static sptr make(bool debug, uint64_t occupied_carriers, uint64_t pilot_carriers);
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_GEN3_CONTROLLER_H */

