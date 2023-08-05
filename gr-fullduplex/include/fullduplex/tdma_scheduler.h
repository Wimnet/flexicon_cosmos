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

#ifndef INCLUDED_FULLDUPLEX_TDMA_SCHEDULER_H
#define INCLUDED_FULLDUPLEX_TDMA_SCHEDULER_H

#include <fullduplex/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace fullduplex {

    /*!
     * \brief <+description of block+>
     * \ingroup fullduplex
     *
     */
    class FULLDUPLEX_API tdma_scheduler : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<tdma_scheduler> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of fullduplex::tdma_scheduler.
       *
       * To avoid accidental use of raw pointers, fullduplex::tdma_scheduler's
       * constructor is in a private implementation
       * class. fullduplex::tdma_scheduler::make is the public interface for
       * creating new instances.
       */
      static sptr make(bool debug, int num_radios, int num_fd_radios, bool fd_bs, bool iui_free, bool mode);
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_TDMA_SCHEDULER_H */

