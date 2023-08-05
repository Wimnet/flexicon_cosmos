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


#ifndef INCLUDED_FULLDUPLEX_ADD_PADDING_H
#define INCLUDED_FULLDUPLEX_ADD_PADDING_H

#include <fullduplex/api.h>
#include <gnuradio/tagged_stream_block.h>

namespace gr {
  namespace fullduplex {

    /*!
     * \brief <+description of block+>
     * \ingroup fullduplex
     *
     */
    class FULLDUPLEX_API add_padding : virtual public gr::tagged_stream_block
    {
     public:
      typedef boost::shared_ptr<add_padding> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of fullduplex::add_Padding.
       *
       * To avoid accidental use of raw pointers, fullduplex::add_Padding's
       * constructor is in a private implementation
       * class. fullduplex::add_Padding::make is the public interface for
       * creating new instances.
       */
      static sptr make(bool debug, unsigned int pad_front, unsigned int pad_tail);
      protected:
      gr::thread::mutex d_mutex;
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_ADD_PADDING_H */

