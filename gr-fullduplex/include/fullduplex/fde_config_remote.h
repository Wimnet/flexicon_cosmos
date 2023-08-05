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

#ifndef INCLUDED_FULLDUPLEX_FDE_CONFIG_REMOTE_H
#define INCLUDED_FULLDUPLEX_FDE_CONFIG_REMOTE_H

#include <fullduplex/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace fullduplex {

    /*!
     * \brief <+description of block+>
     * \ingroup fullduplex
     *
     */
    class FULLDUPLEX_API fde_config_remote : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<fde_config_remote> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of fullduplex::fde_config_remote.
       *
       * To avoid accidental use of raw pointers, fullduplex::fde_config_remote's
       * constructor is in a private implementation
       * class. fullduplex::fde_config_remote::make is the public interface for
       * creating new instances.
       */
      static sptr make(bool debug, char* init_sn);

      virtual void set_all(int att_0_val, int att_1_val, int att_2_val,
                           int cf_cap_0_val, int cf_cap_1_val, int qf_cap_0_val, int qf_cap_1_val,
                           int dac_0_val, int dac_1_val, int dac_2_val, int canc_path_val,
                           int tuner_cap_0_val, int tuner_cap_1_val, int tuner_cap_2_val) = 0;
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_FDE_CONFIG_REMOTE_H */

