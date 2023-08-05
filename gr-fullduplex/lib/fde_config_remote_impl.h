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

#ifndef INCLUDED_FULLDUPLEX_FDE_CONFIG_REMOTE_IMPL_H
#define INCLUDED_FULLDUPLEX_FDE_CONFIG_REMOTE_IMPL_H

#include <fullduplex/fde_config_remote.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <sstream>

namespace gr {
  namespace fullduplex {

    class fde_config_remote_impl : public fde_config_remote
    {
     private:
      // Nothing to declare in this block.
      int d_socket_descriptor;
      struct sockaddr_in d_server_addr;
      char d_recv_message[128]; // not used yet
      int d_server_addr_length;

      bool d_debug;

     public:
      fde_config_remote_impl(bool debug, char* init_sn);
      ~fde_config_remote_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      void set_all(int att_0_val, int att_1_val, int att_2_val,
                   int cf_cap_0_val, int cf_cap_1_val, int qf_cap_0_val, int qf_cap_1_val,
                   int dac_0_val, int dac_1_val, int dac_2_val, int canc_path_val,
                   int tuner_cap_0_val, int tuner_cap_1_val, int tuner_cap_2_val);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_FDE_CONFIG_REMOTE_IMPL_H */

