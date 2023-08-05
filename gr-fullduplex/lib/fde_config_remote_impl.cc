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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "fde_config_remote_impl.h"


namespace gr {
  namespace fullduplex {

    fde_config_remote::sptr
    fde_config_remote::make(bool debug, char* init_sn)
    {
      return gnuradio::get_initial_sptr
        (new fde_config_remote_impl(debug, init_sn));
    }


    /*
     * The private constructor
     */
    fde_config_remote_impl::fde_config_remote_impl(bool debug, char* init_sn)
      : gr::block("fde_config_remote",
              gr::io_signature::make(0, 0, sizeof(int)),
              gr::io_signature::make(0, 0, sizeof(int))),
        d_debug(debug)
    {
      d_server_addr_length = sizeof(d_server_addr);

      memset(d_recv_message, '\0', sizeof(d_recv_message));

      d_socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

      if (d_socket_descriptor < 0) 
      {
        std::cout << "Could not create socket!" << std::endl;
      }
      std::cout << "Created socket" << std::endl;

      d_server_addr.sin_family = AF_INET;
      d_server_addr.sin_port = htons(31415);
      d_server_addr.sin_addr.s_addr = inet_addr("10.116.7.3");

      std::stringstream ss;
      ss << "+" << init_sn;
      const char* msg = ss.str().c_str();

      int send_rv = 0;
      send_rv = sendto(d_socket_descriptor, msg, ss.str().length(), 0, (struct sockaddr*)&d_server_addr, d_server_addr_length);
      if (send_rv < 0)
      {
        std::cout << "Could not send message!" << std::endl;
      }
    }

    /*
     * Our virtual destructor.
     */
    fde_config_remote_impl::~fde_config_remote_impl()
    {
      close(d_socket_descriptor);
    }

    void
    fde_config_remote_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = noutput_items;
    }

    int
    fde_config_remote_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      int *in = (int *) input_items[0];
      int *out = (int *) output_items[0];

      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      // consume_each (noutput_items);
      for (int i = 0; i < noutput_items; i++) {
          out[i] = in[i] * 2;
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    void
    fde_config_remote_impl::set_all(int att_0_val, int att_1_val, int att_2_val,
                             int cf_cap_0_val, int cf_cap_1_val, int qf_cap_0_val, int qf_cap_1_val,
                             int dac_0_val, int dac_1_val, int dac_2_val, int canc_path_val,
                             int tuner_cap_0_val, int tuner_cap_1_val, int tuner_cap_2_val) {

      std::stringstream ss;
      ss << "-" << tuner_cap_0_val << ";";
      ss << tuner_cap_1_val << ";";
      ss << tuner_cap_2_val << ";";
      ss << dac_0_val << ";";
      ss << dac_1_val << ";";
      ss << dac_2_val << ";";
      ss << cf_cap_0_val << ";";
      ss << cf_cap_1_val << ";";
      ss << qf_cap_0_val << ";";
      ss << qf_cap_1_val << ";";
      ss << att_0_val << ";";
      ss << att_1_val << ";";
      ss << att_2_val << ";";
      ss << canc_path_val;
      const char* msg = ss.str().c_str();

      int send_rv = 0;
      send_rv = sendto(d_socket_descriptor, msg, ss.str().length(), 0, (struct sockaddr*)&d_server_addr, d_server_addr_length);
      if (send_rv < 0)
      {
        std::cout << "Could not send message!" << std::endl;
      }
      else
      {
        if (d_debug)
        {
          std::cout << "Sending " << ss.str() << std::endl;
        }
      }
    }

  } /* namespace fullduplex */
} /* namespace gr */

