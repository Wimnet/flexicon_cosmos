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

#ifndef INCLUDED_FULLDUPLEX_GEN3_TAP_MEASUREMENTS_IMPL_H
#define INCLUDED_FULLDUPLEX_GEN3_TAP_MEASUREMENTS_IMPL_H

#include <fullduplex/gen3_tap_measurements.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

namespace gr {
  namespace fullduplex {

    class gen3_tap_measurements_impl : public gen3_tap_measurements
    {
     private:
      bool d_debug;
      uint64_t d_active_subcarriers;

      // channel estimate arrays and flags
      int16_t* Channel_Est_Real;
      int16_t* Channel_Est_Imag;
      bool d_fresh_estimate;
      bool d_ready;
      
      // Channel estimate writing tools
      int currAddress;
      int currCount;
      const int maxCount = 100;   // Maximum number of measurements per (tap, delay)  
      const int delay = 1000000;    // 1000000 microseconds = 1.0 second
      std::ofstream outfile;
      
      // serial port connection
      int serial_port;

      pmt::pmt_t d_input_port_name;
      pmt::pmt_t d_channel_est_request_port_name;

      void debug_out(std::string msg);
      int setup_FPGA_connection();
      void initialize_canceller(int serial_port);
      void on_channel_estimate(pmt::pmt_t msg);
      
      // Frame sizes
      static const int TX_FRAME_LEN = 2*242+24+1;
      static const int TX_FRAME_LEN_BYTES = TX_FRAME_LEN*2;
      static const int RX_FRAME_LEN = 24;
      static const int RX_FRAME_LEN_BYTES = RX_FRAME_LEN*2;

      // Frame structures
      union Frame_to_FPGA {
        int16_t data_int[TX_FRAME_LEN];
        char  data_char[TX_FRAME_LEN_BYTES];
      };
      union Frame_from_FPGA {
        int16_t data_int[RX_FRAME_LEN];
        char  data_char[RX_FRAME_LEN_BYTES];
      };
  
      // Write Flag
        //  0 --> Do nothing
        //  1 --> Run OMP
        //  2 --> Run GD
        //  3 --> Configure IC
      void write_Flag(int16_t data_int[TX_FRAME_LEN], int16_t flag)
      {
        data_int[0] = flag;
      }

      // Write Channel Estimate
      void write_ChanEst(int16_t data_int[TX_FRAME_LEN], int16_t ChanEst_Real[242], int16_t ChanEst_Imag[242])
      {
        // Offset from beginning
        const int offset = 1;
        
        // Iterate through channel estimate and fill in the data_int array
        for (int ticker = 0; ticker < 242; ticker++) {
        data_int[2*ticker + offset + 0] = ChanEst_Real[ticker];
        data_int[2*ticker + offset + 1] = ChanEst_Imag[ticker];
        }
      }

      // Write Config Values
      void write_ConfigValues(int16_t data_int[TX_FRAME_LEN], int16_t ConfigValues[24])
      {
        // Offset from beginning
        const int offset = 485; //2*242+1;
        
        // Iterate through config values and fill in the data_int array
        for (int ticker = 0; ticker < 24; ticker++) {
        data_int[ticker + offset] = ConfigValues[ticker];
        }
      }

     public:
      gen3_tap_measurements_impl(bool debug);
      ~gen3_tap_measurements_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_GEN3_TAP_MEASUREMENTS_IMPL_H */

