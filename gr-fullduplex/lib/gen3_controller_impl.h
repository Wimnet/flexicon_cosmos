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

#ifndef INCLUDED_FULLDUPLEX_GEN3_CONTROLLER_IMPL_H
#define INCLUDED_FULLDUPLEX_GEN3_CONTROLLER_IMPL_H

#include <fullduplex/gen3_controller.h>
#include <iostream>
#include <string>
#include <sstream>
#include <bits/stdc++.h>

namespace gr {
  namespace fullduplex {

    class gen3_controller_impl : public gen3_controller
    {
     private:
      // Nothing to declare in this block.
      bool d_debug;
      uint64_t d_active_subcarriers;

      // History block
      //  0:    Initial primary
      //  Odd:  Plus channel
      //  Even: Minus channel
      static const int numGD = 50; // Number of gradient descent steps we need
      gr_complex channel_history[1 + 2 + 2*numGD][242];
      int16_t config_history[1 + 2 + 2*numGD][24];
      int history_ticker;
      int primary_channel_ticker;
      int best_idx;

      int pckt_counter;


      // channel estimate arrays and flags
      int16_t curr_Channel_Est_Real[242];
      int16_t curr_Channel_Est_Imag[242];
      bool d_fresh_estimate;
      bool d_ready;
      int FPGA_operation;
      int channelType;

      std::ofstream outfile_channel;
      std::ofstream outfile_config;

      // serial port connection
      int serial_port;

      pmt::pmt_t d_input_port_name;
      pmt::pmt_t d_channel_est_request_port_name;

      void debug_out(std::string msg);
      int setup_pc_connection();
      void initialize_canceller(int FPGA_hyperparameter_flag);
      void on_channel_estimate(pmt::pmt_t msg);\
      const int delay = 2000000;   // microseconds ==> 2sec

      // Frame structures
      static const int TX_FRAME_LEN = 2*242+24+1;
      static const int TX_FRAME_LEN_BYTES = TX_FRAME_LEN*2;
      static const int RX_FRAME_LEN = 24;
      static const int RX_FRAME_LEN_BYTES = RX_FRAME_LEN*2;
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
        const int offset = 2*242+1;
        
        // Iterate through config values and fill in the data_int array
        for (int ticker = 0; ticker < 24; ticker++) {
        data_int[ticker + offset] = ConfigValues[ticker];
        }
      }

      int select_next_primary(int index) {
        float sum_plus=0;
        float sum_minus=0;
        for(int i=0;i<242;i++) {
          //sum_plus  += pow(abs(channel_history[index][i]), 2);
          //sum_minus += pow(abs(channel_history[index+1][i]), 2);
          sum_plus += pow(channel_history[index][i].real(), 2) + pow(channel_history[index][i].imag(), 2);
          sum_minus += pow(channel_history[index+1][i].real(), 2) + pow(channel_history[index+1][i].imag(), 2);
        }
        float mag_dB_plus = 10*log10(sum_plus/242);
        float mag_dB_minus = 10*log10(sum_minus/242);
        std::cout<<"Magnitude of Plus channel (dB)"<<mag_dB_plus<< "at index"<<index<<"\n";
        std::cout<<"Magnitude of Minus channel (dB)"<<mag_dB_minus<<"at index"<<index+1<<"\n";

        int temp = (mag_dB_plus <= mag_dB_minus) ? index : index+1;
        std::cout<<"Selected channel index under select_next_primary"<<temp<< std::endl;
        return temp;
      }

      int select_best_run() {
        float sum=0;
        float avg_dB;
        float min = 0;
        int min_idx =0;
        for(int ticker = 0; ticker < (1 + 2 + 2*numGD); ticker++) {
          sum = 0;
          for(int counter = 0; counter < 242; counter++) {
            //sum += pow(abs(channel_history[ticker][counter]), 2);
            sum += pow(channel_history[ticker][counter].real(), 2) + pow(channel_history[ticker][counter].imag(), 2);
          }
          avg_dB = 10*log10(sum/242);
          
          if(ticker == 0) {
            min = avg_dB;
            min_idx = 0;
            std::cout<<"Circulator cancellation: " << min << std::endl;
            continue;
          }
          std::cout<<"Round " << ticker << " cancellation: " << avg_dB << std::endl;
          if(avg_dB <= min){
              min = avg_dB;
              min_idx = ticker;
          }
        }

        return min_idx;
      }

     public:
      gen3_controller_impl(bool debug, uint64_t occupied_carriers, uint64_t pilot_carriers);
      ~gen3_controller_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_GEN3_CONTROLLER_IMPL_H */

