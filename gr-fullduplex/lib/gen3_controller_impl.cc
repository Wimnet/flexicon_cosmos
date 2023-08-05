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
#include "gen3_controller_impl.h"

#include <iostream>
#include <bits/stdc++.h>

// C library headers
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// headers in automatically generated hello_world.c
#include <stdlib.h>
#include <math.h>

namespace gr {
  namespace fullduplex {

    gen3_controller::sptr
    gen3_controller::make(bool debug, uint64_t occupied_carriers, uint64_t pilot_carriers)
    {
      return gnuradio::get_initial_sptr
        (new gen3_controller_impl(debug, occupied_carriers, pilot_carriers));
    }

    /*
     * The private constructor
     */
    gen3_controller_impl::gen3_controller_impl(bool debug, uint64_t occupied_carriers, uint64_t pilot_carriers)
      : gr::block("gen3_controller",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(gr_complex))),
    d_debug(debug),
    d_input_port_name(pmt::mp("est_msg_in")),
    d_channel_est_request_port_name(pmt::mp("channel_request_out")),
    d_active_subcarriers(occupied_carriers+pilot_carriers),
    d_fresh_estimate(false),
    FPGA_operation(1),
    history_ticker(0),
    primary_channel_ticker(0),
    best_idx(0),
    pckt_counter(0)
    {
      message_port_register_in(d_input_port_name);
      message_port_register_out(d_channel_est_request_port_name);

      //curr_Channel_Est_Real = (int16_t*)malloc(d_active_subcarriers*sizeof(int16_t));
      //curr_Channel_Est_Imag = (int16_t*)malloc(d_active_subcarriers*sizeof(int16_t));

      serial_port = setup_pc_connection();
      
      // only set up the message handler when we are done with the initial setup
      // this avoids issues where the channel estimator (which starts in an active state)
      // will send a message before it is time

      outfile_channel.open("/home/wimnet-gen3/Documents/Gen-3 Canceller/channel_History.csv");
      outfile_config.open("/home/wimnet-gen3/Documents/Gen-3 Canceller/config_History.csv");

      // Wait for a bit, and request a new channel estimate from the channel estimate block
      usleep(delay); 
      message_port_pub(d_channel_est_request_port_name, pmt::cons(pmt::PMT_NIL, pmt::PMT_T));
      set_msg_handler(d_input_port_name, boost::bind(&gen3_controller_impl::on_channel_estimate, this, _1));
      message_port_pub(d_channel_est_request_port_name, pmt::cons(pmt::PMT_NIL, pmt::PMT_T));
    }

    /*
     * Our virtual destructor.
     */
    gen3_controller_impl::~gen3_controller_impl()
    {
      // End session
      close(serial_port);
      outfile_channel.close();
      outfile_config.close();

      std::cout << EXIT_SUCCESS;
    }
    
    void
  gen3_controller_impl::debug_out(std::string msg)
  {
    if (d_debug)
    {
      std::cout << "[GEN-3 SYSTEM CONTROL] " << msg << std::endl;
    }
  }

  void
  gen3_controller_impl::on_channel_estimate(pmt::pmt_t msg)
  {
    if(pckt_counter < 30) {
      //std::cout << "Ensuring alignment " << pckt_counter << "..." << std::endl;
      pckt_counter++;
      message_port_pub(d_channel_est_request_port_name, pmt::cons(pmt::PMT_NIL, pmt::PMT_T));
      return;
    }

    // STEP 0: Initialize canceller
    if (best_idx > -1) {
      // If best_idx is not -1, then we have a termination and the previous run finished.
      std::cout << "###########################################" << std::endl;
      history_ticker = 0;
      primary_channel_ticker = 0;
      best_idx = -1;

      initialize_canceller(3);

      usleep(delay);
      message_port_pub(d_channel_est_request_port_name, pmt::cons(pmt::PMT_NIL, pmt::PMT_T));
      return;
    }

    // STEP 1: Measure new channel estimate
    const gr_complex* val = pmt::c32vector_elements(pmt::cdr(msg), d_active_subcarriers);
    std::cout << "ROUND " << history_ticker << ": Measured channel." << std::endl;
    
    // STEP 2: Record channel estimate
    for (uint64_t idx = 0; idx < d_active_subcarriers; idx++)
    {
      channel_history[history_ticker][idx] = val[idx];
      if (d_debug) 
      {
        //std::cout << "Re: " << curr_Channel_Est_Real[idx] << ", Im: " << curr_Channel_Est_Imag[idx] << std::endl;
        std::cout << "Re: " << channel_history[history_ticker][idx].real() << ", Im: " << channel_history[history_ticker][idx].imag() << std::endl; 
      }
      // Write to file
        outfile_channel << val[idx].real() << ',' << val[idx].imag() << ',';
    }
    outfile_channel << std::endl;
    d_fresh_estimate = true;
    history_ticker++;


    // STEP 2.5: Do we need to continue computing configuration?
    if (history_ticker == 1+2+2*numGD) {
       
      // Determine best run
      best_idx = select_best_run();
      std::cout << "The best run index is "<< best_idx <<std::endl;
      
      // Create frame, send to FPGA, wait for ACK, end
        
      for (uint64_t idx = 0; idx < d_active_subcarriers; idx++) {
       curr_Channel_Est_Real[idx] = int16_t(channel_history[best_idx][idx].real() * pow(2, 15));
       curr_Channel_Est_Imag[idx] = int16_t(channel_history[best_idx][idx].imag() * pow(2, 15));
      }

      // Record best index config values
      int16_t configValues[24] = {};
      std::cout << "Sending to FPGA: ";
      for (uint64_t idx = 0; idx < 24; idx++) {
        configValues[idx] = config_history[best_idx][idx];
        std::cout << configValues[idx] << ", ";
      }
      std::cout << std::endl;

      // Build the frame
      union Frame_to_FPGA send_to_FPGA;
      write_ChanEst(send_to_FPGA.data_int, curr_Channel_Est_Real, curr_Channel_Est_Imag);
      write_ConfigValues(send_to_FPGA.data_int, configValues);
      write_Flag(send_to_FPGA.data_int, 3);
      
      // Send to FPGA
      write(serial_port, &send_to_FPGA.data_char, TX_FRAME_LEN_BYTES);

      // Receive ACK from FPGA
      int received_bytes = 0;
      int total_received_bytes = 0;
      Frame_from_FPGA got_from_FPGA;
      while (total_received_bytes < RX_FRAME_LEN_BYTES) {
        received_bytes = read(serial_port, &got_from_FPGA.data_char[total_received_bytes], RX_FRAME_LEN_BYTES);
        total_received_bytes += received_bytes;
      }
      std::cout << "Received from FPGA: ";
      for (int ticker = 0; ticker < 24; ticker++) {
        config_history[history_ticker][ticker] = got_from_FPGA.data_int[ticker];
        std::cout << config_history[history_ticker][ticker] << ", ";
        
        outfile_config << config_history[history_ticker][ticker] << ','; 
      }
      outfile_config << std::endl;

      std::cout << std::endl;
    
      printf("Termination ACKed successfully!\n");
      usleep(30000000); // Wait 30 seconds, restart
      return;
    }

    // STEP 3: Send primary channel and config values to the FPGA
    // If we just measured a minus channel, choose next primary channel
    if (history_ticker % 2 == 1 && history_ticker >= 3) {
      primary_channel_ticker = select_next_primary(history_ticker-2);
      std::cout << "ROUND " << history_ticker << ": Selected PRIMARY CHANNEL "<< primary_channel_ticker << std::endl;
      
    }

    // If we recorded a plus (odd) channel, we are sending a minus channel.
    // If we recorded a minus (even) channel, we are sending a plus channel.
    int channelSign = 0;
    if (history_ticker % 2 == 0) {
      channelSign = 1;
      std::cout << "ROUND " << history_ticker << ": Sending MINUS CHANNEL to FPGA." << std::endl;
    }
    else {
      channelSign = -1;
      std::cout << "ROUND " << history_ticker << ": Sending PLUS CHANNEL to FPGA." << std::endl;
    }
    for (uint64_t idx = 0; idx < d_active_subcarriers; idx++) {
      curr_Channel_Est_Real[idx] = int16_t(channelSign * channel_history[primary_channel_ticker][idx].real() * pow(2, 15));
      curr_Channel_Est_Imag[idx] = int16_t(channelSign * channel_history[primary_channel_ticker][idx].imag() * pow(2, 15));
    }
    
    // Record primary config values
    int16_t configValues[24] = {};
    std::cout << "Sending to FPGA: ";
    for (uint64_t idx = 0; idx < 24; idx++) {
      configValues[idx] = config_history[primary_channel_ticker][idx];
      std::cout << configValues[idx] << ", ";
    }
    std::cout << std::endl;

    // Build the frame
    union Frame_to_FPGA send_to_FPGA;
    write_Flag(send_to_FPGA.data_int, FPGA_operation);
    write_ChanEst(send_to_FPGA.data_int, curr_Channel_Est_Real, curr_Channel_Est_Imag);
    write_ConfigValues(send_to_FPGA.data_int, configValues);

    // Send to FPGA
    write(serial_port, &send_to_FPGA.data_char, TX_FRAME_LEN_BYTES);
    switch (FPGA_operation) {
      case 1:
        printf("OMP requested successfully!\n");

        // Set to GD mode if finished second OMP
        if (history_ticker == 2) {
          FPGA_operation = 2;
        }
        break;

      case 2: // Run GD
        printf("GD requested successfully!\n");
        break;
    }       

    // Step 4: Receive ACK from FPGA with new config values
    int received_bytes = 0;
    int total_received_bytes = 0;
    Frame_from_FPGA got_from_FPGA;
    while (total_received_bytes < RX_FRAME_LEN_BYTES) {
      received_bytes = read(serial_port, &got_from_FPGA.data_char[total_received_bytes], RX_FRAME_LEN_BYTES);
      total_received_bytes += received_bytes;
    }

    // Record new configuration obtained in ACK
    std::cout << "Received from FPGA: ";
    for (int ticker = 0; ticker < 24; ticker++) {
      config_history[history_ticker][ticker] = got_from_FPGA.data_int[ticker];
      std::cout << config_history[history_ticker][ticker] << ", ";
      outfile_config << config_history[history_ticker][ticker] << ", ";
    }
    outfile_config << std::endl;
    std::cout << std::endl;
    std::cout << "ROUND " << history_ticker << ": Recording configuration history." << std::endl;

    // Step 5: Wait for a bit, and request a new channel estimate from the channel estimate block
    usleep(delay); 
    message_port_pub(d_channel_est_request_port_name, pmt::cons(pmt::PMT_NIL, pmt::PMT_T));
  }


  int gen3_controller_impl::setup_pc_connection()
  {
    // Set up serial port
    int serial_port = open("/dev/ttyUSB2", O_RDWR);

    // Check for errors
    if (serial_port < 0) {
      printf("Error %i from open: %s\n", errno, strerror(errno));
    }
    else {
      printf("Serial Port %i initialization success!\n", serial_port);
    }

    // Create new termios struct, we call it 'tty' for convention
    // No need for "= {0}" at the end as we'll immediately write the existing
    // config to this struct
    struct termios tty;

    // Read in existing settings, and handle any error
    // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
    // must have been initialized with a call to tcgetattr() overwise behaviour
    // is undefined
    if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    // Setting the parity bit
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    //tty.c_cflag |= PARENB;  // Set parity bit, enabling parity

    // Setting the number of stop bit
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    //tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication

    // Setting number of bits per byte
    tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
    //tty.c_cflag |= CS5; // 5 bits per byte
    //tty.c_cflag |= CS6; // 6 bits per byte
    //tty.c_cflag |= CS7; // 7 bits per byte
    tty.c_cflag |= CS8; // 8 bits per byte (most common)

    // flow control
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    //tty.c_cflag |= CRTSCTS; // Enable RTS/CTS hardware flow control

    // CREAD AND CLOCAL
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)


    // Disabling canonical mode
    tty.c_lflag &= ~ICANON;

    // Disable echo
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo

    // Disable signal chars
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

    // Input Modes (c_iflag)

    // Software Flow Control (IXOFF, IXON, IXANY)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl

    // Disabling Special Handling Of Bytes On Receive
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    //Output Modes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

    // VMIN and VTIME (c_cc)
    // See: http://unixwiz.net/techtips/termios-vmin-vtime.html
    tty.c_cc[VTIME] = 0;  
    tty.c_cc[VMIN] = 48;  

    // Baud Rate
    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Saving termios
    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return -1;
    }

    return serial_port;
  }
  
  
  void
  gen3_controller_impl::initialize_canceller(int FPGA_hyperparameter_flag)
  {
    // Build init frame
    union Frame_to_FPGA initFrame; 
    int16_t init_flag = 3;
    int16_t init_chan_real[d_active_subcarriers] = {};
    int16_t init_chan_imag[d_active_subcarriers] = {};
    int16_t init_configValues[24] = {
      0, 0, 0, 0, 0, 0,     /* RF Gains */
      0, 0,               /* BB Gains */
      0, 0, 
      0, 0, 
      0, 0, 
      64, 64, 64, 64, 64, 64,   /* RF Addresses */
      0,                  /* BB Addresses */
      0, 
      0, 
      0
    };
    write_Flag(initFrame.data_int, FPGA_hyperparameter_flag);
    write_ChanEst(initFrame.data_int, init_chan_real, init_chan_imag);
    write_ConfigValues(initFrame.data_int, init_configValues);

    std::cout << "Sending to FPGA: ";
    /*for (uint64_t idx = 0; idx < 24; idx++) {
      std::cout << init_configValues[idx] << ", ";
    }*/
    for (uint64_t idx = 0; idx < TX_FRAME_LEN; idx++) {
      std::cout << initFrame.data_int[idx] << ", ";      
    }
    std::cout << std::endl;
    
    // Write to serial port
    write(serial_port, &initFrame.data_char, TX_FRAME_LEN_BYTES);
    // Wait for response
    int received_bytes = 0;
    int total_received_bytes = 0;
    Frame_from_FPGA got_from_FPGA;
    while (total_received_bytes < RX_FRAME_LEN_BYTES) {
      std::cout << "Nothing yet..." << std::endl;
      received_bytes = read(serial_port, &got_from_FPGA.data_char[total_received_bytes], RX_FRAME_LEN_BYTES);
      total_received_bytes += received_bytes;
    }
    printf("Initialization ACKed successfully!\n");
    

    // Record new configuration obtained in ACK
    std::cout << "Received from FPGA: ";
    for (int ticker = 0; ticker < 24; ticker++) {
      config_history[history_ticker][ticker] = got_from_FPGA.data_int[ticker];
      std::cout << config_history[history_ticker][ticker] << ", ";
    }
    std::cout << std::endl;
    std::cout << "ROUND " << history_ticker << ": Recording configuration history." << std::endl;

    // Set FPGA operation flag to OMP mode
    FPGA_operation = 1;
  }
    void
    gen3_controller_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    gen3_controller_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      gr_complex* taps_out = (gr_complex *) output_items[0];
    gr_complex taps_out_temp[24] = {};
    
    int currAddr;
    int currGain_real;
    int currGain_imag;
    for (uint64_t RF_ticker = 0; RF_ticker < 6; RF_ticker++) {
      // Obtain current gain and address
      currGain_real = config_history[history_ticker][RF_ticker];
      currAddr = config_history[history_ticker][RF_ticker+14];

      // Map to a tap
      switch (currAddr) {
        case 64:
          taps_out_temp[0] = gr_complex(currGain_real, 0);
          break;
        case 65 ... 72:
          taps_out_temp[1] = gr_complex(currGain_real, 0);
          break;
        case 73 ... 80:
          taps_out_temp[2] = gr_complex(currGain_real, 0);
          break;
        case 81 ... 88:
          taps_out_temp[3] = gr_complex(currGain_real, 0);
          break;
        case 89 ... 96:
          taps_out_temp[4] = gr_complex(currGain_real, 0);
          break;
        case 97 ... 104:
          taps_out_temp[5] = gr_complex(currGain_real, 0);
          break;
        case 105 ... 136:
          taps_out_temp[6] = gr_complex(currGain_real, 0);
          break;
        case 137 ... 168:
          taps_out_temp[7] = gr_complex(currGain_real, 0);
          break;
        case 169 ... 200:
          taps_out_temp[8] = gr_complex(currGain_real, 0);
          break;
        case 201 ... 232:
          taps_out_temp[9] = gr_complex(currGain_real, 0);
          break;
        case 233 ... 264:
          taps_out_temp[10] = gr_complex(currGain_real, 0);
          break;
        case 265 ... 296:
          taps_out_temp[11] = gr_complex(currGain_real, 0);
          break;
        case 297 ... 328:
          taps_out_temp[12] = gr_complex(currGain_real, 0);
          break;
        case 329 ... 360:
          taps_out_temp[13] = gr_complex(currGain_real, 0);
          break;
        case 361 ... 392:
          taps_out_temp[14] = gr_complex(currGain_real, 0);
          break;
        case 393 ... 424:
          taps_out_temp[15] = gr_complex(currGain_real, 0);
          break;
      }
    }
    for (uint64_t BB_ticker = 6; BB_ticker < 14; BB_ticker+=2) {
      // Obtain current gain and address
      currGain_real = config_history[history_ticker][BB_ticker];
      currGain_imag = config_history[history_ticker][BB_ticker+1];
      currAddr = config_history[history_ticker][17 + (BB_ticker)/2];

      // Map to a tap
      switch (currAddr) {
        case 0 ... 7:
          taps_out_temp[16] = gr_complex(currGain_real, currGain_imag);
          break;
        case 8 ... 15:
          taps_out_temp[17] = gr_complex(currGain_real, currGain_imag);
          break;
        case 16 ... 23:
          taps_out_temp[18] = gr_complex(currGain_real, currGain_imag);
          break;
        case 24 ... 31:
          taps_out_temp[19] = gr_complex(currGain_real, currGain_imag);
          break;
        case 32 ... 39:
          taps_out_temp[20] = gr_complex(currGain_real, currGain_imag);
          break;
        case 40 ... 47:
          taps_out_temp[21] = gr_complex(currGain_real, currGain_imag);
          break;
        case 48 ... 55:
          taps_out_temp[22] = gr_complex(currGain_real, currGain_imag);
          break;
        case 56 ... 63:
          taps_out_temp[23] = gr_complex(currGain_real, currGain_imag);
          break;
      }
    }


    for (uint64_t ticker; ticker < 24; ticker++)
    {
      taps_out[ticker] = taps_out_temp[ticker];
    }
    // Do <+signal processing+>
    // Tell runtime system how many input items we consumed on
    // each input stream.
    consume_each (0);
    // Tell runtime system how many output items we produced.
    return 24;
    }

  } /* namespace fullduplex */
} /* namespace gr */

