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

#include <gnuradio/io_signature.h>
#include "fpga_hold_state_impl.h"

namespace gr {
  namespace fullduplex {

    fpga_hold_state::sptr
    fpga_hold_state::make()
    {
      return gnuradio::get_initial_sptr
        (new fpga_hold_state_impl());
    }

    /*
     * The private constructor
     */
    fpga_hold_state_impl::fpga_hold_state_impl()
      : gr::block("fpga_hold_state",
               gr::io_signature::make(0, 0, 0),
      gr::io_signature::make(0, 0, 0)),
      //d_debug(debug),
      d_input_port_name(pmt::mp("est_msg_in")),
      d_channel_est_request_port_name(pmt::mp("channel_request_out")),
      d_active_subcarriers(52),
      d_fresh_estimate(false),
      FPGA_operation(50)
    {

    message_port_register_in(d_input_port_name);
    message_port_register_out(d_channel_est_request_port_name);

    //curr_Channel_Est_Real = (int16_t*)malloc(d_active_subcarriers*sizeof(int16_t));
    //curr_Channel_Est_Imag = (int16_t*)malloc(d_active_subcarriers*sizeof(int16_t));

    serial_port = setup_pc_connection();

    // only set up the message handler when we are done with the initial setup
    // this avoids issues where the channel estimator (which starts in an active state)
    // will send a message before it is time
    

    // OMP

    /*
    // Build init frame
    union Frame_to_FPGA initFrame; 
      float temp_chan_real[52] = {
      -0.17927, 
      -0.184881,
      -0.205506,
      -0.210857,
      -0.237466,
      -0.19733, 
      -0.253837,
      -0.261499,
      -0.29201, 
      -0.287356,
      -0.304153,
      -0.322014,
      -0.354569,
      -0.348917,
      -0.359554,
      -0.391707,
      -0.393066,
      -0.417541,
      -0.419498,
      -0.469659,
      -0.443204,
      -0.445401,
      -0.456601,
      -0.466806,
      -0.48032, 
      -0.454212,
      -0.42703, 
      -0.493421,
      -0.516782,
      -0.518115,
      -0.496927,
      -0.524572,
      -0.540657,
      -0.52949, 
      -0.53131, 
      -0.52831, 
      -0.494166,
      -0.534393,
      -0.501648,
      -0.47909, 
      -0.487637,
      -0.47784, 
      -0.472676,
      -0.453777,
      -0.454213,
      -0.418424,
      -0.456938,
      -0.37677, 
      -0.375661,
      -0.375692,
      -0.338189,
      -0.278995
    };
    float temp_chan_imag[52] = {
      -0.259583,
      -0.263438,
      -0.255111,
      -0.246658,
      -0.264864,
      -0.228144,
      -0.247002,
      -0.24282,
      -0.233269,
      -0.230057,
      -0.218091,
      -0.207372,
      -0.193767,
      -0.192501,
      -0.213068,
      -0.161113,
      -0.164622,
      -0.149645,
      -0.132126,
      -0.0806436,
      -0.095742,
      -0.0630179,
      -0.0316918,
      -0.00797447,
      0.0436537,
      0.183411,
      -0.216835,
      -0.0878293,
      -0.0206764,
      0.00273584,
      0.0265272,
      0.0525986,
      0.178038,
      0.111139,
      0.142459,
      0.157265,
      0.20872,
      0.212585,
      0.224486,
      0.27663,
      0.281551,
      0.308271,
      0.311408,
      0.386651,
      0.391935,
      0.40552,
      0.344017,
      0.446679,
      0.499764,
      0.489213,
      0.521042,
      0.495996
    };

    int16_t init_chan_real[52] = {};
    int16_t init_chan_imag[52] = {};
      for (uint64_t idx = 0; idx < d_active_subcarriers; idx++) {
      init_chan_real[idx] = int16_t(temp_chan_real[idx]* pow(2, 15));
      init_chan_imag[idx] = int16_t(temp_chan_imag[idx]* pow(2, 15));
    }
  
    int16_t init_configValues[24] = {
      0, 0, 0, 0, 0, 0,     // RF Gains 
      0, 0,               // BB Gains 
      0, 0, 
      0, 0, 
      0, 0, 
      64, 64, 64, 64, 64, 64,   // RF Addresses 
      0,                  // BB Addresses 
      0, 
      0, 
      0
    };
    write_Flag(initFrame.data_int, 1);
    write_ChanEst(initFrame.data_int, init_chan_real, init_chan_imag);
    write_ConfigValues(initFrame.data_int, init_configValues);

    std::cout << "Sending to FPGA: ";
    for (uint64_t idx = 0; idx < 24; idx++) {
      std::cout << init_configValues[idx] << ", ";
    }
     std::cout << std::endl;
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
    printf("OMP Finished!\n");

    std::cout << "Received from FPGA: ";
    for (int ticker = 0; ticker < 24; ticker++) {
      config_history[ticker] = got_from_FPGA.data_int[ticker];
      std::cout << config_history[ticker] << ", ";
    }
    std::cout << std::endl;



 // Gradient Descent


    std::cout<<"Running GD: "<<std::endl;

    write_Flag(initFrame.data_int, 2);
    write_ChanEst(initFrame.data_int, init_chan_real, init_chan_imag);
    write_ConfigValues(initFrame.data_int, config_history);

    std::cout << "Sending to FPGA: ";

    for (uint64_t idx = 0; idx < 24; idx++) {
      std::cout << config_history[idx] << ", ";
    }
     std::cout << std::endl;
    for (uint64_t idx = 0; idx < TX_FRAME_LEN; idx++) {
      std::cout << initFrame.data_int[idx] << ", ";      
    }
    std::cout << std::endl;
    
    // Write to serial port
    write(serial_port, &initFrame.data_char, TX_FRAME_LEN_BYTES);
    // Wait for response
    received_bytes = 0;
    total_received_bytes = 0;
  
    while (total_received_bytes < RX_FRAME_LEN_BYTES) {
      std::cout << "Nothing yet..." << std::endl;
      received_bytes = read(serial_port, &got_from_FPGA.data_char[total_received_bytes], RX_FRAME_LEN_BYTES);
      total_received_bytes += received_bytes;
    }
    printf("GD Finished!\n");

    std::cout << "Received from FPGA: ";
    for (int ticker = 0; ticker < 24; ticker++) {
      config_history[ticker] = got_from_FPGA.data_int[ticker];
      std::cout << config_history[ticker] << ", ";
    }
    std::cout << std::endl;
    */
    //message_port_pub(d_channel_est_request_port_name, pmt::cons(pmt::PMT_NIL, pmt::PMT_T));
    //set_msg_handler(d_input_port_name, boost::bind(&gen3_hyperparameter_tuning_impl::on_channel_estimate, this, _1));
    //message_port_pub(d_channel_est_request_port_name, pmt::cons(pmt::PMT_NIL, pmt::PMT_T));
  


    }

    /*
     * Our virtual destructor.
     */
    fpga_hold_state_impl::~fpga_hold_state_impl()
    {
      // End session
    close(serial_port);

    std::cout << EXIT_SUCCESS;
    }

    int fpga_hold_state_impl::setup_pc_connection()
  {
    // Set up serial port
    int serial_port = open("/dev/ttyUSB1", O_RDWR);

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
    fpga_hold_state_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    fpga_hold_state_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {

      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (0);

      // Tell runtime system how many output items we produced.
      return 0;
    }

  } /* namespace fullduplex */
} /* namespace gr */

