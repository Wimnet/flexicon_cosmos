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
#include "gen3_tap_measurements_impl.h"
#include <iostream>

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

namespace gr {
  namespace fullduplex {

    gen3_tap_measurements::sptr
    gen3_tap_measurements::make(bool debug)
    {
      return gnuradio::get_initial_sptr
        (new gen3_tap_measurements_impl(debug));
    }

    /*
     * The private constructor
     */
    gen3_tap_measurements_impl::gen3_tap_measurements_impl(bool debug)
      : gr::block("gen3_tap_measurements",
              gr::io_signature::make(0, 0, 0),
      gr::io_signature::make(0, 0, 0)),
      d_debug(debug),
      d_input_port_name(pmt::mp("est_msg_in")),
      d_channel_est_request_port_name(pmt::mp("channel_request_out")),
      d_active_subcarriers(242),
      d_fresh_estimate(false)
    { 
      message_port_register_in(d_input_port_name);
      message_port_register_out(d_channel_est_request_port_name);

      Channel_Est_Real = (int16_t*)malloc(d_active_subcarriers*sizeof(int16_t));
      Channel_Est_Imag = (int16_t*)malloc(d_active_subcarriers*sizeof(int16_t));

      serial_port = setup_FPGA_connection();
      initialize_canceller(serial_port);
      
      // Set the current count to 0
      currCount = 0;
      
      // Set the current address to -1 (Representing initialization time)
      currAddress = -1;

      outfile.open("/home/wimnet-gen3/Documents/Gen-3 Canceller/TapMeasurements_256.csv");

      // only set up the message handler when we are done with the initial setup
      // this avoids issues where the channel estimator (which starts in an active state)
      // will send a message before it is time
      set_msg_handler(d_input_port_name, boost::bind(&gen3_tap_measurements_impl::on_channel_estimate, this, _1));
      message_port_pub(d_channel_est_request_port_name, pmt::cons(pmt::PMT_NIL, pmt::PMT_T));
    }

    /*
     * Our virtual destructor.
     */
    gen3_tap_measurements_impl::~gen3_tap_measurements_impl()
    {
      free(Channel_Est_Real);
      free(Channel_Est_Imag);

      // End session
      close(serial_port);
      
      // Close the file
      outfile.close();

      std::cout << EXIT_SUCCESS;
    }

    /*
     * Connecting to the FPGA
     */
    int gen3_tap_measurements_impl::setup_FPGA_connection()
    {
      // Set up serial port
      int serial_port = open("/dev/ttyUSB1", O_RDWR);

      // Check for errors
      if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
      }
      else {
        printf("Serial Port initialization success!\n");
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
      tty.c_cc[VTIME] = 100;  
      tty.c_cc[VMIN] = 0;  

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

    /*
     * Initializing the canceller to all-zeros
     */
    void
    gen3_tap_measurements_impl::initialize_canceller(int serial_port)
    {
      // Build initialization frame
      union Frame_to_FPGA initFrame; 
      int16_t init_flag = 3;
      int16_t init_chan_real[242] = {};
      int16_t init_chan_imag[242] = {};
      int16_t init_configValues[24] = {
        0, 0, 0, 0, 0, 60,    /* RF Gains */
        0, 0,          /* BB Gains */
        0, 0, 
        0, 0, 
        0, 0, 
        64, 64, 64, 64, 64, 64,  /* RF Addresses */
        0,             /* BB Addresses */
        0, 
        0, 
        0
      };
      write_Flag(initFrame.data_int, init_flag);
      write_ChanEst(initFrame.data_int, init_chan_real, init_chan_imag);
      write_ConfigValues(initFrame.data_int, init_configValues);
      
      // Write to serial port
      write(serial_port, &initFrame.data_char, TX_FRAME_LEN_BYTES);
      
      // Wait for response
      int received_bytes = 0;
      int total_received_bytes = 0;
      Frame_from_FPGA got_from_FPGA;
      while (total_received_bytes < RX_FRAME_LEN_BYTES) {
        received_bytes = read(serial_port, &got_from_FPGA.data_char[total_received_bytes], RX_FRAME_LEN_BYTES);
        total_received_bytes += received_bytes;
      }
      
      // Wait for a bit to ensure programming
      usleep(delay);      
      printf("Initialized successfully!\n");
    }

    /*
     * Run when channel estimates arrive
     */
    void
    gen3_tap_measurements_impl::on_channel_estimate(pmt::pmt_t msg)
    {
      // Write to console where we are
      std::cout << "ADDR: " << currAddress  << "\t| COUNT: " << currCount << std::endl;

      // Start case: give time for alignment
      if (currAddress == -1) {
        std::cout << "Ensuring alignment: " << 20 << " seconds" << std::endl;
        std::cout << "Remove circulator now!" << std::endl;
        usleep(20000000);
        
        // Update address
        currAddress += 1;

        // Request a new channel estimate from the channel estimate block
        message_port_pub(d_channel_est_request_port_name, pmt::cons(pmt::PMT_NIL, pmt::PMT_T));
        return;
      }
      /*
        // Prepare frame
        union Frame_to_FPGA currFrame; 
        int16_t curr_flag = 3;
        int16_t curr_chan_real[52] = {};
        int16_t curr_chan_imag[52] = {};
        write_Flag(currFrame.data_int, curr_flag);
        write_ChanEst(currFrame.data_int, curr_chan_real, curr_chan_imag);
        
        // Set appropriate gain and address in config values
        int16_t curr_configValues[24] = {
          0, 0, 0, 0, 0, 0,   // RF Gains 
          0, 0,           // BB Gains 
          0, 0, 
          0, 0, 
          0, 0, 
          64, 64, 64, 64, 64, 64, // RF Addresses 
          0,                  // BB Addresses 
          0, 
          0, 
          0
        };
        if (currAddress < 64) {   // BB tap
          // Even number --> real
          if (currAddress % 2 == 0) {
            curr_configValues[12] = 63;
            curr_configValues[23] = currAddress;
          }
          else {
            curr_configValues[13] = 63;
            curr_configValues[23] = currAddress-1;
          }
        }
        else {                    // RF tap
          curr_configValues[5]  = 63;
          curr_configValues[19] = currAddress;
        }
        write_ConfigValues(currFrame.data_int, curr_configValues);
        
        // Write to serial port
        write(serial_port, &currFrame.data_char, TX_FRAME_LEN_BYTES);

        // Receive ACK
        int received_bytes = 0;
        int total_received_bytes = 0;
        Frame_from_FPGA got_from_FPGA;
        while (total_received_bytes < RX_FRAME_LEN_BYTES) {
          received_bytes = read(serial_port, &got_from_FPGA.data_char[total_received_bytes], RX_FRAME_LEN_BYTES);
          total_received_bytes += received_bytes;
        }

        std::cout << "------------------------" << std::endl;
        std::cout << "----\tADDRESS " << currAddress << "\t----" << std::endl;

        // Set count back to 0
        currCount = 0;

        // Wait for a bit to ensure programming
        usleep(delay);

      }*/

      // Update count
      currCount += 1;

      // If reach maximum count, reprogram
      if (currCount >= maxCount) {
        // Update address
        currAddress += 1;

        // Prepare frame
        union Frame_to_FPGA currFrame; 
        int16_t curr_flag = 3;
        int16_t curr_chan_real[242] = {};
        int16_t curr_chan_imag[242] = {};
        write_Flag(currFrame.data_int, curr_flag);
        write_ChanEst(currFrame.data_int, curr_chan_real, curr_chan_imag);
        
        int16_t curr_configValues[24] = {
          0, 0, 0, 0, 0, 0,   // RF Gains 
          0, 0,           // BB Gains 
          0, 0, 
          0, 0, 
          0, 0, 
          64, 64, 64, 64, 64, 64, // RF Addresses 
          0,                  // BB Addresses 
          0, 
          0, 
          0
        };
        if (currAddress == 425) {        
          // Set appropriate gain and address in config values
          curr_configValues[5] = 20;
          curr_configValues[12] = 63;
        }
        else if (currAddress == 426){
          return;
        }
        else {
          // Set appropriate gain and address in config values
          if (currAddress < 64) {   // BB tap
            // Even number --> real
            if (currAddress % 2 == 0) {
              curr_configValues[12] = 63;
              curr_configValues[23] = currAddress;
            }
            else {
              curr_configValues[13] = 63;
              curr_configValues[23] = currAddress-1;
            }
          }
          else {                    // RF tap
            curr_configValues[5]  = 63;
            curr_configValues[19] = currAddress;
          }
        }
        write_ConfigValues(currFrame.data_int, curr_configValues);
        
        // Write to serial port
        write(serial_port, &currFrame.data_char, TX_FRAME_LEN_BYTES);
        std::cout << "Sending to FPGA: ";
        for (int ticker = 0; ticker < 24; ticker++) {
          std::cout << currFrame.data_int[1+2*242+ticker] << ", ";
        }
        std::cout << std::endl;

        // Receive ACK
        int received_bytes = 0;
        int total_received_bytes = 0;
        Frame_from_FPGA got_from_FPGA;
        while (total_received_bytes < RX_FRAME_LEN_BYTES) {
          received_bytes = read(serial_port, &got_from_FPGA.data_char[total_received_bytes], RX_FRAME_LEN_BYTES);
          total_received_bytes += received_bytes;
        }
        std::cout << "Received from FPGA: ";
        for (int ticker = 0; ticker < 24; ticker++) {
          std::cout << got_from_FPGA.data_int[ticker] << ", ";
        }
        std::cout << std::endl;

        std::cout << "------------------------" << std::endl;
        std::cout << "----\tADDRESS " << currAddress << "\t----" << std::endl;


        // Set count back to 0
        currCount = 0;

        // Wait for a bit to ensure programming
        usleep(delay);
      }

      // Obtain new estimate
      const gr_complex* val = pmt::c32vector_elements(pmt::cdr(msg), d_active_subcarriers);
      for (uint64_t idx = 0; idx < d_active_subcarriers; idx++) {
        Channel_Est_Real[idx] = int16_t(val[idx].real() * pow(2, 15));
        Channel_Est_Imag[idx] = int16_t(val[idx].imag() * pow(2, 15));
        if (d_debug) {
          std::cout << "Re: " << Channel_Est_Real[idx] << ", Im: " << Channel_Est_Imag[idx] << std::endl; 
        }

        // Write to file, update count
        outfile << val[idx].real() << ',' << val[idx].imag() << ',';
        //std::cout << ".";
      }
      outfile << std::endl;
      //std::cout << std::endl;
      d_fresh_estimate = true;

      // Request a new channel estimate from the channel estimate block
      message_port_pub(d_channel_est_request_port_name, pmt::cons(pmt::PMT_NIL, pmt::PMT_T));

    }

    /*
     * Debug messages
     */
    void
    gen3_tap_measurements_impl::debug_out(std::string msg)
    {
      if (d_debug)
      {
        std::cout << "[GEN-3 TAP MEASURE] " << msg << std::endl;
      }
    }

    void
    gen3_tap_measurements_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    gen3_tap_measurements_impl::general_work (int noutput_items,
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

