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

#ifndef INCLUDED_FULLDUPLEX_TDMA_SCHEDULER_IMPL_H
#define INCLUDED_FULLDUPLEX_TDMA_SCHEDULER_IMPL_H

#include <fullduplex/tdma_scheduler.h>
#include <iostream>
#include <chrono>
#include <vector>

namespace gr {
  namespace fullduplex {

    class tdma_scheduler_impl : public tdma_scheduler
    {
     private:
      // class fields
      bool d_debug;
      int d_num_radios;
      int d_num_fd_radios;
      bool d_fd_bs;
      bool d_iui_free;
      int d_slot;
      bool d_started;
      bool d_mode;
      std::vector<std::vector<int>> d_schedule;
      uint64_t d_startup_time;

      // message ports
      // in
      pmt::pmt_t d_clock_port;
      pmt::pmt_t d_start_port;
      void clock_handle(pmt::pmt_t msg);
      void start_handle(pmt::pmt_t msg);

      // out
      pmt::pmt_t d_schedule_port;
      pmt::pmt_t d_full_schedule_port;

      // helpers
      void generate_schedule();
      void generate_fd_schedule();
      void generate_hd_schedule();
      void print_schedule();
      void send_full_schedules();

     public:
      tdma_scheduler_impl(bool debug, int num_radios, int num_fd_radios, bool fd_bs, bool iui_free, bool mode);
      ~tdma_scheduler_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace fullduplex
} // namespace gr

#endif /* INCLUDED_FULLDUPLEX_TDMA_SCHEDULER_IMPL_H */

