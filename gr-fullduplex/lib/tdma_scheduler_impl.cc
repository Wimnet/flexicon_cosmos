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
#include "tdma_scheduler_impl.h"

namespace gr {
  namespace fullduplex {

    tdma_scheduler::sptr
    tdma_scheduler::make(bool debug, int num_radios, int num_fd_radios, bool fd_bs, bool iui_free, bool mode)
    {
      return gnuradio::get_initial_sptr
        (new tdma_scheduler_impl(debug, num_radios, num_fd_radios, fd_bs, iui_free, mode));
    }


    /*
     * The private constructor
     * There are no inputs or outputs in this block - it is message-driven
     */
    tdma_scheduler_impl::tdma_scheduler_impl(bool debug, int num_radios, int num_fd_radios, bool fd_bs, bool iui_free, bool mode)
      : gr::block("tdma_scheduler",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0)),
        d_debug(debug),
        d_num_radios(num_radios),
        d_num_fd_radios(num_fd_radios),
        d_fd_bs(fd_bs),
        d_iui_free(iui_free),
        d_clock_port(pmt::mp("clock")),
        d_start_port(pmt::mp("start")),
        d_schedule_port(pmt::mp("slot")),
        d_full_schedule_port(pmt::mp("schedule")),
        d_slot(0),
        d_started(false),
        d_mode(mode) // if mode == true, then we use the real-time mode. Otherwise, we output the schedule to each radio and they deal with it
        // d_schedule(std::vector<std::vector<int>>())
    {
      // register the clock port (every time a new packet is transmitted, this will )
      message_port_register_in(d_clock_port);
      set_msg_handler(d_clock_port, boost::bind(&tdma_scheduler_impl::clock_handle, this, _1));

      message_port_register_in(d_start_port);
      set_msg_handler(d_start_port, boost::bind(&tdma_scheduler_impl::start_handle, this, _1));

      message_port_register_out(d_schedule_port);

      message_port_register_out(d_full_schedule_port);

      // generate the schedule
      generate_schedule();

      if (d_debug) 
      {
        print_schedule();
      }

      if (!mode) // Non real-time mode. Output the schedule to each radio
      {
        d_startup_time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()
        ).count();
      }
      else 
      {
        d_startup_time = 0;
      }
    }

    /*
     * Our virtual destructor.
     */
    tdma_scheduler_impl::~tdma_scheduler_impl()
    {
    }

    /* the scheduler needs to take into account three things:
     * - the number of radios available
     * - the number of available radios which are in FD mode
     * - if the base station is in FD mode
     * 
     * essentially, the scheduler will first schedule the BS->UE downlinks
     * and then schedule the UE->BS uplinks
     * if the BS can operate in FD mode, then during the downlink, the UE can 
     * also perform an uplink
     * if a UE and BS can both operate in FD mode, then during the uplink, the BS can also
     * perform a downlink
     *
     * Radio 1 = BS, Radios 2-4 = UE1-3

     * FD UEs are specified in order. So if 2 UEs are FD, that means UE1 and UE2 are FD

     * Example: 1 BS, 3 UE. BS is FD-capable, 2 UE are FD-capable
        * T1: BS -> UE1, UE1 -> BS                      [1, 2]
        * T2: BS -> UE2, UE2 -> BS                      [1, 3]
        * T3: BS -> UE3, UE1 or UE2 -> BS (alternates)  [1, 2] or [1, 3]
        * T4: UE1 -> BS, BS -> UE1                      [1, 2]
        * T5: UE2 -> BS, BS -> UE2                      [1, 3]
        * T6: UE3 -> BS, BS -> UE1 or UE2 (alternates)  [1, 4]

    * Example: 1 BS, 3 UE, BS is FD-capable, no UE are FD-capable
        * T1: BS -> UE1, UE2 or UE3 -> BS (alternates)  [1, 3] or [1, 4]
        * T2: BS -> UE2, UE1 or UE3 -> BS (alternates)  [1, 2] or [1, 3]
        * T3: BS -> UE3, UE1 or UE2 -> BS (alternates)  [1, 2] or [1, 4]
        * T4: UE1 -> BS, BS -> UE2 or UE3 (alternates)  [1, 2]
        * T4: UE2 -> BS, BS -> UE1 or UE3 (alternates)  [1, 3]
        * T4: UE3 -> BS, BS -> UE1 or UE2 (alternates)  [1, 4]
    */ 
    void
    tdma_scheduler_impl::generate_schedule()
    {
      // call different functions depending on whether the BS is in HD mode or not
      if (d_fd_bs) 
      {
        generate_fd_schedule();
      }
      else
      {
        generate_hd_schedule();
      }
    }

    void
    tdma_scheduler_impl::generate_fd_schedule()
    {
      // schedule the BS by iterating through UEs
      for (int idx = 0; idx < d_num_radios - 1; idx++)
      {
        // if the current UE is FD, then we can schedule it with the BS
        if (idx < d_num_fd_radios) 
        {
          std::vector<int> slot{1, idx + 2};
          d_schedule.push_back(slot);
        }
        // if it isn't FD
        else
        {
          if (d_iui_free) 
          { // if IUI-free, we can only schedule the BS.
            std::vector<int> slot{1};
            d_schedule.push_back(slot);
          }
          else
          { // if not IUI-free schedule, then we can schedule the other UEs
            for (int j = 0; j < d_num_radios - 1; j++) 
            {
              if (j != idx) {
                std::vector<int> slot{1, j + 2};
                d_schedule.push_back(slot);
              }
            }
          }
        }
      }
      // now schedule the UEs
      // this is almost the same but it's a bit different. When scheduling
      // different BS->UE, the same radio (1) is enabled each time.
      // So two slots to do UE1->BS; BS->UE2 and UE1->BS; BS->UE3 would both be [2, 1].
      for (int idx = 0; idx < d_num_radios - 1; idx++)
      {
        // if the current UE is FD, then we can schedule it with the BS
        if (idx < d_num_fd_radios) 
        {
          std::vector<int> slot{idx + 2, 1};
          d_schedule.push_back(slot);
        }
        // if it isn't FD
        else
        {
          if (d_iui_free)
          { // if IUI-free schedule, we can only schedule the UE
            std::vector<int> slot{idx + 2};
            d_schedule.push_back(slot);
          }
          else
          { // otherwise we can schedule other things.
            for (int j = 0; j < d_num_radios - 1; j++) 
            {
              if (j != idx) {
                std::vector<int> slot{idx + 2, 1};
                d_schedule.push_back(slot);
              }
            }
          }
        }
      }
    }

    // If the BS is not HD, then it doesn't even matter if the 
    // UEs are FD. So the HD BS case is  trivial.
    void
    tdma_scheduler_impl::generate_hd_schedule()
    {
      // schedule the BS by iterating through UEs
      for (int idx = 0; idx < d_num_radios - 1; idx++)
      {
        std::vector<int> slot{1};
        d_schedule.push_back(slot);
      }
      // schedule the UEs
      for (int idx = 0; idx < d_num_radios - 1; idx++)
      {
        std::vector<int> slot{idx + 2};
        d_schedule.push_back(slot);
      }
    }

    // print the schedule to stdout
    void 
    tdma_scheduler_impl::print_schedule()
    {
      for (int idx = 0; idx < d_schedule.size(); idx++)
      {
        std::cout << "Time Slot " << idx + 1 << ": [";
        for (int j = 0; j < d_schedule[idx].size(); j++) 
        {
           std::cout << d_schedule[idx][j];
           if (j < d_schedule[idx].size() - 1)
           {
            std::cout << ", ";
           }
        }
        std::cout << "]" << std::endl;
      }
    }

    void 
    tdma_scheduler_impl::send_full_schedules()
    {
      // send the schedule which was generated to each radio, so they can manage
      // their transmissions by themselves. This is better when >3 radios are 
      // being used at once - the realtime messages can become desynched very easily
      for (int idx = 0; idx < d_num_radios; idx++)
      {
        int radio = idx + 1;
        pmt::pmt_t schedule = pmt::make_vector(d_schedule.size(), pmt::from_long(0)); 
        // std::vector<int> schedule = std::vector<int>(d_schedule.size());
        // go through the schedule and see which time slots the radio should be activated
        for (int jdx = 0; jdx < d_schedule.size(); jdx++)
        {
          int slotactive = 0;
          std::vector<int> slot = d_schedule[jdx];
          for (int kdx = 0; kdx < slot.size(); kdx++) 
          {
            if (radio == slot[kdx])
            {
              slotactive = 1;
              break;
            }
          }
          pmt::vector_set(schedule, jdx, pmt::from_long(slotactive));
        }
        // schedule should be built and look like a vector of 1s and 0s. 
        // now we can send it to the radio.
        pmt::pmt_t data = pmt::cons(pmt::from_long(d_schedule.size()), schedule);
        pmt::pmt_t msg = pmt::cons(pmt::from_long(radio), data);
        message_port_pub(d_full_schedule_port, msg);
      }
    }

    /*
     * Define the event handles for the clock and start ports
     */
    // The clock handle comes in from a packet counter attached to the BS' file source
    // it can be used with anything as long as it has a the packet_len stream tag
    void
    tdma_scheduler_impl::clock_handle(pmt::pmt_t msg)
    {
      if (!d_mode)
      { // if we are not in real time mode, check if a second has elapsed since starting. If so, then
        // send the schedule to the radios.
        if (!d_started) 
        {
          uint64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
          ).count();
          if (current_time - d_startup_time > 1000) 
          {
            d_started = true; 
            // send the schedule to each radio
            send_full_schedules();
          }
        }
        else { return; }
      }
      else
      {
        if (!d_started) 
        { // if not started, then do nothing
          return;
        }
        // send the transmit messages out of the schedule port
        std::vector<int> schedule = d_schedule[d_slot];
        for (int idx = 0; idx < schedule.size(); idx++) {
          int radio = schedule[idx];
          int transmit = 1;
          pmt::pmt_t msg = pmt::cons(pmt::from_long(radio), pmt::from_long(transmit));
          message_port_pub(d_schedule_port, msg);
        }
        d_slot = (d_slot + 1) % d_schedule.size();
      }
    }

    // just sets the started class field to true
    void
    tdma_scheduler_impl::start_handle(pmt::pmt_t msg)
    {
      if (d_mode) { // only care about this in realtime mode
        d_started = true;
      }
    }


    void
    tdma_scheduler_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      // not called
      ninput_items_required[0] = 0;
    }

    int
    tdma_scheduler_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      // not called
      return 0;
    }

  } /* namespace fullduplex */
} /* namespace gr */