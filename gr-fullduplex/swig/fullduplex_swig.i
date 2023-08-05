/* -*- c++ -*- */

#define FULLDUPLEX_API

%include "gnuradio.i"           // the common stuff

//load generated python docstrings
%include "fullduplex_swig_doc.i"

%{
#include "fullduplex/ofdm_constellation.h"
#include "fullduplex/fde_config.h"
#include "fullduplex/packet_encap2.h"
#include "fullduplex/digital_sic.h"
#include "fullduplex/parse_stream_sync.h"
#include "fullduplex/sub20_init.h"
#include "fullduplex/snr_calc.h"
#include "fullduplex/count_packets.h"
#include "fullduplex/async_divide.h"
#include "fullduplex/tdd_control.h"
#include "fullduplex/aloha_control.h"
#include "fullduplex/tx_rx_delay_calc.h"
#include "fullduplex/fd_mute.h"
#include "fullduplex/radio_sync.h"
#include "fullduplex/digital_sic_135.h"
#include "fullduplex/add_FD_pilot.h"
#include "fullduplex/add_padding.h"
#include "fullduplex/channel_estimator.h"
#include "fullduplex/fpga_hold_state.h"
//#include "fullduplex/sync_rx_stream.h"
#include "fullduplex/digital_sic_single.h"
#include "fullduplex/gen3_controller.h"
#include "fullduplex/gen3_tap_measurements.h"
#include "fullduplex/tdma_scheduler.h"
#include "fullduplex/tdma_packet_encap.h"
#include "fullduplex/fde_config_remote.h"
%}

%include "fullduplex/ofdm_constellation.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, ofdm_constellation);
%include "fullduplex/fde_config.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, fde_config);
%include "fullduplex/packet_encap2.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, packet_encap2);
%include "fullduplex/digital_sic.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, digital_sic);
%include "fullduplex/parse_stream_sync.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, parse_stream_sync);
%include "fullduplex/sub20_init.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, sub20_init);
%include "fullduplex/snr_calc.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, snr_calc);
%include "fullduplex/count_packets.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, count_packets);
%include "fullduplex/async_divide.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, async_divide);

%include "fullduplex/tdd_control.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, tdd_control);
%include "fullduplex/aloha_control.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, aloha_control);
%include "fullduplex/tx_rx_delay_calc.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, tx_rx_delay_calc);
%include "fullduplex/fd_mute.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, fd_mute);
%include "fullduplex/radio_sync.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, radio_sync);
%include "fullduplex/digital_sic_135.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, digital_sic_135);
%include "fullduplex/add_FD_pilot.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, add_FD_pilot);
%include "fullduplex/add_padding.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, add_padding);
%include "fullduplex/channel_estimator.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, channel_estimator);
%include "fullduplex/fpga_hold_state.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, fpga_hold_state);
//%include "fullduplex/sync_rx_stream.h"
//GR_SWIG_BLOCK_MAGIC2(fullduplex, sync_rx_stream);
%include "fullduplex/digital_sic_single.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, digital_sic_single);
%include "fullduplex/gen3_controller.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, gen3_controller);
%include "fullduplex/gen3_tap_measurements.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, gen3_tap_measurements);
%include "fullduplex/tdma_scheduler.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, tdma_scheduler);
%include "fullduplex/tdma_packet_encap.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, tdma_packet_encap);
%include "fullduplex/fde_config_remote.h"
GR_SWIG_BLOCK_MAGIC2(fullduplex, fde_config_remote);
