/* -*- c++ -*- */
/* Copyright 2012 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/digital/ofdm_equalizer_simpledfe.h>

namespace gr {
  namespace digital {

    ofdm_equalizer_simpledfe::sptr
    ofdm_equalizer_simpledfe::make(
	  int fft_len,
	  const gr::digital::constellation_sptr &constellation,
	  const std::vector<std::vector<int> > &occupied_carriers,
	  const std::vector<std::vector<int> > &pilot_carriers,
	  const std::vector<std::vector<gr_complex> > &pilot_symbols,
	  int symbols_skipped,
	  float alpha,
	  bool input_is_shifted)
    {
      return ofdm_equalizer_simpledfe::sptr(
	  new ofdm_equalizer_simpledfe(
	      fft_len,
	      constellation,
	      occupied_carriers,
	      pilot_carriers,
	      pilot_symbols,
	      symbols_skipped,
	      alpha,
	      input_is_shifted
	  )
      );
    }

    ofdm_equalizer_simpledfe::ofdm_equalizer_simpledfe(
	int fft_len,
	const gr::digital::constellation_sptr &constellation,
	const std::vector<std::vector<int> > &occupied_carriers,
	const std::vector<std::vector<int> > &pilot_carriers,
	const std::vector<std::vector<gr_complex> > &pilot_symbols,
	int symbols_skipped,
	float alpha,
	bool input_is_shifted)
      : ofdm_equalizer_1d_pilots(fft_len, occupied_carriers, pilot_carriers, pilot_symbols, symbols_skipped, input_is_shifted),
	  d_constellation(constellation),
	  d_alpha(alpha)
    {
    }


    ofdm_equalizer_simpledfe::~ofdm_equalizer_simpledfe()
    {
    }

    /**
     * (DN) Main equalizer method for simple DFE 
     * The equalizer keeps track of the channel estimate on every subcarrier i (H_i) using an averaging IIR filter 
     *  Params:
     *    - frame: packet frame samples
     *    - n_sym: number of symbols to equalize
     *    - initial_taps
     *    - tags
     */
    void
    ofdm_equalizer_simpledfe::equalize(gr_complex *frame,
	      int n_sym,
	      const std::vector<gr_complex> &initial_taps,
	      const std::vector<tag_t> &tags)
    {
      if (!initial_taps.empty()) {
        // (DN) Initialize the channel state vector
	d_channel_state = initial_taps;
      }
      gr_complex sym_eq, sym_est;

      for (int i = 0; i < n_sym; i++) { // loop through the OFDM symbols in the frame
	for (int k = 0; k < d_fft_len; k++) { // loop through the subcarrier index

	  if (!d_occupied_carriers[k]) { // if k is NOT a data subcarrier
	    continue;
	  }
          
          // (DN) IF there are pilot subcarrriers, and this k-th subcarrier is a pilot
	  if (!d_pilot_carriers.empty() && d_pilot_carriers[d_pilot_carr_set][k]) {  
            // update (directly) the channel estimate on the k-th subcarrier, H_k, using an averaging IIR
	    d_channel_state[k] = d_alpha * d_channel_state[k]
			       + (1-d_alpha) * frame[i*d_fft_len + k] / d_pilot_symbols[d_pilot_carr_set][k];
            // Equalize the subcarrier data symbol
	    frame[i*d_fft_len+k] = d_pilot_symbols[d_pilot_carr_set][k];
	  } 
          // (DN) ELSE, k-th subcarrier carries data symbol
          else { 
            // equalize the subcarrier data symbol, using the _previous_ channel estimate
	    sym_eq = frame[i*d_fft_len+k] / d_channel_state[k];
            // map the equalized symbol to a constellation point
	    d_constellation->map_to_points(d_constellation->decision_maker(&sym_eq), &sym_est);
            // calculate the new channel estimate based on this mapping
	    d_channel_state[k] = d_alpha * d_channel_state[k]
                               + (1-d_alpha) * frame[i*d_fft_len + k] / sym_est;
            // update the channel estimate
	    frame[i*d_fft_len+k] = sym_est;
	  }
	}

        // Advancing the pilot carrier set index 
	if (!d_pilot_carriers.empty()) {
	  d_pilot_carr_set = (d_pilot_carr_set + 1) % d_pilot_carriers.size();
	}
      }
    }

  } /* namespace digital */
} /* namespace gr */

