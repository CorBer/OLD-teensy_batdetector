/*
 * Copyright (c) 2018 John-Michael Reed
 * bleeplabs.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include "effect_granular.h"

void AudioEffectGranular::begin(int16_t *sample_bank_def, int16_t max_len_def)
{
	max_sample_len = max_len_def;
	grain_mode = 0;
	read_head = 0;
	write_head = 0;
	prev_input = 0;
	playpack_rate = 65536;
	accumulator = 0;
	allow_len_change = true;
	sample_loaded = false;
	sample_bank = sample_bank_def;
	int subsample=0;
}

void AudioEffectGranular::beginFreeze_int(int grain_samples)
{
	__disable_irq();
	grain_mode = 1;
	if (grain_samples < max_sample_len) {
		freeze_len = grain_samples;
	} else {
		freeze_len = grain_samples;
	}
	sample_loaded = false;
	write_en = false;
	sample_req = true;
	__enable_irq();
}

void AudioEffectGranular::beginPitchShift_int(int grain_samples)
{
	__disable_irq();
	grain_mode = 2;
	if (allow_len_change) {
		if (grain_samples < 100) grain_samples = 100;
		int maximum = (max_sample_len - 1) / 3;
		if (grain_samples > maximum) grain_samples = maximum;
		glitch_len = grain_samples;
	}
	sample_loaded = false;
	write_en = false;
	sample_req = true;
	__enable_irq();
}

void AudioEffectGranular::beginTimeExpansion_int(int grain_samples)
{
	__disable_irq();
	grain_mode = 3;
	if (allow_len_change) {
		if (grain_samples > max_sample_len) {
		grain_samples = max_sample_len;
	     } 
		glitch_len = grain_samples;
	}
	sample_loaded = false;
	write_en = false;
	sample_req = true;

	__enable_irq();
}

void AudioEffectGranular::beginDivider_int(int grain_samples)
{
	__disable_irq();
	divider_sample=0; //samplecounter
	read_head=0;
	grain_mode = 4;
	if (allow_len_change) {
		if (grain_samples > max_sample_len) {
		grain_samples = max_sample_len;
	     } 
		glitch_len = grain_samples;
	}
	sample_loaded = false;
	write_en = false;
	sample_req = true;
	__enable_irq();
}


void AudioEffectGranular::stop()
{
	grain_mode = 0;
	allow_len_change = true;
}

void AudioEffectGranular::update(void)
{
	audio_block_t *block;
	
	if (sample_bank == NULL) {
		block = receiveReadOnly(0);
		if (block) release(block);
		return;
	}

	block = receiveWritable(0);
	
	if (!block) return;

	if (grain_mode == 0) {
		// passthrough, no granular effect
		prev_input = block->data[AUDIO_BLOCK_SAMPLES-1];
	}
	else if (grain_mode == 1) {
		// Freeze - sample 1 grain, then repeatedly play it back
		for (int j = 0; j < AUDIO_BLOCK_SAMPLES; j++) {
			if (sample_req) {
				// only begin capture on zero cross
				int16_t current_input = block->data[j];
				if ((current_input < 0 && prev_input >= 0) ||
				  (current_input >= 0 && prev_input < 0)) {
					write_en = true;
					write_head = 0;
					read_head = 0;
					sample_req = false;
				} else {
					prev_input = current_input;
				}
			}
			if (write_en) {
				sample_bank[write_head++] = block->data[j];
				if (write_head >= freeze_len) {
					sample_loaded = true;
				}
				if (write_head >= max_sample_len) {
					write_en = false;
				}
			}
			if (sample_loaded) {
				if (playpack_rate >= 0) {
					accumulator += playpack_rate;
					read_head = accumulator >> 16;
				}
				if (read_head >= freeze_len) {
					accumulator = 0;
					read_head = 0;
				}
				block->data[j] = sample_bank[read_head];
			}
		}
	}
	else if (grain_mode == 2) {
		//GLITCH SHIFT
		//basic granular synth thingy
		// the shorter the sample the max_sample_len the more tonal it is.
		// Longer it has more definition.  It's a bit roboty either way which
		// is obv great and good enough for noise music.

		for (int k = 0; k < AUDIO_BLOCK_SAMPLES; k++) {
			// only start recording when the audio is crossing zero to minimize pops
			if (sample_req) {
				int16_t current_input = block->data[k];
				if ((current_input < 0 && prev_input >= 0) ||
				  (current_input >= 0 && prev_input < 0)) {
					write_en = true;
				} else {
					prev_input = current_input;
				}
			}

			if (write_en) {
				sample_req = false;
				allow_len_change = true; // Reduces noise by not allowing the
						// length to change after the sample has been
						// recored.  Kind of not too much though
				if (write_head >= glitch_len) {
					write_head = 0;
					sample_loaded = true;
					write_en = false;
					allow_len_change = false;
				}
				sample_bank[write_head] = block->data[k];
				//decimate to half the samples

				write_head++;
			}

			if (sample_loaded) {
				//move it to the middle third of the bank.
				//3 "seperate" banks are used
				float fade_len = 20.00;
				int16_t m2 = fade_len;

				for (int m = 0; m < 2; m++) {
					// I'm off by one somewhere? why is there a tick at the
					// beginning of this only when it's combined with the
					// fade out???? ooor am i osbserving that incorrectly
					// either wait it works enough
					sample_bank[m + glitch_len] = 0;
				}

				for (int m = 2; m < glitch_len-m2; m++) {
					sample_bank[m + glitch_len] = sample_bank[m];
				}

				for (int m = glitch_len-m2; m < glitch_len; m++) {
					// fade out the end. You can just make fadet=0
					// but it's a little too daleky
					float fadet = sample_bank[m] * (m2 / fade_len);
					sample_bank[m + glitch_len] = (int16_t)fadet;
					m2--;
				}
				sample_loaded = false;
				prev_input = block->data[k];
				sample_req = true;
			}

			accumulator += playpack_rate;
			read_head = (accumulator >> 16);

			if (read_head >= glitch_len) {
				read_head -= glitch_len;
				accumulator = 0;

				for (int m = 0; m < glitch_len; m++) {
					sample_bank[m + (glitch_len*2)] = sample_bank[m+glitch_len];
					//  sample_bank[m + (glitch_len*2)] = (m%20)*1000;
				}
			}
			block->data[k] = sample_bank[read_head + (glitch_len*2)];
		}
	} 
	else if (grain_mode == 3) {
		//TIME EXPANSION
		for (int k = 0; k < AUDIO_BLOCK_SAMPLES; k++) 
		 { 
			// if a sample is requested
			// wait with recording until a crossing zero point passes
			if (sample_req) {
				write_en = true; //start collecting a sample by setting write_enabled
				write_head=0; //start at position 0
				read_head=0;
				/*int16_t current_input = block->data[k];
				if ((current_input < 0 && prev_input >= 0) ||
				  (current_input >= 0 && prev_input < 0)) {
					write_en = true; //start collecting a sample by setting write_enabled
					write_head=0; //start at position 0
					read_head=0;
				} else {
					prev_input = current_input;
				}*/

			}

            //active collecting of all incoming data if write_enabled
			if (write_en) {
				sample_req = false; // stop sample request as we are recording
				
				// collect until glitch_len samples are available in the sample_bank
				if (write_head >= glitch_len) {
					//write_head = 0;
					//accumulator= 0;
					sample_loaded = true; //sample_bank is full 
					write_en = false; // stop sampling
				}

				play_sample=true;
				sample_bank[write_head]= block->data[k];
				write_head++; // next sample
			   
			}

            //sample bank is full and can be used to transfer back to the audio system
			if (sample_loaded) {
				
			}
            
			 
			if (play_sample)
			 { accumulator += playpack_rate; // shift sampleposition
			   read_head = (accumulator >> 16); //rightshift 16 == div by 65535 
			 }

			 /*
			 if (play_sample)
			  { accumulator++;
			    read_head = accumulator * Rdivider;
			  }
		*/

			//glitch_len equals array_size
			if (read_head >= write_head) {
				read_head = 0;
				accumulator = 0;
                sample_req = false; //we dont want a new sample
				sample_loaded = false;
				play_sample=false;
			}
         
			block->data[k] = sample_bank[read_head];
			
		}
	} 
	else if (grain_mode == 4) {
		//FREQUENCY DIVIDER
		// todo: change into continous averaged sampling by adding a new block->data and removing the oldest from the samples
		//       array. Average calculation can be done as new_average = last_average - oldest_data*0.1 + latest_data*0.1
        uint8_t dividerset=10;
		
		//needs very little memory, only X=dividerset samples[X] 
		for (int k = 0; k < AUDIO_BLOCK_SAMPLES; k++) 
		 {   sample_bank[divider_sample%dividerset]= block->data[k];  //store current readout in revolving arrayposition
	         block->data[k] = sample_bank[0]; //output is always based on the same arrayposition
			 divider_sample++;
		}
         //roll over the sample_counter at a safe point (block_size*divider) so all blocks are based on equal datasets
		if (divider_sample>=AUDIO_BLOCK_SAMPLES*dividerset)
		 {  divider_sample=0;
		 }
		  
	  }
	  
	transmit(block);
	release(block);
}




