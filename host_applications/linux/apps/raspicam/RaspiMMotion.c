/*
Copyright (c) 2015, Broadcom Europe Ltd
Copyright (c) 2015, Silvan Melchior
Copyright (c) 2015, Robert Tidey
Copyright (c) 2015, James Hughes
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \file RaspiMMotions.c
 * Motion detect processing for RaspiMJPEG.c
 * Relies on a feed of motion vector data from RaspiMCam.
 *
 * \date 18th April 2015
 * \Author: Silvan Melchior / Robert Tidey
 *
 * Description
 *
 * Usage information in README_RaspiMJPEG.md
 */
#include "RaspiMJPEG.h"

int motion_width, motion_height, motion_img_width, motion_img_height;
int motion_frame_count;
int motion_changes;
int motion_state; // 0 search for start, 1 search for stop
int motion_vector_count;
unsigned char *mask_buffer_mem, *mask_buffer;

// initialise variables, set up mask buffer from a pbm file if present
void setup_motiondetect() {
   FILE *mask_file;
   int mask_size, mask_len;
   int mask_valid = 0;
   
   mask_size = motion_width * motion_height;
   printLog("Set up internal detect width=%d height=%d\n", motion_width, motion_height);
   
   motion_frame_count = 0;
   motion_state = 0;
   if (mask_buffer != 0)
      free(mask_buffer_mem);
   
   if (cfg_stru[c_motion_image] != 0) {
      mask_file = fopen(cfg_stru[c_motion_image], "r");
      mask_buffer_mem = (unsigned char *)malloc(mask_size + 256);
      mask_len = fread(mask_buffer_mem, sizeof *mask_buffer_mem, mask_size + 256, mask_file);
      fclose(mask_file);
      //Check for size and header
      if ((mask_len > mask_size + 10) && (*mask_buffer_mem == 'P') && (*(mask_buffer_mem + 1) == '5')) {
         //search for mask size string, data should be 1 byte after this
         mask_buffer = strstr(mask_buffer_mem, "255");
         if (mask_buffer != NULL) {
            mask_buffer +=3;
            //check size from this point
            if ((mask_buffer_mem + mask_len - mask_buffer) >= mask_size) {
               mask_valid = 1;
            }
         }
      }
      if (!mask_valid) {
         free(mask_buffer_mem);
         mask_buffer = 0;
         error("invalid motion mask", 0);
      } else {
         printLog("Motion mask %s loaded\n", cfg_stru[c_motion_image]);
      }
   }
   //Try to delete a motion data capture file if defined
   if (cfg_stru[c_motion_file] != 0) {
      remove(cfg_stru[c_motion_file]);
      motion_vector_count = 0;
   }
}

void send_motion_start() {
   send_schedulecmd("1");
}

void send_motion_stop() {
   send_schedulecmd("0");   
}

void analyse_vectors(unsigned char *buffer) {
   if(cfg_val[c_motion_detection] && !cfg_val[c_motion_external]) {
      unsigned char high_noise = 255 - cfg_val[c_motion_noise], low_noise = cfg_val[c_motion_noise];
      int i, m, row, col;
      i = 0;
      m = 0;
      motion_changes = 0;
      for(row=0; row<motion_height; row++) {
         for(col=0; col<motion_width; col++) {
            if (mask_buffer == 0 || mask_buffer[m++]) {
               if(buffer[i] > low_noise && buffer[i] < high_noise) motion_changes++;
               if(buffer[i+1] > low_noise && buffer[i+1] < high_noise) motion_changes++;
            }
            i+=4;
         }
      }
      switch (motion_state) {
         case 0:
            if (motion_changes >= cfg_val[c_motion_threshold]) {
               motion_frame_count++;
               if (motion_frame_count >= cfg_val[c_motion_startframes]) {
                  send_motion_start();
                  motion_frame_count = 0;
                  motion_state = 1;
               }
            } else {
               motion_frame_count = -2;
            }
            break;
         case 1:
            if (motion_changes < cfg_val[c_motion_threshold]) {
               motion_frame_count++;
               if (motion_frame_count >= cfg_val[c_motion_stopframes]) {
                  send_motion_stop();
                  motion_frame_count = 0;
                  motion_state = 0;
               }
            } else {
               motion_frame_count = -2;
            }
            break;
      }
      if (motion_frame_count < 0) motion_frame_count = 0;
   }
}

