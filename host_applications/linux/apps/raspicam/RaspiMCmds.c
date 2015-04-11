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
 * \file RaspiMCmds.c
 * Process Pipe Commands for RaspiMJPEG
 *
 * \date 9th Aprl 2015
 * \Author: Silvan Melchior / Robert Tidey
 *
 * Description
 *
 * Pipe Commands all start with 2 char identifier
 * Usage information in README_RaspiMJPEG.md
 */

#include "RaspiMJPEG.h"

void process_cmd(char *readbuf, int length) {
   typedef enum pipe_cmd_type{ca,im,tl,px,bo,tv,an,av,as,at,ac,ab,sh,co,br,sa,is,vs,rl,ec,em,wb,mm,ie,ce,ro,fl,ri,ss,qu,bl,ru,md,sc,rs,bu} pipe_cmd_type;
   char pipe_cmds[] = "ca,im,tl,px,bo,tv,an,av,as,at,ac,ab,sh,co,br,sa,is,vs,rl,ec,em,wb,mm,ie,ce,ro,fl,ri,ss,qu,bl,ru,md,sc,rs,bu";
   pipe_cmd_type pipe_cmd;
   int i;
   char pars[128][10];
   long int par0;
   char *parstring=0, *temp;
   int key = -1;
   
   //Sanitise buffer and return if no good
   if (length == 2) length++;
   if (length < 3) return;
   readbuf[length] = 0;
   readbuf[2] = 0;
   
   //find 2 letter command and translate into enum
   temp = strstr(pipe_cmds, readbuf);
   if (temp == NULL) return;
   pipe_cmd = (pipe_cmd_type)((temp - pipe_cmds) / 3);
   
   //extract space separated numeric parameters
   // and make separate string parameter (strtok changes the original)
   asprintf(&parstring, "%s", readbuf+3);
   i = 0;
   temp = strtok(readbuf+3, " ");
   while(i<10 && temp != NULL) {
      strcpy(pars[i], temp);
      i++;
      temp = strtok(NULL, " ");
   }
   par0 = strtol(pars[0], NULL, 10);
   
   switch(pipe_cmd) {
      case ca:
         if(par0 == 1) {
            start_video(0);
         }  else {
            stop_video(0);
         }
         break;
      case im:
         capt_img();
         break;
      case tl:
         if(par0) {
            timelapse = 1;
            lapse_cnt = 1;
            updateStatus();
            printLog("Timelapse started\n");
         }
         else {
            image2_cnt++;
            timelapse = 0;
            updateStatus();
            printLog("Timelapse stopped\n");
         }
         break;
      case px:
         stop_all();
         addUserValue(c_video_width, pars[0]);
         addUserValue(c_video_height, pars[1]);
         addUserValue(c_video_fps, pars[2]);
         addUserValue(c_MP4Box_fps, pars[3]);
         addUserValue(c_image_width, pars[4]);
         addUserValue(c_image_height, pars[5]);
         start_all(0);
         break;
      case bo:
         addUserValue(c_MP4Box, pars[0]);
         break;
      case tv:
         addUserValue(c_tl_interval, pars[0]);
         break;
      case an:
         addUserValue(c_annotation, parstring);
         break;
      case av:
         addUserValue(c_anno_version, pars[0]);
         break;
      case as:
         addUserValue(c_anno_text_size, pars[0]);
         break;
      case at:
         addUserValue(c_anno3_custom_text_colour, pars[0]);
         addUserValue(c_anno3_custom_text_Y, pars[1]);
         addUserValue(c_anno3_custom_text_U, pars[2]);
         addUserValue(c_anno3_custom_text_V, pars[3]);
         break;
      case ac:
         addUserValue(c_anno3_custom_background_colour, pars[0]);
         addUserValue(c_anno3_custom_background_Y, pars[1]);
         addUserValue(c_anno3_custom_background_U, pars[2]);
         addUserValue(c_anno3_custom_background_V, pars[3]);
         break;
      case ab:
         addUserValue(c_anno_background, pars[0]);
         break;
      case sh:
         key = c_sharpness;
         break;
      case co:
         key = c_contrast;
         break;
      case br:
         key = c_brightness;
         break;
      case sa:
         key = c_saturation;
         break;
      case is:
         key = c_iso;
         break;
      case vs:
         key = c_video_stabilisation;
         break;
      case rl:
         key = c_raw_layer;
         break;
      case ec:
         key = 1000 + c_exposure_compensation;
         break;
      case em:
         key = 1000 + c_exposure_mode;
         break;
      case wb:
         key = 1000 + c_white_balance;
         break;
      case mm:
         key = 1000 + c_metering_mode;
         break;
      case ie:
         key = 1000 + c_image_effect;
         break;
      case ce:
         addUserValue(c_colour_effect_u, pars[1]);
         addUserValue(c_colour_effect_v, pars[2]);
         key = c_colour_effect_en;
         break;
      case ro:
         key = c_rotation;
         break;
      case fl:
         if(par0 & 1) addUserValue(c_hflip, "1"); else addUserValue(c_hflip, "0"); 
         if((par0 >> 1) & 1) addUserValue(c_vflip, "1"); else addUserValue(c_vflip, "0"); 
         cam_set(c_hflip);
         break;
      case ri:
         addUserValue(c_sensor_region_y, pars[0]);
         addUserValue(c_sensor_region_w, pars[0]);
         addUserValue(c_sensor_region_h, pars[0]);
         key = c_sensor_region_x;
         break;
      case ss:
         addUserValue(c_shutter_speed, pars[0]);
         key = c_shutter_speed;
         break;
      case qu:
         key = c_image_quality;
         break;
      case bl:
         key = c_video_bitrate;
         break;
      case ru:
         if (par0 == 0) {
            stop_all();
            idle = 1;
            printLog("Stream halted\n");
         } else {
            start_all(1);
            idle = 0;
            printLog("Stream continued\n");
         }
         updateStatus();
         break;
      case md:
         if(par0 == 0) {
            cfg_val[c_motion_detection] = 0;
            if(system("killall motion") == -1) error("Could not stop Motion", 1);
            printLog("Motion detection stopped\n");
         }
         else {
            cfg_val[c_motion_detection] = 1;
            if(system("motion") == -1) error("Could not start Motion", 1);
            printLog("Motion detection started\n");
         }
         updateStatus();
         break;
      case sc:
         set_counts();
         printLog("Scan for highest count\n");
         break;
      case rs:
         printLog("Reset settings to defaults\n");
         stop_all();
         read_config("/etc/raspimjpeg", 1);
         saveUserConfig(cfg_stru[c_user_config]);
         start_all(0);
         break;
      case bu:
         key = c_video_buffer;
         break;
      default:
         printLog("Unrecognised pipe command\n");
         break;
   }
   
   //Action any key settings
   if (key >= 0) {
      if (key < 1000) {
         addUserValue(key, pars[0]);
         cam_set(key);
      } else {
         addUserValue(key - 1000, parstring);
         cam_set(key - 1000);
      }
   }
   saveUserConfig(cfg_stru[c_user_config]);
   if (parstring != 0) free(parstring);
}
