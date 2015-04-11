/*
Copyright (c) 2013, Broadcom Europe Ltd
Copyright (c) 2013, Silvan Melchior
Copyright (c) 2013, James Hughes
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
 * \file RaspiMJPEG.c
 * Command line program to capture a camera video stream and encode it to file.
 * Also optionally stream a preview of current camera input with MJPEG.
 *
 * \date 9th Aprl 2015
 * \Author: Silvan Melchior / Robert Tidey
 *
 * Description
 *
 * RaspiMJPEG is an OpenMAX-Application based on the mmal-library, which is
 * comparable to and inspired by RaspiVid and RaspiStill. RaspiMJPEG can record
 * 1080p 30fps videos and 5 Mpx images into a file. But instead of showing the
 * preview on a screen, RaspiMJPEG streams the preview as MJPEG into a file.
 * The update-rate and the size of the preview are customizable with parameters
 * and independent of the image/video. Once started, the application receives
 * commands with a unix-pipe and showes its status on stdout and writes it into
 * a status-file. The program terminates itself after receiving a SIGINT or
 * SIGTERM.
 *
 * Usage information in README_RaspiMJPEG.md
 */

#include "RaspiMJPEG.h"

MMAL_STATUS_T status;
MMAL_COMPONENT_T *camera = 0, *jpegencoder = 0, *jpegencoder2 = 0, *h264encoder = 0, *resizer = 0;
MMAL_CONNECTION_T *con_cam_res, *con_res_jpeg, *con_cam_h264, *con_cam_jpeg;
FILE *jpegoutput_file = NULL, *jpegoutput2_file = NULL, *h264output_file = NULL, *status_file = NULL;
MMAL_POOL_T *pool_jpegencoder, *pool_jpegencoder2, *pool_h264encoder;
char *cb_buff = NULL;
char header_bytes[29];
int cb_len, cb_wptr, cb_wrap, cb_data;
int iframe_buff[IFRAME_BUFSIZE], iframe_buff_wpos, iframe_buff_rpos, header_wptr;
unsigned int tl_cnt=0, mjpeg_cnt=0, image_cnt=0, image2_cnt=0, lapse_cnt=0, video_cnt=0;
char *filename_recording = 0;
unsigned char timelapse=0, running=1, autostart=1, idle=0, a_error=0, v_capturing=0, i_capturing=0, v_boxing=0;
unsigned char buffering=0, buffering_toggle=0;
char *cfg_strd[KEY_COUNT + 1];
char *cfg_stru[KEY_COUNT + 1];
long int cfg_val[KEY_COUNT + 1];

char *cfg_key[] ={
   "annotation","anno_background","anno_version",
   "anno3_custom_background_colour","anno3_custom_background_Y","anno3_custom_background_U","anno3_custom_background_V",
   "anno3_custom_text_colour","anno3_custom_text_Y","anno3_custom_text_U","anno3_custom_text_V","anno_text_size",
   "sharpness","contrast","brightness","saturation","iso",
   "metering_mode","video_stabilisation","exposure_compensation","exposure_mode","white_balance","image_effect",
   "colour_effect_en","colour_effect_u","colour_effect_v",
   "rotation","hflip","vflip",
   "sensor_region_x","sensor_region_y","sensor_region_w","sensor_region_h",
   "shutter_speed","raw_layer",
   "width","quality","divider",
   "video_width","video_height","video_fps","video_bitrate","video_buffer",
   "MP4Box","MP4Box_fps",
   "image_width","image_height","image_quality","tl_interval",
   "preview_path","image_path","lapse_path","video_path","status_file","control_file","media_path","subdir_char",
   "thumb_gen","autostart","motion_detection","user_config","log_file"
};


void term (int signum) {
   running = 0;
}

void set_counts() {
   image2_cnt = findNextCount(cfg_stru[c_image_path], "it");
   video_cnt = findNextCount(cfg_stru[c_video_path], "v");
}

int getKey(char *key) {
   int i;
   for(i=0; i < KEY_COUNT; i++) {
      if(strcmp(key, cfg_key[i]) == 0) {
         break;
      }
   }
   return i;
}

void addValue(int keyI, char *value, int both){
   long int val=strtol(value, NULL, 10);

   if (cfg_stru[keyI] != 0) free(cfg_stru[keyI]);
      asprintf(&cfg_stru[keyI],"%s", value);
   if (both) {
      if (cfg_strd[keyI] != 0) free(cfg_strd[keyI]);
      asprintf(&cfg_strd[keyI],"%s", value);
   }
   if (strcmp(value, "true") == 0)
      val = 1;
   else if (strcmp(value, "false") == 0)
      val = 0;
   switch(keyI) {
      case c_autostart:
         if(strcmp(value, "idle") == 0) {
            val = 0;
            idle = 1;
         }else if(strcmp(value, "standard") == 0) { 
            val = 1;
            idle = 0;
         };
         updateStatus();
         break;
      case c_MP4Box:
         if(strcmp(value, "background") == 0)
            val = 2;
   }
   cfg_val[keyI] = val;
}

void addUserValue(int key, char *value){
   printLog("Change: %s = %s\n", cfg_key[key], value);
   addValue(key, value, 0);
}

void saveUserConfig(char *cfilename) {
   FILE *fp;
   int i;
   fp = fopen(cfilename, "w");
   if(fp != NULL) {
      for(i = 0; i < KEY_COUNT; i++) {
         if(strlen(cfg_key[i]) > 0) {
            if(cfg_stru[i] != 0 && (cfg_strd[i] == 0 || strcmp(cfg_strd[i], cfg_stru[i]) != 0)) {
               fprintf(fp, "%s %s\n", cfg_key[i], cfg_stru[i]);
            }
         }
      }
      fclose(fp);
   }
}

void read_config(char *cfilename, int type) {
   FILE *fp;
   int length;
   unsigned int len = 0;
   char *line = NULL;
   char *value = NULL;

   fp = fopen(cfilename, "r");
   if(fp != NULL) {
      while((length = getline(&line, &len, fp)) != -1) {
         line[length-1] = 0;
         value = strchr(line, ' ');
         if (value != NULL) {
            // split line into key, value
            *value = 0;
            value++;
            value = trim(value);
            if (strlen(line) > 0 && *line != '#' && strlen(value) > 0) {
               addValue(getKey(line), value, type);
            }
         }
      }
      if(line) free(line);
   }   
}

int main (int argc, char* argv[]) {
   int i, fd, length;
   char readbuf[60];

   bcm_host_init();
   //
   // read arguments
   //
   for(i=1; i<argc; i++) {
      if(strcmp(argv[i], "--version") == 0) {
         printf("RaspiMJPEG Version %s\n", VERSION);
         exit(0);
      }
      else if(strcmp(argv[i], "-md") == 0) {
         cfg_val[c_motion_detection] = 1;
      }
   }

   //default base media path
   asprintf(&cfg_stru[c_media_path], "%s", "/var/www/media");
   
   //
   // read configs and init
   //
   read_config("/etc/raspimjpeg", 1);
   if (cfg_stru[c_user_config] != 0)
      read_config(cfg_stru[c_user_config], 0);

   printLog("RaspiMJPEG Version %s\n", VERSION);
   
   if(cfg_val[c_autostart]) start_all(0);
   if(cfg_val[c_motion_detection]) {
      if(system("motion") == -1) error("Could not start Motion", 1);
   }

   //
   // run
   //
   if(cfg_val[c_autostart]) {
      if(cfg_stru[c_control_file] != 0) printLog("MJPEG streaming, ready to receive commands\n");
      else printLog("MJPEG streaming\n");
   }
   else {
      if(cfg_stru[c_control_file] != 0) printLog("MJPEG idle, ready to receive commands\n");
      else printLog("MJPEG idle\n");
   }

   struct sigaction action;
   memset(&action, 0, sizeof(struct sigaction));
   action.sa_handler = term;
   sigaction(SIGTERM, &action, NULL);
   sigaction(SIGINT, &action, NULL);
   
   //Clear out anything in FIFO first
   do {
      fd = open(cfg_stru[c_control_file], O_RDONLY | O_NONBLOCK);
      if(fd < 0) error("Could not open PIPE", 1);
      fcntl(fd, F_SETFL, 0);
      length = read(fd, readbuf, 60);
      close(fd);
   } while (length != 0); 
  
   // Main forever loop
   while(running) {
      if(cfg_stru[c_control_file] != 0) {

         fd = open(cfg_stru[c_control_file], O_RDONLY | O_NONBLOCK);
         if(fd < 0) error("Could not open PIPE", 1);
         fcntl(fd, F_SETFL, 0);
         length = read(fd, readbuf, 60);
         close(fd);

         if(length) {
            process_cmd(readbuf, length);
         }

      }
      if(timelapse) {
         tl_cnt++;
         if(tl_cnt >= cfg_val[c_tl_interval]) {
            if(i_capturing == 0) {
               capt_img();
               tl_cnt = 0;
            }
         }
      }
      usleep(100000);
   }
  
   printLog("SIGINT/SIGTERM received, stopping\n");
   //
   // tidy up
   //
   if(!idle) stop_all();
   return 0;
}
