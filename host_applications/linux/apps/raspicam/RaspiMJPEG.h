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
 * \file RaspiMJPEG.h
 **/
#define VERSION "5.1.3"
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <semaphore.h>
#include <signal.h>
#include <fcntl.h>
#include <time.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#define IFRAME_BUFSIZE (60*1000)
#define STD_INTRAPERIOD 60
extern MMAL_STATUS_T status;
extern MMAL_COMPONENT_T *camera, *jpegencoder, *jpegencoder2, *h264encoder, *resizer;
extern MMAL_CONNECTION_T *con_cam_res, *con_res_jpeg, *con_cam_h264, *con_cam_jpeg;
extern FILE *jpegoutput_file, *jpegoutput2_file, *h264output_file, *status_file;
extern MMAL_POOL_T *pool_jpegencoder, *pool_jpegencoder2, *pool_h264encoder;
extern char *cb_buff;
extern char header_bytes[29];
extern int cb_len, cb_wptr, cb_wrap, cb_data;
extern int iframe_buff[IFRAME_BUFSIZE], iframe_buff_wpos, iframe_buff_rpos, header_wptr;
extern unsigned int tl_cnt, mjpeg_cnt, image_cnt, image2_cnt, lapse_cnt, video_cnt;
extern char *filename_recording;
extern unsigned char timelapse, running, autostart, idle, a_error, v_capturing, i_capturing, v_boxing;
extern unsigned char buffering, buffering_toggle;

//Box file queue
#define MAX_BOX_FILES 32
extern char *box_files[MAX_BOX_FILES];
extern int box_head;
extern int box_tail;

//hold config file data for both dflt and user config files and u long versions
#define KEY_COUNT 64
extern char *cfg_strd[KEY_COUNT + 1];
extern char *cfg_stru[KEY_COUNT + 1];
extern long int cfg_val[KEY_COUNT + 1];
extern char *cfg_key[];

typedef enum cfgkey_type
   {
   c_annotation,c_anno_background,
   c_anno3_custom_background_colour,c_anno3_custom_background_Y,c_anno3_custom_background_U,c_anno3_custom_background_V,
   c_anno3_custom_text_colour,c_anno3_custom_text_Y,c_anno3_custom_text_U,c_anno3_custom_text_V,c_anno_text_size,
   c_sharpness,c_contrast,c_brightness,c_saturation,c_iso,
   c_metering_mode,c_video_stabilisation,c_exposure_compensation,c_exposure_mode,c_white_balance,c_image_effect,
   c_colour_effect_en,c_colour_effect_u,c_colour_effect_v,
   c_rotation,c_hflip,c_vflip,
   c_sensor_region_x,c_sensor_region_y,c_sensor_region_w,c_sensor_region_h,
   c_shutter_speed,c_raw_layer,
   c_width,c_quality,c_divider,
   c_video_width,c_video_height,c_video_fps,c_video_bitrate,c_video_buffer,
   c_MP4Box,c_MP4Box_fps,
   c_image_width,c_image_height,c_image_quality,c_tl_interval,
   c_preview_path,c_image_path,c_lapse_path,c_video_path,c_status_file,c_control_file,c_media_path,c_macros_path,c_subdir_char,
   c_thumb_gen,c_autostart,c_motion_detection,c_user_config,c_log_file,c_watchdog_interval,c_watchdog_errors
   } cfgkey_type; 

time_t currTime;
struct tm *localTime;

//Utils
void printLog(char *msg, ...);
void updateStatus();
void error (const char *string, char fatal);
int findNextCount(char* folder, char* source);
char* trim(char*s);
void makeFilename(char** filename, char *template);
void createMediaPath(char* filename);
int copy_file(char *from_filename, char *to_filename);
time_t get_mtime(const char *path);

//Camera
void cam_set_annotationV3 (char *filename_temp, MMAL_BOOL_T enable);
void cam_set_annotation();
void thumb_create(char *from_filename, char source);
void capt_img (void);
void start_video(unsigned char prepare_buf);
void stop_video(unsigned char stop_buf);
void cam_stop_buffering ();
void cam_set_buffer ();
void cam_set_ip ();
void cam_set_em ();
void cam_set_wb ();
void cam_set_mm ();
void cam_set_ie ();
void cam_set_ce ();
void cam_set_flip ();
void cam_set_roi ();
void cam_set(int key);
void start_all (int load_conf);
void stop_all (void);

//Cmds
void process_cmd(char *readbuf, int length);
void exec_macro(char *macro);

//Main
void set_counts();
int getKey(char *key);
void addValue(int keyI, char *value, int both);
void addUserValue(int key, char *value);
void saveUserConfig(char *cfilename);
void read_config(char *cfilename, int type);
void monitor();

