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
 * Also optionally stream a preview of current camera input wth MJPEG.
 *
 * \date 25th Nov 2013
 * \Author: Silvan Melchior
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

#define VERSION "4.4.0"

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

#include "bcm_host.h"
#include "interface/vcos/vcos.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

MMAL_STATUS_T status;
MMAL_COMPONENT_T *camera = 0, *jpegencoder = 0, *jpegencoder2 = 0, *h264encoder = 0, *resizer = 0;
MMAL_CONNECTION_T *con_cam_res, *con_res_jpeg, *con_cam_h264, *con_cam_jpeg;
FILE *jpegoutput_file = NULL, *jpegoutput2_file = NULL, *h264output_file = NULL, *status_file = NULL;
MMAL_POOL_T *pool_jpegencoder, *pool_jpegencoder2, *pool_h264encoder;
unsigned int tl_cnt=0, mjpeg_cnt=0, image_cnt=0, image2_cnt=0, lapse_cnt=0, video_cnt=0;
char *filename_recording = 0;
unsigned char timelapse=0, running=1, autostart=1, idle=0, capturing=0;

//hold config file data for both dflt and user config files and u long versions
#define KEY_COUNT 61
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
   "video_width","video_height","video_fps","video_bitrate",
   "MP4Box","MP4Box_fps",
   "image_width","image_height","image_quality","tl_interval",
   "preview_path","image_path","lapse_path","video_path","status_file","control_file","media_path","subdir_char",
   "thumb_gen","autostart","motion_detection","user_config","log_file"
};

typedef enum cfgkey_type
   {
   c_annotation,c_anno_background,c_anno_version,
   c_anno3_custom_background_colour,c_anno3_custom_background_Y,c_anno3_custom_background_U,c_anno3_custom_background_V,
   c_anno3_custom_text_colour,c_anno3_custom_text_Y,c_anno3_custom_text_U,c_anno3_custom_text_V,c_anno_text_size,
   c_sharpness,c_contrast,c_brightness,c_saturation,c_iso,
   c_metering_mode,c_video_stabilisation,c_exposure_compensation,c_exposure_mode,c_white_balance,c_image_effect,
   c_colour_effect_en,c_colour_effect_u,c_colour_effect_v,
   c_rotation,c_hflip,c_vflip,
   c_sensor_region_x,c_sensor_region_y,c_sensor_region_w,c_sensor_region_h,
   c_shutter_speed,c_raw_layer,
   c_width,c_quality,c_divider,
   c_video_width,c_video_height,c_video_fps,c_video_bitrate,
   c_MP4Box,c_MP4Box_fps,
   c_image_width,c_image_height,c_image_quality,c_tl_interval,
   c_preview_path,c_image_path,c_lapse_path,c_video_path,c_status_file,c_control_file,c_media_path,c_subdir_char,
   c_thumb_gen,c_autostart,c_motion_detection,c_user_config,c_log_file
   } cfgkey_type; 

time_t currTime;
struct tm *localTime;

void read_config(char *cfilename, int type);
void cam_set_annotation();
void makeFilename(char** filename, char *template);
void createMediaPath(char* filename);
void printLog(char *msg, ...);

void error (const char *string, char fatal) {
   printLog("Error: %s\n", string);
   if (fatal == 0)
      return;
   if(cfg_stru[c_status_file] != 0) {
      status_file = fopen(cfg_stru[c_status_file], "w");
      if(status_file) {
         fprintf(status_file, "Error: %s", string);
         fclose(status_file);
      }
   }
   exit(1);
}

void term (int signum) {
   running = 0;
}

static void camera_control_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {

   if(buffer->cmd != MMAL_EVENT_PARAMETER_CHANGED) error("Camera sent invalid data", 0);
   mmal_buffer_header_release(buffer);

}

static void jpegencoder_buffer_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
   int bytes_written = buffer->length;
   char *filename_temp, *filename_temp2;

   if(mjpeg_cnt == 0) {
      if(!jpegoutput_file) {
         asprintf(&filename_temp, cfg_stru[c_preview_path], image_cnt);
         asprintf(&filename_temp2, "%s.part", filename_temp);
         jpegoutput_file = fopen(filename_temp2, "wb");
         free(filename_temp);
         free(filename_temp2);
         if(jpegoutput_file == NULL) error("Could not open mjpeg-destination", 1);
      }
      if(buffer->length) {
         mmal_buffer_header_mem_lock(buffer);
         if(jpegoutput_file != NULL)
            bytes_written = fwrite(buffer->data, 1, buffer->length, jpegoutput_file);
         else bytes_written = 0;
         mmal_buffer_header_mem_unlock(buffer);
      }
    if(bytes_written != buffer->length) error("Could not write all bytes", 0);
      }
  
   if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) {
      mjpeg_cnt++;
      if(mjpeg_cnt == cfg_val[c_divider]) {
         if(jpegoutput_file != NULL) {
            fclose(jpegoutput_file);
            jpegoutput_file = NULL;
            asprintf(&filename_temp, cfg_stru[c_preview_path], image_cnt);
            asprintf(&filename_temp2, "%s.part", filename_temp);
            rename(filename_temp2, filename_temp);
            free(filename_temp);
            free(filename_temp2);
         }
         image_cnt++;
         mjpeg_cnt = 0;
         cam_set_annotation();
      }
   }

   mmal_buffer_header_release(buffer);

   if (port->is_enabled) {
      MMAL_STATUS_T status = MMAL_SUCCESS;
      MMAL_BUFFER_HEADER_T *new_buffer;

      new_buffer = mmal_queue_get(pool_jpegencoder->queue);

      if (new_buffer) status = mmal_port_send_buffer(port, new_buffer);
      if (!new_buffer || status != MMAL_SUCCESS) error("Could not send buffers to port", 1);
   }
}

static void jpegencoder2_buffer_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {

   int bytes_written = buffer->length;

   if(buffer->length) {
      mmal_buffer_header_mem_lock(buffer);
      if(jpegoutput2_file != NULL)
         bytes_written = fwrite(buffer->data, 1, buffer->length, jpegoutput2_file);
      else
         bytes_written = 0;
      mmal_buffer_header_mem_unlock(buffer);
   }
   if(bytes_written != buffer->length) error("Could not write all bytes", 0);

   if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) {
      if(jpegoutput2_file != NULL) fclose(jpegoutput2_file);
      jpegoutput2_file = NULL;
      if(cfg_stru[c_status_file] != 0) {
         if(!timelapse) {
            status_file = fopen(cfg_stru[c_status_file], "w");
            fprintf(status_file, "ready");
            fclose(status_file);
         }
      }
      if (timelapse && strlen(cfg_stru[c_lapse_path]) > 10)
         lapse_cnt++;
      else
         image2_cnt++;
      capturing = 0;
   }

   mmal_buffer_header_release(buffer);

   if (port->is_enabled) {
      MMAL_STATUS_T status = MMAL_SUCCESS;
      MMAL_BUFFER_HEADER_T *new_buffer;

      new_buffer = mmal_queue_get(pool_jpegencoder2->queue);

      if (new_buffer) status = mmal_port_send_buffer(port, new_buffer);
      if (!new_buffer || status != MMAL_SUCCESS) error("Could not send buffers to port", 1);
   }

}

static void h264encoder_buffer_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)  {

   int bytes_written = buffer->length;

   if(buffer->length) {
      mmal_buffer_header_mem_lock(buffer);
      if(h264output_file != NULL)
         bytes_written = fwrite(buffer->data, 1, buffer->length, h264output_file);
      else
         bytes_written = 0;
      mmal_buffer_header_mem_unlock(buffer);
      if(bytes_written != buffer->length) error("Could not write all bytes", 0);
   }

   mmal_buffer_header_release(buffer);

   if (port->is_enabled) {
      MMAL_STATUS_T status = MMAL_SUCCESS;
      MMAL_BUFFER_HEADER_T *new_buffer;

      new_buffer = mmal_queue_get(pool_h264encoder->queue);

      if (new_buffer) status = mmal_port_send_buffer(port, new_buffer);
      if (!new_buffer || status != MMAL_SUCCESS) error("Could not send buffers to port", 1);
   }

}

void cam_set_em () {
   MMAL_PARAM_EXPOSUREMODE_T mode;
   if(strcmp(cfg_stru[c_exposure_mode], "off") == 0) mode = MMAL_PARAM_EXPOSUREMODE_OFF;
   else if(strcmp(cfg_stru[c_exposure_mode], "auto") == 0) mode = MMAL_PARAM_EXPOSUREMODE_AUTO;
   else if(strcmp(cfg_stru[c_exposure_mode], "night") == 0) mode = MMAL_PARAM_EXPOSUREMODE_NIGHT;
   else if(strcmp(cfg_stru[c_exposure_mode], "nightpreview") == 0) mode = MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW;
   else if(strcmp(cfg_stru[c_exposure_mode], "backlight") == 0) mode = MMAL_PARAM_EXPOSUREMODE_BACKLIGHT;
   else if(strcmp(cfg_stru[c_exposure_mode], "spotlight") == 0) mode = MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT;
   else if(strcmp(cfg_stru[c_exposure_mode], "sports") == 0) mode = MMAL_PARAM_EXPOSUREMODE_SPORTS;
   else if(strcmp(cfg_stru[c_exposure_mode], "snow") == 0) mode = MMAL_PARAM_EXPOSUREMODE_SNOW;
   else if(strcmp(cfg_stru[c_exposure_mode], "beach") == 0) mode = MMAL_PARAM_EXPOSUREMODE_BEACH;
   else if(strcmp(cfg_stru[c_exposure_mode], "verylong") == 0) mode = MMAL_PARAM_EXPOSUREMODE_VERYLONG;
   else if(strcmp(cfg_stru[c_exposure_mode], "fixedfps") == 0) mode = MMAL_PARAM_EXPOSUREMODE_FIXEDFPS;
   else if(strcmp(cfg_stru[c_exposure_mode], "antishake") == 0) mode = MMAL_PARAM_EXPOSUREMODE_ANTISHAKE;
   else if(strcmp(cfg_stru[c_exposure_mode], "fireworks") == 0) mode = MMAL_PARAM_EXPOSUREMODE_FIREWORKS;
   else {error("Invalid exposure mode", 1); return;}
   MMAL_PARAMETER_EXPOSUREMODE_T exp_mode = {{MMAL_PARAMETER_EXPOSURE_MODE,sizeof(exp_mode)}, mode};
   status = mmal_port_parameter_set(camera->control, &exp_mode.hdr);
   if(status != MMAL_SUCCESS) error("Could not set exposure mode", 0);
}

void cam_set_wb () {
   MMAL_PARAM_AWBMODE_T awb_mode;
   if(strcmp(cfg_stru[c_white_balance], "off") == 0) awb_mode = MMAL_PARAM_AWBMODE_OFF;
   else if(strcmp(cfg_stru[c_white_balance], "auto") == 0) awb_mode = MMAL_PARAM_AWBMODE_AUTO;
   else if(strcmp(cfg_stru[c_white_balance], "sun") == 0) awb_mode = MMAL_PARAM_AWBMODE_SUNLIGHT;
   else if(strcmp(cfg_stru[c_white_balance], "cloudy") == 0) awb_mode = MMAL_PARAM_AWBMODE_CLOUDY;
   else if(strcmp(cfg_stru[c_white_balance], "shade") == 0) awb_mode = MMAL_PARAM_AWBMODE_SHADE;
   else if(strcmp(cfg_stru[c_white_balance], "tungsten") == 0) awb_mode = MMAL_PARAM_AWBMODE_TUNGSTEN;
   else if(strcmp(cfg_stru[c_white_balance], "fluorescent") == 0) awb_mode = MMAL_PARAM_AWBMODE_FLUORESCENT;
   else if(strcmp(cfg_stru[c_white_balance], "incandescent") == 0) awb_mode = MMAL_PARAM_AWBMODE_INCANDESCENT;
   else if(strcmp(cfg_stru[c_white_balance], "flash") == 0) awb_mode = MMAL_PARAM_AWBMODE_FLASH;
   else if(strcmp(cfg_stru[c_white_balance], "horizon") == 0) awb_mode = MMAL_PARAM_AWBMODE_HORIZON;
   else {error("Invalid white balance", 0); return;}
   MMAL_PARAMETER_AWBMODE_T param = {{MMAL_PARAMETER_AWB_MODE,sizeof(param)}, awb_mode};
   status = mmal_port_parameter_set(camera->control, &param.hdr);
   if(status != MMAL_SUCCESS) error("Could not set white balance", 0);
}

void cam_set_mm () {
   MMAL_PARAM_EXPOSUREMETERINGMODE_T m_mode;
   if(strcmp(cfg_stru[c_metering_mode], "average") == 0) m_mode = MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
   else if(strcmp(cfg_stru[c_metering_mode], "spot") == 0) m_mode = MMAL_PARAM_EXPOSUREMETERINGMODE_SPOT;
   else if(strcmp(cfg_stru[c_metering_mode], "backlit") == 0) m_mode = MMAL_PARAM_EXPOSUREMETERINGMODE_BACKLIT;
   else if(strcmp(cfg_stru[c_metering_mode], "matrix") == 0) m_mode = MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX;
   else {error("Invalid metering mode", 0); return;}
   MMAL_PARAMETER_EXPOSUREMETERINGMODE_T meter_mode = {{MMAL_PARAMETER_EXP_METERING_MODE,sizeof(meter_mode)}, m_mode};
   status = mmal_port_parameter_set(camera->control, &meter_mode.hdr);
   if(status != MMAL_SUCCESS) error("Could not set metering mode", 0);
}

void cam_set_ie () {
   MMAL_PARAM_IMAGEFX_T imageFX;
   if(strcmp(cfg_stru[c_image_effect], "none") == 0) imageFX = MMAL_PARAM_IMAGEFX_NONE;
   else if(strcmp(cfg_stru[c_image_effect], "negative") == 0) imageFX = MMAL_PARAM_IMAGEFX_NEGATIVE;
   else if(strcmp(cfg_stru[c_image_effect], "solarise") == 0) imageFX = MMAL_PARAM_IMAGEFX_SOLARIZE;
   else if(strcmp(cfg_stru[c_image_effect], "sketch") == 0) imageFX = MMAL_PARAM_IMAGEFX_SKETCH;
   else if(strcmp(cfg_stru[c_image_effect], "denoise") == 0) imageFX = MMAL_PARAM_IMAGEFX_DENOISE;
   else if(strcmp(cfg_stru[c_image_effect], "emboss") == 0) imageFX = MMAL_PARAM_IMAGEFX_EMBOSS;
   else if(strcmp(cfg_stru[c_image_effect], "oilpaint") == 0) imageFX = MMAL_PARAM_IMAGEFX_OILPAINT;
   else if(strcmp(cfg_stru[c_image_effect], "hatch") == 0) imageFX = MMAL_PARAM_IMAGEFX_HATCH;
   else if(strcmp(cfg_stru[c_image_effect], "gpen") == 0) imageFX = MMAL_PARAM_IMAGEFX_GPEN;
   else if(strcmp(cfg_stru[c_image_effect], "pastel") == 0) imageFX = MMAL_PARAM_IMAGEFX_PASTEL;
   else if(strcmp(cfg_stru[c_image_effect], "watercolour") == 0) imageFX = MMAL_PARAM_IMAGEFX_WATERCOLOUR;
   else if(strcmp(cfg_stru[c_image_effect], "film") == 0) imageFX = MMAL_PARAM_IMAGEFX_FILM;
   else if(strcmp(cfg_stru[c_image_effect], "blur") == 0) imageFX = MMAL_PARAM_IMAGEFX_BLUR;
   else if(strcmp(cfg_stru[c_image_effect], "saturation") == 0) imageFX = MMAL_PARAM_IMAGEFX_SATURATION;
   else if(strcmp(cfg_stru[c_image_effect], "colourswap") == 0) imageFX = MMAL_PARAM_IMAGEFX_COLOURSWAP;
   else if(strcmp(cfg_stru[c_image_effect], "washedout") == 0) imageFX = MMAL_PARAM_IMAGEFX_WASHEDOUT;
   else if(strcmp(cfg_stru[c_image_effect], "posterise") == 0) imageFX = MMAL_PARAM_IMAGEFX_POSTERISE;
   else if(strcmp(cfg_stru[c_image_effect], "colourpoint") == 0) imageFX = MMAL_PARAM_IMAGEFX_COLOURPOINT;
   else if(strcmp(cfg_stru[c_image_effect], "colourbalance") == 0) imageFX = MMAL_PARAM_IMAGEFX_COLOURBALANCE;
   else if(strcmp(cfg_stru[c_image_effect], "cartoon") == 0) imageFX = MMAL_PARAM_IMAGEFX_CARTOON;
   else {error("Invalid image effect", 0); return;}
   MMAL_PARAMETER_IMAGEFX_T imgFX = {{MMAL_PARAMETER_IMAGE_EFFECT,sizeof(imgFX)}, imageFX};
   status = mmal_port_parameter_set(camera->control, &imgFX.hdr);
   if(status != MMAL_SUCCESS) error("Could not set image effect", 0);
}

void cam_set_ce () {
   MMAL_PARAMETER_COLOURFX_T colfx = {{MMAL_PARAMETER_COLOUR_EFFECT,sizeof(colfx)}, 0, 0, 0};
   colfx.enable = cfg_val[c_colour_effect_en];
   colfx.u = cfg_val[c_colour_effect_u];
   colfx.v = cfg_val[c_colour_effect_v];
   status = mmal_port_parameter_set(camera->control, &colfx.hdr);
   if(status != MMAL_SUCCESS) error("Could not set exposure compensation", 0);
}

void cam_set_flip () {
   MMAL_PARAMETER_MIRROR_T mirror = {{MMAL_PARAMETER_MIRROR, sizeof(MMAL_PARAMETER_MIRROR_T)}, MMAL_PARAM_MIRROR_NONE};
   if (cfg_val[c_hflip] && cfg_val[c_vflip]) mirror.value = MMAL_PARAM_MIRROR_BOTH;
   else if (cfg_val[c_hflip]) mirror.value = MMAL_PARAM_MIRROR_HORIZONTAL;
   else if (cfg_val[c_vflip]) mirror.value = MMAL_PARAM_MIRROR_VERTICAL;
   status = mmal_port_parameter_set(camera->output[0], &mirror.hdr);
   if(status != MMAL_SUCCESS) {error("Could not set flip (0)", 0); return;}
   status = mmal_port_parameter_set(camera->output[1], &mirror.hdr);
   if(status != MMAL_SUCCESS) {error("Could not set flip (1)", 0); return;}
   status = mmal_port_parameter_set(camera->output[2], &mirror.hdr);
   if(status != MMAL_SUCCESS) {error("Could not set flip (2)", 0); return;}
}

void cam_set_roi () {
   MMAL_PARAMETER_INPUT_CROP_T crop = {{MMAL_PARAMETER_INPUT_CROP, sizeof(MMAL_PARAMETER_INPUT_CROP_T)}};
   crop.rect.x = cfg_val[c_sensor_region_x];
   crop.rect.y = cfg_val[c_sensor_region_y];
   crop.rect.width = cfg_val[c_sensor_region_w];
   crop.rect.height = cfg_val[c_sensor_region_h];
   status = mmal_port_parameter_set(camera->control, &crop.hdr);
   if(status != MMAL_SUCCESS) error("Could not set sensor area", 0);
}

void cam_set(int key) {
   int control = 0;
   unsigned int id;
   MMAL_RATIONAL_T value;
   value.den = 100;
   
   switch(key) {
      case c_sharpness:
         control = 1;id = MMAL_PARAMETER_SHARPNESS;break;
      case c_contrast:
         control = 1;id = MMAL_PARAMETER_CONTRAST;break;
      case c_brightness:
         control = 1;id = MMAL_PARAMETER_BRIGHTNESS;break;
      case c_saturation:
         control = 1;id = MMAL_PARAMETER_SATURATION;break;
      case c_iso:
         control = 3;id = MMAL_PARAMETER_ISO;break;
      case c_video_stabilisation:
         control = 4;id = MMAL_PARAMETER_VIDEO_STABILISATION;break;
      case c_raw_layer:
         control = 4;id = MMAL_PARAMETER_ENABLE_RAW_CAPTURE;break;
      case c_exposure_compensation:
         control = 2;id = MMAL_PARAMETER_EXPOSURE_COMP;break;
      case c_shutter_speed:
         control = 3;id = MMAL_PARAMETER_SHUTTER_SPEED;break;
      case c_rotation:
         status = mmal_port_parameter_set_int32(camera->output[0], MMAL_PARAMETER_ROTATION, cfg_val[c_rotation]);
         if(status != MMAL_SUCCESS) printLog("Could not set rotation (0)\n");
         status = mmal_port_parameter_set_int32(camera->output[1], MMAL_PARAMETER_ROTATION, cfg_val[c_rotation]);
         if(status != MMAL_SUCCESS) printLog("Could not set rotation (1)\n");
         status = mmal_port_parameter_set_int32(camera->output[2], MMAL_PARAMETER_ROTATION, cfg_val[c_rotation]);
         if(status != MMAL_SUCCESS) printLog("Could not set rotation (2)\n");
         break;
      case c_image_quality:
         status = mmal_port_parameter_set_uint32(jpegencoder2->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, cfg_val[c_image_quality]);
         if(status != MMAL_SUCCESS) printLog("Could not set quality\n");
         break;
      case c_video_bitrate:
         h264encoder->output[0]->format->bitrate = cfg_val[c_video_bitrate];
         status = mmal_port_format_commit(h264encoder->output[0]);
         if(status != MMAL_SUCCESS) printLog("Could not set bitrate\n");
         break;
      case c_exposure_mode:
         cam_set_em();
         break;
      case c_white_balance:
         cam_set_wb();
         break;
      case c_metering_mode:
         cam_set_mm();
         break;
      case c_image_effect:
         cam_set_ie();
         break;
      case c_colour_effect_en:
         cam_set_ce();
         break;
      case c_hflip:
         cam_set_flip();
         break;
      case c_sensor_region_x:
         cam_set_roi();
         break;
   }
   
   switch(control) {
      case 1: //rational
         value.num = cfg_val[key];
         status = mmal_port_parameter_set_rational(camera->control, id, value);
         if(status != MMAL_SUCCESS) printLog("Could not set %s\n", cfg_key[key]);
         break;
      case 2: //int32
         status = mmal_port_parameter_set_int32(camera->control, id, cfg_val[key]);
         if(status != MMAL_SUCCESS) printLog("Could not set %s\n", cfg_key[key]);
         break;
      case 3: //uint32
         status = mmal_port_parameter_set_uint32(camera->control, id, cfg_val[key]);
         if(status != MMAL_SUCCESS) printLog("Could not set %s\n", cfg_key[key]);
         break;
      case 4: //boolean
         status = mmal_port_parameter_set_boolean(camera->control, id, cfg_val[key]);
         if(status != MMAL_SUCCESS) printLog("Could not set %s\n", cfg_key[key]);
         break;
   }
}

void cam_set_annotationV2 (char *filename_temp, MMAL_BOOL_T enable) {
   MMAL_PARAMETER_CAMERA_ANNOTATE_V2_T anno = {{MMAL_PARAMETER_ANNOTATE, sizeof(MMAL_PARAMETER_CAMERA_ANNOTATE_V2_T)}};

   if (filename_temp != 0) strcpy(anno.text, filename_temp);
   anno.enable = enable;
   anno.show_shutter = 0;
   anno.show_analog_gain = 0;
   anno.show_lens = 0;
   anno.show_caf = 0;
   anno.show_motion = 0;
   anno.black_text_background = cfg_val[c_anno_background];

   status = mmal_port_parameter_set(camera->control, &anno.hdr);
   if(status != MMAL_SUCCESS) error("Could not set annotation", 0);
}

void cam_set_annotationV3 (char *filename_temp, MMAL_BOOL_T enable) {
   MMAL_PARAMETER_CAMERA_ANNOTATE_V3_T anno = {{MMAL_PARAMETER_ANNOTATE, sizeof(MMAL_PARAMETER_CAMERA_ANNOTATE_V3_T)}};
   
   if (filename_temp != 0) strcpy(anno.text, filename_temp);
   anno.enable = enable;
   anno.show_shutter = 0;
   anno.show_analog_gain = 0;
   anno.show_lens = 0;
   anno.show_caf = 0;
   anno.show_motion = 0;
   anno.enable_text_background = cfg_val[c_anno_background];
   anno.custom_background_colour = cfg_val[c_anno3_custom_background_colour];
   anno.custom_background_Y = cfg_val[c_anno3_custom_background_Y];
   anno.custom_background_U = cfg_val[c_anno3_custom_background_U];
   anno.custom_background_V = cfg_val[c_anno3_custom_background_V];
   anno.custom_text_colour = cfg_val[c_anno3_custom_text_colour];
   anno.custom_text_Y = cfg_val[c_anno3_custom_text_Y];
   anno.custom_text_U = cfg_val[c_anno3_custom_text_U];
   anno.custom_text_V = cfg_val[c_anno3_custom_text_V];
   anno.text_size = cfg_val[c_anno_text_size];
   
   status = mmal_port_parameter_set(camera->control, &anno.hdr);
   if(status != MMAL_SUCCESS) error("Could not set annotation", 0);
}

void cam_set_annotation() {
   char *filename_temp = 0;
   MMAL_BOOL_T enable;
   if(cfg_stru[c_annotation] != 0) {
      currTime = time(NULL);
      localTime = localtime (&currTime);
      makeFilename(&filename_temp, cfg_stru[c_annotation]);
      enable = MMAL_TRUE;
   } else {
      enable = MMAL_FALSE;
   }
   if (cfg_val[c_anno_version] == 3) 
      cam_set_annotationV3(filename_temp, enable);
   else
      cam_set_annotationV2(filename_temp, enable);
   
   if (filename_temp != 0) free(filename_temp);
}

void set_counts() {
   image2_cnt = findNextCount(cfg_stru[c_image_path], "it");
   video_cnt = findNextCount(cfg_stru[c_video_path], "v");
}

void start_all (int load_conf) {
   MMAL_ES_FORMAT_T *format;
   int max, i;

   set_counts();
   //reload config if requested
   if (load_conf != 0) {
      read_config("/etc/raspimjpeg",1);
      if (cfg_stru[c_user_config] != 0)
         read_config(cfg_stru[c_user_config],0);
   }
   //
   // create camera
   //
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
   if(status != MMAL_SUCCESS) error("Could not create camera", 1);
   status = mmal_port_enable(camera->control, camera_control_callback);
   if(status != MMAL_SUCCESS) error("Could not enable camera control port", 1);

   MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
      {MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config)},
      .max_stills_w = cfg_val[c_image_width],
      .max_stills_h = cfg_val[c_image_height],
      .stills_yuv422 = 0,
      .one_shot_stills = 1,
      .max_preview_video_w = cfg_val[c_video_width],
      .max_preview_video_h = cfg_val[c_video_height],
      .num_preview_video_frames = 3,
      .stills_capture_circular_buffer_height = 0,
      .fast_preview_resume = 0,
      .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
   };
   mmal_port_parameter_set(camera->control, &cam_config.hdr);

   format = camera->output[0]->format;
   format->es->video.width = cfg_val[c_video_width];
   format->es->video.height = cfg_val[c_video_height];
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = cfg_val[c_video_width];
   format->es->video.crop.height = cfg_val[c_video_height];
   format->es->video.frame_rate.num = 0;
   format->es->video.frame_rate.den = 1;
   status = mmal_port_format_commit(camera->output[0]);
   if(status != MMAL_SUCCESS) error("Could not set preview format", 1);

   format = camera->output[1]->format;
   format->encoding_variant = MMAL_ENCODING_I420;
   format->encoding = MMAL_ENCODING_OPAQUE;
   format->es->video.width = cfg_val[c_video_width];
   format->es->video.height = cfg_val[c_video_height];
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = cfg_val[c_video_width];
   format->es->video.crop.height = cfg_val[c_video_height];
   format->es->video.frame_rate.num = cfg_val[c_video_fps];
   format->es->video.frame_rate.den = 1;
   status = mmal_port_format_commit(camera->output[1]);
   if(status != MMAL_SUCCESS) error("Could not set video format", 1);
   if(camera->output[1]->buffer_num < 3)
      camera->output[1]->buffer_num = 3;
  
   format = camera->output[2]->format;
   format->encoding = MMAL_ENCODING_OPAQUE;
   format->es->video.width = cfg_val[c_image_width];
   format->es->video.height = cfg_val[c_image_height];
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = cfg_val[c_image_width];
   format->es->video.crop.height = cfg_val[c_image_height];
   format->es->video.frame_rate.num = 0;
   format->es->video.frame_rate.den = 1;
   status = mmal_port_format_commit(camera->output[2]);
   if(status != MMAL_SUCCESS) error("Could not set still format", 1);
   if(camera->output[2]->buffer_num < 3)
      camera->output[2]->buffer_num = 3;

   status = mmal_component_enable(camera);
   if(status != MMAL_SUCCESS) error("Could not enable camera", 1);

   //
   // create jpeg-encoder
   //
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &jpegencoder);
   if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create image encoder", 1);

   mmal_format_copy(jpegencoder->output[0]->format, jpegencoder->input[0]->format);
   jpegencoder->output[0]->format->encoding = MMAL_ENCODING_JPEG;
   jpegencoder->output[0]->buffer_size = jpegencoder->output[0]->buffer_size_recommended;
   if(jpegencoder->output[0]->buffer_size < jpegencoder->output[0]->buffer_size_min)
      jpegencoder->output[0]->buffer_size = jpegencoder->output[0]->buffer_size_min;
   jpegencoder->output[0]->buffer_num = jpegencoder->output[0]->buffer_num_recommended;
   if(jpegencoder->output[0]->buffer_num < jpegencoder->output[0]->buffer_num_min)
      jpegencoder->output[0]->buffer_num = jpegencoder->output[0]->buffer_num_min;
   status = mmal_port_format_commit(jpegencoder->output[0]);
   if(status != MMAL_SUCCESS) error("Could not set image format", 1);
   status = mmal_port_parameter_set_uint32(jpegencoder->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, cfg_val[c_quality]);
   if(status != MMAL_SUCCESS) error("Could not set jpeg quality", 1);

   status = mmal_component_enable(jpegencoder);
   if(status != MMAL_SUCCESS) error("Could not enable image encoder", 1);
   pool_jpegencoder = mmal_port_pool_create(jpegencoder->output[0], jpegencoder->output[0]->buffer_num, jpegencoder->output[0]->buffer_size);
   if(!pool_jpegencoder) error("Could not create image buffer pool", 1);

   //
   // create second jpeg-encoder
   //
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &jpegencoder2);
   if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create image encoder 2", 1);

   mmal_format_copy(jpegencoder2->output[0]->format, jpegencoder2->input[0]->format);
   jpegencoder2->output[0]->format->encoding = MMAL_ENCODING_JPEG;
   jpegencoder2->output[0]->buffer_size = jpegencoder2->output[0]->buffer_size_recommended;
   if(jpegencoder2->output[0]->buffer_size < jpegencoder2->output[0]->buffer_size_min)
      jpegencoder2->output[0]->buffer_size = jpegencoder2->output[0]->buffer_size_min;
   jpegencoder2->output[0]->buffer_num = jpegencoder2->output[0]->buffer_num_recommended;
   if(jpegencoder2->output[0]->buffer_num < jpegencoder2->output[0]->buffer_num_min)
      jpegencoder2->output[0]->buffer_num = jpegencoder2->output[0]->buffer_num_min;
   status = mmal_port_format_commit(jpegencoder2->output[0]);
   if(status != MMAL_SUCCESS) error("Could not set image format 2", 1);
   status = mmal_port_parameter_set_uint32(jpegencoder2->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, 85);
   if(status != MMAL_SUCCESS) error("Could not set jpeg quality 2", 1);

   status = mmal_component_enable(jpegencoder2);
   if(status != MMAL_SUCCESS) error("Could not enable image encoder 2", 1);
   pool_jpegencoder2 = mmal_port_pool_create(jpegencoder2->output[0], jpegencoder2->output[0]->buffer_num, jpegencoder2->output[0]->buffer_size);
   if(!pool_jpegencoder2) error("Could not create image buffer pool 2", 1);

   //
   // create h264-encoder
   //
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &h264encoder);
   if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create video encoder", 1);

   mmal_format_copy(h264encoder->output[0]->format, h264encoder->input[0]->format);
   h264encoder->output[0]->format->encoding = MMAL_ENCODING_H264;
   h264encoder->output[0]->format->bitrate = 17000000;
   h264encoder->output[0]->buffer_size = h264encoder->output[0]->buffer_size_recommended;
   if(h264encoder->output[0]->buffer_size < h264encoder->output[0]->buffer_size_min)
      h264encoder->output[0]->buffer_size = h264encoder->output[0]->buffer_size_min;
   h264encoder->output[0]->buffer_num = h264encoder->output[0]->buffer_num_recommended;
   if(h264encoder->output[0]->buffer_num < h264encoder->output[0]->buffer_num_min)
      h264encoder->output[0]->buffer_num = h264encoder->output[0]->buffer_num_min;
   h264encoder->output[0]->format->es->video.frame_rate.num = 0;
   h264encoder->output[0]->format->es->video.frame_rate.den = 1;
   status = mmal_port_format_commit(h264encoder->output[0]);
   if(status != MMAL_SUCCESS) error("Could not set video format", 1);

   MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param2)}, 25};
   status = mmal_port_parameter_set(h264encoder->output[0], &param2.hdr);
   if(status != MMAL_SUCCESS) error("Could not set video quantisation", 1);

   MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_QP_P, sizeof(param3)}, 31};
   status = mmal_port_parameter_set(h264encoder->output[0], &param3.hdr);
   if(status != MMAL_SUCCESS) error("Could not set video quantisation", 1);

   MMAL_PARAMETER_VIDEO_PROFILE_T param4;
   param4.hdr.id = MMAL_PARAMETER_PROFILE;
   param4.hdr.size = sizeof(param4);
   param4.profile[0].profile = MMAL_VIDEO_PROFILE_H264_HIGH;
   param4.profile[0].level = MMAL_VIDEO_LEVEL_H264_4;
   status = mmal_port_parameter_set(h264encoder->output[0], &param4.hdr);
   if(status != MMAL_SUCCESS) error("Could not set video port format", 1);

   status = mmal_port_parameter_set_boolean(h264encoder->input[0], MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, 1);
   if(status != MMAL_SUCCESS) error("Could not set immutable flag", 1);

   status = mmal_port_parameter_set_boolean(h264encoder->output[0], MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, 0);
   if(status != MMAL_SUCCESS) error("Could not set inline flag", 1);

   //
   // create image-resizer
   //
   unsigned int height_temp = (unsigned long int)cfg_val[c_width]*cfg_val[c_video_height]/cfg_val[c_video_width];
   height_temp -= height_temp%16;
   status = mmal_component_create("vc.ril.resize", &resizer);
   if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create image resizer", 1);
  
   format = resizer->output[0]->format;
   format->es->video.width = cfg_val[c_width];
   format->es->video.height = height_temp;
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = cfg_val[c_width];
   format->es->video.crop.height = height_temp;
   format->es->video.frame_rate.num = 30;
   format->es->video.frame_rate.den = 1;
   status = mmal_port_format_commit(resizer->output[0]);
   if(status != MMAL_SUCCESS) error("Could not set image resizer output", 1);

   status = mmal_component_enable(resizer);
   if(status != MMAL_SUCCESS) error("Could not enable image resizer", 1);

   //
   // connect
   //
   status = mmal_connection_create(&con_cam_res, camera->output[0], resizer->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
   if(status != MMAL_SUCCESS) error("Could not create connection camera -> resizer", 1);
   status = mmal_connection_enable(con_cam_res);
   if(status != MMAL_SUCCESS) error("Could not enable connection camera -> resizer", 1);
  
   status = mmal_connection_create(&con_res_jpeg, resizer->output[0], jpegencoder->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
   if(status != MMAL_SUCCESS) error("Could not create connection resizer -> encoder", 1);
   status = mmal_connection_enable(con_res_jpeg);
   if(status != MMAL_SUCCESS) error("Could not enable connection resizer -> encoder", 1);

   status = mmal_port_enable(jpegencoder->output[0], jpegencoder_buffer_callback);
   if(status != MMAL_SUCCESS) error("Could not enable jpeg port", 1);
   max = mmal_queue_length(pool_jpegencoder->queue);
   for(i=0;i<max;i++) {
      MMAL_BUFFER_HEADER_T *jpegbuffer = mmal_queue_get(pool_jpegencoder->queue);

      if(!jpegbuffer) error("Could not create jpeg buffer header", 1);
      status = mmal_port_send_buffer(jpegencoder->output[0], jpegbuffer);
      if(status != MMAL_SUCCESS) error("Could not send buffers to jpeg port", 1);
   }

   status = mmal_connection_create(&con_cam_jpeg, camera->output[2], jpegencoder2->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
   if(status != MMAL_SUCCESS) error("Could not create connection camera -> encoder", 1);
   status = mmal_connection_enable(con_cam_jpeg);
   if(status != MMAL_SUCCESS) error("Could not enable connection camera -> encoder", 1);
  
   status = mmal_port_enable(jpegencoder2->output[0], jpegencoder2_buffer_callback);
   if(status != MMAL_SUCCESS) error("Could not enable jpeg port 2", 1);
   max = mmal_queue_length(pool_jpegencoder2->queue);
   for(i=0;i<max;i++) {
      MMAL_BUFFER_HEADER_T *jpegbuffer2 = mmal_queue_get(pool_jpegencoder2->queue);

      if(!jpegbuffer2) error("Could not create jpeg buffer header 2", 1);
      status = mmal_port_send_buffer(jpegencoder2->output[0], jpegbuffer2);
      if(status != MMAL_SUCCESS) error("Could not send buffers to jpeg port 2", 1);
   }
  
   //
   // settings
   //
   cam_set(c_sharpness);
   cam_set(c_contrast);
   cam_set(c_brightness);
   cam_set(c_saturation);
   cam_set(c_iso);
   cam_set(c_video_stabilisation);
   cam_set(c_exposure_compensation);
   cam_set(c_raw_layer);
   cam_set(c_shutter_speed);
   cam_set(c_image_quality);
   cam_set(c_video_bitrate);
   cam_set(c_rotation);
   cam_set(c_exposure_mode);
   cam_set(c_white_balance);
   cam_set(c_metering_mode);
   cam_set(c_image_effect);
   cam_set(c_colour_effect_en);
   cam_set(c_hflip);
   cam_set(c_sensor_region_x);
   cam_set_annotation();
}


void stop_all (void) {
   mmal_port_disable(jpegencoder->output[0]);
   mmal_connection_destroy(con_cam_res);
   mmal_connection_destroy(con_res_jpeg);
   mmal_port_pool_destroy(jpegencoder->output[0], pool_jpegencoder);
   mmal_component_disable(jpegencoder);
   mmal_component_disable(camera);
   mmal_component_destroy(jpegencoder);
   mmal_component_destroy(h264encoder);
   mmal_component_destroy(camera);
}

char* trim(char*s) {
   char *end = s + strlen(s)-1;
   while(*s && isspace(*s))
      *s++ = 0;
   while(isspace(*end))
      *end-- = 0;
   return s;
}

int copy_file(char *from_filename, char *to_filename)
{
   FILE  *fd_from, *fd_to;
   char buffer[4096];
   size_t bytes;

   if ((fd_from = fopen(from_filename, "r")) == NULL)
      return  -1;
   
   if ((fd_to = fopen(to_filename, "w")) == NULL) {
      fclose(fd_from);
      return  -1;
   }

   while (0 < (bytes = fread(buffer, 1, sizeof(buffer), fd_from))) {
       fwrite(buffer, 1, bytes, fd_to);
   }
   fclose(fd_from);
   fclose(fd_to);
   return  0;
}

void createMediaPath(char* filename) {
   char* s;
   char* t;
   int r = 0;
   struct stat buf;
   //Create folders under media in filename as needed
   if (strncmp(filename, cfg_stru[c_media_path], strlen(cfg_stru[c_media_path])) == 0) {
      stat(cfg_stru[c_media_path], &buf);
      //s to trailing path
      s = filename + strlen(cfg_stru[c_media_path]) + 1;
      do {
         t = strchr(s, '/');
         if (t != NULL) {
            *t = 0;
            r = mkdir(filename, 0777);
            if (r !=0 && errno == EEXIST) {
               chmod(filename, 0666);
               r = 0;
            } else if (r == 0) {
               chown(filename, buf.st_uid, buf.st_gid);
            }
            *t = '/';
            s = t + 1;
         }
      } while (t != NULL && r == 0);
   }
}

void makeFilename(char** filename, char *template) {
   //Create filename from template
   const int max_subs = 16;
   char spec[11] = "%YyMDhmsvit";
   char *template1;
   char p[max_subs][10];
   char *s, *e, *f;
   int sp, pi;
   
   memset(p, 0, sizeof p);
   //get copy of template to work with
   asprintf(&template1, "%s", template);
   pi=0;
   //start and end pointers
   s = template1;
   e = template1 + strlen(s) - 1;
   //successively search through template1 for % specifiers
   do {
      s = strchr(s, '%');
      if (s != NULL && s < e) {
         s++;
         //find which specifier it is or default to unknown
         f = strchr(spec, *s);
         if (f == NULL) {
            sp = strlen(spec);
         } else {
            sp = f-spec;
         }
         switch(sp) {
            case 0: sprintf(p[pi], "%s", "%");break;
            case 1: sprintf(p[pi], "%04d", localTime->tm_year+1900);break;
            case 2: sprintf(p[pi], "%02d", (localTime->tm_year+1900) % 100);break;
            case 3: sprintf(p[pi], "%02d", localTime->tm_mon+1);break;
            case 4: sprintf(p[pi], "%02d", localTime->tm_mday);break;
            case 5: sprintf(p[pi], "%02d", localTime->tm_hour);break;
            case 6: sprintf(p[pi], "%02d", localTime->tm_min);break;
            case 7: sprintf(p[pi], "%02d", localTime->tm_sec);break;
            case 8: sprintf(p[pi], "%04d", video_cnt);break;
            case 9: sprintf(p[pi], "%04d", image2_cnt);break;
            case 10: sprintf(p[pi], "%04d", lapse_cnt);break;
         }
         if (pi < (max_subs-1)) pi++;
         *s = 's';
         s++;
      } else {
         break;
      }
   } while(s != NULL);
   
   asprintf(filename, template1, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]); 
   free(template1);
}

void thumb_create(char *from_filename, char source) {
   //make a thumbnail with source+count.th.jpg appended
   char *filename = 0;
   char *thumb_name = 0;
   char *f = 0, *s = 0, *t = 0;
   unsigned int xcount=0;
   
   //Check if thumbnail needed for this type of source
   if (cfg_stru[c_thumb_gen] != 0 && strchr(cfg_stru[c_thumb_gen], source) != NULL) {
      if (source == 'v')
         xcount = video_cnt;
      else if (source == 'i')
         xcount = image2_cnt;
      else if (source == 't')
         xcount = image2_cnt;
      
      asprintf(&filename, "%s", from_filename);
      if (strncmp(filename, cfg_stru[c_media_path], strlen(cfg_stru[c_media_path])) == 0) {
         f = filename + strlen(cfg_stru[c_media_path]) + 1;
         //remove .h264 if present
         if (strcmp((f + strlen(f) - 5), ".h264") == 0) {
            *(f + strlen(f) - 5) = 0; 
         }
         s = f;
         do {
            t = strchr(s, '/');
            if (t != NULL) {
               *t = *cfg_stru[c_subdir_char];
               s = t + 1;
            }
         } while (t != NULL);
         //generate thumbnail name
         asprintf(&thumb_name, "%s/%s.%c%04d.th.jpg", cfg_stru[c_media_path], f, source, xcount);
         copy_file(cfg_stru[c_preview_path], thumb_name);
         free(thumb_name);
      }
      free(filename);
   }
}

void capt_img (void) {

   char *filename_temp;

   currTime = time(NULL);
   localTime = localtime (&currTime);
   if(timelapse && strlen(cfg_stru[c_lapse_path]) > 10) {
      makeFilename(&filename_temp, cfg_stru[c_lapse_path]);
      if (lapse_cnt == 1) {
         //Only first capture of a lapse sequence
         thumb_create(filename_temp, 't');
      }
   } else {
      makeFilename(&filename_temp, cfg_stru[c_image_path]);
      thumb_create(filename_temp, 'i');
   }
   createMediaPath(filename_temp);
   jpegoutput2_file = fopen(filename_temp, "wb");
   free(filename_temp);
   if(jpegoutput2_file != NULL){ 
      status = mmal_port_parameter_set_boolean(camera->output[2], MMAL_PARAMETER_CAPTURE, 1);
      if(status == MMAL_SUCCESS) {
         printLog("Capturing image\n");
         if(cfg_stru[c_status_file] != 0) {
            if(!timelapse) {
               status_file = fopen(cfg_stru[c_status_file], "w");
               fprintf(status_file, "image");
               fclose(status_file);
            }
         }
         capturing = 1;
      } else {
         error("Could not start image capture", 0);
      }
   } else {
      error("Could not open/create image-file", 0);
   }
}

void start_video(void) {
   int i, max;
   char *filename_temp;

   if(!capturing) {
      status = mmal_component_enable(h264encoder);
      if(status != MMAL_SUCCESS) {error("Could not enable h264encoder", 0); return;}
      pool_h264encoder = mmal_port_pool_create(h264encoder->output[0], h264encoder->output[0]->buffer_num, h264encoder->output[0]->buffer_size);
      if(!pool_h264encoder) {error("Could not create pool", 0); return;}
      status = mmal_connection_create(&con_cam_h264, camera->output[1], h264encoder->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
      if(status != MMAL_SUCCESS) {error("Could not create connection camera -> video converter", 0); return;}
      status = mmal_connection_enable(con_cam_h264);
      if(status != MMAL_SUCCESS) {error("Could not enable connection camera -> video converter", 0); return;}
      currTime = time(NULL);
      localTime = localtime (&currTime);
      if(cfg_val[c_MP4Box] != 0) {
         makeFilename(&filename_recording, cfg_stru[c_video_path]);
         asprintf(&filename_temp, "%s.h264", filename_recording);
      }
      else {
         makeFilename(&filename_temp, cfg_stru[c_video_path]);
      }
      thumb_create(filename_temp, 'v');
      createMediaPath(filename_temp);
      h264output_file = fopen(filename_temp, "wb");
      free(filename_temp);
      if(!h264output_file) {error("Could not open/create video-file", 0); return;}
      status = mmal_port_enable(h264encoder->output[0], h264encoder_buffer_callback);
      if(status != MMAL_SUCCESS) {error("Could not enable video port", 0); return;}
      max = mmal_queue_length(pool_h264encoder->queue);
      for(i=0;i<max;i++) {
         MMAL_BUFFER_HEADER_T *h264buffer = mmal_queue_get(pool_h264encoder->queue);
         if(!h264buffer) {error("Could not create video pool header", 0); return;}
         status = mmal_port_send_buffer(h264encoder->output[0], h264buffer);
         if(status != MMAL_SUCCESS) {error("Could not send buffers to video port", 0); return;}
      }
      mmal_port_parameter_set_boolean(camera->output[1], MMAL_PARAMETER_CAPTURE, 1);
      if(status != MMAL_SUCCESS) {error("Could not start capture", 0); return;}
      printLog("Capturing started\n");
      if(cfg_stru[c_status_file] != 0) {
         status_file = fopen(cfg_stru[c_status_file], "w");
         if(!cfg_val[c_motion_detection]) fprintf(status_file, "video");
         else fprintf(status_file, "md_video");
         fclose(status_file);
      }
      capturing = 1;
   }
}

void stop_video(void) {
   char *filename_temp, *cmd_temp;
   char background;
   if(capturing) {
      mmal_port_parameter_set_boolean(camera->output[1], MMAL_PARAMETER_CAPTURE, 0);
      if(status != MMAL_SUCCESS) error("Could not stop capture", 1);
      status = mmal_port_disable(h264encoder->output[0]);
      if(status != MMAL_SUCCESS) error("Could not disable video port", 1);
      status = mmal_connection_destroy(con_cam_h264);
      if(status != MMAL_SUCCESS) error("Could not destroy connection camera -> video encoder", 1);
      mmal_port_pool_destroy(h264encoder->output[0], pool_h264encoder);
      if(status != MMAL_SUCCESS) error("Could not destroy video buffer pool", 1);
      status = mmal_component_disable(h264encoder);
      if(status != MMAL_SUCCESS) error("Could not disable video converter", 1);
      fclose(h264output_file);
      h264output_file = NULL;
      printLog("Capturing stopped\n");
      if(cfg_val[c_MP4Box]) {
         asprintf(&filename_temp, "%s.h264", filename_recording);
         if(cfg_val[c_MP4Box] == 1) {
            printLog("Boxing started\n");
            status_file = fopen(cfg_stru[c_status_file], "w");
            if(!cfg_val[c_motion_detection]) fprintf(status_file, "boxing");
            else fprintf(status_file, "md_boxing");
            fclose(status_file);
            background = ' ';
         } else {
            background = '&';
         }
         asprintf(&cmd_temp, "(MP4Box -fps %i -add %s.h264 %s > /dev/null;rm \"%s\";) %c", cfg_val[c_MP4Box_fps], filename_recording, filename_recording, filename_temp, background);
         if(cfg_val[c_MP4Box] == 1) {
            if(system(cmd_temp) == -1) error("Could not start MP4Box", 0);
            printLog("Boxing operation stopped\n");
         } else {
            system(cmd_temp);
            printLog("Boxing in background\n");
         }
         free(filename_temp);
         free(filename_recording);
         free(cmd_temp);
      }
      video_cnt++;
      if(cfg_stru[c_status_file] != 0) {
         status_file = fopen(cfg_stru[c_status_file], "w");
         if(!cfg_val[c_motion_detection]) fprintf(status_file, "ready");
         else fprintf(status_file, "md_ready");
         fclose(status_file);
      }
      capturing = 0;
   }
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
   long int val=strtol(value, NULL, 10);;

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


void process_cmd(char *readbuf, int length) {
   typedef enum pipe_cmd_type{ca,im,tl,px,bo,tv,an,av,as,at,ac,ab,sh,co,br,sa,is,vs,rl,ec,em,wb,mm,ie,ce,ro,fl,ri,ss,qu,bl,ru,md,sc,rs} pipe_cmd_type;
   char pipe_cmds[] = "ca,im,tl,px,bo,tv,an,av,as,at,ac,ab,sh,co,br,sa,is,vs,rl,ec,em,wb,mm,ie,ce,ro,fl,ri,ss,qu,bl,ru,md,sc,rs";
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
            start_video();
         }  else {
            stop_video();
         }
         break;
      case im:
         capt_img();
         break;
      case tl:
         if(par0) {
            if(cfg_stru[c_status_file] != 0) {
               status_file = fopen(cfg_stru[c_status_file], "w");
               fprintf(status_file, "timelapse");
               fclose(status_file);
            }
            timelapse = 1;
            lapse_cnt = 1;
            printLog("Timelapse started\n");
         }
         else {
            if(cfg_stru[c_status_file] != 0) {
              status_file = fopen(cfg_stru[c_status_file], "w");
              if(!cfg_val[c_motion_detection]) fprintf(status_file, "ready");
              else fprintf(status_file, "md_ready");
              fclose(status_file);
            }
            image2_cnt++;
            timelapse = 0;
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
            if(cfg_stru[c_status_file] != 0) {
              status_file = fopen(cfg_stru[c_status_file], "w");
              fprintf(status_file, "halted");
              fclose(status_file);
            }
         } else {
            start_all(1);
            idle = 0;
            printLog("Stream continued\n");
            if(cfg_stru[c_status_file] != 0) {
              status_file = fopen(cfg_stru[c_status_file], "w");
              fprintf(status_file, "ready");
              fclose(status_file);
            }
         }
         break;
      case md:
         if(par0 == 0) {
            cfg_val[c_motion_detection] = 0;
            if(system("killall motion") == -1) error("Could not stop Motion", 1);
            printLog("Motion detection stopped\n");
            if(cfg_stru[c_status_file] != 0) {
               status_file = fopen(cfg_stru[c_status_file], "w");
               fprintf(status_file, "ready");
               fclose(status_file);
            }
         }
         else {
            cfg_val[c_motion_detection] = 1;
            if(system("motion") == -1) error("Could not start Motion", 1);
            printLog("Motion detection started\n");
            if(cfg_stru[c_status_file] != 0) {
               status_file = fopen(cfg_stru[c_status_file], "w");
               fprintf(status_file, "md_ready");
               fclose(status_file);
            }
         }
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

int findNextCount(char* folder, char* source) {
   char* search;
   char *s, *e;
   unsigned int current_count, max_count = 0;
   int found = 0;
   DIR *dp;
   struct dirent *fp;
   
   // get working copy of a path
   asprintf(&search,"%s", folder);
   // find base path by searching forward for first %sub then back for /
   s = strchr(search, '%');
   if (s != NULL) {
      *s = 0;
      s = strrchr(search, '/');
      if (s != NULL) {
         //truncate off to get base path and open it
         *s = 0;
         dp = opendir(search);
         if (dp != NULL) {
            //scan the contents
            while ((fp = readdir(dp))) {
               s = fp->d_name;
               // check if name is a thumbnail
               e = s + strlen(s) - 7;
               if (e > s && strcmp(e, ".th.jpg") == 0) {
                  // truncate where number should end
                  *e = 0;
                  //search to find beginning of field
                  s = strrchr(s, '.');
                  if (s != NULL) {
                     //set start to beginning
                     s++;
                     //see if it a comparison type
                     if (strchr(source, *s) != NULL) {
                        //extract number and set maximum
                        found = 1;
                        current_count = strtoul(s+1, &e, 10);
                        if (current_count > max_count) {
                           max_count = current_count;
                        }
                     }
                  }
               }
            }
            closedir (dp);
         }
      }
   }
   free(search);
   return max_count + found;
}

void printLog(char *msg, ...) {
   char *timestamp;
   va_list args;
   va_start(args, msg);
   int nofile;
   FILE *fp;

   if (cfg_stru[c_log_file] != 0) {
      nofile = (access(cfg_stru[c_log_file], F_OK ) == -1 );
      fp = fopen(cfg_stru[c_log_file], "a");
   } else {
      fp = stdout;
   }
   if (fp != NULL) {
      currTime = time(NULL);
      localTime = localtime (&currTime);
      makeFilename(&timestamp, "{%Y/%M/%D %h:%m:%s} ");
      fprintf(fp, "%s",timestamp);
      vfprintf(fp, msg, args);
      if (cfg_stru[c_log_file] != 0) {
         fclose(fp);
         if (nofile) chmod(cfg_stru[c_log_file], 0666);
      }
      if (timestamp != 0) free(timestamp);
   }
   va_end(args);
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
  
   if(cfg_stru[c_status_file] != 0) {
      status_file = fopen(cfg_stru[c_status_file], "w");
      if(!status_file) error("Could not open/create status-file", 1);
      if(cfg_val[c_autostart]) {
         if(!cfg_val[c_motion_detection]) {
            fprintf(status_file, "ready");
         }
         else fprintf(status_file, "md_ready");
      }
      else fprintf(status_file, "halted");
      fclose(status_file);
   }
   
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
            if(capturing == 0) {
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
