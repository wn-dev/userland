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

#define VERSION "4.2.3"

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
unsigned int tl_cnt=0, mjpeg_cnt=0, width=320, divider=5, image_cnt=0, image2_cnt=0, lapse_cnt=0, video_cnt=0, mp4box=0;
unsigned int cam_setting_sharpness=0, cam_setting_contrast=0, cam_setting_brightness=50, cam_setting_saturation=0, cam_setting_iso=0, cam_setting_vs=0, cam_setting_ec=0, cam_setting_rotation=0, cam_setting_quality=85, cam_setting_raw=0, cam_setting_ce_en=0, cam_setting_ce_u=128, cam_setting_ce_v=128, cam_setting_hflip=0, cam_setting_vflip=0, cam_setting_annback=0;
char cam_setting_em[20]="auto", cam_setting_wb[20]="auto", cam_setting_ie[20]="none", cam_setting_mm[20]="average", subdir_char='@';
unsigned long int cam_setting_bitrate=17000000, cam_setting_roi_x=0, cam_setting_roi_y=0, cam_setting_roi_w=65536, cam_setting_roi_h=65536, cam_setting_ss=0;
unsigned int video_width=1920, video_height=1080, video_fps=25, MP4Box_fps=25, image_width=2592, image_height=1944;
char *jpeg_filename = 0, *jpeg2_filename = 0, *lapse_filename = 0, *h264_filename = 0, *pipe_filename = 0, *status_filename = 0, *cam_setting_annotation = 0, *thumb_gen = 0, *user_config = 0, *media_path = 0, *filename_recording = 0;
unsigned char timelapse=0, running=1, autostart=1, quality=85, idle=0, capturing=0, motion_detection=0;
//v3 annotation
unsigned int anno_version = 2, anno3_text_size = 0, anno3_custom_background[4]={0,0,128,128}, anno3_custom_text[4]={0,255,128,128};

int time_between_pic;
time_t currTime;
struct tm *localTime;

void read_config(char *cfilename);
void cam_set_annotation();
void makeFilename(char** filename, char *template);
void createMediaPath(char* filename);
  
void error (const char *string) {
   fprintf(stderr, "Error: %s\n", string);
   if(status_filename != 0) {
      status_file = fopen(status_filename, "w");
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

   if(buffer->cmd != MMAL_EVENT_PARAMETER_CHANGED) error("Camera sent invalid data");
   mmal_buffer_header_release(buffer);

}

static void jpegencoder_buffer_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
   int bytes_written = buffer->length;
   char *filename_temp, *filename_temp2;

   if(mjpeg_cnt == 0) {
      if(!jpegoutput_file) {
         asprintf(&filename_temp, jpeg_filename, image_cnt);
         asprintf(&filename_temp2, "%s.part", filename_temp);
         jpegoutput_file = fopen(filename_temp2, "wb");
         free(filename_temp);
         free(filename_temp2);
         if(!jpegoutput_file) error("Could not open mjpeg-destination");
      }
      if(buffer->length) {
         mmal_buffer_header_mem_lock(buffer);
         bytes_written = fwrite(buffer->data, 1, buffer->length, jpegoutput_file);
         mmal_buffer_header_mem_unlock(buffer);
      }
    if(bytes_written != buffer->length) error("Could not write all bytes");
      }
  
   if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) {
      mjpeg_cnt++;
      if(mjpeg_cnt == divider) {
         fclose(jpegoutput_file);
         jpegoutput_file = NULL;
         asprintf(&filename_temp, jpeg_filename, image_cnt);
         asprintf(&filename_temp2, "%s.part", filename_temp);
         rename(filename_temp2, filename_temp);
         free(filename_temp);
         free(filename_temp2);
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
      if (!new_buffer || status != MMAL_SUCCESS) error("Could not send buffers to port");
   }

}

static void jpegencoder2_buffer_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {

   int bytes_written = buffer->length;

   if(buffer->length) {
      mmal_buffer_header_mem_lock(buffer);
      bytes_written = fwrite(buffer->data, 1, buffer->length, jpegoutput2_file);
      mmal_buffer_header_mem_unlock(buffer);
   }
   if(bytes_written != buffer->length) error("Could not write all bytes");

   if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) {
      fclose(jpegoutput2_file);
      if(status_filename != 0) {
         if(!timelapse) {
            status_file = fopen(status_filename, "w");
            fprintf(status_file, "ready");
            fclose(status_file);
         }
      }
      if (timelapse && strlen(lapse_filename) > 10)
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
      if (!new_buffer || status != MMAL_SUCCESS) error("Could not send buffers to port");
   }

}

static void h264encoder_buffer_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)  {

   int bytes_written = buffer->length;

   if(buffer->length) {
      mmal_buffer_header_mem_lock(buffer);
      bytes_written = fwrite(buffer->data, 1, buffer->length, h264output_file);
      mmal_buffer_header_mem_unlock(buffer);
      if(bytes_written != buffer->length) error("Could not write all bytes");
   }

   mmal_buffer_header_release(buffer);

   if (port->is_enabled) {
      MMAL_STATUS_T status = MMAL_SUCCESS;
      MMAL_BUFFER_HEADER_T *new_buffer;

      new_buffer = mmal_queue_get(pool_h264encoder->queue);

      if (new_buffer) status = mmal_port_send_buffer(port, new_buffer);
      if (!new_buffer || status != MMAL_SUCCESS) error("Could not send buffers to port");
   }

}

void cam_set_sharpness () {
   MMAL_RATIONAL_T value = {cam_setting_sharpness, 100};
   status = mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_SHARPNESS, value);
   if(status != MMAL_SUCCESS) error("Could not set sharpness");
}

void cam_set_contrast () {
   MMAL_RATIONAL_T value = {cam_setting_contrast, 100};
   status = mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_CONTRAST, value);
   if(status != MMAL_SUCCESS) error("Could not set contrast");
}

void cam_set_brightness () {
   MMAL_RATIONAL_T value = {cam_setting_brightness, 100};
   status = mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_BRIGHTNESS, value);
   if(status != MMAL_SUCCESS) error("Could not set brightness");
}

void cam_set_saturation () {
   MMAL_RATIONAL_T value = {cam_setting_saturation, 100};
   status = mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_SATURATION, value);
   if(status != MMAL_SUCCESS) error("Could not set saturation");
}

void cam_set_iso () {
   status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_ISO, cam_setting_iso);
   if(status != MMAL_SUCCESS) error("Could not set ISO");
}

void cam_set_vs () {
   status = mmal_port_parameter_set_boolean(camera->control, MMAL_PARAMETER_VIDEO_STABILISATION, cam_setting_vs);
   if(status != MMAL_SUCCESS) error("Could not set video stabilisation");
}

void cam_set_ec () {
   status = mmal_port_parameter_set_int32(camera->control, MMAL_PARAMETER_EXPOSURE_COMP, cam_setting_ec);
   if(status != MMAL_SUCCESS) error("Could not set exposure compensation");
}

void cam_set_em () {
   MMAL_PARAM_EXPOSUREMODE_T mode;
   if(strcmp(cam_setting_em, "off") == 0) mode = MMAL_PARAM_EXPOSUREMODE_OFF;
   else if(strcmp(cam_setting_em, "auto") == 0) mode = MMAL_PARAM_EXPOSUREMODE_AUTO;
   else if(strcmp(cam_setting_em, "night") == 0) mode = MMAL_PARAM_EXPOSUREMODE_NIGHT;
   else if(strcmp(cam_setting_em, "nightpreview") == 0) mode = MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW;
   else if(strcmp(cam_setting_em, "backlight") == 0) mode = MMAL_PARAM_EXPOSUREMODE_BACKLIGHT;
   else if(strcmp(cam_setting_em, "spotlight") == 0) mode = MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT;
   else if(strcmp(cam_setting_em, "sports") == 0) mode = MMAL_PARAM_EXPOSUREMODE_SPORTS;
   else if(strcmp(cam_setting_em, "snow") == 0) mode = MMAL_PARAM_EXPOSUREMODE_SNOW;
   else if(strcmp(cam_setting_em, "beach") == 0) mode = MMAL_PARAM_EXPOSUREMODE_BEACH;
   else if(strcmp(cam_setting_em, "verylong") == 0) mode = MMAL_PARAM_EXPOSUREMODE_VERYLONG;
   else if(strcmp(cam_setting_em, "fixedfps") == 0) mode = MMAL_PARAM_EXPOSUREMODE_FIXEDFPS;
   else if(strcmp(cam_setting_em, "antishake") == 0) mode = MMAL_PARAM_EXPOSUREMODE_ANTISHAKE;
   else if(strcmp(cam_setting_em, "fireworks") == 0) mode = MMAL_PARAM_EXPOSUREMODE_FIREWORKS;
   else error("Invalid exposure mode");
   MMAL_PARAMETER_EXPOSUREMODE_T exp_mode = {{MMAL_PARAMETER_EXPOSURE_MODE,sizeof(exp_mode)}, mode};
   status = mmal_port_parameter_set(camera->control, &exp_mode.hdr);
   if(status != MMAL_SUCCESS) error("Could not set exposure mode");
}

void cam_set_wb () {
   MMAL_PARAM_AWBMODE_T awb_mode;
   if(strcmp(cam_setting_wb, "off") == 0) awb_mode = MMAL_PARAM_AWBMODE_OFF;
   else if(strcmp(cam_setting_wb, "auto") == 0) awb_mode = MMAL_PARAM_AWBMODE_AUTO;
   else if(strcmp(cam_setting_wb, "sun") == 0) awb_mode = MMAL_PARAM_AWBMODE_SUNLIGHT;
   else if(strcmp(cam_setting_wb, "cloudy") == 0) awb_mode = MMAL_PARAM_AWBMODE_CLOUDY;
   else if(strcmp(cam_setting_wb, "shade") == 0) awb_mode = MMAL_PARAM_AWBMODE_SHADE;
   else if(strcmp(cam_setting_wb, "tungsten") == 0) awb_mode = MMAL_PARAM_AWBMODE_TUNGSTEN;
   else if(strcmp(cam_setting_wb, "fluorescent") == 0) awb_mode = MMAL_PARAM_AWBMODE_FLUORESCENT;
   else if(strcmp(cam_setting_wb, "incandescent") == 0) awb_mode = MMAL_PARAM_AWBMODE_INCANDESCENT;
   else if(strcmp(cam_setting_wb, "flash") == 0) awb_mode = MMAL_PARAM_AWBMODE_FLASH;
   else if(strcmp(cam_setting_wb, "horizon") == 0) awb_mode = MMAL_PARAM_AWBMODE_HORIZON;
   else error("Invalid white balance");
   MMAL_PARAMETER_AWBMODE_T param = {{MMAL_PARAMETER_AWB_MODE,sizeof(param)}, awb_mode};
   status = mmal_port_parameter_set(camera->control, &param.hdr);
   if(status != MMAL_SUCCESS) error("Could not set white balance");
}

void cam_set_mm () {
   MMAL_PARAM_EXPOSUREMETERINGMODE_T m_mode;
   if(strcmp(cam_setting_mm, "average") == 0) m_mode = MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
   else if(strcmp(cam_setting_mm, "spot") == 0) m_mode = MMAL_PARAM_EXPOSUREMETERINGMODE_SPOT;
   else if(strcmp(cam_setting_mm, "backlit") == 0) m_mode = MMAL_PARAM_EXPOSUREMETERINGMODE_BACKLIT;
   else if(strcmp(cam_setting_mm, "matrix") == 0) m_mode = MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX;
   else error("Invalid metering mode");
   MMAL_PARAMETER_EXPOSUREMETERINGMODE_T meter_mode = {{MMAL_PARAMETER_EXP_METERING_MODE,sizeof(meter_mode)}, m_mode};
   status = mmal_port_parameter_set(camera->control, &meter_mode.hdr);
   if(status != MMAL_SUCCESS) error("Could not set metering mode");
}

void cam_set_ie () {
   MMAL_PARAM_IMAGEFX_T imageFX;
   if(strcmp(cam_setting_ie, "none") == 0) imageFX = MMAL_PARAM_IMAGEFX_NONE;
   else if(strcmp(cam_setting_ie, "negative") == 0) imageFX = MMAL_PARAM_IMAGEFX_NEGATIVE;
   else if(strcmp(cam_setting_ie, "solarise") == 0) imageFX = MMAL_PARAM_IMAGEFX_SOLARIZE;
   else if(strcmp(cam_setting_ie, "sketch") == 0) imageFX = MMAL_PARAM_IMAGEFX_SKETCH;
   else if(strcmp(cam_setting_ie, "denoise") == 0) imageFX = MMAL_PARAM_IMAGEFX_DENOISE;
   else if(strcmp(cam_setting_ie, "emboss") == 0) imageFX = MMAL_PARAM_IMAGEFX_EMBOSS;
   else if(strcmp(cam_setting_ie, "oilpaint") == 0) imageFX = MMAL_PARAM_IMAGEFX_OILPAINT;
   else if(strcmp(cam_setting_ie, "hatch") == 0) imageFX = MMAL_PARAM_IMAGEFX_HATCH;
   else if(strcmp(cam_setting_ie, "gpen") == 0) imageFX = MMAL_PARAM_IMAGEFX_GPEN;
   else if(strcmp(cam_setting_ie, "pastel") == 0) imageFX = MMAL_PARAM_IMAGEFX_PASTEL;
   else if(strcmp(cam_setting_ie, "watercolour") == 0) imageFX = MMAL_PARAM_IMAGEFX_WATERCOLOUR;
   else if(strcmp(cam_setting_ie, "film") == 0) imageFX = MMAL_PARAM_IMAGEFX_FILM;
   else if(strcmp(cam_setting_ie, "blur") == 0) imageFX = MMAL_PARAM_IMAGEFX_BLUR;
   else if(strcmp(cam_setting_ie, "saturation") == 0) imageFX = MMAL_PARAM_IMAGEFX_SATURATION;
   else if(strcmp(cam_setting_ie, "colourswap") == 0) imageFX = MMAL_PARAM_IMAGEFX_COLOURSWAP;
   else if(strcmp(cam_setting_ie, "washedout") == 0) imageFX = MMAL_PARAM_IMAGEFX_WASHEDOUT;
   else if(strcmp(cam_setting_ie, "posterise") == 0) imageFX = MMAL_PARAM_IMAGEFX_POSTERISE;
   else if(strcmp(cam_setting_ie, "colourpoint") == 0) imageFX = MMAL_PARAM_IMAGEFX_COLOURPOINT;
   else if(strcmp(cam_setting_ie, "colourbalance") == 0) imageFX = MMAL_PARAM_IMAGEFX_COLOURBALANCE;
   else if(strcmp(cam_setting_ie, "cartoon") == 0) imageFX = MMAL_PARAM_IMAGEFX_CARTOON;
   else error("Invalid image effect");
   MMAL_PARAMETER_IMAGEFX_T imgFX = {{MMAL_PARAMETER_IMAGE_EFFECT,sizeof(imgFX)}, imageFX};
   status = mmal_port_parameter_set(camera->control, &imgFX.hdr);
   if(status != MMAL_SUCCESS) error("Could not set image effect");
}

void cam_set_ce () {
   MMAL_PARAMETER_COLOURFX_T colfx = {{MMAL_PARAMETER_COLOUR_EFFECT,sizeof(colfx)}, 0, 0, 0};
   colfx.enable = cam_setting_ce_en;
   colfx.u = cam_setting_ce_u;
   colfx.v = cam_setting_ce_v;
   status = mmal_port_parameter_set(camera->control, &colfx.hdr);
   if(status != MMAL_SUCCESS) error("Could not set exposure compensation");
}

void cam_set_rotation () {
   status = mmal_port_parameter_set_int32(camera->output[0], MMAL_PARAMETER_ROTATION, cam_setting_rotation);
   if(status != MMAL_SUCCESS) error("Could not set rotation (0)");
   status = mmal_port_parameter_set_int32(camera->output[1], MMAL_PARAMETER_ROTATION, cam_setting_rotation);
   if(status != MMAL_SUCCESS) error("Could not set rotation (1)");
   status = mmal_port_parameter_set_int32(camera->output[2], MMAL_PARAMETER_ROTATION, cam_setting_rotation);
   if(status != MMAL_SUCCESS) error("Could not set rotation (2)");
}

void cam_set_flip () {
   MMAL_PARAMETER_MIRROR_T mirror = {{MMAL_PARAMETER_MIRROR, sizeof(MMAL_PARAMETER_MIRROR_T)}, MMAL_PARAM_MIRROR_NONE};
   if (cam_setting_hflip && cam_setting_vflip) mirror.value = MMAL_PARAM_MIRROR_BOTH;
   else if (cam_setting_hflip) mirror.value = MMAL_PARAM_MIRROR_HORIZONTAL;
   else if (cam_setting_vflip) mirror.value = MMAL_PARAM_MIRROR_VERTICAL;
   status = mmal_port_parameter_set(camera->output[0], &mirror.hdr);
   if(status != MMAL_SUCCESS) error("Could not set flip (0)");
   status = mmal_port_parameter_set(camera->output[1], &mirror.hdr);
   if(status != MMAL_SUCCESS) error("Could not set flip (1)");
   status = mmal_port_parameter_set(camera->output[2], &mirror.hdr);
   if(status != MMAL_SUCCESS) error("Could not set flip (2)");
}

void cam_set_roi () {
   MMAL_PARAMETER_INPUT_CROP_T crop = {{MMAL_PARAMETER_INPUT_CROP, sizeof(MMAL_PARAMETER_INPUT_CROP_T)}};
   crop.rect.x = cam_setting_roi_x;
   crop.rect.y = cam_setting_roi_y;
   crop.rect.width = cam_setting_roi_w;
   crop.rect.height = cam_setting_roi_h;
   status = mmal_port_parameter_set(camera->control, &crop.hdr);
   if(status != MMAL_SUCCESS) error("Could not set sensor area");
}

void cam_set_ss () {
   status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_SHUTTER_SPEED, cam_setting_ss);
   if(status != MMAL_SUCCESS) error("Could not set shutter speed");
}

void cam_set_quality () {
   status = mmal_port_parameter_set_uint32(jpegencoder2->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, cam_setting_quality);
   if(status != MMAL_SUCCESS) error("Could not set quality");
}

void cam_set_raw () {
   status = mmal_port_parameter_set_boolean(camera->output[2], MMAL_PARAMETER_ENABLE_RAW_CAPTURE, cam_setting_raw);
   if(status != MMAL_SUCCESS) error("Could not set raw layer");
}

void cam_set_bitrate () {
   h264encoder->output[0]->format->bitrate = cam_setting_bitrate;
   status = mmal_port_format_commit(h264encoder->output[0]);
   if(status != MMAL_SUCCESS) error("Could not set bitrate");
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
   anno.black_text_background = cam_setting_annback;

   status = mmal_port_parameter_set(camera->control, &anno.hdr);
   if(status != MMAL_SUCCESS) error("Could not set annotation");
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
   anno.enable_text_background = cam_setting_annback;
   anno.custom_background_colour = anno3_custom_background[0];
   anno.custom_background_Y = anno3_custom_background[1];
   anno.custom_background_U = anno3_custom_background[2];
   anno.custom_background_V = anno3_custom_background[3];
   anno.custom_text_colour = anno3_custom_text[0];
   anno.custom_text_Y = anno3_custom_text[1];
   anno.custom_text_U = anno3_custom_text[2];
   anno.custom_text_V = anno3_custom_text[3];
   anno.text_size = anno3_text_size;

   status = mmal_port_parameter_set(camera->control, &anno.hdr);
   if(status != MMAL_SUCCESS) error("Could not set annotation");
}

void cam_set_annotation() {
   char *filename_temp = 0;
   MMAL_BOOL_T enable;
   if(cam_setting_annotation != 0) {
      currTime = time(NULL);
      localTime = localtime (&currTime);
      makeFilename(&filename_temp, cam_setting_annotation);
      enable = MMAL_TRUE;
   } else {
      enable = MMAL_FALSE;
   }
   if (anno_version == 3) 
      cam_set_annotationV3(filename_temp, enable);
   else
      cam_set_annotationV2(filename_temp, enable);
   
   if (filename_temp != 0) free(filename_temp);
}

void start_all (int load_conf) {
   MMAL_ES_FORMAT_T *format;
   int max, i;

   //reload config if requested
   if (load_conf != 0) {
      read_config("/etc/raspimjpeg");
      if (user_config != 0)
         read_config(user_config);
   }
   //
   // create camera
   //
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
   if(status != MMAL_SUCCESS) error("Could not create camera");
   status = mmal_port_enable(camera->control, camera_control_callback);
   if(status != MMAL_SUCCESS) error("Could not enable camera control port");

   MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
      {MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config)},
      .max_stills_w = image_width,
      .max_stills_h = image_height,
      .stills_yuv422 = 0,
      .one_shot_stills = 1,
      .max_preview_video_w = video_width,
      .max_preview_video_h = video_height,
      .num_preview_video_frames = 3,
      .stills_capture_circular_buffer_height = 0,
      .fast_preview_resume = 0,
      .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
   };
   mmal_port_parameter_set(camera->control, &cam_config.hdr);

   format = camera->output[0]->format;
   format->es->video.width = video_width;
   format->es->video.height = video_height;
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = video_width;
   format->es->video.crop.height = video_height;
   format->es->video.frame_rate.num = 0;
   format->es->video.frame_rate.den = 1;
   status = mmal_port_format_commit(camera->output[0]);
   if(status != MMAL_SUCCESS) error("Coult not set preview format");

   format = camera->output[1]->format;
   format->encoding_variant = MMAL_ENCODING_I420;
   format->encoding = MMAL_ENCODING_OPAQUE;
   format->es->video.width = video_width;
   format->es->video.height = video_height;
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = video_width;
   format->es->video.crop.height = video_height;
   format->es->video.frame_rate.num = video_fps;
   format->es->video.frame_rate.den = 1;
   status = mmal_port_format_commit(camera->output[1]);
   if(status != MMAL_SUCCESS) error("Could not set video format");
   if(camera->output[1]->buffer_num < 3)
      camera->output[1]->buffer_num = 3;
  
   format = camera->output[2]->format;
   format->encoding = MMAL_ENCODING_OPAQUE;
   format->es->video.width = image_width;
   format->es->video.height = image_height;
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = image_width;
   format->es->video.crop.height = image_height;
   format->es->video.frame_rate.num = 0;
   format->es->video.frame_rate.den = 1;
   status = mmal_port_format_commit(camera->output[2]);
   if(status != MMAL_SUCCESS) error("Could not set still format");
   if(camera->output[2]->buffer_num < 3)
      camera->output[2]->buffer_num = 3;

   status = mmal_component_enable(camera);
   if(status != MMAL_SUCCESS) error("Could not enable camera");

   //
   // create jpeg-encoder
   //
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &jpegencoder);
   if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create image encoder");

   mmal_format_copy(jpegencoder->output[0]->format, jpegencoder->input[0]->format);
   jpegencoder->output[0]->format->encoding = MMAL_ENCODING_JPEG;
   jpegencoder->output[0]->buffer_size = jpegencoder->output[0]->buffer_size_recommended;
   if(jpegencoder->output[0]->buffer_size < jpegencoder->output[0]->buffer_size_min)
      jpegencoder->output[0]->buffer_size = jpegencoder->output[0]->buffer_size_min;
   jpegencoder->output[0]->buffer_num = jpegencoder->output[0]->buffer_num_recommended;
   if(jpegencoder->output[0]->buffer_num < jpegencoder->output[0]->buffer_num_min)
      jpegencoder->output[0]->buffer_num = jpegencoder->output[0]->buffer_num_min;
   status = mmal_port_format_commit(jpegencoder->output[0]);
   if(status != MMAL_SUCCESS) error("Could not set image format");
   status = mmal_port_parameter_set_uint32(jpegencoder->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, quality);
   if(status != MMAL_SUCCESS) error("Could not set jpeg quality");

   status = mmal_component_enable(jpegencoder);
   if(status != MMAL_SUCCESS) error("Could not enable image encoder");
   pool_jpegencoder = mmal_port_pool_create(jpegencoder->output[0], jpegencoder->output[0]->buffer_num, jpegencoder->output[0]->buffer_size);
   if(!pool_jpegencoder) error("Could not create image buffer pool");

   //
   // create second jpeg-encoder
   //
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &jpegencoder2);
   if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create image encoder 2");

   mmal_format_copy(jpegencoder2->output[0]->format, jpegencoder2->input[0]->format);
   jpegencoder2->output[0]->format->encoding = MMAL_ENCODING_JPEG;
   jpegencoder2->output[0]->buffer_size = jpegencoder2->output[0]->buffer_size_recommended;
   if(jpegencoder2->output[0]->buffer_size < jpegencoder2->output[0]->buffer_size_min)
      jpegencoder2->output[0]->buffer_size = jpegencoder2->output[0]->buffer_size_min;
   jpegencoder2->output[0]->buffer_num = jpegencoder2->output[0]->buffer_num_recommended;
   if(jpegencoder2->output[0]->buffer_num < jpegencoder2->output[0]->buffer_num_min)
      jpegencoder2->output[0]->buffer_num = jpegencoder2->output[0]->buffer_num_min;
   status = mmal_port_format_commit(jpegencoder2->output[0]);
   if(status != MMAL_SUCCESS) error("Could not set image format 2");
   status = mmal_port_parameter_set_uint32(jpegencoder2->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, 85);
   if(status != MMAL_SUCCESS) error("Could not set jpeg quality 2");

   status = mmal_component_enable(jpegencoder2);
   if(status != MMAL_SUCCESS) error("Could not enable image encoder 2");
   pool_jpegencoder2 = mmal_port_pool_create(jpegencoder2->output[0], jpegencoder2->output[0]->buffer_num, jpegencoder2->output[0]->buffer_size);
   if(!pool_jpegencoder2) error("Could not create image buffer pool 2");

   //
   // create h264-encoder
   //
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &h264encoder);
   if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create video encoder");

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
   if(status != MMAL_SUCCESS) error("Could not set video format");

   MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param2)}, 25};
   status = mmal_port_parameter_set(h264encoder->output[0], &param2.hdr);
   if(status != MMAL_SUCCESS) error("Could not set video quantisation");

   MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_QP_P, sizeof(param3)}, 31};
   status = mmal_port_parameter_set(h264encoder->output[0], &param3.hdr);
   if(status != MMAL_SUCCESS) error("Could not set video quantisation");

   MMAL_PARAMETER_VIDEO_PROFILE_T param4;
   param4.hdr.id = MMAL_PARAMETER_PROFILE;
   param4.hdr.size = sizeof(param4);
   param4.profile[0].profile = MMAL_VIDEO_PROFILE_H264_HIGH;
   param4.profile[0].level = MMAL_VIDEO_LEVEL_H264_4;
   status = mmal_port_parameter_set(h264encoder->output[0], &param4.hdr);
   if(status != MMAL_SUCCESS) error("Could not set video port format");

   status = mmal_port_parameter_set_boolean(h264encoder->input[0], MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, 1);
   if(status != MMAL_SUCCESS) error("Could not set immutable flag");

   status = mmal_port_parameter_set_boolean(h264encoder->output[0], MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, 0);
   if(status != MMAL_SUCCESS) error("Could not set inline flag");

   //
   // create image-resizer
   //
   unsigned int height_temp = (unsigned long int)width*video_height/video_width;
   height_temp -= height_temp%16;
   status = mmal_component_create("vc.ril.resize", &resizer);
   if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create image resizer");
  
   format = resizer->output[0]->format;
   format->es->video.width = width;
   format->es->video.height = height_temp;
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = width;
   format->es->video.crop.height = height_temp;
   format->es->video.frame_rate.num = 30;
   format->es->video.frame_rate.den = 1;
   status = mmal_port_format_commit(resizer->output[0]);
   if(status != MMAL_SUCCESS) error("Could not set image resizer output");

   status = mmal_component_enable(resizer);
   if(status != MMAL_SUCCESS) error("Could not enable image resizer");

   //
   // connect
   //
   status = mmal_connection_create(&con_cam_res, camera->output[0], resizer->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
   if(status != MMAL_SUCCESS) error("Could not create connection camera -> resizer");
   status = mmal_connection_enable(con_cam_res);
   if(status != MMAL_SUCCESS) error("Could not enable connection camera -> resizer");
  
   status = mmal_connection_create(&con_res_jpeg, resizer->output[0], jpegencoder->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
   if(status != MMAL_SUCCESS) error("Could not create connection resizer -> encoder");
   status = mmal_connection_enable(con_res_jpeg);
   if(status != MMAL_SUCCESS) error("Could not enable connection resizer -> encoder");

   status = mmal_port_enable(jpegencoder->output[0], jpegencoder_buffer_callback);
   if(status != MMAL_SUCCESS) error("Could not enable jpeg port");
   max = mmal_queue_length(pool_jpegencoder->queue);
   for(i=0;i<max;i++) {
      MMAL_BUFFER_HEADER_T *jpegbuffer = mmal_queue_get(pool_jpegencoder->queue);

      if(!jpegbuffer) error("Could not create jpeg buffer header");
      status = mmal_port_send_buffer(jpegencoder->output[0], jpegbuffer);
      if(status != MMAL_SUCCESS) error("Could not send buffers to jpeg port");
   }

   status = mmal_connection_create(&con_cam_jpeg, camera->output[2], jpegencoder2->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
   if(status != MMAL_SUCCESS) error("Could not create connection camera -> encoder");
   status = mmal_connection_enable(con_cam_jpeg);
   if(status != MMAL_SUCCESS) error("Could not enable connection camera -> encoder");
  
   status = mmal_port_enable(jpegencoder2->output[0], jpegencoder2_buffer_callback);
   if(status != MMAL_SUCCESS) error("Could not enable jpeg port 2");
   max = mmal_queue_length(pool_jpegencoder2->queue);
   for(i=0;i<max;i++) {
      MMAL_BUFFER_HEADER_T *jpegbuffer2 = mmal_queue_get(pool_jpegencoder2->queue);

      if(!jpegbuffer2) error("Could not create jpeg buffer header 2");
      status = mmal_port_send_buffer(jpegencoder2->output[0], jpegbuffer2);
      if(status != MMAL_SUCCESS) error("Could not send buffers to jpeg port 2");
   }
  
   //
   // settings
   //
   cam_set_sharpness();
   cam_set_contrast();
   cam_set_brightness();
   cam_set_saturation();
   cam_set_iso();
   cam_set_vs();
   cam_set_ec();
   cam_set_em();
   cam_set_wb();
   cam_set_mm();
   cam_set_ie();
   cam_set_ce();
   cam_set_rotation();
   cam_set_flip();
   cam_set_roi();
   cam_set_ss();
   cam_set_quality();
   cam_set_raw();
   cam_set_bitrate();
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

void dbg(char *msg) {
   FILE *fdbg = NULL;
   fdbg = fopen("/var/www/debug.txt", "a");
   fwrite(msg, 1, strlen(msg), fdbg);
   fwrite("\n", 1, 1, fdbg);
   fclose(fdbg);
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
   if (strncmp(filename, media_path, strlen(media_path)) == 0) {
      stat(media_path, &buf);
      //s to trailing path
      s = filename + strlen(media_path) + 1;
      do {
         t = strchr(s, '/');
         if (t != NULL) {
            *t = 0;
            r = mkdir(filename, 0777);
            if (r !=0 && errno == EEXIST) {
               chmod(filename, 0777);
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
            case 0: sprintf(p[pi], "%s", "s");break;
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
   if (thumb_gen != 0 && strchr(thumb_gen, source) != NULL) {
      if (source == 'v')
         xcount = video_cnt;
      else if (source == 'i')
         xcount = image2_cnt;
      else if (source == 't')
         xcount = image2_cnt;
      
      asprintf(&filename, "%s", from_filename);
      if (strncmp(filename, media_path, strlen(media_path)) == 0) {
         f = filename + strlen(media_path) + 1;
         //remove .h264 if present
         if (strcmp((f + strlen(f) - 5), ".h264") == 0) {
            *(f + strlen(f) - 5) = 0; 
         }
         s = f;
         do {
            t = strchr(s, '/');
            if (t != NULL) {
               *t = subdir_char;
               s = t + 1;
            }
         } while (t != NULL);
         //generate thumbnail name
         asprintf(&thumb_name, "%s/%s.%c%04d.th.jpg", media_path, f, source, xcount);
         copy_file(jpeg_filename, thumb_name);
         free(thumb_name);
      }
      free(filename);
   }
}

void capt_img (void) {

   char *filename_temp;

   currTime = time(NULL);
   localTime = localtime (&currTime);
   if(timelapse && strlen(lapse_filename) > 10) {
      makeFilename(&filename_temp, lapse_filename);
      if (lapse_cnt == 1) {
         //Only first capture of a lapse sequence
         thumb_create(filename_temp, 't');
      }
   } else {
      makeFilename(&filename_temp, jpeg2_filename);
      thumb_create(filename_temp, 'i');
   }
   createMediaPath(filename_temp);
   jpegoutput2_file = fopen(filename_temp, "wb");
   free(filename_temp);
   if(!jpegoutput2_file) error("Could not open/create image-file");
   status = mmal_port_parameter_set_boolean(camera->output[2], MMAL_PARAMETER_CAPTURE, 1);
   if(status != MMAL_SUCCESS) error("Could not start image capture");
   printf("Capturing image\n");
   if(status_filename != 0) {
      if(!timelapse) {
         status_file = fopen(status_filename, "w");
         fprintf(status_file, "image");
         fclose(status_file);
      }
   }
   capturing = 1;
}

void start_video(void) {
   int i, max;
   char *filename_temp;

   if(!capturing) {
      status = mmal_component_enable(h264encoder);
      if(status != MMAL_SUCCESS) error("Could not enable h264encoder");
      pool_h264encoder = mmal_port_pool_create(h264encoder->output[0], h264encoder->output[0]->buffer_num, h264encoder->output[0]->buffer_size);
      if(!pool_h264encoder) error("Could not create pool");
      status = mmal_connection_create(&con_cam_h264, camera->output[1], h264encoder->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
      if(status != MMAL_SUCCESS) error("Could not create connecton camera -> video converter");
      status = mmal_connection_enable(con_cam_h264);
      if(status != MMAL_SUCCESS) error("Could not enable connection camera -> video converter");
      currTime = time(NULL);
      localTime = localtime (&currTime);
      if(mp4box != 0) {
         makeFilename(&filename_recording, h264_filename);
         asprintf(&filename_temp, "%s.h264", filename_recording);
      }
      else {
         makeFilename(&filename_temp, h264_filename);
      }
      thumb_create(filename_temp, 'v');
      createMediaPath(filename_temp);
      h264output_file = fopen(filename_temp, "wb");
      free(filename_temp);
      if(!h264output_file) error("Could not open/create video-file");
      status = mmal_port_enable(h264encoder->output[0], h264encoder_buffer_callback);
      if(status != MMAL_SUCCESS) error("Could not enable video port");
      max = mmal_queue_length(pool_h264encoder->queue);
      for(i=0;i<max;i++) {
         MMAL_BUFFER_HEADER_T *h264buffer = mmal_queue_get(pool_h264encoder->queue);
         if(!h264buffer) error("Could not create video pool header");
         status = mmal_port_send_buffer(h264encoder->output[0], h264buffer);
         if(status != MMAL_SUCCESS) error("Could not send buffers to video port");
      }
      mmal_port_parameter_set_boolean(camera->output[1], MMAL_PARAMETER_CAPTURE, 1);
      if(status != MMAL_SUCCESS) error("Could not start capture");
      printf("Capturing started\n");
      if(status_filename != 0) {
         status_file = fopen(status_filename, "w");
         if(!motion_detection) fprintf(status_file, "video");
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
      if(status != MMAL_SUCCESS) error("Could not stop capture");
      status = mmal_port_disable(h264encoder->output[0]);
      if(status != MMAL_SUCCESS) error("Could not disable video port");
      status = mmal_connection_destroy(con_cam_h264);
      if(status != MMAL_SUCCESS) error("Could not destroy connection camera -> video encoder");
      mmal_port_pool_destroy(h264encoder->output[0], pool_h264encoder);
      if(status != MMAL_SUCCESS) error("Could not destroy video buffer pool");
      status = mmal_component_disable(h264encoder);
      if(status != MMAL_SUCCESS) error("Could not disable video converter");
      fclose(h264output_file);
      h264output_file = NULL;
      printf("Capturing stopped\n");
      if(mp4box) {
         asprintf(&filename_temp, "%s.h264", filename_recording);
         if(mp4box == 1) {
            printf("Boxing started\n");
            status_file = fopen(status_filename, "w");
            if(!motion_detection) fprintf(status_file, "boxing");
            else fprintf(status_file, "md_boxing");
            fclose(status_file);
            background = ' ';
         } else {
            background = '&';
         }
         asprintf(&cmd_temp, "(MP4Box -fps %i -add %s.h264 %s > /dev/null;rm \"%s\";) %c", MP4Box_fps, filename_recording, filename_recording, filename_temp, background);
         if(mp4box == 1) {
            if(system(cmd_temp) == -1) error("Could not start MP4Box");
            printf("Boxing operation stopped\n");
         } else {
            system(cmd_temp);
            printf("Boxing in background");
         }
         //remove(filename_temp);
         free(filename_temp);
         free(filename_recording);
         free(cmd_temp);
      }
      video_cnt++;
      if(status_filename != 0) {
         status_file = fopen(status_filename, "w");
         if(!motion_detection) fprintf(status_file, "ready");
         else fprintf(status_file, "md_ready");
         fclose(status_file);
      }
      capturing = 0;
   }
}

void read_config(char *cfilename) {
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
               if(strcmp(line, "width") == 0) {
                  width = atoi(value);
               }
               else if(strcmp(line, "quality") == 0) {
                  quality = atoi(value);
               }
               else if(strcmp(line, "divider") == 0) {
                  divider = atoi(value);
               }
               else if(strcmp(line, "media_path") == 0) {
                  if (media_path != 0) free(media_path);
                  asprintf(&media_path, "%s", value);
               }
               else if(strcmp(line, "preview_path") == 0) {
                  if (jpeg_filename != 0) free(jpeg_filename);
                  asprintf(&jpeg_filename, "%s", value);
               }
               else if(strcmp(line, "image_path") == 0) {
                  if (jpeg2_filename != 0) free(jpeg2_filename);
                  asprintf(&jpeg2_filename, "%s", value);
               }
               else if(strcmp(line, "lapse_path") == 0) {
                  if (lapse_filename != 0) free(lapse_filename);
                  asprintf(&lapse_filename, "%s", value);
               }
               else if(strcmp(line, "video_path") == 0) {
                  if (h264_filename != 0) free(h264_filename);
                  asprintf(&h264_filename, "%s", value);
               }
               else if(strcmp(line, "status_file") == 0) {
                  if (status_filename != 0) free(status_filename);
                  asprintf(&status_filename, "%s", value);
               }
               else if(strcmp(line, "thumb_gen") == 0) {
                  if (thumb_gen != 0) free(thumb_gen);
                  asprintf(&thumb_gen, "%s", value);
               }
               else if(strcmp(line, "control_file") == 0) {
                  if (pipe_filename != 0) free(pipe_filename);
                  asprintf(&pipe_filename, "%s", value);
               }
               else if(strcmp(line, "annotation") == 0) {
                  if (cam_setting_annotation != 0) free(cam_setting_annotation);
                  asprintf(&cam_setting_annotation, "%s", value);
               }
               else if(strcmp(line, "user_config") == 0) {
                  if (user_config != 0) free(user_config);
                  asprintf(&user_config, "%s", value);
               }
               else if(strcmp(line, "anno_background") == 0) {
                  if(strcmp(value, "true") == 0) cam_setting_annback = 1;
               }
               else if(strcmp(line, "MP4Box") == 0) {
                  if(strcmp(value, "true") == 0) mp4box = 1;
                  else if(strcmp(value, "false") == 0) mp4box = 0;
                  else if(strcmp(value, "background") == 0) mp4box = 2;
                  else mp4box = atoi(value);
               }
               else if(strcmp(line, "autostart") == 0) {
                  if(strcmp(value, "idle") == 0) {
                     autostart = 0;
                     idle = 1;
                  }
               }
               else if(strcmp(line, "motion_detection") == 0) {
                  if(strcmp(value, "true") == 0) motion_detection = 1;
               }
               else if(strcmp(line, "sharpness") == 0) {
                  cam_setting_sharpness = atoi(value);
               }
               else if(strcmp(line, "contrast") == 0) {
                  cam_setting_contrast = atoi(value);
               }
               else if(strcmp(line, "brightness") == 0) {
                  cam_setting_brightness = atoi(value);
               }
               else if(strcmp(line, "saturation") == 0) {
                  cam_setting_saturation = atoi(value);
               }
               else if(strcmp(line, "iso") == 0) {
                  cam_setting_iso = atoi(value);
               }
               else if(strcmp(line, "video_stabilisation") == 0) {
                  if(strcmp(value, "true") == 0) cam_setting_vs = 1;
               }
               else if(strcmp(line, "exposure_compensation") == 0) {
                  cam_setting_ec = atoi(value);
               }
               else if(strcmp(line, "exposure_mode") == 0) {
                  sprintf(cam_setting_em, "%s", value);
               }
               else if(strcmp(line, "white_balance") == 0) {
                  sprintf(cam_setting_wb, "%s", value);
               }
               else if(strcmp(line, "metering_mode") == 0) {
                  sprintf(cam_setting_mm, "%s", value);
               }
               else if(strcmp(line, "image_effect") == 0) {
                  sprintf(cam_setting_ie, "%s", value);
               }
               else if(strcmp(line, "colour_effect_en") == 0) {
                  if(strcmp(value, "true") == 0) cam_setting_ce_en = 1;
               }
               else if(strcmp(line, "colour_effect_u") == 0) {
                  cam_setting_ce_u = atoi(value);
               }
               else if(strcmp(line, "colour_effect_v") == 0) {
                  cam_setting_ce_v = atoi(value);
               }
               else if(strcmp(line, "rotation") == 0) {
                  cam_setting_rotation = atoi(value);
               }
               else if(strcmp(line, "hflip") == 0) {
                  if(strcmp(value, "true") == 0) cam_setting_hflip = 1;
               }
               else if(strcmp(line, "vflip") == 0) {
                  if(strcmp(value, "true") == 0) cam_setting_vflip = 1;
               }
               else if(strcmp(line, "sensor_region_x") == 0) {
                  cam_setting_roi_x = strtoull(value, NULL, 0);
               }
               else if(strcmp(line, "sensor_region_y") == 0) {
                  cam_setting_roi_y = strtoull(value, NULL, 0);
               }
               else if(strcmp(line, "sensor_region_w") == 0) {
                  cam_setting_roi_w = strtoull(value, NULL, 0);
               }
               else if(strcmp(line, "sensor_region_h") == 0) {
                  cam_setting_roi_h = strtoull(value, NULL, 0);
               }
               else if(strcmp(line, "shutter_speed") == 0) {
                  cam_setting_ss = strtoull(value, NULL, 0);
               }
               else if(strcmp(line, "image_quality") == 0) {
                  cam_setting_quality = atoi(value);
               }
               else if(strcmp(line, "raw_layer") == 0) {
                  if(strcmp(value, "true") == 0) cam_setting_raw = 1;
               }
               else if(strcmp(line, "video_bitrate") == 0) {
                  cam_setting_bitrate = strtoull(value, NULL, 0);
               }
               else if(strcmp(line, "video_width") == 0) {
                  video_width = atoi(value);
               }
               else if(strcmp(line, "video_height") == 0) {
                  video_height = atoi(value);
               }
               else if(strcmp(line, "video_fps") == 0) {
                  video_fps = atoi(value);
               }
               else if(strcmp(line, "MP4Box_fps") == 0) {
                  MP4Box_fps = atoi(value);
               }
               else if(strcmp(line, "image_width") == 0) {
                  image_width = atoi(value);
               }
               else if(strcmp(line, "image_height") == 0) {
                  image_height = atoi(value);
               }
               else if(strcmp(line, "subdir_char") == 0) {
                  subdir_char = *value;
               }
               else if(strcmp(line, "anno_version") == 0) {
                  anno_version = atoi(value);
               }
               else if(strcmp(line, "anno_text_size") == 0) {
                  anno3_text_size = atoi(value);
               }
               else if(strcmp(line, "anno3_custom_background_colour") == 0) {
                  anno3_custom_background[0] = atoi(value);
               }
               else if(strcmp(line, "anno3_custom_background_Y") == 0) {
                  anno3_custom_background[1] = atoi(value);
               }
               else if(strcmp(line, "anno3_custom_background_U") == 0) {
                  anno3_custom_background[2] = atoi(value);
               }
               else if(strcmp(line, "anno3_custom_background_V") == 0) {
                  anno3_custom_background[3] = atoi(value);
               }
               else if(strcmp(line, "anno3_custom_text_colour") == 0) {
                  anno3_custom_text[0] = atoi(value);
               }
               else if(strcmp(line, "anno3_custom_text_Y") == 0) {
                  anno3_custom_text[1] = atoi(value);
               }
               else if(strcmp(line, "anno3_custom_text_U") == 0) {
                  anno3_custom_text[2] = atoi(value);
               }
               else if(strcmp(line, "anno3_custom_text_V") == 0) {
                  anno3_custom_text[3] = atoi(value);
               }
               else {
                 printf("Unknown command in config file: %s\n", line);
                 //error("Invalid config file");
               }
            }
         }
      }
   if(line) free(line);
   }   
}


void process_cmd(char *readbuf, int length) {
   typedef enum pipe_cmd_type{ca,im,tl,px,bo,an,av,as,at,ac,ab,sh,co,br,sa,is,vs,rl,ec,em,wb,mm,ie,ce,ro,fl,ri,ss,qu,bl,ru,md} pipe_cmd_type;
   char pipe_cmds[] = "ca,im,tl,px,bo,an,av,as,at,ac,ab,sh,co,br,sa,is,vs,rl,ec,em,wb,mm,ie,ce,ro,fl,ri,ss,qu,bl,ru,md";
   pipe_cmd_type pipe_cmd;
   int i;
   unsigned long int pars[10];
   char *parstring=0, *temp;
   
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
      pars[i] = strtoull(temp, NULL, 10);
      i++;
      temp = strtok(NULL, " ");
   }
   
   switch(pipe_cmd) {
      case ca:
         if(pars[0] == 1) {
            start_video();
         }  else {
            stop_video();
         }
         break;
      case im:
         capt_img();
         break;
      case tl:
         time_between_pic = (int)pars[0];
         if(time_between_pic) {
            if(status_filename != 0) {
               status_file = fopen(status_filename, "w");
               fprintf(status_file, "timelapse");
               fclose(status_file);
            }
            timelapse = 1;
            lapse_cnt = 1;
            printf("Timelapse started\n");
         }
         else {
            if(status_filename != 0) {
              status_file = fopen(status_filename, "w");
              if(!motion_detection) fprintf(status_file, "ready");
              else fprintf(status_file, "md_ready");
              fclose(status_file);
            }
            image2_cnt++;
            timelapse = 0;
            printf("Timelapse stopped\n");
         }
         break;
      case px:
         stop_all();
         video_width = (unsigned int)pars[0];
         video_height = (unsigned int)pars[1];
         video_fps = (unsigned int)pars[2];
         MP4Box_fps = (unsigned int)pars[3];
         image_width = (unsigned int)pars[4];
         image_height = (unsigned int)pars[5];
         start_all(0);
         printf("Changed resolutions and framerates\n");
         break;
      case bo:
         mp4box = (unsigned int)pars[0];
         if (mp4box > 2) mp4box = 2;
         printf("MP4Box mode changed\n");
         break;
      case an:
         asprintf(&cam_setting_annotation, "%s", parstring);
         printf("Annotation changed\n");
         break;
      case av:
         anno_version = (unsigned int)pars[0];
         printf("Annotation version changed\n");
         break;
      case as:
         anno3_text_size = (unsigned int)pars[0];
         printf("Annotation text size changed\n");
         break;
      case at:
         anno3_custom_text[0] = (unsigned int)pars[0];
         anno3_custom_text[1] = (unsigned int)pars[1];
         anno3_custom_text[2] = (unsigned int)pars[2];
         anno3_custom_text[3] = (unsigned int)pars[3];
         printf("Annotation custom text colour changed\n");
         break;
      case ac:
         anno3_custom_background[0] = (unsigned int)pars[0];
         anno3_custom_background[1] = (unsigned int)pars[1];
         anno3_custom_background[2] = (unsigned int)pars[2];
         anno3_custom_background[3] = (unsigned int)pars[3];
         printf("Annotation custom background colour changed\n");
         break;
      case ab:
         cam_setting_annback = (pars[0] == 0)?0:1;
         printf("Annotation background changed\n");
         break;
      case sh:
         cam_setting_sharpness = (unsigned int)pars[0];
         cam_set_sharpness();
         printf("Sharpness: %d\n", cam_setting_sharpness);
         break;
      case co:
         cam_setting_contrast = (unsigned int)pars[0];
         cam_set_contrast();
         printf("Contrast: %d\n", cam_setting_contrast);
         break;
      case br:
         cam_setting_brightness = (unsigned int)pars[0];
         cam_set_brightness();
         printf("Brightness: %d\n", cam_setting_brightness);
         break;
      case sa:
         cam_setting_saturation = (unsigned int)pars[0];
         cam_set_saturation();
         printf("Saturation: %d\n", cam_setting_saturation);
         break;
      case is:
         cam_setting_iso = (unsigned int)pars[0];
         cam_set_iso();
         printf("ISO: %d\n", cam_setting_iso);
         break;
      case vs:
         cam_setting_vs = (pars[0] == 1)?1:0;
         cam_set_vs();
         printf("Changed video stabilisation\n");
         break;
      case rl:
         cam_setting_raw = (pars[0] == 1)?1:0;
         cam_set_raw();
         printf("Changed raw layer\n");
         break;
      case ec:
         cam_setting_ec = (unsigned int)pars[0];
         cam_set_ec();
         printf("Exposure compensation: %d\n", cam_setting_ec);
         break;
      case em:
         sprintf(cam_setting_em, "%s", parstring);
         cam_set_em();
         printf("Exposure mode changed\n");
         break;
      case wb:
         sprintf(cam_setting_wb, "%s", parstring);
         cam_set_wb();
         printf("White balance changed\n");
         break;
      case mm:
         sprintf(cam_setting_mm, "%s", parstring);
         cam_set_mm();
         printf("Metering mode changed\n");
         break;
      case ie:
         sprintf(cam_setting_ie, "%s", parstring);
         cam_set_ie();
         printf("Image effect changed\n");
         break;
      case ce:
         cam_setting_ce_en = (unsigned int)pars[0];
         cam_setting_ce_u = (unsigned int)pars[1];
         cam_setting_ce_v = (unsigned int)pars[2];;
         cam_set_ce();
         printf("Colour effect changed\n");
         break;
      case ro:
         cam_setting_rotation = (unsigned int)pars[0];
         cam_set_rotation();
         printf("Rotation: %d\n", cam_setting_rotation);
         break;
      case fl:
         cam_setting_hflip = pars[0] & 1;
         cam_setting_vflip = pars[0] >> 1;
         cam_set_flip();
         printf("Flip changed\n");
         break;
      case ri:
         cam_setting_roi_x = pars[0];
         cam_setting_roi_y = pars[1];
         cam_setting_roi_w = pars[2];
         cam_setting_roi_h = pars[3];
         cam_set_roi();
         printf("Changed Sensor Region\n");
         break;
      case ss:
         cam_setting_ss = pars[0];
         cam_set_ss();
         printf("Shutter Speed: %lu\n", cam_setting_ss);
         break;
      case qu:
         cam_setting_quality = (unsigned int)pars[0];
         cam_set_quality();
         printf("Quality: %d\n", cam_setting_quality);
         break;
      case bl:
         cam_setting_bitrate = pars[0];
         cam_set_bitrate();
         printf("Bitrate: %lu\n", cam_setting_bitrate);
         break;
      case ru:
         if (pars[0] == 0) {
            stop_all();
            idle = 1;
            printf("Stream halted\n");
            if(status_filename != 0) {
              status_file = fopen(status_filename, "w");
              fprintf(status_file, "halted");
              fclose(status_file);
            }
         } else {
            start_all(1);
            idle = 0;
            printf("Stream continued\n");
            if(status_filename != 0) {
              status_file = fopen(status_filename, "w");
              fprintf(status_file, "ready");
              fclose(status_file);
            }
         }
         break;
      case md:
         if(pars[0]==0) {
            motion_detection = 0;
            if(system("killall motion") == -1) error("Could not stop Motion");
            printf("Motion detection stopped\n");
            if(status_filename != 0) {
               status_file = fopen(status_filename, "w");
               fprintf(status_file, "ready");
               fclose(status_file);
            }
         }
         else {
            motion_detection = 1;
            if(system("motion") == -1) error("Could not start Motion");
            printf("Motion detection started\n");
            if(status_filename != 0) {
               status_file = fopen(status_filename, "w");
               fprintf(status_file, "md_ready");
               fclose(status_file);
            }
         }
         break;
      default:
         printf("Unrecognised pipe command\n");
         break;
   }
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
   return max_count + found;;
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
         printf("RaspiMJPEG Version ");
         printf(VERSION);
         printf("\n");
         exit(0);
      }
      else if(strcmp(argv[i], "-md") == 0) {
         motion_detection = 1;
      }
   }

   //default base media path
   asprintf(&media_path, "%s", "/var/www/media");
   //
   // read configs and init
   //
   read_config("/etc/raspimjpeg");
   if (user_config != 0)
      read_config(user_config);
 
   image2_cnt = findNextCount(jpeg2_filename, "it");
   video_cnt = findNextCount(h264_filename, "v");

   if(autostart) start_all(0);
   if(motion_detection) {
      if(system("motion") == -1) error("Could not start Motion");
   }

   //
   // run
   //
   if(autostart) {
      if(pipe_filename != 0) printf("MJPEG streaming, ready to receive commands\n");
      else printf("MJPEG streaming\n");
   }
   else {
      if(pipe_filename != 0) printf("MJPEG idle, ready to receive commands\n");
      else printf("MJPEG idle\n");
   }

   struct sigaction action;
   memset(&action, 0, sizeof(struct sigaction));
   action.sa_handler = term;
   sigaction(SIGTERM, &action, NULL);
   sigaction(SIGINT, &action, NULL);
  
   if(status_filename != 0) {
      status_file = fopen(status_filename, "w");
      if(!status_file) error("Could not open/create status-file");
      if(autostart) {
         if(!motion_detection) {
            fprintf(status_file, "ready");
         }
         else fprintf(status_file, "md_ready");
      }
      else fprintf(status_file, "halted");
      fclose(status_file);
   }
   
   //Clear out anything in FIFO first
   do {
      fd = open(pipe_filename, O_RDONLY | O_NONBLOCK);
      if(fd < 0) error("Could not open PIPE");
      fcntl(fd, F_SETFL, 0);
      length = read(fd, readbuf, 60);
      close(fd);
   } while (length != 0); 
  
   // Main forever loop
   while(running) {
      if(pipe_filename != 0) {

         fd = open(pipe_filename, O_RDONLY | O_NONBLOCK);
         if(fd < 0) error("Could not open PIPE");
         fcntl(fd, F_SETFL, 0);
         length = read(fd, readbuf, 60);
         close(fd);

         if(length) {
            process_cmd(readbuf, length);
         }

      }
      if(timelapse) {
         tl_cnt++;
         if(tl_cnt >= time_between_pic) {
            if(capturing == 0) {
               capt_img();
               tl_cnt = 0;
            }
         }
      }
      usleep(100000);
   }
  
   printf("SIGINT/SIGTERM received, stopping\n");
   //
   // tidy up
   //
   if(!idle) stop_all();
   return 0;
}
