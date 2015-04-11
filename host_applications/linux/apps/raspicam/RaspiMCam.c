/*
Copyright (c) 2015, Broadcom Europe Ltd
Copyright (c) 2015, Silvan Melchior
Copyright (c) 2015, Robert Tidey
Copyright (c) 2015, ethanol100
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
 * \file RaspiMCam.c
 * Camera access routines
 *
 * \date 9th Aprl 2015
 * \Author: Silvan Melchior / Robert Tidey
 *
 * Description
 *
 * MMAL access routines and callbacks to support two images streams and one video
 *
 * Usage information in README_RaspiMJPEG.md
 */

#include "RaspiMJPEG.h"

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
      if (timelapse && strlen(cfg_stru[c_lapse_path]) > 10)
         lapse_cnt++;
      else
         image2_cnt++;
      i_capturing = 0;
      updateStatus();
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

  if(buffering_toggle) {
    
    int space_in_buff = cb_len - cb_wptr;
    int copy_to_end = space_in_buff > buffer->length ? buffer->length : space_in_buff;
    int copy_to_start = buffer->length - copy_to_end;
    
    if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) {
      if(header_wptr + buffer->length > sizeof(header_bytes)) {
        error("Error in header bytes\n", 0);
      }
      else  {
        mmal_buffer_header_mem_lock(buffer);
        memcpy(header_bytes + header_wptr, buffer->data, buffer->length);
        mmal_buffer_header_mem_unlock(buffer);
        header_wptr += buffer->length;
      }
    }
    else {
      static int frame_start = -1;
      int i;

      if(frame_start == -1)
        frame_start = cb_wptr;

      if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_KEYFRAME) {
        iframe_buff[iframe_buff_wpos] = frame_start;
        iframe_buff_wpos = (iframe_buff_wpos + 1) % IFRAME_BUFSIZE;
      }

      if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END)
        frame_start = -1;

      // If we overtake the iframe rptr then move the rptr along
      if((iframe_buff_rpos + 1) % IFRAME_BUFSIZE != iframe_buff_wpos)
        while(
              (
                cb_wptr <= iframe_buff[iframe_buff_rpos] &&
                (cb_wptr + buffer->length) > iframe_buff[iframe_buff_rpos]
              ) ||
              (
                (cb_wptr > iframe_buff[iframe_buff_rpos]) &&
                (cb_wptr + buffer->length) > (iframe_buff[iframe_buff_rpos] + cb_len)
              )
             )
          iframe_buff_rpos = (iframe_buff_rpos + 1) % IFRAME_BUFSIZE;

      mmal_buffer_header_mem_lock(buffer);
      // We are pushing data into a circular buffer
      memcpy(cb_buff + cb_wptr, buffer->data, copy_to_end);
      memcpy(cb_buff, buffer->data + copy_to_end, copy_to_start);
      mmal_buffer_header_mem_unlock(buffer);

      if((cb_wptr + buffer->length) > cb_len)
        cb_wrap = 1;

      cb_wptr = (cb_wptr + buffer->length) % cb_len;

      for(i = iframe_buff_rpos; i != iframe_buff_wpos; i = (i + 1) % IFRAME_BUFSIZE) {
        int p = iframe_buff[i];
        if(cb_buff[p] != 0 || cb_buff[p+1] != 0 || cb_buff[p+2] != 0 || cb_buff[p+3] != 1) {
          error("Error in iframe list", 0);
        }
      }
    }
  }
  else if(buffer->length) {
  
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
   cam_set_annotationV3(filename_temp, enable);
   if (filename_temp != 0) free(filename_temp);
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
         i_capturing = 1;
         updateStatus();
      } else {
         error("Could not start image capture", 0);
      }
   } else {
      error("Could not open/create image-file", 0);
   }
}

void start_video(unsigned char prepare_buf) {
  int i, max;
  char *filename_temp;

  if(!v_capturing || prepare_buf) {
    if(prepare_buf) {
      cb_wptr = 0;
      cb_wrap = 0;
      cb_data = 0;
      iframe_buff_wpos = 0;
      iframe_buff_rpos = 0;
      header_wptr = 0;
    }
    if(prepare_buf || !buffering) {
      status = mmal_component_enable(h264encoder);
      if(status != MMAL_SUCCESS) {error("Could not enable h264encoder", 0); return;}
      pool_h264encoder = mmal_port_pool_create(h264encoder->output[0], h264encoder->output[0]->buffer_num, h264encoder->output[0]->buffer_size);
      if(!pool_h264encoder) {error("Could not create pool", 0); return;}
      status = mmal_connection_create(&con_cam_h264, camera->output[1], h264encoder->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
      if(status != MMAL_SUCCESS) {error("Could not create connection camera -> video converter", 0); return;}
      status = mmal_connection_enable(con_cam_h264);
      if(status != MMAL_SUCCESS) {error("Could not enable connection camera -> video converter", 0); return;}
    }
    if(!prepare_buf) {
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
      if(buffering) {
        int copy_from_end, copy_from_start;
        copy_from_end = cb_len - iframe_buff[iframe_buff_rpos];
        copy_from_start = cb_len - copy_from_end;
        copy_from_start = cb_wptr < copy_from_start ? cb_wptr : copy_from_start;
        if(!cb_wrap) {
           copy_from_start = cb_wptr;
           copy_from_end = 0;
        }
        long fileSizeCircularBuffer=copy_from_start+copy_from_end+header_wptr;
        fseek(h264output_file, fileSizeCircularBuffer, SEEK_SET);
        buffering_toggle = 0;
      }
    }
    if(prepare_buf || !buffering) {
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
    }
    if(!prepare_buf) {
      printLog("Capturing started\n");
      v_capturing = 1;
    }
    updateStatus();
  }
}

void stop_video(unsigned char stop_buf) {
  char *filename_temp, *cmd_temp;
  char background;
  if(v_capturing || stop_buf) {
    if(stop_buf || !buffering) {
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
    }
    if(!stop_buf) {
      if(buffering) {
        buffering_toggle = 1;
        int copy_from_end, copy_from_start;
        copy_from_end = cb_len - iframe_buff[iframe_buff_rpos];
        copy_from_start = cb_len - copy_from_end;
        copy_from_start = cb_wptr < copy_from_start ? cb_wptr : copy_from_start;
        if(!cb_wrap) {
          copy_from_start = cb_wptr;
          copy_from_end = 0;
        }
        fseek(h264output_file, 0, SEEK_SET);
        fwrite(header_bytes, 1, header_wptr, h264output_file);
        fwrite(cb_buff + iframe_buff[iframe_buff_rpos], 1, copy_from_end, h264output_file);
        fwrite(cb_buff, 1, copy_from_start, h264output_file);
      }
      fclose(h264output_file);
      h264output_file = NULL;
      printLog("Capturing stopped\n");
      v_capturing = 0;
      if(cfg_val[c_MP4Box]) {
        asprintf(&filename_temp, "%s.h264", filename_recording);
        if(cfg_val[c_MP4Box] == 1) {
          printLog("Boxing started\n");
          v_boxing = 1;
          updateStatus();
          background = ' ';
        }
        else {
          background = '&';
        }
        asprintf(&cmd_temp, "(MP4Box -fps %i -add %s.h264 %s > /dev/null;rm \"%s\";) %c", cfg_val[c_MP4Box_fps], filename_recording, filename_recording, filename_temp, background);
        if(cfg_val[c_MP4Box] == 1) {
          if(system(cmd_temp) == -1) error("Could not start MP4Box", 0);
          printLog("Boxing operation stopped\n");
          v_boxing = 0;
        }
        else {
          system(cmd_temp);
          printLog("Boxing in background\n");
        }
        free(filename_temp);
        free(filename_recording);
        free(cmd_temp);
      }
      video_cnt++;
    }
    updateStatus();
  }
}

void cam_set_buffer() {
  if(buffering) {
    buffering = 0;
    buffering_toggle = 0;
    stop_video(1);
    if(cb_buff != NULL) {
      free(cb_buff);
    }
  }
  if(cfg_val[c_video_buffer] != 0) {
    int count = ((long long)cfg_val[c_video_bitrate]/8 * (long long)cfg_val[c_video_buffer]) / 1000;
    
    cb_buff = (char *) malloc(count);
    if(cb_buff == NULL) {
      error("Unable to allocate circular buffer", 0);
    }
    else {
      cb_len = count;
      buffering = 1;
      buffering_toggle = 1;
      start_video(1);
    }
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
      case c_video_buffer:
         cam_set_buffer();
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
   cam_set(c_video_buffer);
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

