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
 * \file RaspiMUtils.c
 * Utilities for RaspiMJPEG.c
 * Also optionally stream a preview of current camera input wth MJPEG.
 *
 * \date 9th Aprl 2015
 * \Author: Silvan Melchior / Robert Tidey
 *
 * Description
 *
 * Usage information in README_RaspiMJPEG.md
 */
#include "RaspiMJPEG.h"

void mmalLog(char *msg, ...) {
	if (cfg_stru[c_mmal_logfile]) {
		va_list args;
		va_start(args, msg);
		int nofile = 0;
		FILE *fp;

		nofile = (access(cfg_stru[c_mmal_logfile], F_OK ) == -1 );
		fp = fopen(cfg_stru[c_mmal_logfile], "a");
		if (fp != NULL) {
			clock_gettime(CLOCK_REALTIME, &currTime);
			localTime = localtime (&(currTime.tv_sec));
			fprintf(fp, "%02d %lu ",localTime->tm_sec,currTime.tv_nsec);
			vfprintf(fp, msg, args);
			fclose(fp);
			if (nofile) chmod(cfg_stru[c_mmal_logfile], 0777);
		}
		va_end(args);
	}
}

void printLogEx(int logfile, char *msg, ...) {
	va_list args;
	va_start(args, msg);
	int nofile = 0;
	char *timestamp;
	FILE *fp;
	vfprintf(stdout, msg, args);

	if (cfg_stru[logfile] != 0 && cfg_val[c_log_size] != 0) {
		nofile = (access(cfg_stru[logfile], F_OK ) == -1 );
		fp = fopen(cfg_stru[logfile], "a");
	} else {
		fp = stdout;
	}
	if (fp != NULL) {
		clock_gettime(CLOCK_REALTIME, &currTime);
		localTime = localtime (&(currTime.tv_sec));
		makeName(&timestamp, "{%Y/%M/%D %h:%m:%s} ");
		fprintf(fp, "%s",timestamp);
		vfprintf(fp, msg, args);
		if (cfg_stru[logfile] != 0) {
			fclose(fp);
			if (nofile) chmod(cfg_stru[logfile], 0777);
		}
		free(timestamp);
	}
	va_end(args);
}

void printLog(char *msg, ...) {
	char *timestamp;
	va_list args;
	va_start(args, msg);
	int nofile = 0;
	FILE *fp;
	vfprintf(stdout, msg, args);


	if (cfg_stru[c_log_file] != 0 && cfg_val[c_log_size] != 0) {
		nofile = (access(cfg_stru[c_log_file], F_OK ) == -1 );
		fp = fopen(cfg_stru[c_log_file], "a");
	} else {
		fp = stdout;
	}
	if (fp != NULL) {
		clock_gettime(CLOCK_REALTIME, &currTime);
		localTime = localtime (&(currTime.tv_sec));
		makeName(&timestamp, "{%Y/%M/%D %h:%m:%s} ");
		fprintf(fp, "%s",timestamp);
		vfprintf(fp, msg, args);
		if (cfg_stru[c_log_file] != 0) {
			fclose(fp);
			if (nofile) chmod(cfg_stru[c_log_file], 0777);
		}
		free(timestamp);
	}
	va_end(args);
}

void updateStatus() {
	char status[20];
   
	if(cfg_stru[c_status_file] != 0) {
      
		if (a_error) {
			strcpy(status, "Error");
		}
		else if (idle) {
			strcpy(status, "halted");
		}
		else if (i_capturing) {
			strcpy(status, "image");
		}
		else if (v_capturing) {
			if(!cfg_val[c_motion_detection]) 
				if (timelapse)
					strcpy(status, "tl_video");
				else
					strcpy(status, "video");
			else
				if (timelapse)
					strcpy(status, "tl_md_video");
				else
					strcpy(status, "md_video");
			}
		else {
			if(!cfg_val[c_motion_detection]) 
				if (timelapse)
					strcpy(status, "timelapse");
				else
					strcpy(status, "ready");
			else
				if (timelapse)
					strcpy(status, "tl_md_ready");
				else
					strcpy(status, "md_ready");
		}
      
		status_file = fopen(cfg_stru[c_status_file], "w");
		if(status_file) {
			fprintf(status_file, status);
			fclose(status_file);
		}
	}
}

void error (const char *string, char fatal) {
	printLog("Error: %s\n", string);
	if (fatal == 0) {
		exec_macro(cfg_stru[c_error_soft], string);
		return;
	}
	exec_macro(cfg_stru[c_error_hard], string);
	a_error = 1;
	updateStatus();
	exit(1);
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


char* trim(char*s) {
	if (s == NULL) return NULL;
	char *end = s + strlen(s)-1;
	while(*s && isspace(*s))
		*s++ = 0;
	while(isspace(*end))
		*end-- = 0;
	return s;
}

void makeName(char** name, char *template) {
	//Create name from template
	const int max_subs = 24;
	char spec[16] = "%YyMDhmsuvitfcka";
	char *template1;
	char p[512];
	char *s, *f, *q;
	int sp, si=0;
	int read_size;
	FILE *fp;

	memset(p, 0, sizeof p);
	//get copy of template to work with
	asprintf(&template1, "%s", template);
	s = template1;
	if(s != NULL) {
		//start and end pointers
		//successively search through template for % specifiers
		while(*s && si < max_subs && strlen(p) < 255) {
			if (*s == '%') {
				s++;
				//find which specifier it is or default to unknown
				f = strchr(spec, *s);
				if (f == NULL) {
					sp = strlen(spec);
				} else {
					sp = f-spec;
				}
				q = p + strlen(p);
				switch(sp) {
					case 0: sprintf(q, "%s", "%");break;
					case 1: sprintf(q, "%04d", localTime->tm_year+1900);break;
					case 2: sprintf(q, "%02d", (localTime->tm_year+1900) % 100);break;
					case 3: sprintf(q, "%02d", localTime->tm_mon+1);break;
					case 4: sprintf(q, "%02d", localTime->tm_mday);break;
					case 5: sprintf(q, "%02d", localTime->tm_hour);break;
					case 6: sprintf(q, "%02d", localTime->tm_min);break;
					case 7: sprintf(q, "%02d", localTime->tm_sec);break;
					case 8: sprintf(q, "%03d", currTime.tv_nsec / 1000000);break;
					case 9: sprintf(q, cfg_stru[c_count_format], video_cnt);break;
					case 10: sprintf(q, cfg_stru[c_count_format], image2_cnt);break;
					case 11: sprintf(q, cfg_stru[c_count_format], lapse_cnt);break;
					case 12: sprintf(q, "%04d", motion_frame_count);break;
					case 13: sprintf(q, "%04d", motion_changes);break;
					case 14: sprintf(q, "%02d", video_frame);break;
					case 15: 
						if (cfg_stru[c_user_annotate] != NULL) {
							fp = fopen(cfg_stru[c_user_annotate], "r");
							if (fp != NULL) {
								read_size = fread(q,sizeof(char),255, fp);
								fclose(fp);
							}
						}
						break;
				}
				si++;
				s++;
			} else {
				p[strlen(p)] = *s;
				s++;
			}
		}
	}
	asprintf(name, "%s", p); 
	free(template1);
}

void makeFilename(char** filename, char* template) {
	char *template1;
	//allow paths to be relative to media path
	if (*template != '/') {
		asprintf(&template1,"%s/%s",cfg_stru[c_media_path], template);
		makeName(filename, template1);
		free(template1);
	} else {
		makeName(filename, template);
	}
}

void createPath(char* filename, char* path) {
	char* s;
	char* t;
	int r = 0;
	struct stat buf;
	//Create folders under path in filename as needed
	if (filename != NULL && strncmp(filename, path, strlen(path)) == 0) {
		stat(path, &buf);
		//s to trailing path
		s = filename + strlen(path) + 1;
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

void createMediaPath(char *filename) {
	createPath(filename, cfg_stru[c_media_path]);
}

int copy_file(char *from_filename, char *to_filename) {
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

time_t get_mtime(const char *path) {
	struct stat statbuf;
	if (stat(path, &statbuf) == -1) {
		return 0;
	}
	return statbuf.st_mtime;
}

void makeBoxname(char** boxname, char *filename) {
	char *temp;
	//trim off extension
	char *ext = strrchr(filename, '.');
	if (ext != NULL) *ext = 0;
	if (cfg_stru[c_boxing_path] != NULL) {
		temp = strrchr(filename, '/');
		if (temp != NULL) {
			asprintf(boxname, "%s/%s.h264", cfg_stru[c_boxing_path], temp+1);
		}
	} else {
		asprintf(boxname, "%s.h264", filename);
	}
	if (ext != NULL) *ext = '.';
}

int get_box_count() {
	if (box_head >= box_tail)
		return box_head - box_tail;
	else
		return MAX_BOX_FILES + box_head - box_tail;
}

void add_box_file(char *boxfile) {
	if ((MAX_BOX_FILES - get_box_count()) > 2 ) {
		asprintf(&box_files[box_head], "%s", boxfile);
		printLog("Add %s to Box Queue at pos %d\n", box_files[box_head], box_head);
		box_head++;
		if (box_head >= MAX_BOX_FILES) box_head = 0;
	} else {
		printLog("Box queue full. Skipped %s", boxfile);
	}
}

int check_box_files() {
	char *cmd_temp = 0, *filename_temp = 0;
	int ret = 0;
	if (v_boxing > 0) {
		makeBoxname(&filename_temp, box_files[box_tail]);
		// check if current MP4Box finished by seeing if h264 now deleted
		if (access(filename_temp, F_OK ) == -1) {
			printLog("Finished boxing %s from Box Queue at pos %d\n", box_files[box_tail], box_tail);
			exec_macro(cfg_stru[c_end_box], box_files[box_tail]);
			free(box_files[box_tail]);
			box_tail++;
			if (box_tail >= MAX_BOX_FILES) box_tail = 0;
			v_boxing = 0;
			ret = 1;
		}
		free(filename_temp);
	}
	if(v_boxing == 0 && get_box_count() > 0) {
		//start new MP4Box operation
		makeBoxname(&filename_temp, box_files[box_tail]);
		if(cfg_stru[c_MP4Box_cmd] == 0) cfg_stru[c_MP4Box_cmd] = "(set -e;MP4Box -fps %i -add %s %s > /dev/null 2>&1;rm \"%s\";) &";
		asprintf(&cmd_temp, cfg_stru[c_MP4Box_cmd], cfg_val[c_MP4Box_fps], filename_temp, box_files[box_tail], filename_temp);
		printLog("Start boxing %s to %s Queue pos %d\n", filename_temp, box_files[box_tail], box_tail);
		system(cmd_temp);
		v_boxing = 1;
		free(cmd_temp);
		free(filename_temp);
	}
	return ret;
}

void check_h264_toBox() {
	char* search;
	char *n, *s, *e;
	char *h264;
	DIR *dp;
	struct dirent *fp;
    if(get_box_count() == 0) {
		// get working copy of video path
		makeFilename(&search, cfg_stru[c_video_path]);
		// find base path from last /
		s = strrchr(search, '/');
		printLog("h264 search %s\n", search);
		if (s != NULL) {
			//truncate off to get base path and open it
			*s = 0;
			dp = opendir(search);
			if (dp != NULL) {
				//scan the contents
				while ((fp = readdir(dp))) {
					n = fp->d_name;
					// check if file is h264
					e = n + strlen(n) - 5;
					if (e > n && strcmp(e, ".h264") == 0) {
						*(n + strlen(n) - 5) = 0;
						asprintf(&h264, "%s/%s.mp4", search, n);
						printLog("h264 found %s\n", h264);
						add_box_file(h264);
						free(h264);
					}
				}
				closedir (dp);
			}
		}
		free(search);
	}
}   

void send_schedulecmd(char *cmd) {
	FILE *m_pipe;

	printLog("send smd %s\n", cmd);
	if (cfg_stru[c_motion_pipe] != NULL) {
		m_pipe = fopen(cfg_stru[c_motion_pipe], "w");
		if (m_pipe != NULL) {
			fwrite(cmd, 1, 1, m_pipe);
			fclose(m_pipe);
		}
	}
}

