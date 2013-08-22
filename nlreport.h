/* @@--
 * 
 * Copyright (C) 2010-2013 Alberto Vigata
 *       
 * This file is part of vgtmpeg
 * 
 * a Versed Generalist Transcoder
 * 
 * vgtmpeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 * 
 * vgtmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#ifndef __NLREPORT_H
#define __NLREPORT_H

#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "ffmpeg.h"

void print_nlreport( OutputFile **output_files,
                         OutputStream **ost_table, int nb_ostreams,
                         int is_last_report, int64_t timer_start, int nb_frames_dup, int nb_frames_drop );


void c_strfree(char *str);  
char *c_strescape (const char *source);


void show_codecs_json(void);
void show_formats_json(void);
void show_options_json(void);



#endif
