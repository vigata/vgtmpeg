/* @@--
 * 
 * Copyright (C) 2010-2015 Alberto Vigata
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

#ifndef __NLDUMP_FORMAT_H
#define __NLDUMP_FORMAT_H

#include "libavcodec/avcodec.h"
#include "libavutil/avstring.h"
#include "libavutil/dict.h"
#include "libavutil/pixdesc.h"


void dump_nlformat(AVFormatContext *ic,
                 int index,
                 const char *url,
                 int is_output);
#endif
