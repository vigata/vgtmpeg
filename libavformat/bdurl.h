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

#ifndef HB_BDURL_H
#define HB_BDURL_H

#include "dvdurl_common.h"
#include "libbluray/bluray.h"

struct hb_bd_s
{
    char         * path;
    BLURAY       * bd;
    int            title_count;
    BLURAY_TITLE_INFO  ** title_info;
    uint64_t       pkt_count;
//    hb_stream_t  * stream;
    int            chapter;
    int            next_chap;

    /* vgtmpeg */
    hb_buffer_t     *read_buffer;
};

typedef struct hb_bd_s hb_bd_t;

hb_optmedia_func_t *hb_optmedia_bd_methods(void);

typedef struct bdurl {
	const AVClass *class;
    hb_bd_t *hb_bd;
    hb_list_t *list_title;
    hb_title_t *selected_title;
    //int selected_title_idx;
    int selected_chapter;
    hb_buffer_t *cur_read_buffer;
    int wide_support;
    int min_title_duration;
} bdurl_t;


#endif // HB_BDURL_H


