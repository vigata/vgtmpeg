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

#ifndef HB_DVD_H
#define HB_DVD_H

#include "url.h"
#include "dvdurl_common.h"
#include "dvdread/ifo_read.h"
#include "dvdread/nav_read.h"



struct hb_dvdread_s
{
    char         * path;

    dvd_reader_t * reader;
    ifo_handle_t * vmg;

    int            vts;
    int            ttn;
    ifo_handle_t * ifo;
    dvd_file_t   * file;

    pgc_t        * pgc;
    int            cell_start;
    int            cell_end;
    int            title_start;
    int            title_end;
    int            title_block_count;
    int            cell_cur;
    int            cell_next;
    int            cell_overlap;
    int            block;
    int            pack_len;
    int            next_vobu;
    int            in_cell;
    int            in_sync;
    uint16_t       cur_vob_id;
    uint8_t        cur_cell_id;

    /* vgtmpeg */
    hb_buffer_t     *read_buffer;
};


typedef struct hb_dvdread_s hb_dvdread_t;

union hb_dvd_s
{
    hb_dvdread_t dvdread;
};

typedef union  hb_dvd_s hb_dvd_t;

//struct hb_dvd_func_s
//{
//    hb_dvd_t *    (* init)        ( char * );
//    void          (* close)       ( hb_dvd_t ** );
//    char        * (* name)        ( char * );
//    int           (* title_count) ( hb_dvd_t * );
//    hb_title_t  * (* title_scan)  ( hb_dvd_t *, int, uint64_t );
//    int           (* start)       ( hb_dvd_t *, hb_title_t *, int );
//    void          (* stop)        ( hb_dvd_t * );
//    int           (* seek)        ( hb_dvd_t *, float );
//    hb_buffer_t * (* read)        ( hb_dvd_t * );
//    int           (* chapter)     ( hb_dvd_t * );
//    int           (* angle_count) ( hb_dvd_t * );
//    void          (* set_angle)   ( hb_dvd_t *, int );
//    int           (* main_feature)( hb_dvd_t *, hb_list_t * );
//};
//typedef struct hb_dvd_func_s hb_dvd_func_t;
//
//hb_dvd_func_t * hb_dvdread_methods( void );
hb_optmedia_func_t *hb_optmedia_dvd_methods(void);



typedef struct dvdurl_s {
	const AVClass *class;
    hb_list_t *list_title;
    hb_dvd_t *hb_dvd;
    hb_title_t *selected_title;
    int selected_title_idx;
    int selected_chapter;
    hb_buffer_t *cur_read_buffer;
    int wide_support;
    int min_title_duration;
} dvdurl_t;

/* returns 1 if the path indicated contains a valid path that will be opened
 * with dvd url
 */
int parse_dvd_path(void *ctx, char *opt, const char *path,  int (* parse_file)(void *ctx, char *opt, char *filename), void (* select_default_program)(int programid) );


#endif // HB_DVD_H


