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

#ifndef OPTMEDIA_H
#define OPTMEDIA_H

/* public functions of optical media protocols */
struct ff_input_func_s
{
	int (* parse_file)(void *ctx, char *filename);
	void (* select_default_program)(int programid);
};

typedef struct ff_input_func_s ff_input_func_t;

/* returns 0 if path is not an optical media supported
 * if its an optical media path, calls parse_file with the right url for the optical media
 *
 * It will also call select_default_program
 * */
int parse_optmedia_path( void *ctx, const char *path, ff_input_func_t *ff_input_func );

#ifdef __GNUC__
#define BDNOT_USED __attribute__ ((unused))
#else
#define BDNOT_USED
#endif

#define OPTMEDIA_NOT_USED BDNOT_USED


#endif //!OPTMEDIA_H

