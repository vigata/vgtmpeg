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
#ifndef HB_LANG_H
#define HB_LANG_H

typedef struct iso639_lang_t
{
    const char * eng_name;        /* Description in English */
    const char * native_name;     /* Description in native language */
    const char * iso639_1;       /* ISO-639-1 (2 const characters) code */
    const char * iso639_2;        /* ISO-639-2/t (3 const character) code */
    const char * iso639_2b;       /* ISO-639-2/b code (if different from above) */

} iso639_lang_t;

#ifdef __cplusplus
extern "C" {
#endif
/* find language associated with ISO-639-1 language code */
const iso639_lang_t * lang_for_code( int code );

/* find language associated with ISO-639-2 language code */
const iso639_lang_t * lang_for_code2( const char *code2 );

/* ISO-639-1 code for language */
int lang_to_code(const iso639_lang_t *lang);

const iso639_lang_t * lang_for_english( const char * english );
#ifdef __cplusplus
}
#endif
#endif
