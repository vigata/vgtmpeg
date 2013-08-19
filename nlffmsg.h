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

#ifndef __NLFFMSG_H
#define __NLFFMSG_H

#include <inttypes.h>

#define FFMSG_VERSION_MAJOR 0
#define FFMSG_VERSION_MINOR 3

#define FFMSG_MSGTYPE_STREAMINFO "streaminfo"
#define FFMSG_MSGTYPE_PROGRESSINFO "progressinfo"


#define FFMSG_START "<nlffmsg>\n"
#define FFMSG_STOP "</nlffmsg>\n"


#define FFMSG_NODE_START(x) "<" #x ">\n"
#define FFMSG_NODE_STOP(x)   "</" #x ">\n" 
#define FFMSG_NODE_START_FMT(x) "<" x ">\n"
#define FFMSG_NODE_STOP_FMT(x)   "</" x ">\n" 


#define FFMSG_STRING_FMT(name) "<" #name " type=\"string\" val=\"%s\"/>\n"
#define FFMSG_INTEGER_FMT(name) "<" #name " type=\"integer\" val=\"%" PRIi64 "\"/>\n"
#define FFMSG_INT32_FMT(name) "<" #name " type=\"integer\" val=\"%d\"/>\n"

#define FFMSG_LOG(...)  av_log ( NULL, AV_LOG_INFO, __VA_ARGS__ )

#define FFMSG_PICTURE_START(width,height,format) FFMSG_LOG("<nlpicmsg width=\"%d\" height=\"%d\" format=\"%d\">\n", width, height, format )
#define FFMSG_PICTURE_DATA(b64data) fputs(b64data,stderr)
#define FFMSG_PICTURE_STOP()   FFMSG_LOG("</nlpicmsg>\n")

#define FFMSG_START_MSGTYPE( type, mainkey) \
    FFMSG_LOG( FFMSG_START );   \
    FFMSG_LOG( FFMSG_INT32_FMT(version_major), FFMSG_VERSION_MAJOR );\
    FFMSG_LOG( FFMSG_INT32_FMT(version_minor), FFMSG_VERSION_MINOR );\
    FFMSG_LOG( FFMSG_STRING_FMT(msgtype), type  ); \
    FFMSG_LOG( FFMSG_NODE_START(mainkey) );

#define FFMSG_STOP_MSGTYPE(type, mainkey) \
    FFMSG_LOG( FFMSG_NODE_STOP(mainkey) ); \
    FFMSG_LOG( FFMSG_STOP );

/* buffer to use for escaped xml strings. this allows for a maximum of 2048 escaped characters
 * FIXME the buffer needs to move into an app context to ensure thread safety  */
#define MAX_FFGMT_STRING_LEN 2048
static char xmlesc1[MAX_FFGMT_STRING_LEN*6];
static char xmlesc2[MAX_FFGMT_STRING_LEN*6];

static char *xescape(char *buf, char *s) {
	int i=MAX_FFGMT_STRING_LEN;
	char *o = buf;
	while(i-- && *s ) {
		switch(*s) {
		case '<':
			*o++ = '&'; *o++ = 'l'; *o++ = 't'; *o++ = ';';
			break;
		case '>':
			*o++ = '&'; *o++ = 'g'; *o++ = 't'; *o++ = ';';
			break;
		case '"':
			*o++ = '&'; *o++ = 'q'; *o++ = 'u'; *o++ = 'o'; *o++ = 't'; *o++ = ';';
			break;
		case '\'':
			*o++ = '&'; *o++ = 'a'; *o++ = 'p'; *o++ = 'o'; *o++ = 's'; *o++ = ';';
			break;
		case '&':
			*o++ = '&'; *o++ = 'a'; *o++ = 'm'; *o++ = 'p'; *o++ = ';';
			break;        
		default:
            /* ignore not allowed xml characters */
            if( ((unsigned char)(*s)) >= 0x20 ) *o++ = *s;
			break;
		}
		s++;
	}
	*o=0;
	return buf;
}

#define FFMSG_STRING_VALUE(name,value) {FFMSG_LOG("<%s type=\"string\" val=\"%s\"/>\n", xescape(xmlesc1,name), xescape(xmlesc2,value));}
/* definitions 
 *
 * ffnlmsg.version.major        (integer)
 * ffnlmsg.version.minor        (integer)
 * ffnlmsg.muxinfo.direction (string)   [input|output]
 * ffnlmsg.muxinfo.index      (integer) 
 * ffnlmsg.muxinfo.timebase   (integer) 
 * ffnlmsg.muxinfo.mux_format (string)  [mpeg4|...]
 * ffnlmsg.muxinfo.program_count  (integer) 
 * ffnlmsg.muxinfo.stream_count     (integer)
 * ffnlmsg.muxinfo.duration                 (integer)
 * ffnlmsg.muxinfo.start_time               (integer)
 * ffnlmsg.muxinfo.bitrate                  (integer)
 * ffnlmsg.muxinfo.programs                 (array)
 * ffnlmsg.muxinfo.programs.id[n]           (program info)
 * ffnlmsg.muxinfo.programs.id[n].id        (integer)
 * ffnlmsg.muxinfo.programs.id[n].name      (string)
 *  
 *  ...
 */
#endif /* __NLFFMSG_H */
