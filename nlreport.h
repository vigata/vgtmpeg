/* @@--
 * 
 * Copyright (C) 2010-2012 Alberto Vigata
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

//#define _XOPEN_SOURCE 600
//#define STATS_DELAY 100000
#define STATS_DELAY 200000  /* delay between progress info messages */
static void print_nlreport( OutputFile **output_files,
                         OutputStream **ost_table, int nb_ostreams,
                         int is_last_report, int64_t timer_start, int nb_frames_dup, int nb_frames_drop )
{
    //char buf[1024];
    OutputStream *ost;
    AVFormatContext *oc;
    int64_t total_size;
    AVCodecContext *enc;
    int frame_number, vid, i;
    double bitrate, ti1, pts;
    static int64_t last_time = -1;
    //static int qp_histogram[52];

    if (!is_last_report) {
        int64_t cur_time;
        /* display the report every 0.5 seconds */
        cur_time = av_gettime();
        if (last_time == -1) {
            last_time = cur_time;
            return;
        }
        if ((cur_time - last_time) < STATS_DELAY )
            return;
        last_time = cur_time;
    }

    
    FFMSG_LOG( FFMSG_START );
    FFMSG_LOG( FFMSG_INT32_FMT(version_major), FFMSG_VERSION_MAJOR );
    FFMSG_LOG( FFMSG_INT32_FMT(version_minor), FFMSG_VERSION_MINOR );
    FFMSG_LOG( FFMSG_STRING_FMT(msgtype), FFMSG_MSGTYPE_PROGRESSINFO );

    FFMSG_LOG( FFMSG_NODE_START(progress) );

    oc = output_files[0]->ctx;

    total_size = avio_size(oc->pb);
    if (total_size < 0) { // FIXME improve avio_size() so it works with non seekable output too
        total_size= avio_tell(oc->pb);
        if (total_size < 0)
            total_size = 0;
    }

    //buf[0] = '\0';
    ti1 = 1e10;
    vid = 0;
    for(i=0;i<nb_ostreams;i++) {
        ost = ost_table[i];
        enc = ost->st->codec;
//        if (vid && enc->codec_type == AVMEDIA_TYPE_VIDEO) {
//            snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "q=%2.1f ",
//                     !ost->st->stream_copy ?
//                     enc->coded_frame->quality/(float)FF_QP2LAMBDA : -1);
//        }
        if (!vid && enc->codec_type == AVMEDIA_TYPE_VIDEO) {
            int fps;
            float t = (av_gettime()-timer_start) / 1000000.0;

            frame_number = ost->frame_number;
            fps = (t>1)?(int)(frame_number/t+0.5) : 0;
            /* snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "frame=%5d fps=%3d q=%3.1f ", */
                     /* frame_number, fps, */
                     /* !ost->st->stream_copy ? */
                     /* enc->coded_frame->quality/(float)FF_QP2LAMBDA : -1); */

            FFMSG_LOG( FFMSG_INT32_FMT(curframe), frame_number );
            FFMSG_LOG( FFMSG_INT32_FMT(fps), fps );

//            if(is_last_report)
//                snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "L");
            vid = 1;
        }
        /* compute min output value */
        pts = (double)ost->st->pts.val * av_q2d(ost->st->time_base);
        if ((pts < ti1) && (pts > 0))
            ti1 = pts;
    }
    if (ti1 < 0.01)
        ti1 = 0.01;

    if (1) {
        bitrate = (double)(total_size * 8) / ti1 / 1000.0;

//        snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
//            "size=%8.0fkB time=%0.2f bitrate=%6.1fkbits/s",
//            (double)total_size / 1024, ti1, bitrate);

        FFMSG_LOG( FFMSG_INTEGER_FMT(size), total_size );
        FFMSG_LOG( FFMSG_INT32_FMT(bitrate), (int)(bitrate*1000.0) );
        FFMSG_LOG( FFMSG_INT32_FMT(frames_dup), nb_frames_dup );
        FFMSG_LOG( FFMSG_INT32_FMT(frames_drop), nb_frames_drop );
        FFMSG_LOG( FFMSG_INT32_FMT(is_last_report), is_last_report );
        FFMSG_LOG( FFMSG_INT32_FMT(curtime), (int)(ti1*1000.0) );

//        if (nb_frames_dup || nb_frames_drop)
//          snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), " dup=%d drop=%d",
//                  nb_frames_dup, nb_frames_drop);

        /* if (verbose >= 0) */
            /* fprintf(stderr, "%s    \r", buf); */
/*  */
        /* fflush(stderr); */
    }

//    if (is_last_report && verbose >= 0){
//        int64_t raw= audio_size + video_size + extra_size;
//        /* fprintf(stderr, "\n"); */
//        /* fprintf(stderr, "video:%1.0fkB audio:%1.0fkB global headers:%1.0fkB muxing overhead %f%%\n", */
//                /* video_size/1024.0, */
//                /* audio_size/1024.0, */
//                /* extra_size/1024.0, */
//                /* 100.0*(total_size - raw)/raw */
//        /* ); */
//    }


    FFMSG_LOG( FFMSG_NODE_STOP(progress) );
    FFMSG_LOG( FFMSG_STOP );
}

/* codec output report */
/* outputs and array of codecs with format:
 *
 * {
 *  codecs: [
 *      {
 *          decode: true,
 *          encode: false,
 *          type: one of 'video','audio','subtitle'
 *          features: int
 *      }
 *  ]
 *  }
 */


/* escape C string so it can be used as a string literal. Make sure to call
 * c_strfree to free the string returned by c_strescape  */
static void c_strfree(char *str) { 
    free(str);
}
static char *c_strescape (const char *source)
{
    const unsigned char *p;
    char *dest;
    char *q;
    unsigned char excmap[256];

    //g_return_val_if_fail (source != NULL, NULL);
    if(!source) return NULL;

    p = (const unsigned char *) source;
    /* Each source byte needs maximally four destination chars (\777) */
    q = dest = malloc (strlen (source) * 4 + 1);

    memset (excmap, 0, 256);

    while (*p)
    {
        if (excmap[*p])
            *q++ = *p;
        else
        {
            switch (*p)
            {
                case '\b':
                    *q++ = '\\';
                    *q++ = 'b';
                    break;
                case '\f':
                    *q++ = '\\';
                    *q++ = 'f';
                    break;
                case '\n':
                    *q++ = '\\';
                    *q++ = 'n';
                    break;
                case '\r':
                    *q++ = '\\';
                    *q++ = 'r';
                    break;
                case '\t':
                    *q++ = '\\';
                    *q++ = 't';
                    break;
                case '\\':
                    *q++ = '\\';
                    *q++ = '\\';
                    break;
                case '"':
                    *q++ = '\\';
                    *q++ = '"';
                    break;
                default:
                    if ((*p < ' ') || (*p >= 0177))
                    {
                        *q++ = '\\';
                        *q++ = '0' + (((*p) >> 6) & 07);
                        *q++ = '0' + (((*p) >> 3) & 07);
                        *q++ = '0' + ((*p) & 07);
                    }
                    else
                        *q++ = *p;
                    break;
            }
        }
        p++;
    }
    *q = 0;
    return dest;
}


#define JSON_LOG(...)  av_log ( NULL, AV_LOG_INFO, __VA_ARGS__ )

#define JSON_OBJECT(x) JSON_LOG("{"); {x}; JSON_LOG("}"); 
#define JSON_PROPERTY( first, name, val) { if(!first) {JSON_LOG(",");}; JSON_LOG( "\""#name "\":"  ); {val}; }


static char *tmpstrcptr;
#define JSON_STRING_C(cstring)  { JSON_LOG("\"%s\"", tmpstrcptr=c_strescape(cstring) ); c_strfree(tmpstrcptr); }
//#define JSON_STRING_C(cstring)  { JSON_LOG("\"%s\"", cstring) ;  }
#define JSON_INT_C(val)  JSON_LOG("%d", (val));
/* if double is NaN set to zero on output. Checking NaN as compiler is supposed to return true on NaN!=NaN */
#define JSON_DOUBLE_C(val) JSON_LOG("%f", ((double)(val)!=(double)(val)) ? 0.0 : (double)(val));
#define JSON_BOOLEAN_C(val)     JSON_LOG("%s", (val) ? "true" : "false");


#define JSON_ARRAY(x) {JSON_LOG("["); {x}; JSON_LOG("]");}
#define JSON_ARRAY_ITEM(first, x) {if(!first) {JSON_LOG(",");}; {x}; }


static void show_codecs_json(void)
{
    AVCodec *p=NULL, *p2;
    const char *last_name;
    last_name= "000";
    


    JSON_OBJECT( 
            JSON_PROPERTY( 1, codecs, JSON_ARRAY(
                    int first = 1;
                    for(;;)                    {
                        int decode=0;
                        int encode=0;
                        int cap=0;
                        const char *type_str;

                        p2=NULL;
                        while((p= av_codec_next(p))) {
                            if((p2==NULL || strcmp(p->name, p2->name)<0) &&
                                strcmp(p->name, last_name)>0){
                            p2= p;
                            decode= encode= cap=0;
                        }
                        if(p2 && strcmp(p->name, p2->name)==0){
                            if(p->decode ) decode=1;
                            if(p->encode2) encode=1;
                            cap |= p->capabilities;
                            }
                        }
                        if(p2==NULL)
                            break;
        
                        last_name= p2->name;

                        switch(p2->type) {
                            case AVMEDIA_TYPE_VIDEO:
                                type_str = "video";
                                break;
                            case AVMEDIA_TYPE_AUDIO:
                                type_str = "audio";
                                break;
                            case AVMEDIA_TYPE_SUBTITLE:
                                type_str = "subtitle";
                                break;
                            default:
                                type_str = "?";
                                break;
                        }

                        JSON_ARRAY_ITEM( first, JSON_OBJECT( 
                            JSON_PROPERTY( 1, name,  JSON_STRING_C( p2->name ) );
                            JSON_PROPERTY( 0, decode, JSON_BOOLEAN_C(decode) );
                            JSON_PROPERTY( 0, encode, JSON_BOOLEAN_C(encode) );
                            JSON_PROPERTY( 0, type, JSON_STRING_C(type_str) );
                            JSON_PROPERTY( 0, caps, JSON_INT_C(cap) );
                            JSON_PROPERTY( 0, long_name,  JSON_STRING_C( p2->long_name ) );
                        ))
                        first = 0;
                    }
        ))
   );
}

static void show_formats_json(void) {
    AVInputFormat *ifmt=NULL;
    AVOutputFormat *ofmt=NULL;
    const char *last_name;
    last_name= "000";

    JSON_OBJECT( 
            JSON_PROPERTY( 1, formats, JSON_ARRAY(
                    int first = 1;
                    for(;;)                    {
                        int decode=0;
                        int encode=0;
                        const char *name=NULL;
                        const char *long_name=NULL;

                        while((ofmt= av_oformat_next(ofmt))) {
                            if((name == NULL || strcmp(ofmt->name, name)<0) &&
                                strcmp(ofmt->name, last_name)>0){
                            name= ofmt->name;
                            long_name= ofmt->long_name;
                            encode=1;
                            }
                        }

                        while((ifmt= av_iformat_next(ifmt))) {
                            if((name == NULL || strcmp(ifmt->name, name)<0) &&
                            strcmp(ifmt->name, last_name)>0){
                            name= ifmt->name;
                            long_name= ifmt->long_name;
                            encode=0;
                           }
                        
                            if(name && strcmp(ifmt->name, name)==0)
                            decode=1;
                        }
                        if(name==NULL)
                            break;
                        last_name= name;
        
                        JSON_ARRAY_ITEM( first, JSON_OBJECT( 
                            JSON_PROPERTY( 1, name,  JSON_STRING_C( name ) );
                            JSON_PROPERTY( 0, decode, JSON_BOOLEAN_C(decode) );
                            JSON_PROPERTY( 0, encode, JSON_BOOLEAN_C(encode) );
                            JSON_PROPERTY( 0, long_name,  JSON_STRING_C( long_name ) );
                        ))
                        first = 0;

                  }

        ))
   );
}


static void show_avoptions_opt_list_enum(void *obj,  const char *unit,
                     int req_flags, int rej_flags) {
    const AVOption *opt=NULL;
    //const char *class_name = (*(AVClass**)obj)->class_name;
    int first=1;
    double dblval = 0;


    while ((opt= av_opt_next(obj, opt))) {
        if (!(opt->flags & req_flags) || (opt->flags & rej_flags))
            continue;

        /* Don't print CONST's on level one.
         * Don't print anything but CONST's on level two.
         * Only print items from the requested unit.
         */
        if (!unit && opt->type==FF_OPT_TYPE_CONST)
            continue;
        else if (unit && opt->type!=FF_OPT_TYPE_CONST)
            continue;
        else if (unit && opt->type==FF_OPT_TYPE_CONST && strcmp(unit, opt->unit))
            continue;

        /* flatten numeric value to a double */
        switch(opt->type) {
        case     AV_OPT_TYPE_CONST:
        case     AV_OPT_TYPE_INT:
        case     AV_OPT_TYPE_INT64:
        	dblval = (double)opt->default_val.i64;
        	break;
        case     AV_OPT_TYPE_DOUBLE:
        case     AV_OPT_TYPE_FLOAT:
        	dblval = opt->default_val.dbl;
        	break;
        case     AV_OPT_TYPE_RATIONAL:
        	dblval = (double)opt->default_val.q.num / (double)opt->default_val.q.den;
        	break;
        case     AV_OPT_TYPE_FLAGS:
        case     AV_OPT_TYPE_STRING:
        case     AV_OPT_TYPE_BINARY:

        case     AV_OPT_TYPE_IMAGE_SIZE:
        case     AV_OPT_TYPE_PIXEL_FMT:
        case     AV_OPT_TYPE_SAMPLE_FMT:
        case     AV_OPT_TYPE_VIDEO_RATE:
        case     AV_OPT_TYPE_DURATION:
        	break;
        }


        JSON_ARRAY_ITEM( first, 
                JSON_OBJECT( 
                    JSON_PROPERTY( 1, name, JSON_STRING_C(opt->name) );
                    JSON_PROPERTY( 0, value, JSON_DOUBLE_C(dblval) );
                    if( opt->help ) JSON_PROPERTY( 0, help, JSON_STRING_C(opt->help) );
                    );
                );
        first = 0;
    }
}


static void show_avoptions_json(void *obj, int req_flags, int rej_flags, const char *klass, int add_flags )
{
    const AVOption *opt=NULL;
    const char *type;
    const char *unit=NULL;
    //char *class_name = (*(AVClass**)obj)->class_name;
    if (!obj)
        return;


    while ((opt= av_opt_next(obj, opt))) {
        if (!(opt->flags & req_flags) || (opt->flags & rej_flags))
            continue;

        /* Don't print CONST's on level one.
         * Don't print anything but CONST's on level two.
         * Only print items from the requested unit.
         */
        if (!unit && opt->type==FF_OPT_TYPE_CONST)
            continue;
        else if (unit && opt->type!=FF_OPT_TYPE_CONST)
            continue;
        else if (unit && opt->type==FF_OPT_TYPE_CONST && strcmp(unit, opt->unit))
            continue;
        /* else if (unit && opt->type == FF_OPT_TYPE_CONST) */
            /* av_log(av_log_obj, AV_LOG_INFO, "   %-15s ", opt->name); */
        /* else */
            /* av_log(av_log_obj, AV_LOG_INFO, "-%-17s ", opt->name); */

        switch (opt->type) {
            case FF_OPT_TYPE_FLAGS:
                type = "flags";
                break;
            case FF_OPT_TYPE_INT:
                type = "int";
                break;
            case FF_OPT_TYPE_INT64:
                type = "int64";
                break;
            case FF_OPT_TYPE_DOUBLE:
                type = "double";
                break;
            case FF_OPT_TYPE_FLOAT:
                type = "float";
                break;
            case FF_OPT_TYPE_STRING:
                type = "string";
                break;
            case FF_OPT_TYPE_RATIONAL:
                type = "rational";
                break;
            case FF_OPT_TYPE_BINARY:
                type = "binary";
                break;
            case FF_OPT_TYPE_CONST:
                type = "const";
            default:
                type = "uknown";
                break;
        }

    JSON_ARRAY_ITEM( 0, 
    JSON_OBJECT( 
            JSON_PROPERTY( 1, domain,  JSON_STRING_C( "avcontext" ) );                    
            JSON_PROPERTY( 0, flags, JSON_INT_C(opt->flags | add_flags) ); 
            JSON_PROPERTY( 0, klass, JSON_STRING_C(klass) );
            JSON_PROPERTY( 0, name, JSON_STRING_C(opt->name) );
            JSON_PROPERTY( 0, type, JSON_STRING_C(type) );
            JSON_PROPERTY( 0, typeint, JSON_INT_C(opt->type) );
            JSON_PROPERTY( 0, def, JSON_DOUBLE_C(opt->default_val.dbl) );
            /* JSON_PROPERTY( 0, encode, JSON_BOOLEAN_C( opt->flags & AV_OPT_FLAG_ENCODING_PARAM ) ); */
            /* JSON_PROPERTY( 0, decode, JSON_BOOLEAN_C( opt->flags & AV_OPT_FLAG_DECODING_PARAM ) ); */
            /* JSON_PROPERTY( 0, video,  JSON_BOOLEAN_C( opt->flags & AV_OPT_FLAG_VIDEO_PARAM ) ); */
            /* JSON_PROPERTY( 0, audio,    JSON_BOOLEAN_C( opt->flags & AV_OPT_FLAG_AUDIO_PARAM ) ); */
            /* JSON_PROPERTY( 0, subtitle, JSON_BOOLEAN_C( opt->flags & AV_OPT_FLAG_SUBTITLE_PARAM ) ); */
            if( opt->help ) JSON_PROPERTY( 0, help, JSON_STRING_C(opt->help) );
           

        /* av_log(av_log_obj, AV_LOG_INFO, "%c", (opt->flags & AV_OPT_FLAG_ENCODING_PARAM) ? 'E' : '.'); */
        /* av_log(av_log_obj, AV_LOG_INFO, "%c", (opt->flags & AV_OPT_FLAG_DECODING_PARAM) ? 'D' : '.'); */
        /* av_log(av_log_obj, AV_LOG_INFO, "%c", (opt->flags & AV_OPT_FLAG_VIDEO_PARAM   ) ? 'V' : '.'); */
        /* av_log(av_log_obj, AV_LOG_INFO, "%c", (opt->flags & AV_OPT_FLAG_AUDIO_PARAM   ) ? 'A' : '.'); */
        /* av_log(av_log_obj, AV_LOG_INFO, "%c", (opt->flags & AV_OPT_FLAG_SUBTITLE_PARAM) ? 'S' : '.'); */

        /* if (opt->help) */
            /* av_log(av_log_obj, AV_LOG_INFO, " %s", opt->help); */
        /* av_log(av_log_obj, AV_LOG_INFO, "\n"); */

            {
                int has_enum = opt->unit && opt->type!=FF_OPT_TYPE_CONST;
                JSON_PROPERTY( 0, has_enum, JSON_BOOLEAN_C(has_enum) );

                if ( has_enum ) {
                    JSON_PROPERTY(0, enums, JSON_ARRAY(
                                show_avoptions_opt_list_enum(obj,  opt->unit, req_flags, rej_flags);
                                ));
                }
            }
            )
        );
    }
}



static void show_help_options_json(const OptionDef *po)
{
    JSON_OBJECT(
            JSON_PROPERTY( 1, domain,  JSON_STRING_C( "general" ) );                    
            JSON_PROPERTY( 0, flags, JSON_INT_C(po->flags) );
            JSON_PROPERTY( 0, name, JSON_STRING_C(po->name) );
            JSON_PROPERTY( 0, has_arg, JSON_BOOLEAN_C(po->flags & HAS_ARG) );
            if( po->flags &HAS_ARG ) JSON_PROPERTY(0, arg, JSON_STRING_C(po->argname));
            JSON_PROPERTY( 0, help, JSON_STRING_C(po->help) );
    );
}

#define AV_OPT_FLAG_FORMAT_PARAM (1<<17)
static void avcontext_flagdef(void) {
    JSON_OBJECT(
            JSON_PROPERTY( 1, name, JSON_STRING_C("avflags") );
            JSON_PROPERTY( 0, def,  JSON_OBJECT(
                    JSON_PROPERTY(1, AV_OPT_FLAG_ENCODING_PARAM, JSON_INT_C(AV_OPT_FLAG_ENCODING_PARAM));
                    JSON_PROPERTY(0, AV_OPT_FLAG_DECODING_PARAM, JSON_INT_C(AV_OPT_FLAG_DECODING_PARAM));
                    JSON_PROPERTY(0, AV_OPT_FLAG_METADATA, JSON_INT_C(AV_OPT_FLAG_METADATA));
                    JSON_PROPERTY(0, AV_OPT_FLAG_AUDIO_PARAM, JSON_INT_C(AV_OPT_FLAG_AUDIO_PARAM));
                    JSON_PROPERTY(0, AV_OPT_FLAG_VIDEO_PARAM, JSON_INT_C(AV_OPT_FLAG_VIDEO_PARAM));
                    JSON_PROPERTY(0, AV_OPT_FLAG_FORMAT_PARAM, JSON_INT_C(AV_OPT_FLAG_FORMAT_PARAM));
                    JSON_PROPERTY(0, AV_OPT_FLAG_SUBTITLE_PARAM, JSON_INT_C(AV_OPT_FLAG_SUBTITLE_PARAM));
                    ));
            )
}

static void avcontext_typedef(void) {
    JSON_OBJECT(
            JSON_PROPERTY( 1, name, JSON_STRING_C("avtype") );
            JSON_PROPERTY( 0, def,  JSON_OBJECT(
                    JSON_PROPERTY(1, FF_OPT_TYPE_FLAGS, JSON_INT_C(FF_OPT_TYPE_FLAGS));
                    JSON_PROPERTY(0, FF_OPT_TYPE_INT, JSON_INT_C(FF_OPT_TYPE_INT));
                    JSON_PROPERTY(0, FF_OPT_TYPE_INT64, JSON_INT_C(FF_OPT_TYPE_INT64));
                    JSON_PROPERTY(0, FF_OPT_TYPE_DOUBLE, JSON_INT_C(FF_OPT_TYPE_DOUBLE));
                    JSON_PROPERTY(0, FF_OPT_TYPE_FLOAT, JSON_INT_C(FF_OPT_TYPE_FLOAT));
                    JSON_PROPERTY(0, FF_OPT_TYPE_STRING, JSON_INT_C(FF_OPT_TYPE_STRING));
                    JSON_PROPERTY(0, FF_OPT_TYPE_RATIONAL, JSON_INT_C(FF_OPT_TYPE_RATIONAL));
                    JSON_PROPERTY(0, FF_OPT_TYPE_BINARY, JSON_INT_C(FF_OPT_TYPE_BINARY));
                    JSON_PROPERTY(0, FF_OPT_TYPE_CONST, JSON_INT_C(FF_OPT_TYPE_CONST));
                    ));
            )
}  
static void show_help_options_json_flagdef(void) {
    JSON_OBJECT(
            JSON_PROPERTY( 1, name, JSON_STRING_C("gflags") );
            JSON_PROPERTY( 0, def,  JSON_OBJECT(
                    JSON_PROPERTY(1, HAS_ARG    ,JSON_INT_C(HAS_ARG));
                    JSON_PROPERTY(0, OPT_BOOL   ,JSON_INT_C(OPT_BOOL));
                    JSON_PROPERTY(0, OPT_EXPERT ,JSON_INT_C(OPT_EXPERT));
                    JSON_PROPERTY(0, OPT_STRING ,JSON_INT_C(OPT_STRING));
                    JSON_PROPERTY(0, OPT_VIDEO  ,JSON_INT_C(OPT_VIDEO));
                    JSON_PROPERTY(0, OPT_AUDIO  ,JSON_INT_C(OPT_AUDIO));
                    JSON_PROPERTY(0, OPT_INT    ,JSON_INT_C(OPT_INT));
                    JSON_PROPERTY(0, OPT_FLOAT  ,JSON_INT_C(OPT_FLOAT));
                    JSON_PROPERTY(0, OPT_SUBTITLE ,JSON_INT_C(OPT_SUBTITLE));
                    JSON_PROPERTY(0, OPT_INT64  ,JSON_INT_C(OPT_INT64));
                    JSON_PROPERTY(0, OPT_EXIT   ,JSON_INT_C(OPT_EXIT));
                    JSON_PROPERTY(0, OPT_DATA   ,JSON_INT_C(OPT_DATA));
                    ));
            )
}

static void show_options_children(const AVClass *class, int flags, int addflags, const char  *context_name_def ) {
    const AVClass *child = NULL;
    const char *context_name = context_name_def;
    AVCodec *c;
    AVOutputFormat *of;
    AVInputFormat *ifo;

    /* retrieve context name for this class */
    c = NULL;
    while((c=av_codec_next(c))) {
        if(c->priv_class && !strcmp(c->priv_class->class_name, class->class_name)) {
            context_name = c->name;
        }
    }

    /* maybe its a format */
    of = NULL;
    while((of=av_oformat_next(of))) {
        if(of->priv_class && !strcmp(of->priv_class->class_name, class->class_name)) {
            context_name = of->name;
        }
    }

    /* maybe its a format */
    ifo = NULL;
    while((ifo=av_iformat_next(ifo))) {
        if(ifo->priv_class && !strcmp(ifo->priv_class->class_name, class->class_name)) {
            context_name = ifo->name;
        }
    }

    show_avoptions_json(&class, flags, 0, context_name, addflags);
    printf("\n");

    while (child = av_opt_child_class_next(class, child))
        show_options_children(child, flags, addflags, child->class_name );
}

static void show_options_json(void) {
    /* general options  */
    const OptionDef *po;
    int first = 1;

    JSON_OBJECT(
            JSON_PROPERTY(1, flagdef, JSON_ARRAY(
                    JSON_ARRAY_ITEM(1, show_help_options_json_flagdef(); );
                    JSON_ARRAY_ITEM(0, avcontext_flagdef(); );
                    JSON_ARRAY_ITEM(0, avcontext_typedef(); );
                    )); 
            JSON_PROPERTY(0, options, JSON_ARRAY( 

            for( po=options; po->name!=NULL; po++ ) {
            JSON_ARRAY_ITEM( first, show_help_options_json(po); );
            first = 0;
            }

            /* codec options */
            show_options_children(avcodec_get_class(), AV_OPT_FLAG_ENCODING_PARAM|AV_OPT_FLAG_DECODING_PARAM,0, "" );
            /* muxer options */
            show_options_children(avformat_get_class(), AV_OPT_FLAG_ENCODING_PARAM|AV_OPT_FLAG_DECODING_PARAM,AV_OPT_FLAG_FORMAT_PARAM, "" );
            /* sws */
            show_options_children(sws_get_class(), AV_OPT_FLAG_ENCODING_PARAM|AV_OPT_FLAG_DECODING_PARAM,0, sws_get_class()->class_name );


            //show_avoptions_json(avcodec_opts[1], , 2, "", 0);

            /* individual codec options */
            /* c = NULL; */
            /* while ((c = av_codec_next(c))) { */
                /* if (c->priv_class) { */
                    /* show_avoptions_json(&c->priv_class, AV_OPT_FLAG_ENCODING_PARAM|AV_OPT_FLAG_DECODING_PARAM, 0, c->name, 0); */
                /* } */
            /* } */


            /*muxer options */
            //show_avoptions_json(avformat_opts, AV_OPT_FLAG_ENCODING_PARAM|AV_OPT_FLAG_DECODING_PARAM, 0, "", AV_OPT_FLAG_FORMAT_PARAM );

            /* individual muxer options */
            /* while ((oformat = av_oformat_next(oformat))) { */
                /* if (oformat->priv_class) { */
                /* show_avoptions_json(&oformat->priv_class, AV_OPT_FLAG_ENCODING_PARAM, 0, oformat->name, AV_OPT_FLAG_FORMAT_PARAM ); */
                /* } */
            /* } */
/*  */
            /* show_avoptions_json(sws_opts, AV_OPT_FLAG_ENCODING_PARAM|AV_OPT_FLAG_DECODING_PARAM, 0, "", AV_OPT_FLAG_FORMAT_PARAM ); */
    ))
        )

}



#endif
