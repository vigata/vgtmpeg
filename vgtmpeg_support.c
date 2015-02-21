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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "nlffmsg.h"
#include "nlinput.h"
#include "nlreport.h"
#include "nldump_format.h"
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "cmdutils.h"



/****************************************************************/
/* utils                                                        */
/****************************************************************/
/* buffer to use for escaped xml strings. this allows for a maximum of 2048 escaped characters
 * FIXME the buffer needs to move into an app context to ensure thread safety  */
#define MAX_FFGMT_STRING_LEN 2048
static char xmlesc1[MAX_FFGMT_STRING_LEN*6];
static char xmlesc2[MAX_FFGMT_STRING_LEN*6];


char *xescape(char *buf, char *s) {
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
            /* ignore not ascii characters */
            if( (((unsigned char)(*s)) >= 0x20) && (((unsigned char)(*s)) <= 0x7f )  ) *o++ = *s;
			break;
		}
		s++;
	}
	*o=0;
	return buf;
}


/****************************************************************/
/* nlinput                                                      */
/****************************************************************/
#define sl0(x) (x)
#define sl8(x) ((x)<<8)
#define sl16(x) ((x)<<16)
#define sl24(x) ((x)<<24)

#define VGTM_W  ( sl24('V') | sl16('G') | sl8('T') | sl0('M') )
#define MTGV_W  ( sl24('M') | sl16('T') | sl8('G') | sl0('V') )

#define CB2INT(x) ( sl24(x[0]) | sl16(x[1]) | sl8(x[2]) | sl0(x[3]) ) 
static int nlinput_readbyte(char *val) {
    int read = fread( val, 1, 1, stdin );
    return read==1;
}


/* message code definitions */
#define EXIT                101
#define CANCEL_TRANSCODE    99

static void * nlinput_start(void *p) {
    nlinput_t *ctx = (nlinput_t *)p;
    unsigned char b;
    int loop = 1;

    while(loop) {

       printf("nlinput: about to read\n");
       if(!nlinput_readbyte(&b)) 
           break;

       printf("nlinput: read %x(%c)\n",b,b);
       switch(b) {
           case EXIT:
               printf("nlinput: exiting\n");
               ctx->exit = 1;
               loop = 0;
               break;
           case CANCEL_TRANSCODE:
               printf("nlinput: canceling transcode\n");
               ctx->cancel_transcode = 1;
               break;
       }
    }

    return 0;
}

/* fires up input thread */
nlinput_t *nlinput_prepare(void) {
    /* starting nlinput */
    nlinput_t *ret = malloc( sizeof(nlinput_t) );
    memset( ret, 0, sizeof (nlinput_t) );

    //pthread_attr_init(&nlin_attr);
    //pthread_attr_setdetachstate(&nlin_attr, PTHREAD_CREATE_JOINABLE );
    pthread_create( &ret->nlin_th, NULL /*nlin_attr*/, nlinput_start, (void *)ret );

    return ret;
}

/* shutdown input thread */
void nlinput_cancel(nlinput_t *ctx) {
    void *status;
    printf("nlinput: cancel\n");

    pthread_join( ctx->nlin_th, &status );
}







/****************************************************************/
/* JSON output                                                  */
/****************************************************************/
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


void show_codecs_json(void)
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

void show_formats_json(void) {
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

/**
 * Given an avoption returns the default in double format.
 * This is useful to transform default value to a 80bit double for JSON output.
 * Returns 0 if it was not possible to coalesce value
 */
static double show_avoptions_get_double_default(const AVOption *opt)
{
	double dblval = 0;

    /* flatten numeric value to a double */
	switch (opt->type) {
	case AV_OPT_TYPE_FLAGS:
	case AV_OPT_TYPE_DURATION:
	case AV_OPT_TYPE_CHANNEL_LAYOUT:
	case AV_OPT_TYPE_CONST:
	case AV_OPT_TYPE_INT:
	case AV_OPT_TYPE_INT64:
	case AV_OPT_TYPE_PIXEL_FMT:
	case AV_OPT_TYPE_SAMPLE_FMT:
		dblval = (double) opt->default_val.i64;
		break;
	case AV_OPT_TYPE_DOUBLE:
	case AV_OPT_TYPE_FLOAT:
		dblval = opt->default_val.dbl;
		break;
	case AV_OPT_TYPE_RATIONAL:
		dblval = (double) opt->default_val.q.num
				/ (double) opt->default_val.q.den;
		break;
	case AV_OPT_TYPE_STRING:
	case AV_OPT_TYPE_BINARY:
	case AV_OPT_TYPE_COLOR:
	case AV_OPT_TYPE_IMAGE_SIZE:
	case AV_OPT_TYPE_VIDEO_RATE:
		break;
	}

	return dblval;
}

/**
 * Give an avoption returns the default in string format.
 * returns "" if not relevant
 */
static const char * show_avoptions_get_string_default(const AVOption *opt)
{
	const char *ret = "";

	switch(opt->type) {
	case AV_OPT_TYPE_COLOR:
	case AV_OPT_TYPE_IMAGE_SIZE:
	case AV_OPT_TYPE_STRING:
	case AV_OPT_TYPE_VIDEO_RATE:
		ret = opt->default_val.str;
		break;
	default:
		ret = "";
		break;
	}

	return ret;
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

        dblval = show_avoptions_get_double_default(opt);

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
        if (!unit && opt->type==AV_OPT_TYPE_CONST)
            continue;
        else if (unit && opt->type!=AV_OPT_TYPE_CONST)
            continue;
        else if (unit && opt->type==AV_OPT_TYPE_CONST && strcmp(unit, opt->unit))
            continue;
        /* else if (unit && opt->type == FF_OPT_TYPE_CONST) */
            /* av_log(av_log_obj, AV_LOG_INFO, "   %-15s ", opt->name); */
        /* else */
            /* av_log(av_log_obj, AV_LOG_INFO, "-%-17s ", opt->name); */

        switch (opt->type) {
            case AV_OPT_TYPE_FLAGS:
                type = "flags";
                break;
            case AV_OPT_TYPE_INT:
                type = "int";
                break;
            case AV_OPT_TYPE_INT64:
                type = "int64";
                break;
            case AV_OPT_TYPE_DOUBLE:
                type = "double";
                break;
            case AV_OPT_TYPE_FLOAT:
                type = "float";
                break;
            case AV_OPT_TYPE_STRING:
                type = "string";
                break;
            case AV_OPT_TYPE_RATIONAL:
                type = "rational";
                break;
            case AV_OPT_TYPE_BINARY:
                type = "binary";
                break;
            case AV_OPT_TYPE_IMAGE_SIZE:
                type = "image_size";
                break;
            case AV_OPT_TYPE_VIDEO_RATE:
                type = "video_rate";
                break;
            case AV_OPT_TYPE_PIXEL_FMT:
                type = "pixel_fmt";
                break;
            case AV_OPT_TYPE_SAMPLE_FMT:
                type = "sample_fmt";
                break;
            case AV_OPT_TYPE_DURATION:
                type = "duration";
                break;
            case AV_OPT_TYPE_COLOR:
                type = "color";
                break;
            case AV_OPT_TYPE_CHANNEL_LAYOUT:
                type = "channel_layout";
                break;
            case AV_OPT_TYPE_CONST:
                type = "const";
                break;
            default:
                type = "unknown";
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
            JSON_PROPERTY( 0, def, JSON_DOUBLE_C( show_avoptions_get_double_default(opt)) );
            JSON_PROPERTY( 0, defstr,JSON_STRING_C( show_avoptions_get_string_default(opt)) );


            if( opt->help ) JSON_PROPERTY( 0, help, JSON_STRING_C(opt->help) );

            {
                int has_enum = opt->unit && opt->type!=AV_OPT_TYPE_CONST;
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
                    JSON_PROPERTY(1, AV_OPT_TYPE_FLAGS, JSON_INT_C(AV_OPT_TYPE_FLAGS));
                    JSON_PROPERTY(0, AV_OPT_TYPE_INT, JSON_INT_C(AV_OPT_TYPE_INT));
                    JSON_PROPERTY(0, AV_OPT_TYPE_INT64, JSON_INT_C(AV_OPT_TYPE_INT64));
                    JSON_PROPERTY(0, AV_OPT_TYPE_DOUBLE, JSON_INT_C(AV_OPT_TYPE_DOUBLE));
                    JSON_PROPERTY(0, AV_OPT_TYPE_FLOAT, JSON_INT_C(AV_OPT_TYPE_FLOAT));
                    JSON_PROPERTY(0, AV_OPT_TYPE_STRING, JSON_INT_C(AV_OPT_TYPE_STRING));
                    JSON_PROPERTY(0, AV_OPT_TYPE_RATIONAL, JSON_INT_C(AV_OPT_TYPE_RATIONAL));
                    JSON_PROPERTY(0, AV_OPT_TYPE_BINARY, JSON_INT_C(AV_OPT_TYPE_BINARY));
                    JSON_PROPERTY(0, AV_OPT_TYPE_CONST, JSON_INT_C(AV_OPT_TYPE_CONST));
                    JSON_PROPERTY(0, AV_OPT_TYPE_IMAGE_SIZE, JSON_INT_C(AV_OPT_TYPE_IMAGE_SIZE));
                    JSON_PROPERTY(0, AV_OPT_TYPE_PIXEL_FMT, JSON_INT_C(AV_OPT_TYPE_PIXEL_FMT));
                    JSON_PROPERTY(0, AV_OPT_TYPE_SAMPLE_FMT, JSON_INT_C(AV_OPT_TYPE_SAMPLE_FMT));
                    JSON_PROPERTY(0, AV_OPT_TYPE_VIDEO_RATE, JSON_INT_C(AV_OPT_TYPE_VIDEO_RATE));
                    JSON_PROPERTY(0, AV_OPT_TYPE_DURATION, JSON_INT_C(AV_OPT_TYPE_DURATION));
                    JSON_PROPERTY(0, AV_OPT_TYPE_COLOR, JSON_INT_C(AV_OPT_TYPE_COLOR));
                    JSON_PROPERTY(0, AV_OPT_TYPE_CHANNEL_LAYOUT, JSON_INT_C(AV_OPT_TYPE_CHANNEL_LAYOUT));
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
                    JSON_PROPERTY(0, OPT_PERFILE   ,JSON_INT_C(OPT_PERFILE));
                    JSON_PROPERTY(0, OPT_OFFSET   ,JSON_INT_C(OPT_OFFSET));
                    JSON_PROPERTY(0, OPT_SPEC   ,JSON_INT_C(OPT_SPEC));
                    JSON_PROPERTY(0, OPT_TIME   ,JSON_INT_C(OPT_TIME));
                    JSON_PROPERTY(0, OPT_DOUBLE   ,JSON_INT_C(OPT_DOUBLE));
                    JSON_PROPERTY(0, OPT_INPUT   ,JSON_INT_C(OPT_INPUT));
                    JSON_PROPERTY(0, OPT_OUTPUT   ,JSON_INT_C(OPT_OUTPUT));
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

/**
 * show_options_json()
 *
 * shows all the available options for vgtmpeg on stdout
 *
 */
void show_options_json(void) {
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


/****************************************************************/
/* nldump format                                                */
/****************************************************************/
static int get_bit_rate(AVCodecContext *ctx)
{
    int bit_rate;
    int bits_per_sample;

    switch(ctx->codec_type) {
    case AVMEDIA_TYPE_VIDEO:
    case AVMEDIA_TYPE_DATA:
    case AVMEDIA_TYPE_SUBTITLE:
    case AVMEDIA_TYPE_ATTACHMENT:
        bit_rate = ctx->bit_rate;
        break;
    case AVMEDIA_TYPE_AUDIO:
        bits_per_sample = av_get_bits_per_sample(ctx->codec_id);
        bit_rate = bits_per_sample ? ctx->sample_rate * ctx->channels * bits_per_sample : ctx->bit_rate;
        break;
    default:
        bit_rate = 0;
        break;
    }
    return bit_rate;
}

static void nl_dump_metadata(AVDictionary *m)
{
    if(m ){
        AVDictionaryEntry *tag=NULL;

        FFMSG_LOG(FFMSG_NODE_START(metadata));

        while((tag=av_dict_get(m, "", tag, AV_DICT_IGNORE_SUFFIX))) {
            char tmp[1024];
            int i;
            av_strlcpy(tmp, tag->value, sizeof(tmp));
            for(i=0; i<strlen(tmp); i++) if(tmp[i]==0xd) tmp[i]=' ';

            FFMSG_STRING_VALUE(tag->key, tmp);
        }
        FFMSG_LOG(FFMSG_NODE_STOP(metadata));
    }
}

static void avcodec_nlstring(char *buf, int buf_size, AVCodecContext *enc, int encode) {
    const char *codec_name;
    const char *profile = "";
    const AVCodec *p;
    // char tag_buf[32];
    int bitrate;
    AVRational display_aspect_ratio;


    codec_name = avcodec_get_name( enc->codec_id );
    if (enc->profile != FF_PROFILE_UNKNOWN) {
        if (enc->codec)
            p = enc->codec;
        else
            p = encode ? avcodec_find_encoder(enc->codec_id) :
                        avcodec_find_decoder(enc->codec_id);
        if (p)
            profile = av_get_profile_name(p, enc->profile);
    }

    /*
    if (enc->codec_tag) {        
        av_get_codec_tag_string(tag_buf, sizeof(tag_buf), enc->codec_tag);
        codec_name = tag_buf;
    }*/


    switch(enc->codec_type) {
    case AVMEDIA_TYPE_VIDEO:
        FFMSG_LOG( FFMSG_STRING_FMT(codectype), "video" );
        FFMSG_LOG( FFMSG_STRING_FMT(codecname), codec_name );
        FFMSG_LOG( FFMSG_STRING_FMT(profile), profile );

        if (enc->pix_fmt != PIX_FMT_NONE) {
            FFMSG_LOG( FFMSG_STRING_FMT(picfmt), av_get_pix_fmt_name(enc->pix_fmt) );
        }

        if (enc->width) {
            FFMSG_LOG( FFMSG_INT32_FMT(width), enc->width );
            FFMSG_LOG( FFMSG_INT32_FMT(height),enc->height );

            if (enc->sample_aspect_ratio.num) {
                av_reduce(&display_aspect_ratio.num, &display_aspect_ratio.den,
                          enc->width*enc->sample_aspect_ratio.num,
                          enc->height*enc->sample_aspect_ratio.den,
                          1024*1024);
                FFMSG_LOG( FFMSG_INT32_FMT(darnum), display_aspect_ratio.num );
                FFMSG_LOG( FFMSG_INT32_FMT(darden), display_aspect_ratio.den );
                FFMSG_LOG( FFMSG_INT32_FMT(sarnum), enc->sample_aspect_ratio.num );
                FFMSG_LOG( FFMSG_INT32_FMT(sarden), enc->sample_aspect_ratio.den );
            }

            /* if(av_log_get_level() >= AV_LOG_DEBUG){ */
                /* int g= av_gcd(enc->time_base.num, enc->time_base.den); */
                /* snprintf(buf + strlen(buf), buf_size - strlen(buf), */
                     /* ", %d/%d", */
                     /* enc->time_base.num/g, enc->time_base.den/g); */
            /* } */
        }
        break;
    case AVMEDIA_TYPE_AUDIO:
        FFMSG_LOG( FFMSG_STRING_FMT(codectype), "audio" );
        FFMSG_LOG( FFMSG_STRING_FMT(codecname), codec_name );
        if (enc->sample_rate) {
                FFMSG_LOG( FFMSG_INT32_FMT(samplerate), enc->sample_rate  );
        }
        //avcodec_get_channel_layout_string(buf , buf_size , enc->channels, enc->channel_layout);
        FFMSG_LOG( FFMSG_INTEGER_FMT(channel_layout),  enc->channel_layout );

        if (enc->sample_fmt != AV_SAMPLE_FMT_NONE) {
            FFMSG_LOG( FFMSG_STRING_FMT(audfmt), av_get_sample_fmt_name(enc->sample_fmt));
        }
        break;
    case AVMEDIA_TYPE_DATA:
        FFMSG_LOG( FFMSG_STRING_FMT(codectype), "data" );
        FFMSG_LOG( FFMSG_STRING_FMT(codecname), codec_name );
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        FFMSG_LOG( FFMSG_STRING_FMT(codectype), "subtitle" );
        FFMSG_LOG( FFMSG_STRING_FMT(codecname), codec_name );
        break;
    case AVMEDIA_TYPE_ATTACHMENT:
        FFMSG_LOG( FFMSG_STRING_FMT(codectype), "attachment" );
        FFMSG_LOG( FFMSG_STRING_FMT(codecname), codec_name );
        break;
    default:
        FFMSG_LOG( FFMSG_STRING_FMT(codectype), "invalid" );
        FFMSG_LOG( FFMSG_INT32_FMT(codecname), enc->codec_type );
        return;
    }

    bitrate = get_bit_rate(enc);
    if (bitrate != 0) {
        FFMSG_LOG( FFMSG_INT32_FMT(bitrate), bitrate );
    }
}



static void dump_stream_nlformat(AVFormatContext *ic, int i, int index, int is_output)
{
    char buf[256];
    AVStream *st = ic->streams[i];
    AVDictionaryEntry *lang;

    FFMSG_LOG( FFMSG_NODE_START_FMT("stream_%d_%d"), index, i );
    FFMSG_LOG( FFMSG_INT32_FMT(index), index );
    FFMSG_LOG( FFMSG_INT32_FMT(stid), i );

    avcodec_nlstring(buf, sizeof(buf), st->codec, is_output);

    nl_dump_metadata(st->metadata);
    lang = av_dict_get(st->metadata, "language", 0, 0);
    if (lang) {
        FFMSG_LOG(FFMSG_STRING_FMT(lang), lang->value);
    }
 
    if(st->codec->codec_type == AVMEDIA_TYPE_VIDEO){
        if(st->avg_frame_rate.den && st->avg_frame_rate.num) {
            FFMSG_LOG( FFMSG_INT32_FMT(avg_framerate_num), st->avg_frame_rate.num );
            FFMSG_LOG( FFMSG_INT32_FMT(avg_framerate_den), st->avg_frame_rate.den );
        }

        if(st->r_frame_rate.den && st->r_frame_rate.num) {
            FFMSG_LOG( FFMSG_INT32_FMT(r_framerate_num), st->r_frame_rate.num );
            FFMSG_LOG( FFMSG_INT32_FMT(r_framerate_den), st->r_frame_rate.den );
        }
        if(st->time_base.den && st->time_base.num) {
            FFMSG_LOG( FFMSG_INT32_FMT(mux_timebase_num), st->time_base.num );
            FFMSG_LOG( FFMSG_INT32_FMT(mux_timebase_den), st->time_base.den );
        }

        if(st->codec->time_base.den && st->codec->time_base.num) {
            FFMSG_LOG( FFMSG_INT32_FMT(codec_timebase_num), st->codec->time_base.num );
            FFMSG_LOG( FFMSG_INT32_FMT(codec_timebase_den), st->codec->time_base.den );
        }
    }

    FFMSG_LOG( FFMSG_INTEGER_FMT(duration), st->duration != AV_NOPTS_VALUE ? st->duration : 0  );
    FFMSG_LOG( FFMSG_INT32_FMT(inspected_frame_count), st->codec_info_nb_frames );

    FFMSG_LOG( FFMSG_NODE_STOP_FMT("stream_%d_%d"), index, i );
}


void dump_nlformat(AVFormatContext *ic,
                 int index,
                 const char *url,
                 int is_output)
{
    int i,rscount;
    AVDictionaryEntry *srctype;
    uint8_t *printed = av_mallocz(ic->nb_streams);
    if (ic->nb_streams && !printed)
        return;

    /* stream info */
    av_log(NULL, AV_LOG_INFO, FFMSG_START );
    FFMSG_LOG( FFMSG_INT32_FMT(version_major), FFMSG_VERSION_MAJOR );
    FFMSG_LOG( FFMSG_INT32_FMT(version_minor), FFMSG_VERSION_MINOR );
    FFMSG_LOG( FFMSG_STRING_FMT(msgtype), FFMSG_MSGTYPE_STREAMINFO );

    av_log(NULL, AV_LOG_INFO, FFMSG_NODE_START(muxinfo) );

    av_log(NULL, AV_LOG_INFO, FFMSG_STRING_FMT(direction), is_output ? "output" : "input" );
    av_log(NULL, AV_LOG_INFO, FFMSG_INTEGER_FMT(index), (int64_t)index );
    av_log(NULL, AV_LOG_INFO, FFMSG_INTEGER_FMT(timebase), (int64_t)AV_TIME_BASE );
    av_log(NULL, AV_LOG_INFO, FFMSG_STRING_FMT(mux_format), is_output ? ic->oformat->name : ic->iformat->name  );
    srctype = av_dict_get(ic->metadata, "source_type", 0,0);
    av_log(NULL, AV_LOG_INFO, FFMSG_STRING_FMT(source_type), srctype ? srctype->value : "file"  );

    av_log(NULL, AV_LOG_INFO, FFMSG_INTEGER_FMT(program_count), (int64_t)ic->nb_programs );
    FFMSG_LOG( FFMSG_INT32_FMT(stream_count), ic->nb_streams );

    if (!is_output) {
        av_log(NULL, AV_LOG_INFO, FFMSG_INTEGER_FMT(duration), ic->duration != AV_NOPTS_VALUE ? ic->duration : 0 );
        av_log(NULL, AV_LOG_INFO, FFMSG_INTEGER_FMT(start_time), ic->start_time != AV_NOPTS_VALUE ? ic->start_time : 0 );
        if( ic->bit_rate ) {
            av_log(NULL, AV_LOG_INFO, FFMSG_INTEGER_FMT(bitrate), (int64_t)ic->bit_rate );
        }        

    }

    if(ic->nb_programs) {
        int j, k, total = 0;
        FFMSG_LOG( FFMSG_NODE_START(programs) );
        for(j=0; j<ic->nb_programs; j++) {
            AVDictionaryEntry *name = av_dict_get(ic->programs[j]->metadata,  "name", NULL, 0);
            FFMSG_LOG( FFMSG_NODE_START_FMT("id%d"), ic->programs[j]->id );
            FFMSG_LOG( FFMSG_INTEGER_FMT(id), (int64_t) ic->programs[j]->id );
            FFMSG_LOG( FFMSG_STRING_FMT(name), name ? name->value : ""  );

            for(k=0; k<ic->programs[j]->nb_stream_indexes; k++) {
                dump_stream_nlformat(ic, ic->programs[j]->stream_index[k], index, is_output);
                printed[ic->programs[j]->stream_index[k]] = 1;
            }
            total += ic->programs[j]->nb_stream_indexes;

            FFMSG_LOG( FFMSG_NODE_STOP_FMT("id%d"), ic->programs[j]->id );
        }
        /* if (total < ic->nb_streams) */
            /* av_log(NULL, AV_LOG_INFO, "  No Program\n"); */
        FFMSG_LOG( FFMSG_NODE_STOP(programs) );
    }

    FFMSG_LOG( FFMSG_NODE_START(streams) );
    rscount = 0;
    for(i=0;i<ic->nb_streams;i++)
        if (!printed[i]) {
            dump_stream_nlformat(ic, i, index, is_output);
            rscount++;
        }

    FFMSG_LOG( FFMSG_NODE_STOP(streams) );
 
    FFMSG_LOG( FFMSG_INT32_FMT(rawstream_count), rscount );

    av_log(NULL, AV_LOG_INFO, FFMSG_NODE_STOP(muxinfo) );
    av_log(NULL, AV_LOG_INFO, FFMSG_STOP );
    av_free(printed); 
}




/****************************************************************/
/* nlreport                                                     */
/****************************************************************/
//#define _XOPEN_SOURCE 600
//#define STATS_DELAY 100000
#define STATS_DELAY 200000  /* delay between progress info messages */
void print_nlreport( OutputFile **output_files,
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
void c_strfree(char *str) { 
    free(str);
}
char *c_strescape (const char *source)
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




