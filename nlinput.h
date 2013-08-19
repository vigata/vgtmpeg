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

#ifndef __NLINPUT_H
#define __NLINPUT_H

#include "nlffmsg.h"
#include "config.h"

#ifndef attribute_align_arg
#if ARCH_X86_32 && AV_GCC_VERSION_AT_LEAST(4,2)
#    define attribute_align_arg __attribute__((force_align_arg_pointer))
#else
#    define attribute_align_arg
#endif
#endif

#if HAVE_PTHREADS
#include <pthread.h>
#elif HAVE_W32THREADS
#include "libavcodec/w32pthreads.h"
#elif HAVE_OS2THREADS
#include "os2threads.h"
#endif

#include <stdio.h>


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


/* cross thread signal struct */
typedef struct {
    int exit;
    int cancel_transcode;
} nlinput_t;

static void *nlinput_start(void *c) {
    nlinput_t *ctx = (nlinput_t *)c;
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

/* static objects to be imported on module */
static nlinput_t nli;
static pthread_t nlin_th;

/* fires up input thread */
static void nlinput_prepare(void) {
    /* starting nlinput */
    memset( &nli, 0, sizeof (nlinput_t) );

    //pthread_attr_init(&nlin_attr);
    //pthread_attr_setdetachstate(&nlin_attr, PTHREAD_CREATE_JOINABLE );
    pthread_create( &nlin_th, NULL /*nlin_attr*/, nlinput_start, (void *)&nli );
}

/* shutdown input thread */
static void nlinput_cancel(void) {
    void *status;
    printf("nlinput: cancel\n");

    pthread_join( nlin_th, &status );
}


#endif /* __NLINPUT_H */
