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

#ifndef __NLINPUT_H
#define __NLINPUT_H

#include "nlffmsg.h"
#include "config.h"



#if HAVE_PTHREADS
#include <pthread.h>
#elif HAVE_W32THREADS
#include "libavcodec/w32pthreads.h"
#elif HAVE_OS2THREADS
#include "os2threads.h"
#endif

/* cross thread signal struct */
typedef struct {
    int exit;
    int cancel_transcode;
    pthread_t nlin_th;
} nlinput_t;


/* fires up input thread */
nlinput_t *nlinput_prepare(void);
void nlinput_cancel(nlinput_t *);


#endif /* __NLINPUT_H */
