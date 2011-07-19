/* @@--
 * 
 * Copyright (C) 2011 Alberto Vigata
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

static void print_nlreport( AVFormatContext **output_files,
                         AVOutputStream **ost_table, int nb_ostreams,
                         int is_last_report)
{
    char buf[1024];
    AVOutputStream *ost;
    AVFormatContext *oc;
    int64_t total_size;
    AVCodecContext *enc;
    int frame_number, vid, i;
    double bitrate, ti1, pts;
    static int64_t last_time = -1;
    static int qp_histogram[52];

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

    oc = output_files[0];

    total_size = url_fsize(oc->pb);
    if(total_size<0) // FIXME improve url_fsize() so it works with non seekable output too
        total_size= url_ftell(oc->pb);

    buf[0] = '\0';
    ti1 = 1e10;
    vid = 0;
    for(i=0;i<nb_ostreams;i++) {
        ost = ost_table[i];
        enc = ost->st->codec;
        if (vid && enc->codec_type == AVMEDIA_TYPE_VIDEO) {
            snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "q=%2.1f ",
                     !ost->st->stream_copy ?
                     enc->coded_frame->quality/(float)FF_QP2LAMBDA : -1);
        }
        if (!vid && enc->codec_type == AVMEDIA_TYPE_VIDEO) {
            float t = (av_gettime()-timer_start) / 1000000.0;

            frame_number = ost->frame_number;
            int fps = (t>1)?(int)(frame_number/t+0.5) : 0;
            /* snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "frame=%5d fps=%3d q=%3.1f ", */
                     /* frame_number, fps, */
                     /* !ost->st->stream_copy ? */
                     /* enc->coded_frame->quality/(float)FF_QP2LAMBDA : -1); */

            FFMSG_LOG( FFMSG_INT32_FMT(curframe), frame_number );
            FFMSG_LOG( FFMSG_INT32_FMT(fps), fps );

            if(is_last_report)
                snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "L");
            vid = 1;
        }
        /* compute min output value */
        pts = (double)ost->st->pts.val * av_q2d(ost->st->time_base);
        if ((pts < ti1) && (pts > 0))
            ti1 = pts;
    }
    if (ti1 < 0.01)
        ti1 = 0.01;

    if (verbose || is_last_report) {
        bitrate = (double)(total_size * 8) / ti1 / 1000.0;

        snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
            "size=%8.0fkB time=%0.2f bitrate=%6.1fkbits/s",
            (double)total_size / 1024, ti1, bitrate);

        FFMSG_LOG( FFMSG_INTEGER_FMT(size), total_size );
        FFMSG_LOG( FFMSG_INT32_FMT(bitrate), (int)(bitrate*1000.0) );
        FFMSG_LOG( FFMSG_INT32_FMT(frames_dup), nb_frames_dup );
        FFMSG_LOG( FFMSG_INT32_FMT(frames_drop), nb_frames_drop );
        FFMSG_LOG( FFMSG_INT32_FMT(is_last_report), is_last_report );
        FFMSG_LOG( FFMSG_INT32_FMT(curtime), (int)(ti1*1000.0) );

        if (nb_frames_dup || nb_frames_drop)
          snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), " dup=%d drop=%d",
                  nb_frames_dup, nb_frames_drop);

        /* if (verbose >= 0) */
            /* fprintf(stderr, "%s    \r", buf); */
/*  */
        /* fflush(stderr); */
    }

    if (is_last_report && verbose >= 0){
        int64_t raw= audio_size + video_size + extra_size;
        /* fprintf(stderr, "\n"); */
        /* fprintf(stderr, "video:%1.0fkB audio:%1.0fkB global headers:%1.0fkB muxing overhead %f%%\n", */
                /* video_size/1024.0, */
                /* audio_size/1024.0, */
                /* extra_size/1024.0, */
                /* 100.0*(total_size - raw)/raw */
        /* ); */
    }


    FFMSG_LOG( FFMSG_NODE_STOP(progress) );
    FFMSG_LOG( FFMSG_STOP );
}


#endif
