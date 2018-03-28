/*
 * VP9 compatible video decoder
 *
 * Copyright (C) 2013 Ronald S. Bultje <rsbultje gmail com>
 * Copyright (C) 2013 Clément Bœsch <u pkh me>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "avcodec.h"
#include "get_bits.h"
#include "internal.h"
#include "profiles.h"
#include "thread.h"
#include "videodsp.h"
#include "vp56.h"
#include "vp9.h"
#include "vp9data.h"
#include "vp9dec.h"
#include "libavutil/avassert.h"
#include "libavutil/pixdesc.h"

#define VP9_SYNCCODE 0x498342

#if HAVE_THREADS
static void vp9_free_entries(AVCodecContext *avctx) {
    VP9Context *s = avctx->priv_data;

    if (avctx->active_thread_type & FF_THREAD_SLICE)  {
        pthread_mutex_destroy(&s->progress_mutex);
        pthread_cond_destroy(&s->progress_cond);
        av_freep(&s->entries);
    }
}

static int vp9_alloc_entries(AVCodecContext *avctx, int n) {
    VP9Context *s = avctx->priv_data;
    int i;

    if (avctx->active_thread_type & FF_THREAD_SLICE)  {
        if (s->entries)
            av_freep(&s->entries);

        s->entries = av_malloc_array(n, sizeof(atomic_int));

        if (!s->entries) {
            av_freep(&s->entries);
            return AVERROR(ENOMEM);
        }

        for (i  = 0; i < n; i++)
            atomic_init(&s->entries[i], 0);

        pthread_mutex_init(&s->progress_mutex, NULL);
        pthread_cond_init(&s->progress_cond, NULL);
    }
    return 0;
}

static void vp9_report_tile_progress(VP9Context *s, int field, int n) {
    pthread_mutex_lock(&s->progress_mutex);
    atomic_fetch_add_explicit(&s->entries[field], n, memory_order_release);
    pthread_cond_signal(&s->progress_cond);
    pthread_mutex_unlock(&s->progress_mutex);
}

static void vp9_await_tile_progress(VP9Context *s, int field, int n) {
    if (atomic_load_explicit(&s->entries[field], memory_order_acquire) >= n)
        return;

    pthread_mutex_lock(&s->progress_mutex);
    while (atomic_load_explicit(&s->entries[field], memory_order_relaxed) != n)
        pthread_cond_wait(&s->progress_cond, &s->progress_mutex);
    pthread_mutex_unlock(&s->progress_mutex);
}
#else
static void vp9_free_entries(AVCodecContext *avctx) {}
static int vp9_alloc_entries(AVCodecContext *avctx, int n) { return 0; }
#endif

static void vp9_frame_unref(AVCodecContext *avctx, VP9Frame *f)
{
    ff_thread_release_buffer(avctx, &f->tf);
    av_buffer_unref(&f->extradata);
    av_buffer_unref(&f->hwaccel_priv_buf);
    f->segmentation_map = NULL;
    f->hwaccel_picture_private = NULL;
}

static int vp9_frame_alloc(AVCodecContext *avctx, VP9Frame *f)
{
    VP9Context *s = avctx->priv_data;
    int ret, sz;

    ret = ff_thread_get_buffer(avctx, &f->tf, AV_GET_BUFFER_FLAG_REF);
    if (ret < 0)
        return ret;

    sz = 64 * s->sb_cols * s->sb_rows;
    f->extradata = av_buffer_allocz(sz * (1 + sizeof(VP9mvrefPair)));
    if (!f->extradata) {
        goto fail;
    }

    f->segmentation_map = f->extradata->data;
    f->mv = (VP9mvrefPair *) (f->extradata->data + sz);

    if (avctx->hwaccel) {
        const AVHWAccel *hwaccel = avctx->hwaccel;
        av_assert0(!f->hwaccel_picture_private);
        if (hwaccel->frame_priv_data_size) {
            f->hwaccel_priv_buf = av_buffer_allocz(hwaccel->frame_priv_data_size);
            if (!f->hwaccel_priv_buf)
                goto fail;
            f->hwaccel_picture_private = f->hwaccel_priv_buf->data;
        }
    }

    return 0;

fail:
    vp9_frame_unref(avctx, f);
    return AVERROR(ENOMEM);
}

static int vp9_frame_ref(AVCodecContext *avctx, VP9Frame *dst, VP9Frame *src)
{
    int ret;

    ret = ff_thread_ref_frame(&dst->tf, &src->tf);
    if (ret < 0)
        return ret;

    dst->extradata = av_buffer_ref(src->extradata);
    if (!dst->extradata)
        goto fail;

    dst->segmentation_map = src->segmentation_map;
    dst->mv = src->mv;
    dst->uses_2pass = src->uses_2pass;

    if (src->hwaccel_picture_private) {
        dst->hwaccel_priv_buf = av_buffer_ref(src->hwaccel_priv_buf);
        if (!dst->hwaccel_priv_buf)
            goto fail;
        dst->hwaccel_picture_private = dst->hwaccel_priv_buf->data;
    }

    return 0;

fail:
    vp9_frame_unref(avctx, dst);
    return AVERROR(ENOMEM);
}

static int update_size(AVCodecContext *avctx, int w, int h)
{
#define HWACCEL_MAX (CONFIG_VP9_DXVA2_HWACCEL + CONFIG_VP9_D3D11VA_HWACCEL * 2 + CONFIG_VP9_VAAPI_HWACCEL)
    enum AVPixelFormat pix_fmts[HWACCEL_MAX + 2], *fmtp = pix_fmts;
    VP9Context *s = avctx->priv_data;
    uint8_t *p;
    int bytesperpixel = s->bytesperpixel, ret, cols, rows;
    int lflvl_len, i;

    av_assert0(w > 0 && h > 0);

    if (!(s->pix_fmt == s->gf_fmt && w == s->w && h == s->h)) {
        if ((ret = ff_set_dimensions(avctx, w, h)) < 0)
            return ret;

        switch (s->pix_fmt) {
        case AV_PIX_FMT_YUV420P:
#if CONFIG_VP9_DXVA2_HWACCEL
            *fmtp++ = AV_PIX_FMT_DXVA2_VLD;
#endif
#if CONFIG_VP9_D3D11VA_HWACCEL
            *fmtp++ = AV_PIX_FMT_D3D11VA_VLD;
            *fmtp++ = AV_PIX_FMT_D3D11;
#endif
#if CONFIG_VP9_VAAPI_HWACCEL
            *fmtp++ = AV_PIX_FMT_VAAPI;
#endif
            break;
        case AV_PIX_FMT_YUV420P10:
        case AV_PIX_FMT_YUV420P12:
#if CONFIG_VP9_VAAPI_HWACCEL
            *fmtp++ = AV_PIX_FMT_VAAPI;
#endif
            break;
        }

        *fmtp++ = s->pix_fmt;
        *fmtp = AV_PIX_FMT_NONE;

        ret = ff_thread_get_format(avctx, pix_fmts);
        if (ret < 0)
            return ret;

        avctx->pix_fmt = ret;
        s->gf_fmt  = s->pix_fmt;
        s->w = w;
        s->h = h;
    }

    cols = (w + 7) >> 3;
    rows = (h + 7) >> 3;

    if (s->intra_pred_data[0] && cols == s->cols && rows == s->rows && s->pix_fmt == s->last_fmt)
        return 0;

    s->last_fmt  = s->pix_fmt;
    s->sb_cols   = (w + 63) >> 6;
    s->sb_rows   = (h + 63) >> 6;
    s->cols      = (w + 7) >> 3;
    s->rows      = (h + 7) >> 3;
    lflvl_len    = avctx->active_thread_type == FF_THREAD_SLICE ? s->sb_rows : 1;

#define assign(var, type, n) var = (type) p; p += s->sb_cols * (n) * sizeof(*var)
    av_freep(&s->intra_pred_data[0]);
    // FIXME we slightly over-allocate here for subsampled chroma, but a little
    // bit of padding shouldn't affect performance...
    p = av_malloc(s->sb_cols * (128 + 192 * bytesperpixel +
                                lflvl_len * sizeof(*s->lflvl) + 16 * sizeof(*s->above_mv_ctx)));
    if (!p)
        return AVERROR(ENOMEM);
    assign(s->intra_pred_data[0],  uint8_t *,             64 * bytesperpixel);
    assign(s->intra_pred_data[1],  uint8_t *,             64 * bytesperpixel);
    assign(s->intra_pred_data[2],  uint8_t *,             64 * bytesperpixel);
    assign(s->above_y_nnz_ctx,     uint8_t *,             16);
    assign(s->above_mode_ctx,      uint8_t *,             16);
    assign(s->above_mv_ctx,        VP56mv(*)[2],          16);
    assign(s->above_uv_nnz_ctx[0], uint8_t *,             16);
    assign(s->above_uv_nnz_ctx[1], uint8_t *,             16);
    assign(s->above_partition_ctx, uint8_t *,              8);
    assign(s->above_skip_ctx,      uint8_t *,              8);
    assign(s->above_txfm_ctx,      uint8_t *,              8);
    assign(s->above_segpred_ctx,   uint8_t *,              8);
    assign(s->above_intra_ctx,     uint8_t *,              8);
    assign(s->above_comp_ctx,      uint8_t *,              8);
    assign(s->above_ref_ctx,       uint8_t *,              8);
    assign(s->above_filter_ctx,    uint8_t *,              8);
    assign(s->lflvl,               VP9Filter *,            lflvl_len);
#undef assign

    if (s->td) {
        for (i = 0; i < s->active_tile_cols; i++) {
            av_freep(&s->td[i].b_base);
            av_freep(&s->td[i].block_base);
        }
    }

    if (s->s.h.bpp != s->last_bpp) {
        ff_vp9dsp_init(&s->dsp, s->s.h.bpp, avctx->flags & AV_CODEC_FLAG_BITEXACT);
        ff_videodsp_init(&s->vdsp, s->s.h.bpp);
        s->last_bpp = s->s.h.bpp;
    }

    return 0;
}

static int update_block_buffers(AVCodecContext *avctx)
{
    int i;
    VP9Context *s = avctx->priv_data;
    int chroma_blocks, chroma_eobs, bytesperpixel = s->bytesperpixel;
    VP9TileData *td = &s->td[0];

    if (td->b_base && td->block_base && s->block_alloc_using_2pass == s->s.frames[CUR_FRAME].uses_2pass)
        return 0;

    av_free(td->b_base);
    av_free(td->block_base);
    chroma_blocks = 64 * 64 >> (s->ss_h + s->ss_v);
    chroma_eobs   = 16 * 16 >> (s->ss_h + s->ss_v);
    if (s->s.frames[CUR_FRAME].uses_2pass) {
        int sbs = s->sb_cols * s->sb_rows;

        td->b_base = av_malloc_array(s->cols * s->rows, sizeof(VP9Block));
        td->block_base = av_mallocz(((64 * 64 + 2 * chroma_blocks) * bytesperpixel * sizeof(int16_t) +
                                    16 * 16 + 2 * chroma_eobs) * sbs);
        if (!td->b_base || !td->block_base)
            return AVERROR(ENOMEM);
        td->uvblock_base[0] = td->block_base + sbs * 64 * 64 * bytesperpixel;
        td->uvblock_base[1] = td->uvblock_base[0] + sbs * chroma_blocks * bytesperpixel;
        td->eob_base = (uint8_t *) (td->uvblock_base[1] + sbs * chroma_blocks * bytesperpixel);
        td->uveob_base[0] = td->eob_base + 16 * 16 * sbs;
        td->uveob_base[1] = td->uveob_base[0] + chroma_eobs * sbs;
    } else {
        for (i = 1; i < s->active_tile_cols; i++) {
            if (s->td[i].b_base && s->td[i].block_base) {
                av_free(s->td[i].b_base);
                av_free(s->td[i].block_base);
            }
        }
        for (i = 0; i < s->active_tile_cols; i++) {
            s->td[i].b_base = av_malloc(sizeof(VP9Block));
            s->td[i].block_base = av_mallocz((64 * 64 + 2 * chroma_blocks) * bytesperpixel * sizeof(int16_t) +
                                       16 * 16 + 2 * chroma_eobs);
            if (!s->td[i].b_base || !s->td[i].block_base)
                return AVERROR(ENOMEM);
            s->td[i].uvblock_base[0] = s->td[i].block_base + 64 * 64 * bytesperpixel;
            s->td[i].uvblock_base[1] = s->td[i].uvblock_base[0] + chroma_blocks * bytesperpixel;
            s->td[i].eob_base = (uint8_t *) (s->td[i].uvblock_base[1] + chroma_blocks * bytesperpixel);
            s->td[i].uveob_base[0] = s->td[i].eob_base + 16 * 16;
            s->td[i].uveob_base[1] = s->td[i].uveob_base[0] + chroma_eobs;
        }
    }
    s->block_alloc_using_2pass = s->s.frames[CUR_FRAME].uses_2pass;

    return 0;
}

// The sign bit is at the end, not the start, of a bit sequence
static av_always_inline int get_sbits_inv(GetBitContext *gb, int n)
{
    int v = get_bits(gb, n);
    return get_bits1(gb) ? -v : v;
}

static av_always_inline int inv_recenter_nonneg(int v, int m)
{
    if (v > 2 * m)
        return v;
    if (v & 1)
        return m - ((v + 1) >> 1);
    return m + (v >> 1);
}

// differential forward probability updates
static int update_prob(VP56RangeCoder *c, int p)
{
    static const int inv_map_table[255] = {
          7,  20,  33,  46,  59,  72,  85,  98, 111, 124, 137, 150, 163, 176,
        189, 202, 215, 228, 241, 254,   1,   2,   3,   4,   5,   6,   8,   9,
         10,  11,  12,  13,  14,  15,  16,  17,  18,  19,  21,  22,  23,  24,
         25,  26,  27,  28,  29,  30,  31,  32,  34,  35,  36,  37,  38,  39,
         40,  41,  42,  43,  44,  45,  47,  48,  49,  50,  51,  52,  53,  54,
         55,  56,  57,  58,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,
         70,  71,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84,
         86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  99, 100,
        101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 112, 113, 114, 115,
        116, 117, 118, 119, 120, 121, 122, 123, 125, 126, 127, 128, 129, 130,
        131, 132, 133, 134, 135, 136, 138, 139, 140, 141, 142, 143, 144, 145,
        146, 147, 148, 149, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160,
        161, 162, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175,
        177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 190, 191,
        192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 203, 204, 205, 206,
        207, 208, 209, 210, 211, 212, 213, 214, 216, 217, 218, 219, 220, 221,
        222, 223, 224, 225, 226, 227, 229, 230, 231, 232, 233, 234, 235, 236,
        237, 238, 239, 240, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251,
        252, 253, 253,
    };
    int d;

    /* This code is trying to do a differential probability update. For a
     * current probability A in the range [1, 255], the difference to a new
     * probability of any value can be expressed differentially as 1-A, 255-A
     * where some part of this (absolute range) exists both in positive as
     * well as the negative part, whereas another part only exists in one
     * half. We're trying to code this shared part differentially, i.e.
     * times two where the value of the lowest bit specifies the sign, and
     * the single part is then coded on top of this. This absolute difference
     * then again has a value of [0, 254], but a bigger value in this range
     * indicates that we're further away from the original value A, so we
     * can code this as a VLC code, since higher values are increasingly
     * unlikely. The first 20 values in inv_map_table[] allow 'cheap, rough'
     * updates vs. the 'fine, exact' updates further down the range, which
     * adds one extra dimension to this differential update model. */

    if (!vp8_rac_get(c)) {
        d = vp8_rac_get_uint(c, 4) + 0;
    } else if (!vp8_rac_get(c)) {
        d = vp8_rac_get_uint(c, 4) + 16;
    } else if (!vp8_rac_get(c)) {
        d = vp8_rac_get_uint(c, 5) + 32;
    } else {
        d = vp8_rac_get_uint(c, 7);
        if (d >= 65)
            d = (d << 1) - 65 + vp8_rac_get(c);
        d += 64;
        av_assert2(d < FF_ARRAY_ELEMS(inv_map_table));
    }

    return p <= 128 ? 1 + inv_recenter_nonneg(inv_map_table[d], p - 1) :
                    255 - inv_recenter_nonneg(inv_map_table[d], 255 - p);
}

static int read_colorspace_details(AVCodecContext *avctx)
{
    static const enum AVColorSpace colorspaces[8] = {
        AVCOL_SPC_UNSPECIFIED, AVCOL_SPC_BT470BG, AVCOL_SPC_BT709, AVCOL_SPC_SMPTE170M,
        AVCOL_SPC_SMPTE240M, AVCOL_SPC_BT2020_NCL, AVCOL_SPC_RESERVED, AVCOL_SPC_RGB,
    };
    VP9Context *s = avctx->priv_data;
    int bits = avctx->profile <= 1 ? 0 : 1 + get_bits1(&s->gb); // 0:8, 1:10, 2:12

    s->bpp_index = bits;
    s->s.h.bpp = 8 + bits * 2;
    s->bytesperpixel = (7 + s->s.h.bpp) >> 3;
    avctx->colorspace = colorspaces[get_bits(&s->gb, 3)];
    if (avctx->colorspace == AVCOL_SPC_RGB) { // RGB = profile 1
        static const enum AVPixelFormat pix_fmt_rgb[3] = {
            AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRP10, AV_PIX_FMT_GBRP12
        };
        s->ss_h = s->ss_v = 0;
        avctx->color_range = AVCOL_RANGE_JPEG;
        s->pix_fmt = pix_fmt_rgb[bits];
        if (avctx->profile & 1) {
            if (get_bits1(&s->gb)) {
                av_log(avctx, AV_LOG_ERROR, "Reserved bit set in RGB\n");
                return AVERROR_INVALIDDATA;
            }
        } else {
            av_log(avctx, AV_LOG_ERROR, "RGB not supported in profile %d\n",
                   avctx->profile);
            return AVERROR_INVALIDDATA;
        }
    } else {
        static const enum AVPixelFormat pix_fmt_for_ss[3][2 /* v */][2 /* h */] = {
            { { AV_PIX_FMT_YUV444P, AV_PIX_FMT_YUV422P },
              { AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV420P } },
            { { AV_PIX_FMT_YUV444P10, AV_PIX_FMT_YUV422P10 },
              { AV_PIX_FMT_YUV440P10, AV_PIX_FMT_YUV420P10 } },
            { { AV_PIX_FMT_YUV444P12, AV_PIX_FMT_YUV422P12 },
              { AV_PIX_FMT_YUV440P12, AV_PIX_FMT_YUV420P12 } }
        };
        avctx->color_range = get_bits1(&s->gb) ? AVCOL_RANGE_JPEG : AVCOL_RANGE_MPEG;
        if (avctx->profile & 1) {
            s->ss_h = get_bits1(&s->gb);
            s->ss_v = get_bits1(&s->gb);
            s->pix_fmt = pix_fmt_for_ss[bits][s->ss_v][s->ss_h];
            if (s->pix_fmt == AV_PIX_FMT_YUV420P) {
                av_log(avctx, AV_LOG_ERROR, "YUV 4:2:0 not supported in profile %d\n",
                       avctx->profile);
                return AVERROR_INVALIDDATA;
            } else if (get_bits1(&s->gb)) {
                av_log(avctx, AV_LOG_ERROR, "Profile %d color details reserved bit set\n",
                       avctx->profile);
                return AVERROR_INVALIDDATA;
            }
        } else {
            s->ss_h = s->ss_v = 1;
            s->pix_fmt = pix_fmt_for_ss[bits][1][1];
        }
    }

    return 0;
}

static int decode_frame_header(AVCodecContext *avctx,
                               const uint8_t *data, int size, int *ref)
{
    VP9Context *s = avctx->priv_data;
    int c, i, j, k, l, m, n, w, h, max, size2, ret, sharp;
    int last_invisible;
    const uint8_t *data2;

    /* general header */
    if ((ret = init_get_bits8(&s->gb, data, size)) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to initialize bitstream reader\n");
        return ret;
    }
    if (get_bits(&s->gb, 2) != 0x2) { // frame marker
        av_log(avctx, AV_LOG_ERROR, "Invalid frame marker\n");
        return AVERROR_INVALIDDATA;
    }
    avctx->profile  = get_bits1(&s->gb);
    avctx->profile |= get_bits1(&s->gb) << 1;
    if (avctx->profile == 3) avctx->profile += get_bits1(&s->gb);
    if (avctx->profile > 3) {
        av_log(avctx, AV_LOG_ERROR, "Profile %d is not yet supported\n", avctx->profile);
        return AVERROR_INVALIDDATA;
    }
    s->s.h.profile = avctx->profile;
    if (get_bits1(&s->gb)) {
        *ref = get_bits(&s->gb, 3);
        return 0;
    }

    s->last_keyframe  = s->s.h.keyframe;
    s->s.h.keyframe   = !get_bits1(&s->gb);

    last_invisible   = s->s.h.invisible;
    s->s.h.invisible = !get_bits1(&s->gb);
    s->s.h.errorres  = get_bits1(&s->gb);
    s->s.h.use_last_frame_mvs = !s->s.h.errorres && !last_invisible;

    if (s->s.h.keyframe) {
        if (get_bits_long(&s->gb, 24) != VP9_SYNCCODE) { // synccode
            av_log(avctx, AV_LOG_ERROR, "Invalid sync code\n");
            return AVERROR_INVALIDDATA;
        }
        if ((ret = read_colorspace_details(avctx)) < 0)
            return ret;
        // for profile 1, here follows the subsampling bits
        s->s.h.refreshrefmask = 0xff;
        w = get_bits(&s->gb, 16) + 1;
        h = get_bits(&s->gb, 16) + 1;
        if (get_bits1(&s->gb)) // display size
            skip_bits(&s->gb, 32);
    } else {
        s->s.h.intraonly = s->s.h.invisible ? get_bits1(&s->gb) : 0;
        s->s.h.resetctx  = s->s.h.errorres ? 0 : get_bits(&s->gb, 2);
        if (s->s.h.intraonly) {
            if (get_bits_long(&s->gb, 24) != VP9_SYNCCODE) { // synccode
                av_log(avctx, AV_LOG_ERROR, "Invalid sync code\n");
                return AVERROR_INVALIDDATA;
            }
            if (avctx->profile >= 1) {
                if ((ret = read_colorspace_details(avctx)) < 0)
                    return ret;
            } else {
                s->ss_h = s->ss_v = 1;
                s->s.h.bpp = 8;
                s->bpp_index = 0;
                s->bytesperpixel = 1;
                s->pix_fmt = AV_PIX_FMT_YUV420P;
                avctx->colorspace = AVCOL_SPC_BT470BG;
                avctx->color_range = AVCOL_RANGE_MPEG;
            }
            s->s.h.refreshrefmask = get_bits(&s->gb, 8);
            w = get_bits(&s->gb, 16) + 1;
            h = get_bits(&s->gb, 16) + 1;
            if (get_bits1(&s->gb)) // display size
                skip_bits(&s->gb, 32);
        } else {
            s->s.h.refreshrefmask = get_bits(&s->gb, 8);
            s->s.h.refidx[0]      = get_bits(&s->gb, 3);
            s->s.h.signbias[0]    = get_bits1(&s->gb) && !s->s.h.errorres;
            s->s.h.refidx[1]      = get_bits(&s->gb, 3);
            s->s.h.signbias[1]    = get_bits1(&s->gb) && !s->s.h.errorres;
            s->s.h.refidx[2]      = get_bits(&s->gb, 3);
            s->s.h.signbias[2]    = get_bits1(&s->gb) && !s->s.h.errorres;
            if (!s->s.refs[s->s.h.refidx[0]].f->buf[0] ||
                !s->s.refs[s->s.h.refidx[1]].f->buf[0] ||
                !s->s.refs[s->s.h.refidx[2]].f->buf[0]) {
                av_log(avctx, AV_LOG_ERROR, "Not all references are available\n");
                return AVERROR_INVALIDDATA;
            }
            if (get_bits1(&s->gb)) {
                w = s->s.refs[s->s.h.refidx[0]].f->width;
                h = s->s.refs[s->s.h.refidx[0]].f->height;
            } else if (get_bits1(&s->gb)) {
                w = s->s.refs[s->s.h.refidx[1]].f->width;
                h = s->s.refs[s->s.h.refidx[1]].f->height;
            } else if (get_bits1(&s->gb)) {
                w = s->s.refs[s->s.h.refidx[2]].f->width;
                h = s->s.refs[s->s.h.refidx[2]].f->height;
            } else {
                w = get_bits(&s->gb, 16) + 1;
                h = get_bits(&s->gb, 16) + 1;
            }
            // Note that in this code, "CUR_FRAME" is actually before we
            // have formally allocated a frame, and thus actually represents
            // the _last_ frame
            s->s.h.use_last_frame_mvs &= s->s.frames[CUR_FRAME].tf.f->width == w &&
                                       s->s.frames[CUR_FRAME].tf.f->height == h;
            if (get_bits1(&s->gb)) // display size
                skip_bits(&s->gb, 32);
            s->s.h.highprecisionmvs = get_bits1(&s->gb);
            s->s.h.filtermode = get_bits1(&s->gb) ? FILTER_SWITCHABLE :
                                                  get_bits(&s->gb, 2);
            s->s.h.allowcompinter = s->s.h.signbias[0] != s->s.h.signbias[1] ||
                                  s->s.h.signbias[0] != s->s.h.signbias[2];
            if (s->s.h.allowcompinter) {
                if (s->s.h.signbias[0] == s->s.h.signbias[1]) {
                    s->s.h.fixcompref    = 2;
                    s->s.h.varcompref[0] = 0;
                    s->s.h.varcompref[1] = 1;
                } else if (s->s.h.signbias[0] == s->s.h.signbias[2]) {
                    s->s.h.fixcompref    = 1;
                    s->s.h.varcompref[0] = 0;
                    s->s.h.varcompref[1] = 2;
                } else {
                    s->s.h.fixcompref    = 0;
                    s->s.h.varcompref[0] = 1;
                    s->s.h.varcompref[1] = 2;
                }
            }
        }
    }
    s->s.h.refreshctx   = s->s.h.errorres ? 0 : get_bits1(&s->gb);
    s->s.h.parallelmode = s->s.h.errorres ? 1 : get_bits1(&s->gb);
    s->s.h.framectxid   = c = get_bits(&s->gb, 2);
    if (s->s.h.keyframe || s->s.h.intraonly)
        s->s.h.framectxid = 0; // BUG: libvpx ignores this field in keyframes

    /* loopfilter header data */
    if (s->s.h.keyframe || s->s.h.errorres || s->s.h.intraonly) {
        // reset loopfilter defaults
        s->s.h.lf_delta.ref[0] = 1;
        s->s.h.lf_delta.ref[1] = 0;
        s->s.h.lf_delta.ref[2] = -1;
        s->s.h.lf_delta.ref[3] = -1;
        s->s.h.lf_delta.mode[0] = 0;
        s->s.h.lf_delta.mode[1] = 0;
        memset(s->s.h.segmentation.feat, 0, sizeof(s->s.h.segmentation.feat));
    }
    s->s.h.filter.level = get_bits(&s->gb, 6);
    sharp = get_bits(&s->gb, 3);
    // if sharpness changed, reinit lim/mblim LUTs. if it didn't change, keep
    // the old cache values since they are still valid
    if (s->s.h.filter.sharpness != sharp) {
        for (i = 1; i <= 63; i++) {
            int limit = i;

            if (sharp > 0) {
                limit >>= (sharp + 3) >> 2;
                limit = FFMIN(limit, 9 - sharp);
            }
            limit = FFMAX(limit, 1);

            s->filter_lut.lim_lut[i] = limit;
            s->filter_lut.mblim_lut[i] = 2 * (i + 2) + limit;
        }
    }
    s->s.h.filter.sharpness = sharp;
    if ((s->s.h.lf_delta.enabled = get_bits1(&s->gb))) {
        if ((s->s.h.lf_delta.updated = get_bits1(&s->gb))) {
            for (i = 0; i < 4; i++)
                if (get_bits1(&s->gb))
                    s->s.h.lf_delta.ref[i] = get_sbits_inv(&s->gb, 6);
            for (i = 0; i < 2; i++)
                if (get_bits1(&s->gb))
                    s->s.h.lf_delta.mode[i] = get_sbits_inv(&s->gb, 6);
        }
    }

    /* quantization header data */
    s->s.h.yac_qi      = get_bits(&s->gb, 8);
    s->s.h.ydc_qdelta  = get_bits1(&s->gb) ? get_sbits_inv(&s->gb, 4) : 0;
    s->s.h.uvdc_qdelta = get_bits1(&s->gb) ? get_sbits_inv(&s->gb, 4) : 0;
    s->s.h.uvac_qdelta = get_bits1(&s->gb) ? get_sbits_inv(&s->gb, 4) : 0;
    s->s.h.lossless    = s->s.h.yac_qi == 0 && s->s.h.ydc_qdelta == 0 &&
                       s->s.h.uvdc_qdelta == 0 && s->s.h.uvac_qdelta == 0;
    if (s->s.h.lossless)
        avctx->properties |= FF_CODEC_PROPERTY_LOSSLESS;

    /* segmentation header info */
    if ((s->s.h.segmentation.enabled = get_bits1(&s->gb))) {
        if ((s->s.h.segmentation.update_map = get_bits1(&s->gb))) {
            for (i = 0; i < 7; i++)
                s->s.h.segmentation.prob[i] = get_bits1(&s->gb) ?
                                 get_bits(&s->gb, 8) : 255;
            if ((s->s.h.segmentation.temporal = get_bits1(&s->gb)))
                for (i = 0; i < 3; i++)
                    s->s.h.segmentation.pred_prob[i] = get_bits1(&s->gb) ?
                                         get_bits(&s->gb, 8) : 255;
        }

        if (get_bits1(&s->gb)) {
            s->s.h.segmentation.absolute_vals = get_bits1(&s->gb);
            for (i = 0; i < 8; i++) {
                if ((s->s.h.segmentation.feat[i].q_enabled = get_bits1(&s->gb)))
                    s->s.h.segmentation.feat[i].q_val = get_sbits_inv(&s->gb, 8);
                if ((s->s.h.segmentation.feat[i].lf_enabled = get_bits1(&s->gb)))
                    s->s.h.segmentation.feat[i].lf_val = get_sbits_inv(&s->gb, 6);
                if ((s->s.h.segmentation.feat[i].ref_enabled = get_bits1(&s->gb)))
                    s->s.h.segmentation.feat[i].ref_val = get_bits(&s->gb, 2);
                s->s.h.segmentation.feat[i].skip_enabled = get_bits1(&s->gb);
            }
        }
    }

    // set qmul[] based on Y/UV, AC/DC and segmentation Q idx deltas
    for (i = 0; i < (s->s.h.segmentation.enabled ? 8 : 1); i++) {
        int qyac, qydc, quvac, quvdc, lflvl, sh;

        if (s->s.h.segmentation.enabled && s->s.h.segmentation.feat[i].q_enabled) {
            if (s->s.h.segmentation.absolute_vals)
                qyac = av_clip_uintp2(s->s.h.segmentation.feat[i].q_val, 8);
            else
                qyac = av_clip_uintp2(s->s.h.yac_qi + s->s.h.segmentation.feat[i].q_val, 8);
        } else {
            qyac  = s->s.h.yac_qi;
        }
        qydc  = av_clip_uintp2(qyac + s->s.h.ydc_qdelta, 8);
        quvdc = av_clip_uintp2(qyac + s->s.h.uvdc_qdelta, 8);
        quvac = av_clip_uintp2(qyac + s->s.h.uvac_qdelta, 8);
        qyac  = av_clip_uintp2(qyac, 8);

        s->s.h.segmentation.feat[i].qmul[0][0] = ff_vp9_dc_qlookup[s->bpp_index][qydc];
        s->s.h.segmentation.feat[i].qmul[0][1] = ff_vp9_ac_qlookup[s->bpp_index][qyac];
        s->s.h.segmentation.feat[i].qmul[1][0] = ff_vp9_dc_qlookup[s->bpp_index][quvdc];
        s->s.h.segmentation.feat[i].qmul[1][1] = ff_vp9_ac_qlookup[s->bpp_index][quvac];

        sh = s->s.h.filter.level >= 32;
        if (s->s.h.segmentation.enabled && s->s.h.segmentation.feat[i].lf_enabled) {
            if (s->s.h.segmentation.absolute_vals)
                lflvl = av_clip_uintp2(s->s.h.segmentation.feat[i].lf_val, 6);
            else
                lflvl = av_clip_uintp2(s->s.h.filter.level + s->s.h.segmentation.feat[i].lf_val, 6);
        } else {
            lflvl  = s->s.h.filter.level;
        }
        if (s->s.h.lf_delta.enabled) {
            s->s.h.segmentation.feat[i].lflvl[0][0] =
            s->s.h.segmentation.feat[i].lflvl[0][1] =
                av_clip_uintp2(lflvl + (s->s.h.lf_delta.ref[0] * (1 << sh)), 6);
            for (j = 1; j < 4; j++) {
                s->s.h.segmentation.feat[i].lflvl[j][0] =
                    av_clip_uintp2(lflvl + ((s->s.h.lf_delta.ref[j] +
                                             s->s.h.lf_delta.mode[0]) * (1 << sh)), 6);
                s->s.h.segmentation.feat[i].lflvl[j][1] =
                    av_clip_uintp2(lflvl + ((s->s.h.lf_delta.ref[j] +
                                             s->s.h.lf_delta.mode[1]) * (1 << sh)), 6);
            }
        } else {
            memset(s->s.h.segmentation.feat[i].lflvl, lflvl,
                   sizeof(s->s.h.segmentation.feat[i].lflvl));
        }
    }

    /* tiling info */
    if ((ret = update_size(avctx, w, h)) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to initialize decoder for %dx%d @ %d\n",
               w, h, s->pix_fmt);
        return ret;
    }
    for (s->s.h.tiling.log2_tile_cols = 0;
         s->sb_cols > (64 << s->s.h.tiling.log2_tile_cols);
         s->s.h.tiling.log2_tile_cols++) ;
    for (max = 0; (s->sb_cols >> max) >= 4; max++) ;
    max = FFMAX(0, max - 1);
    while (max > s->s.h.tiling.log2_tile_cols) {
        if (get_bits1(&s->gb))
            s->s.h.tiling.log2_tile_cols++;
        else
            break;
    }
    s->s.h.tiling.log2_tile_rows = decode012(&s->gb);
    s->s.h.tiling.tile_rows = 1 << s->s.h.tiling.log2_tile_rows;
    if (s->s.h.tiling.tile_cols != (1 << s->s.h.tiling.log2_tile_cols)) {
        int n_range_coders;
        VP56RangeCoder *rc;

        if (s->td) {
            for (i = 0; i < s->active_tile_cols; i++) {
                av_free(s->td[i].b_base);
                av_free(s->td[i].block_base);
            }
            av_free(s->td);
        }

        s->s.h.tiling.tile_cols = 1 << s->s.h.tiling.log2_tile_cols;
        vp9_free_entries(avctx);
        s->active_tile_cols = avctx->active_thread_type == FF_THREAD_SLICE ?
                              s->s.h.tiling.tile_cols : 1;
        vp9_alloc_entries(avctx, s->sb_rows);
        if (avctx->active_thread_type == FF_THREAD_SLICE) {
            n_range_coders = 4; // max_tile_rows
        } else {
            n_range_coders = s->s.h.tiling.tile_cols;
        }
        s->td = av_mallocz_array(s->active_tile_cols, sizeof(VP9TileData) +
                                 n_range_coders * sizeof(VP56RangeCoder));
        if (!s->td)
            return AVERROR(ENOMEM);
        rc = (VP56RangeCoder *) &s->td[s->active_tile_cols];
        for (i = 0; i < s->active_tile_cols; i++) {
            s->td[i].s = s;
            s->td[i].c_b = rc;
            rc += n_range_coders;
        }
    }

    /* check reference frames */
    if (!s->s.h.keyframe && !s->s.h.intraonly) {
        for (i = 0; i < 3; i++) {
            AVFrame *ref = s->s.refs[s->s.h.refidx[i]].f;
            int refw = ref->width, refh = ref->height;

            if (ref->format != avctx->pix_fmt) {
                av_log(avctx, AV_LOG_ERROR,
                       "Ref pixfmt (%s) did not match current frame (%s)",
                       av_get_pix_fmt_name(ref->format),
                       av_get_pix_fmt_name(avctx->pix_fmt));
                return AVERROR_INVALIDDATA;
            } else if (refw == w && refh == h) {
                s->mvscale[i][0] = s->mvscale[i][1] = 0;
            } else {
                if (w * 2 < refw || h * 2 < refh || w > 16 * refw || h > 16 * refh) {
                    av_log(avctx, AV_LOG_ERROR,
                           "Invalid ref frame dimensions %dx%d for frame size %dx%d\n",
                           refw, refh, w, h);
                    return AVERROR_INVALIDDATA;
                }
                s->mvscale[i][0] = (refw << 14) / w;
                s->mvscale[i][1] = (refh << 14) / h;
                s->mvstep[i][0] = 16 * s->mvscale[i][0] >> 14;
                s->mvstep[i][1] = 16 * s->mvscale[i][1] >> 14;
            }
        }
    }

    if (s->s.h.keyframe || s->s.h.errorres || (s->s.h.intraonly && s->s.h.resetctx == 3)) {
        s->prob_ctx[0].p = s->prob_ctx[1].p = s->prob_ctx[2].p =
                           s->prob_ctx[3].p = ff_vp9_default_probs;
        memcpy(s->prob_ctx[0].coef, ff_vp9_default_coef_probs,
               sizeof(ff_vp9_default_coef_probs));
        memcpy(s->prob_ctx[1].coef, ff_vp9_default_coef_probs,
               sizeof(ff_vp9_default_coef_probs));
        memcpy(s->prob_ctx[2].coef, ff_vp9_default_coef_probs,
               sizeof(ff_vp9_default_coef_probs));
        memcpy(s->prob_ctx[3].coef, ff_vp9_default_coef_probs,
               sizeof(ff_vp9_default_coef_probs));
    } else if (s->s.h.intraonly && s->s.h.resetctx == 2) {
        s->prob_ctx[c].p = ff_vp9_default_probs;
        memcpy(s->prob_ctx[c].coef, ff_vp9_default_coef_probs,
               sizeof(ff_vp9_default_coef_probs));
    }

    // next 16 bits is size of the rest of the header (arith-coded)
    s->s.h.compressed_header_size = size2 = get_bits(&s->gb, 16);
    s->s.h.uncompressed_header_size = (get_bits_count(&s->gb) + 7) / 8;

    data2 = align_get_bits(&s->gb);
    if (size2 > size - (data2 - data)) {
        av_log(avctx, AV_LOG_ERROR, "Invalid compressed header size\n");
        return AVERROR_INVALIDDATA;
    }
    ret = ff_vp56_init_range_decoder(&s->c, data2, size2);
    if (ret < 0)
        return ret;

    if (vp56_rac_get_prob_branchy(&s->c, 128)) { // marker bit
        av_log(avctx, AV_LOG_ERROR, "Marker bit was set\n");
        return AVERROR_INVALIDDATA;
    }

    for (i = 0; i < s->active_tile_cols; i++) {
        if (s->s.h.keyframe || s->s.h.intraonly) {
            memset(s->td[i].counts.coef, 0, sizeof(s->td[0].counts.coef));
            memset(s->td[i].counts.eob,  0, sizeof(s->td[0].counts.eob));
        } else {
            memset(&s->td[i].counts, 0, sizeof(s->td[0].counts));
        }
    }

    /* FIXME is it faster to not copy here, but do it down in the fw updates
     * as explicit copies if the fw update is missing (and skip the copy upon
     * fw update)? */
    s->prob.p = s->prob_ctx[c].p;

    // txfm updates
    if (s->s.h.lossless) {
        s->s.h.txfmmode = TX_4X4;
    } else {
        s->s.h.txfmmode = vp8_rac_get_uint(&s->c, 2);
        if (s->s.h.txfmmode == 3)
            s->s.h.txfmmode += vp8_rac_get(&s->c);

        if (s->s.h.txfmmode == TX_SWITCHABLE) {
            for (i = 0; i < 2; i++)
                if (vp56_rac_get_prob_branchy(&s->c, 252))
                    s->prob.p.tx8p[i] = update_prob(&s->c, s->prob.p.tx8p[i]);
            for (i = 0; i < 2; i++)
                for (j = 0; j < 2; j++)
                    if (vp56_rac_get_prob_branchy(&s->c, 252))
                        s->prob.p.tx16p[i][j] =
                            update_prob(&s->c, s->prob.p.tx16p[i][j]);
            for (i = 0; i < 2; i++)
                for (j = 0; j < 3; j++)
                    if (vp56_rac_get_prob_branchy(&s->c, 252))
                        s->prob.p.tx32p[i][j] =
                            update_prob(&s->c, s->prob.p.tx32p[i][j]);
        }
    }

    // coef updates
    for (i = 0; i < 4; i++) {
        uint8_t (*ref)[2][6][6][3] = s->prob_ctx[c].coef[i];
        if (vp8_rac_get(&s->c)) {
            for (j = 0; j < 2; j++)
                for (k = 0; k < 2; k++)
                    for (l = 0; l < 6; l++)
                        for (m = 0; m < 6; m++) {
                            uint8_t *p = s->prob.coef[i][j][k][l][m];
                            uint8_t *r = ref[j][k][l][m];
                            if (m >= 3 && l == 0) // dc only has 3 pt
                                break;
                            for (n = 0; n < 3; n++) {
                                if (vp56_rac_get_prob_branchy(&s->c, 252))
                                    p[n] = update_prob(&s->c, r[n]);
                                else
                                    p[n] = r[n];
                            }
                            memcpy(&p[3], ff_vp9_model_pareto8[p[2]], 8);
                        }
        } else {
            for (j = 0; j < 2; j++)
                for (k = 0; k < 2; k++)
                    for (l = 0; l < 6; l++)
                        for (m = 0; m < 6; m++) {
                            uint8_t *p = s->prob.coef[i][j][k][l][m];
                            uint8_t *r = ref[j][k][l][m];
                            if (m > 3 && l == 0) // dc only has 3 pt
                                break;
                            memcpy(p, r, 3);
                            memcpy(&p[3], ff_vp9_model_pareto8[p[2]], 8);
                        }
        }
        if (s->s.h.txfmmode == i)
            break;
    }

    // mode updates
    for (i = 0; i < 3; i++)
        if (vp56_rac_get_prob_branchy(&s->c, 252))
            s->prob.p.skip[i] = update_prob(&s->c, s->prob.p.skip[i]);
    if (!s->s.h.keyframe && !s->s.h.intraonly) {
        for (i = 0; i < 7; i++)
            for (j = 0; j < 3; j++)
                if (vp56_rac_get_prob_branchy(&s->c, 252))
                    s->prob.p.mv_mode[i][j] =
                        update_prob(&s->c, s->prob.p.mv_mode[i][j]);

        if (s->s.h.filtermode == FILTER_SWITCHABLE)
            for (i = 0; i < 4; i++)
                for (j = 0; j < 2; j++)
                    if (vp56_rac_get_prob_branchy(&s->c, 252))
                        s->prob.p.filter[i][j] =
                            update_prob(&s->c, s->prob.p.filter[i][j]);

        for (i = 0; i < 4; i++)
            if (vp56_rac_get_prob_branchy(&s->c, 252))
                s->prob.p.intra[i] = update_prob(&s->c, s->prob.p.intra[i]);

        if (s->s.h.allowcompinter) {
            s->s.h.comppredmode = vp8_rac_get(&s->c);
            if (s->s.h.comppredmode)
                s->s.h.comppredmode += vp8_rac_get(&s->c);
            if (s->s.h.comppredmode == PRED_SWITCHABLE)
                for (i = 0; i < 5; i++)
                    if (vp56_rac_get_prob_branchy(&s->c, 252))
                        s->prob.p.comp[i] =
                            update_prob(&s->c, s->prob.p.comp[i]);
        } else {
            s->s.h.comppredmode = PRED_SINGLEREF;
        }

        if (s->s.h.comppredmode != PRED_COMPREF) {
            for (i = 0; i < 5; i++) {
                if (vp56_rac_get_prob_branchy(&s->c, 252))
                    s->prob.p.single_ref[i][0] =
                        update_prob(&s->c, s->prob.p.single_ref[i][0]);
                if (vp56_rac_get_prob_branchy(&s->c, 252))
                    s->prob.p.single_ref[i][1] =
                        update_prob(&s->c, s->prob.p.single_ref[i][1]);
            }
        }

        if (s->s.h.comppredmode != PRED_SINGLEREF) {
            for (i = 0; i < 5; i++)
                if (vp56_rac_get_prob_branchy(&s->c, 252))
                    s->prob.p.comp_ref[i] =
                        update_prob(&s->c, s->prob.p.comp_ref[i]);
        }

        for (i = 0; i < 4; i++)
            for (j = 0; j < 9; j++)
                if (vp56_rac_get_prob_branchy(&s->c, 252))
                    s->prob.p.y_mode[i][j] =
                        update_prob(&s->c, s->prob.p.y_mode[i][j]);

        for (i = 0; i < 4; i++)
            for (j = 0; j < 4; j++)
                for (k = 0; k < 3; k++)
                    if (vp56_rac_get_prob_branchy(&s->c, 252))
                        s->prob.p.partition[3 - i][j][k] =
                            update_prob(&s->c,
                                        s->prob.p.partition[3 - i][j][k]);

        // mv fields don't use the update_prob subexp model for some reason
        for (i = 0; i < 3; i++)
            if (vp56_rac_get_prob_branchy(&s->c, 252))
                s->prob.p.mv_joint[i] = (vp8_rac_get_uint(&s->c, 7) << 1) | 1;

        for (i = 0; i < 2; i++) {
            if (vp56_rac_get_prob_branchy(&s->c, 252))
                s->prob.p.mv_comp[i].sign =
                    (vp8_rac_get_uint(&s->c, 7) << 1) | 1;

            for (j = 0; j < 10; j++)
                if (vp56_rac_get_prob_branchy(&s->c, 252))
                    s->prob.p.mv_comp[i].classes[j] =
                        (vp8_rac_get_uint(&s->c, 7) << 1) | 1;

            if (vp56_rac_get_prob_branchy(&s->c, 252))
                s->prob.p.mv_comp[i].class0 =
                    (vp8_rac_get_uint(&s->c, 7) << 1) | 1;

            for (j = 0; j < 10; j++)
                if (vp56_rac_get_prob_branchy(&s->c, 252))
                    s->prob.p.mv_comp[i].bits[j] =
                        (vp8_rac_get_uint(&s->c, 7) << 1) | 1;
        }

        for (i = 0; i < 2; i++) {
            for (j = 0; j < 2; j++)
                for (k = 0; k < 3; k++)
                    if (vp56_rac_get_prob_branchy(&s->c, 252))
                        s->prob.p.mv_comp[i].class0_fp[j][k] =
                            (vp8_rac_get_uint(&s->c, 7) << 1) | 1;

            for (j = 0; j < 3; j++)
                if (vp56_rac_get_prob_branchy(&s->c, 252))
                    s->prob.p.mv_comp[i].fp[j] =
                        (vp8_rac_get_uint(&s->c, 7) << 1) | 1;
        }

        if (s->s.h.highprecisionmvs) {
            for (i = 0; i < 2; i++) {
                if (vp56_rac_get_prob_branchy(&s->c, 252))
                    s->prob.p.mv_comp[i].class0_hp =
                        (vp8_rac_get_uint(&s->c, 7) << 1) | 1;

                if (vp56_rac_get_prob_branchy(&s->c, 252))
                    s->prob.p.mv_comp[i].hp =
                        (vp8_rac_get_uint(&s->c, 7) << 1) | 1;
            }
        }
    }

    return (data2 - data) + size2;
}

(??)static av_always_inline void clamp_mv(VP56mv *dst, const VP56mv *src,
(??)                                      VP9Context *s)
(??){
(??)    dst->x = av_clip(src->x, s->min_mv.x, s->max_mv.x);
(??)    dst->y = av_clip(src->y, s->min_mv.y, s->max_mv.y);
(??)}
(??)
(??)static void find_ref_mvs(VP9Context *s,
(??)                         VP56mv *pmv, int ref, int z, int idx, int sb)
(??){
(??)    static const int8_t mv_ref_blk_off[N_BS_SIZES][8][2] = {
(??)        [BS_64x64] = {{  3, -1 }, { -1,  3 }, {  4, -1 }, { -1,  4 },
(??)                      { -1, -1 }, {  0, -1 }, { -1,  0 }, {  6, -1 }},
(??)        [BS_64x32] = {{  0, -1 }, { -1,  0 }, {  4, -1 }, { -1,  2 },
(??)                      { -1, -1 }, {  0, -3 }, { -3,  0 }, {  2, -1 }},
(??)        [BS_32x64] = {{ -1,  0 }, {  0, -1 }, { -1,  4 }, {  2, -1 },
(??)                      { -1, -1 }, { -3,  0 }, {  0, -3 }, { -1,  2 }},
(??)        [BS_32x32] = {{  1, -1 }, { -1,  1 }, {  2, -1 }, { -1,  2 },
(??)                      { -1, -1 }, {  0, -3 }, { -3,  0 }, { -3, -3 }},
(??)        [BS_32x16] = {{  0, -1 }, { -1,  0 }, {  2, -1 }, { -1, -1 },
(??)                      { -1,  1 }, {  0, -3 }, { -3,  0 }, { -3, -3 }},
(??)        [BS_16x32] = {{ -1,  0 }, {  0, -1 }, { -1,  2 }, { -1, -1 },
(??)                      {  1, -1 }, { -3,  0 }, {  0, -3 }, { -3, -3 }},
(??)        [BS_16x16] = {{  0, -1 }, { -1,  0 }, {  1, -1 }, { -1,  1 },
(??)                      { -1, -1 }, {  0, -3 }, { -3,  0 }, { -3, -3 }},
(??)        [BS_16x8]  = {{  0, -1 }, { -1,  0 }, {  1, -1 }, { -1, -1 },
(??)                      {  0, -2 }, { -2,  0 }, { -2, -1 }, { -1, -2 }},
(??)        [BS_8x16]  = {{ -1,  0 }, {  0, -1 }, { -1,  1 }, { -1, -1 },
(??)                      { -2,  0 }, {  0, -2 }, { -1, -2 }, { -2, -1 }},
(??)        [BS_8x8]   = {{  0, -1 }, { -1,  0 }, { -1, -1 }, {  0, -2 },
(??)                      { -2,  0 }, { -1, -2 }, { -2, -1 }, { -2, -2 }},
(??)        [BS_8x4]   = {{  0, -1 }, { -1,  0 }, { -1, -1 }, {  0, -2 },
(??)                      { -2,  0 }, { -1, -2 }, { -2, -1 }, { -2, -2 }},
(??)        [BS_4x8]   = {{  0, -1 }, { -1,  0 }, { -1, -1 }, {  0, -2 },
(??)                      { -2,  0 }, { -1, -2 }, { -2, -1 }, { -2, -2 }},
(??)        [BS_4x4]   = {{  0, -1 }, { -1,  0 }, { -1, -1 }, {  0, -2 },
(??)                      { -2,  0 }, { -1, -2 }, { -2, -1 }, { -2, -2 }},
(??)    };
(??)    VP9Block *b = s->b;
(??)    int row = s->row, col = s->col, row7 = s->row7;
(??)    const int8_t (*p)[2] = mv_ref_blk_off[b->bs];
(??)#define INVALID_MV 0x80008000U
(??)    uint32_t mem = INVALID_MV;
(??)    int i;
(??)
(??)#define RETURN_DIRECT_MV(mv) \
(??)    do { \
(??)        uint32_t m = AV_RN32A(&mv); \
(??)        if (!idx) { \
(??)            AV_WN32A(pmv, m); \
(??)            return; \
(??)        } else if (mem == INVALID_MV) { \
(??)            mem = m; \
(??)        } else if (m != mem) { \
(??)            AV_WN32A(pmv, m); \
(??)            return; \
(??)        } \
(??)    } while (0)
(??)
(??)    if (sb >= 0) {
(??)        if (sb == 2 || sb == 1) {
(??)            RETURN_DIRECT_MV(b->mv[0][z]);
(??)        } else if (sb == 3) {
(??)            RETURN_DIRECT_MV(b->mv[2][z]);
(??)            RETURN_DIRECT_MV(b->mv[1][z]);
(??)            RETURN_DIRECT_MV(b->mv[0][z]);
(??)        }
(??)
(??)#define RETURN_MV(mv) \
(??)    do { \
(??)        if (sb > 0) { \
(??)            VP56mv tmp; \
(??)            uint32_t m; \
(??)            clamp_mv(&tmp, &mv, s); \
(??)            m = AV_RN32A(&tmp); \
(??)            if (!idx) { \
(??)                AV_WN32A(pmv, m); \
(??)                return; \
(??)            } else if (mem == INVALID_MV) { \
(??)                mem = m; \
(??)            } else if (m != mem) { \
(??)                AV_WN32A(pmv, m); \
(??)                return; \
(??)            } \
(??)        } else { \
(??)            uint32_t m = AV_RN32A(&mv); \
(??)            if (!idx) { \
(??)                clamp_mv(pmv, &mv, s); \
(??)                return; \
(??)            } else if (mem == INVALID_MV) { \
(??)                mem = m; \
(??)            } else if (m != mem) { \
(??)                clamp_mv(pmv, &mv, s); \
(??)                return; \
(??)            } \
(??)        } \
(??)    } while (0)
(??)
(??)        if (row > 0) {
(??)            struct VP9mvrefPair *mv = &s->frames[CUR_FRAME].mv[(row - 1) * s->sb_cols * 8 + col];
(??)            if (mv->ref[0] == ref) {
(??)                RETURN_MV(s->above_mv_ctx[2 * col + (sb & 1)][0]);
(??)            } else if (mv->ref[1] == ref) {
(??)                RETURN_MV(s->above_mv_ctx[2 * col + (sb & 1)][1]);
(??)            }
(??)        }
(??)        if (col > s->tiling.tile_col_start) {
(??)            struct VP9mvrefPair *mv = &s->frames[CUR_FRAME].mv[row * s->sb_cols * 8 + col - 1];
(??)            if (mv->ref[0] == ref) {
(??)                RETURN_MV(s->left_mv_ctx[2 * row7 + (sb >> 1)][0]);
(??)            } else if (mv->ref[1] == ref) {
(??)                RETURN_MV(s->left_mv_ctx[2 * row7 + (sb >> 1)][1]);
(??)            }
(??)        }
(??)        i = 2;
(??)    } else {
(??)        i = 0;
(??)    }
(??)
(??)    // previously coded MVs in this neighbourhood, using same reference frame
(??)    for (; i < 8; i++) {
(??)        int c = p[i][0] + col, r = p[i][1] + row;
(??)
(??)        if (c >= s->tiling.tile_col_start && c < s->cols && r >= 0 && r < s->rows) {
(??)            struct VP9mvrefPair *mv = &s->frames[CUR_FRAME].mv[r * s->sb_cols * 8 + c];
(??)
(??)            if (mv->ref[0] == ref) {
(??)                RETURN_MV(mv->mv[0]);
(??)            } else if (mv->ref[1] == ref) {
(??)                RETURN_MV(mv->mv[1]);
(??)            }
(??)        }
(??)    }
(??)
(??)    // MV at this position in previous frame, using same reference frame
(??)    if (s->use_last_frame_mvs) {
(??)        struct VP9mvrefPair *mv = &s->frames[LAST_FRAME].mv[row * s->sb_cols * 8 + col];
(??)
(??)        if (!s->last_uses_2pass)
(??)            ff_thread_await_progress(&s->frames[LAST_FRAME].tf, row >> 3, 0);
(??)        if (mv->ref[0] == ref) {
(??)            RETURN_MV(mv->mv[0]);
(??)        } else if (mv->ref[1] == ref) {
(??)            RETURN_MV(mv->mv[1]);
(??)        }
(??)    }
(??)
(??)#define RETURN_SCALE_MV(mv, scale) \
(??)    do { \
(??)        if (scale) { \
(??)            VP56mv mv_temp = { -mv.x, -mv.y }; \
(??)            RETURN_MV(mv_temp); \
(??)        } else { \
(??)            RETURN_MV(mv); \
(??)        } \
(??)    } while (0)
(??)
(??)    // previously coded MVs in this neighbourhood, using different reference frame
(??)    for (i = 0; i < 8; i++) {
(??)        int c = p[i][0] + col, r = p[i][1] + row;
(??)
(??)        if (c >= s->tiling.tile_col_start && c < s->cols && r >= 0 && r < s->rows) {
(??)            struct VP9mvrefPair *mv = &s->frames[CUR_FRAME].mv[r * s->sb_cols * 8 + c];
(??)
(??)            if (mv->ref[0] != ref && mv->ref[0] >= 0) {
(??)                RETURN_SCALE_MV(mv->mv[0], s->signbias[mv->ref[0]] != s->signbias[ref]);
(??)            }
(??)            if (mv->ref[1] != ref && mv->ref[1] >= 0 &&
(??)                // BUG - libvpx has this condition regardless of whether
(??)                // we used the first ref MV and pre-scaling
(??)                AV_RN32A(&mv->mv[0]) != AV_RN32A(&mv->mv[1])) {
(??)                RETURN_SCALE_MV(mv->mv[1], s->signbias[mv->ref[1]] != s->signbias[ref]);
(??)            }
(??)        }
(??)    }
(??)
(??)    // MV at this position in previous frame, using different reference frame
(??)    if (s->use_last_frame_mvs) {
(??)        struct VP9mvrefPair *mv = &s->frames[LAST_FRAME].mv[row * s->sb_cols * 8 + col];
(??)
(??)        // no need to await_progress, because we already did that above
(??)        if (mv->ref[0] != ref && mv->ref[0] >= 0) {
(??)            RETURN_SCALE_MV(mv->mv[0], s->signbias[mv->ref[0]] != s->signbias[ref]);
(??)        }
(??)        if (mv->ref[1] != ref && mv->ref[1] >= 0 &&
(??)            // BUG - libvpx has this condition regardless of whether
(??)            // we used the first ref MV and pre-scaling
(??)            AV_RN32A(&mv->mv[0]) != AV_RN32A(&mv->mv[1])) {
(??)            RETURN_SCALE_MV(mv->mv[1], s->signbias[mv->ref[1]] != s->signbias[ref]);
(??)        }
(??)    }
(??)
(??)    AV_ZERO32(pmv);
(??)#undef INVALID_MV
(??)#undef RETURN_MV
(??)#undef RETURN_SCALE_MV
(??)}
(??)
(??)static av_always_inline int read_mv_component(VP9Context *s, int idx, int hp)
(??){
(??)    int bit, sign = vp56_rac_get_prob(&s->c, s->prob.p.mv_comp[idx].sign);
(??)    int n, c = vp8_rac_get_tree(&s->c, vp9_mv_class_tree,
(??)                                s->prob.p.mv_comp[idx].classes);
(??)
(??)    s->counts.mv_comp[idx].sign[sign]++;
(??)    s->counts.mv_comp[idx].classes[c]++;
(??)    if (c) {
(??)        int m;
(??)
(??)        for (n = 0, m = 0; m < c; m++) {
(??)            bit = vp56_rac_get_prob(&s->c, s->prob.p.mv_comp[idx].bits[m]);
(??)            n |= bit << m;
(??)            s->counts.mv_comp[idx].bits[m][bit]++;
(??)        }
(??)        n <<= 3;
(??)        bit = vp8_rac_get_tree(&s->c, vp9_mv_fp_tree, s->prob.p.mv_comp[idx].fp);
(??)        n |= bit << 1;
(??)        s->counts.mv_comp[idx].fp[bit]++;
(??)        if (hp) {
(??)            bit = vp56_rac_get_prob(&s->c, s->prob.p.mv_comp[idx].hp);
(??)            s->counts.mv_comp[idx].hp[bit]++;
(??)            n |= bit;
(??)        } else {
(??)            n |= 1;
(??)            // bug in libvpx - we count for bw entropy purposes even if the
(??)            // bit wasn't coded
(??)            s->counts.mv_comp[idx].hp[1]++;
(??)        }
(??)        n += 8 << c;
(??)    } else {
(??)        n = vp56_rac_get_prob(&s->c, s->prob.p.mv_comp[idx].class0);
(??)        s->counts.mv_comp[idx].class0[n]++;
(??)        bit = vp8_rac_get_tree(&s->c, vp9_mv_fp_tree,
(??)                               s->prob.p.mv_comp[idx].class0_fp[n]);
(??)        s->counts.mv_comp[idx].class0_fp[n][bit]++;
(??)        n = (n << 3) | (bit << 1);
(??)        if (hp) {
(??)            bit = vp56_rac_get_prob(&s->c, s->prob.p.mv_comp[idx].class0_hp);
(??)            s->counts.mv_comp[idx].class0_hp[bit]++;
(??)            n |= bit;
(??)        } else {
(??)            n |= 1;
(??)            // bug in libvpx - we count for bw entropy purposes even if the
(??)            // bit wasn't coded
(??)            s->counts.mv_comp[idx].class0_hp[1]++;
(??)        }
(??)    }
(??)
(??)    return sign ? -(n + 1) : (n + 1);
(??)}
(??)
(??)static void fill_mv(VP9Context *s,
(??)                    VP56mv *mv, int mode, int sb)
(??){
(??)    VP9Block *b = s->b;
(??)
(??)    if (mode == ZEROMV) {
(??)        AV_ZERO64(mv);
(??)    } else {
(??)        int hp;
(??)
(??)        // FIXME cache this value and reuse for other subblocks
(??)        find_ref_mvs(s, &mv[0], b->ref[0], 0, mode == NEARMV,
(??)                     mode == NEWMV ? -1 : sb);
(??)        // FIXME maybe move this code into find_ref_mvs()
(??)        if ((mode == NEWMV || sb == -1) &&
(??)            !(hp = s->highprecisionmvs && abs(mv[0].x) < 64 && abs(mv[0].y) < 64)) {
(??)            if (mv[0].y & 1) {
(??)                if (mv[0].y < 0)
(??)                    mv[0].y++;
(??)                else
(??)                    mv[0].y--;
(??)            }
(??)            if (mv[0].x & 1) {
(??)                if (mv[0].x < 0)
(??)                    mv[0].x++;
(??)                else
(??)                    mv[0].x--;
(??)            }
(??)        }
(??)        if (mode == NEWMV) {
(??)            enum MVJoint j = vp8_rac_get_tree(&s->c, vp9_mv_joint_tree,
(??)                                              s->prob.p.mv_joint);
(??)
(??)            s->counts.mv_joint[j]++;
(??)            if (j >= MV_JOINT_V)
(??)                mv[0].y += read_mv_component(s, 0, hp);
(??)            if (j & 1)
(??)                mv[0].x += read_mv_component(s, 1, hp);
(??)        }
(??)
(??)        if (b->comp) {
(??)            // FIXME cache this value and reuse for other subblocks
(??)            find_ref_mvs(s, &mv[1], b->ref[1], 1, mode == NEARMV,
(??)                         mode == NEWMV ? -1 : sb);
(??)            if ((mode == NEWMV || sb == -1) &&
(??)                !(hp = s->highprecisionmvs && abs(mv[1].x) < 64 && abs(mv[1].y) < 64)) {
(??)                if (mv[1].y & 1) {
(??)                    if (mv[1].y < 0)
(??)                        mv[1].y++;
(??)                    else
(??)                        mv[1].y--;
(??)                }
(??)                if (mv[1].x & 1) {
(??)                    if (mv[1].x < 0)
(??)                        mv[1].x++;
(??)                    else
(??)                        mv[1].x--;
(??)                }
(??)            }
(??)            if (mode == NEWMV) {
(??)                enum MVJoint j = vp8_rac_get_tree(&s->c, vp9_mv_joint_tree,
(??)                                                  s->prob.p.mv_joint);
(??)
(??)                s->counts.mv_joint[j]++;
(??)                if (j >= MV_JOINT_V)
(??)                    mv[1].y += read_mv_component(s, 0, hp);
(??)                if (j & 1)
(??)                    mv[1].x += read_mv_component(s, 1, hp);
(??)            }
(??)        }
(??)    }
(??)}
(??)
(??)static av_always_inline void setctx_2d(uint8_t *ptr, int w, int h,
(??)                                       ptrdiff_t stride, int v)
(??){
(??)    switch (w) {
(??)    case 1:
(??)        do {
(??)            *ptr = v;
(??)            ptr += stride;
(??)        } while (--h);
(??)        break;
(??)    case 2: {
(??)        int v16 = v * 0x0101;
(??)        do {
(??)            AV_WN16A(ptr, v16);
(??)            ptr += stride;
(??)        } while (--h);
(??)        break;
(??)    }
(??)    case 4: {
(??)        uint32_t v32 = v * 0x01010101;
(??)        do {
(??)            AV_WN32A(ptr, v32);
(??)            ptr += stride;
(??)        } while (--h);
(??)        break;
(??)    }
(??)    case 8: {
(??)#if HAVE_FAST_64BIT
(??)        uint64_t v64 = v * 0x0101010101010101ULL;
(??)        do {
(??)            AV_WN64A(ptr, v64);
(??)            ptr += stride;
(??)        } while (--h);
(??)#else
(??)        uint32_t v32 = v * 0x01010101;
(??)        do {
(??)            AV_WN32A(ptr,     v32);
(??)            AV_WN32A(ptr + 4, v32);
(??)            ptr += stride;
(??)        } while (--h);
(??)#endif
(??)        break;
(??)    }
(??)    }
(??)}
(??)
(??)static void decode_mode(AVCodecContext *ctx)
(??){
(??)    static const uint8_t left_ctx[N_BS_SIZES] = {
(??)        0x0, 0x8, 0x0, 0x8, 0xc, 0x8, 0xc, 0xe, 0xc, 0xe, 0xf, 0xe, 0xf
(??)    };
(??)    static const uint8_t above_ctx[N_BS_SIZES] = {
(??)        0x0, 0x0, 0x8, 0x8, 0x8, 0xc, 0xc, 0xc, 0xe, 0xe, 0xe, 0xf, 0xf
(??)    };
(??)    static const uint8_t max_tx_for_bl_bp[N_BS_SIZES] = {
(??)        TX_32X32, TX_32X32, TX_32X32, TX_32X32, TX_16X16, TX_16X16,
(??)        TX_16X16, TX_8X8, TX_8X8, TX_8X8, TX_4X4, TX_4X4, TX_4X4
(??)    };
(??)    VP9Context *s = ctx->priv_data;
(??)    VP9Block *b = s->b;
(??)    int row = s->row, col = s->col, row7 = s->row7;
(??)    enum TxfmMode max_tx = max_tx_for_bl_bp[b->bs];
(??)    int w4 = FFMIN(s->cols - col, bwh_tab[1][b->bs][0]);
(??)    int h4 = FFMIN(s->rows - row, bwh_tab[1][b->bs][1]), y;
(??)    int have_a = row > 0, have_l = col > s->tiling.tile_col_start;
(??)    int vref, filter_id;
(??)
(??)    if (!s->segmentation.enabled) {
(??)        b->seg_id = 0;
(??)    } else if (s->keyframe || s->intraonly) {
(??)        b->seg_id = vp8_rac_get_tree(&s->c, vp9_segmentation_tree, s->prob.seg);
(??)    } else if (!s->segmentation.update_map ||
(??)               (s->segmentation.temporal &&
(??)                vp56_rac_get_prob_branchy(&s->c,
(??)                    s->prob.segpred[s->above_segpred_ctx[col] +
(??)                                    s->left_segpred_ctx[row7]]))) {
(??)        if (!s->errorres) {
(??)            int pred = 8, x;
(??)            uint8_t *refsegmap = s->frames[LAST_FRAME].segmentation_map;
(??)
(??)            if (!s->last_uses_2pass)
(??)                ff_thread_await_progress(&s->frames[LAST_FRAME].tf, row >> 3, 0);
(??)            for (y = 0; y < h4; y++)
(??)                for (x = 0; x < w4; x++)
(??)                    pred = FFMIN(pred, refsegmap[(y + row) * 8 * s->sb_cols + x + col]);
(??)            av_assert1(pred < 8);
(??)            b->seg_id = pred;
(??)        } else {
(??)            b->seg_id = 0;
(??)        }
(??)
(??)        memset(&s->above_segpred_ctx[col], 1, w4);
(??)        memset(&s->left_segpred_ctx[row7], 1, h4);
(??)    } else {
(??)        b->seg_id = vp8_rac_get_tree(&s->c, vp9_segmentation_tree,
(??)                                     s->prob.seg);
(??)
(??)        memset(&s->above_segpred_ctx[col], 0, w4);
(??)        memset(&s->left_segpred_ctx[row7], 0, h4);
(??)    }
(??)    if (s->segmentation.enabled &&
(??)        (s->segmentation.update_map || s->keyframe || s->intraonly)) {
(??)        setctx_2d(&s->frames[CUR_FRAME].segmentation_map[row * 8 * s->sb_cols + col],
(??)                  w4, h4, 8 * s->sb_cols, b->seg_id);
(??)    }
(??)
(??)    b->skip = s->segmentation.enabled &&
(??)        s->segmentation.feat[b->seg_id].skip_enabled;
(??)    if (!b->skip) {
(??)        int c = s->left_skip_ctx[row7] + s->above_skip_ctx[col];
(??)        b->skip = vp56_rac_get_prob(&s->c, s->prob.p.skip[c]);
(??)        s->counts.skip[c][b->skip]++;
(??)    }
(??)
(??)    if (s->keyframe || s->intraonly) {
(??)        b->intra = 1;
(??)    } else if (s->segmentation.feat[b->seg_id].ref_enabled) {
(??)        b->intra = !s->segmentation.feat[b->seg_id].ref_val;
(??)    } else {
(??)        int c, bit;
(??)
(??)        if (have_a && have_l) {
(??)            c = s->above_intra_ctx[col] + s->left_intra_ctx[row7];
(??)            c += (c == 2);
(??)        } else {
(??)            c = have_a ? 2 * s->above_intra_ctx[col] :
(??)                have_l ? 2 * s->left_intra_ctx[row7] : 0;
(??)        }
(??)        bit = vp56_rac_get_prob(&s->c, s->prob.p.intra[c]);
(??)        s->counts.intra[c][bit]++;
(??)        b->intra = !bit;
(??)    }
(??)
(??)    if ((b->intra || !b->skip) && s->txfmmode == TX_SWITCHABLE) {
(??)        int c;
(??)        if (have_a) {
(??)            if (have_l) {
(??)                c = (s->above_skip_ctx[col] ? max_tx :
(??)                     s->above_txfm_ctx[col]) +
(??)                    (s->left_skip_ctx[row7] ? max_tx :
(??)                     s->left_txfm_ctx[row7]) > max_tx;
(??)            } else {
(??)                c = s->above_skip_ctx[col] ? 1 :
(??)                    (s->above_txfm_ctx[col] * 2 > max_tx);
(??)            }
(??)        } else if (have_l) {
(??)            c = s->left_skip_ctx[row7] ? 1 :
(??)                (s->left_txfm_ctx[row7] * 2 > max_tx);
(??)        } else {
(??)            c = 1;
(??)        }
(??)        switch (max_tx) {
(??)        case TX_32X32:
(??)            b->tx = vp56_rac_get_prob(&s->c, s->prob.p.tx32p[c][0]);
(??)            if (b->tx) {
(??)                b->tx += vp56_rac_get_prob(&s->c, s->prob.p.tx32p[c][1]);
(??)                if (b->tx == 2)
(??)                    b->tx += vp56_rac_get_prob(&s->c, s->prob.p.tx32p[c][2]);
(??)            }
(??)            s->counts.tx32p[c][b->tx]++;
(??)            break;
(??)        case TX_16X16:
(??)            b->tx = vp56_rac_get_prob(&s->c, s->prob.p.tx16p[c][0]);
(??)            if (b->tx)
(??)                b->tx += vp56_rac_get_prob(&s->c, s->prob.p.tx16p[c][1]);
(??)            s->counts.tx16p[c][b->tx]++;
(??)            break;
(??)        case TX_8X8:
(??)            b->tx = vp56_rac_get_prob(&s->c, s->prob.p.tx8p[c]);
(??)            s->counts.tx8p[c][b->tx]++;
(??)            break;
(??)        case TX_4X4:
(??)            b->tx = TX_4X4;
(??)            break;
(??)        }
(??)    } else {
(??)        b->tx = FFMIN(max_tx, s->txfmmode);
(??)    }
(??)
(??)    if (s->keyframe || s->intraonly) {
(??)        uint8_t *a = &s->above_mode_ctx[col * 2];
(??)        uint8_t *l = &s->left_mode_ctx[(row7) << 1];
(??)
(??)        b->comp = 0;
(??)        if (b->bs > BS_8x8) {
(??)            // FIXME the memory storage intermediates here aren't really
(??)            // necessary, they're just there to make the code slightly
(??)            // simpler for now
(??)            b->mode[0] = a[0] = vp8_rac_get_tree(&s->c, vp9_intramode_tree,
(??)                                    vp9_default_kf_ymode_probs[a[0]][l[0]]);
(??)            if (b->bs != BS_8x4) {
(??)                b->mode[1] = vp8_rac_get_tree(&s->c, vp9_intramode_tree,
(??)                                 vp9_default_kf_ymode_probs[a[1]][b->mode[0]]);
(??)                l[0] = a[1] = b->mode[1];
(??)            } else {
(??)                l[0] = a[1] = b->mode[1] = b->mode[0];
(??)            }
(??)            if (b->bs != BS_4x8) {
(??)                b->mode[2] = a[0] = vp8_rac_get_tree(&s->c, vp9_intramode_tree,
(??)                                        vp9_default_kf_ymode_probs[a[0]][l[1]]);
(??)                if (b->bs != BS_8x4) {
(??)                    b->mode[3] = vp8_rac_get_tree(&s->c, vp9_intramode_tree,
(??)                                  vp9_default_kf_ymode_probs[a[1]][b->mode[2]]);
(??)                    l[1] = a[1] = b->mode[3];
(??)                } else {
(??)                    l[1] = a[1] = b->mode[3] = b->mode[2];
(??)                }
(??)            } else {
(??)                b->mode[2] = b->mode[0];
(??)                l[1] = a[1] = b->mode[3] = b->mode[1];
(??)            }
(??)        } else {
(??)            b->mode[0] = vp8_rac_get_tree(&s->c, vp9_intramode_tree,
(??)                                          vp9_default_kf_ymode_probs[*a][*l]);
(??)            b->mode[3] = b->mode[2] = b->mode[1] = b->mode[0];
(??)            // FIXME this can probably be optimized
(??)            memset(a, b->mode[0], bwh_tab[0][b->bs][0]);
(??)            memset(l, b->mode[0], bwh_tab[0][b->bs][1]);
(??)        }
(??)        b->uvmode = vp8_rac_get_tree(&s->c, vp9_intramode_tree,
(??)                                     vp9_default_kf_uvmode_probs[b->mode[3]]);
(??)    } else if (b->intra) {
(??)        b->comp = 0;
(??)        if (b->bs > BS_8x8) {
(??)            b->mode[0] = vp8_rac_get_tree(&s->c, vp9_intramode_tree,
(??)                                          s->prob.p.y_mode[0]);
(??)            s->counts.y_mode[0][b->mode[0]]++;
(??)            if (b->bs != BS_8x4) {
(??)                b->mode[1] = vp8_rac_get_tree(&s->c, vp9_intramode_tree,
(??)                                              s->prob.p.y_mode[0]);
(??)                s->counts.y_mode[0][b->mode[1]]++;
(??)            } else {
(??)                b->mode[1] = b->mode[0];
(??)            }
(??)            if (b->bs != BS_4x8) {
(??)                b->mode[2] = vp8_rac_get_tree(&s->c, vp9_intramode_tree,
(??)                                              s->prob.p.y_mode[0]);
(??)                s->counts.y_mode[0][b->mode[2]]++;
(??)                if (b->bs != BS_8x4) {
(??)                    b->mode[3] = vp8_rac_get_tree(&s->c, vp9_intramode_tree,
(??)                                                  s->prob.p.y_mode[0]);
(??)                    s->counts.y_mode[0][b->mode[3]]++;
(??)                } else {
(??)                    b->mode[3] = b->mode[2];
(??)                }
(??)            } else {
(??)                b->mode[2] = b->mode[0];
(??)                b->mode[3] = b->mode[1];
(??)            }
(??)        } else {
(??)            static const uint8_t size_group[10] = {
(??)                3, 3, 3, 3, 2, 2, 2, 1, 1, 1
(??)            };
(??)            int sz = size_group[b->bs];
(??)
(??)            b->mode[0] = vp8_rac_get_tree(&s->c, vp9_intramode_tree,
(??)                                          s->prob.p.y_mode[sz]);
(??)            b->mode[1] = b->mode[2] = b->mode[3] = b->mode[0];
(??)            s->counts.y_mode[sz][b->mode[3]]++;
(??)        }
(??)        b->uvmode = vp8_rac_get_tree(&s->c, vp9_intramode_tree,
(??)                                     s->prob.p.uv_mode[b->mode[3]]);
(??)        s->counts.uv_mode[b->mode[3]][b->uvmode]++;
(??)    } else {
(??)        static const uint8_t inter_mode_ctx_lut[14][14] = {
(??)            { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5 },
(??)            { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5 },
(??)            { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5 },
(??)            { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5 },
(??)            { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5 },
(??)            { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5 },
(??)            { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5 },
(??)            { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5 },
(??)            { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5 },
(??)            { 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5 },
(??)            { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 2, 2, 1, 3 },
(??)            { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 2, 2, 1, 3 },
(??)            { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 1, 1, 0, 3 },
(??)            { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 3, 3, 3, 4 },
(??)        };
(??)
(??)        if (s->segmentation.feat[b->seg_id].ref_enabled) {
(??)            av_assert2(s->segmentation.feat[b->seg_id].ref_val != 0);
(??)            b->comp = 0;
(??)            b->ref[0] = s->segmentation.feat[b->seg_id].ref_val - 1;
(??)        } else {
(??)            // read comp_pred flag
(??)            if (s->comppredmode != PRED_SWITCHABLE) {
(??)                b->comp = s->comppredmode == PRED_COMPREF;
(??)            } else {
(??)                int c;
(??)
(??)                // FIXME add intra as ref=0xff (or -1) to make these easier?
(??)                if (have_a) {
(??)                    if (have_l) {
(??)                        if (s->above_comp_ctx[col] && s->left_comp_ctx[row7]) {
(??)                            c = 4;
(??)                        } else if (s->above_comp_ctx[col]) {
(??)                            c = 2 + (s->left_intra_ctx[row7] ||
(??)                                     s->left_ref_ctx[row7] == s->fixcompref);
(??)                        } else if (s->left_comp_ctx[row7]) {
(??)                            c = 2 + (s->above_intra_ctx[col] ||
(??)                                     s->above_ref_ctx[col] == s->fixcompref);
(??)                        } else {
(??)                            c = (!s->above_intra_ctx[col] &&
(??)                                 s->above_ref_ctx[col] == s->fixcompref) ^
(??)                            (!s->left_intra_ctx[row7] &&
(??)                             s->left_ref_ctx[row & 7] == s->fixcompref);
(??)                        }
(??)                    } else {
(??)                        c = s->above_comp_ctx[col] ? 3 :
(??)                        (!s->above_intra_ctx[col] && s->above_ref_ctx[col] == s->fixcompref);
(??)                    }
(??)                } else if (have_l) {
(??)                    c = s->left_comp_ctx[row7] ? 3 :
(??)                    (!s->left_intra_ctx[row7] && s->left_ref_ctx[row7] == s->fixcompref);
(??)                } else {
(??)                    c = 1;
(??)                }
(??)                b->comp = vp56_rac_get_prob(&s->c, s->prob.p.comp[c]);
(??)                s->counts.comp[c][b->comp]++;
(??)            }
(??)
(??)            // read actual references
(??)            // FIXME probably cache a few variables here to prevent repetitive
(??)            // memory accesses below
(??)            if (b->comp) /* two references */ {
(??)                int fix_idx = s->signbias[s->fixcompref], var_idx = !fix_idx, c, bit;
(??)
(??)                b->ref[fix_idx] = s->fixcompref;
(??)                // FIXME can this codeblob be replaced by some sort of LUT?
(??)                if (have_a) {
(??)                    if (have_l) {
(??)                        if (s->above_intra_ctx[col]) {
(??)                            if (s->left_intra_ctx[row7]) {
(??)                                c = 2;
(??)                            } else {
(??)                                c = 1 + 2 * (s->left_ref_ctx[row7] != s->varcompref[1]);
(??)                            }
(??)                        } else if (s->left_intra_ctx[row7]) {
(??)                            c = 1 + 2 * (s->above_ref_ctx[col] != s->varcompref[1]);
(??)                        } else {
(??)                            int refl = s->left_ref_ctx[row7], refa = s->above_ref_ctx[col];
(??)
(??)                            if (refl == refa && refa == s->varcompref[1]) {
(??)                                c = 0;
(??)                            } else if (!s->left_comp_ctx[row7] && !s->above_comp_ctx[col]) {
(??)                                if ((refa == s->fixcompref && refl == s->varcompref[0]) ||
(??)                                    (refl == s->fixcompref && refa == s->varcompref[0])) {
(??)                                    c = 4;
(??)                                } else {
(??)                                    c = (refa == refl) ? 3 : 1;
(??)                                }
(??)                            } else if (!s->left_comp_ctx[row7]) {
(??)                                if (refa == s->varcompref[1] && refl != s->varcompref[1]) {
(??)                                    c = 1;
(??)                                } else {
(??)                                    c = (refl == s->varcompref[1] &&
(??)                                         refa != s->varcompref[1]) ? 2 : 4;
(??)                                }
(??)                            } else if (!s->above_comp_ctx[col]) {
(??)                                if (refl == s->varcompref[1] && refa != s->varcompref[1]) {
(??)                                    c = 1;
(??)                                } else {
(??)                                    c = (refa == s->varcompref[1] &&
(??)                                         refl != s->varcompref[1]) ? 2 : 4;
(??)                                }
(??)                            } else {
(??)                                c = (refl == refa) ? 4 : 2;
(??)                            }
(??)                        }
(??)                    } else {
(??)                        if (s->above_intra_ctx[col]) {
(??)                            c = 2;
(??)                        } else if (s->above_comp_ctx[col]) {
(??)                            c = 4 * (s->above_ref_ctx[col] != s->varcompref[1]);
(??)                        } else {
(??)                            c = 3 * (s->above_ref_ctx[col] != s->varcompref[1]);
(??)                        }
(??)                    }
(??)                } else if (have_l) {
(??)                    if (s->left_intra_ctx[row7]) {
(??)                        c = 2;
(??)                    } else if (s->left_comp_ctx[row7]) {
(??)                        c = 4 * (s->left_ref_ctx[row7] != s->varcompref[1]);
(??)                    } else {
(??)                        c = 3 * (s->left_ref_ctx[row7] != s->varcompref[1]);
(??)                    }
(??)                } else {
(??)                    c = 2;
(??)                }
(??)                bit = vp56_rac_get_prob(&s->c, s->prob.p.comp_ref[c]);
(??)                b->ref[var_idx] = s->varcompref[bit];
(??)                s->counts.comp_ref[c][bit]++;
(??)            } else /* single reference */ {
(??)                int bit, c;
(??)
(??)                if (have_a && !s->above_intra_ctx[col]) {
(??)                    if (have_l && !s->left_intra_ctx[row7]) {
(??)                        if (s->left_comp_ctx[row7]) {
(??)                            if (s->above_comp_ctx[col]) {
(??)                                c = 1 + (!s->fixcompref || !s->left_ref_ctx[row7] ||
(??)                                         !s->above_ref_ctx[col]);
(??)                            } else {
(??)                                c = (3 * !s->above_ref_ctx[col]) +
(??)                                    (!s->fixcompref || !s->left_ref_ctx[row7]);
(??)                            }
(??)                        } else if (s->above_comp_ctx[col]) {
(??)                            c = (3 * !s->left_ref_ctx[row7]) +
(??)                                (!s->fixcompref || !s->above_ref_ctx[col]);
(??)                        } else {
(??)                            c = 2 * !s->left_ref_ctx[row7] + 2 * !s->above_ref_ctx[col];
(??)                        }
(??)                    } else if (s->above_intra_ctx[col]) {
(??)                        c = 2;
(??)                    } else if (s->above_comp_ctx[col]) {
(??)                        c = 1 + (!s->fixcompref || !s->above_ref_ctx[col]);
(??)                    } else {
(??)                        c = 4 * (!s->above_ref_ctx[col]);
(??)                    }
(??)                } else if (have_l && !s->left_intra_ctx[row7]) {
(??)                    if (s->left_intra_ctx[row7]) {
(??)                        c = 2;
(??)                    } else if (s->left_comp_ctx[row7]) {
(??)                        c = 1 + (!s->fixcompref || !s->left_ref_ctx[row7]);
(??)                    } else {
(??)                        c = 4 * (!s->left_ref_ctx[row7]);
(??)                    }
(??)                } else {
(??)                    c = 2;
(??)                }
(??)                bit = vp56_rac_get_prob(&s->c, s->prob.p.single_ref[c][0]);
(??)                s->counts.single_ref[c][0][bit]++;
(??)                if (!bit) {
(??)                    b->ref[0] = 0;
(??)                } else {
(??)                    // FIXME can this codeblob be replaced by some sort of LUT?
(??)                    if (have_a) {
(??)                        if (have_l) {
(??)                            if (s->left_intra_ctx[row7]) {
(??)                                if (s->above_intra_ctx[col]) {
(??)                                    c = 2;
(??)                                } else if (s->above_comp_ctx[col]) {
(??)                                    c = 1 + 2 * (s->fixcompref == 1 ||
(??)                                                 s->above_ref_ctx[col] == 1);
(??)                                } else if (!s->above_ref_ctx[col]) {
(??)                                    c = 3;
(??)                                } else {
(??)                                    c = 4 * (s->above_ref_ctx[col] == 1);
(??)                                }
(??)                            } else if (s->above_intra_ctx[col]) {
(??)                                if (s->left_intra_ctx[row7]) {
(??)                                    c = 2;
(??)                                } else if (s->left_comp_ctx[row7]) {
(??)                                    c = 1 + 2 * (s->fixcompref == 1 ||
(??)                                                 s->left_ref_ctx[row7] == 1);
(??)                                } else if (!s->left_ref_ctx[row7]) {
(??)                                    c = 3;
(??)                                } else {
(??)                                    c = 4 * (s->left_ref_ctx[row7] == 1);
(??)                                }
(??)                            } else if (s->above_comp_ctx[col]) {
(??)                                if (s->left_comp_ctx[row7]) {
(??)                                    if (s->left_ref_ctx[row7] == s->above_ref_ctx[col]) {
(??)                                        c = 3 * (s->fixcompref == 1 ||
(??)                                                 s->left_ref_ctx[row7] == 1);
(??)                                    } else {
(??)                                        c = 2;
(??)                                    }
(??)                                } else if (!s->left_ref_ctx[row7]) {
(??)                                    c = 1 + 2 * (s->fixcompref == 1 ||
(??)                                                 s->above_ref_ctx[col] == 1);
(??)                                } else {
(??)                                    c = 3 * (s->left_ref_ctx[row7] == 1) +
(??)                                    (s->fixcompref == 1 || s->above_ref_ctx[col] == 1);
(??)                                }
(??)                            } else if (s->left_comp_ctx[row7]) {
(??)                                if (!s->above_ref_ctx[col]) {
(??)                                    c = 1 + 2 * (s->fixcompref == 1 ||
(??)                                                 s->left_ref_ctx[row7] == 1);
(??)                                } else {
(??)                                    c = 3 * (s->above_ref_ctx[col] == 1) +
(??)                                    (s->fixcompref == 1 || s->left_ref_ctx[row7] == 1);
(??)                                }
(??)                            } else if (!s->above_ref_ctx[col]) {
(??)                                if (!s->left_ref_ctx[row7]) {
(??)                                    c = 3;
(??)                                } else {
(??)                                    c = 4 * (s->left_ref_ctx[row7] == 1);
(??)                                }
(??)                            } else if (!s->left_ref_ctx[row7]) {
(??)                                c = 4 * (s->above_ref_ctx[col] == 1);
(??)                            } else {
(??)                                c = 2 * (s->left_ref_ctx[row7] == 1) +
(??)                                2 * (s->above_ref_ctx[col] == 1);
(??)                            }
(??)                        } else {
(??)                            if (s->above_intra_ctx[col] ||
(??)                                (!s->above_comp_ctx[col] && !s->above_ref_ctx[col])) {
(??)                                c = 2;
(??)                            } else if (s->above_comp_ctx[col]) {
(??)                                c = 3 * (s->fixcompref == 1 || s->above_ref_ctx[col] == 1);
(??)                            } else {
(??)                                c = 4 * (s->above_ref_ctx[col] == 1);
(??)                            }
(??)                        }
(??)                    } else if (have_l) {
(??)                        if (s->left_intra_ctx[row7] ||
(??)                            (!s->left_comp_ctx[row7] && !s->left_ref_ctx[row7])) {
(??)                            c = 2;
(??)                        } else if (s->left_comp_ctx[row7]) {
(??)                            c = 3 * (s->fixcompref == 1 || s->left_ref_ctx[row7] == 1);
(??)                        } else {
(??)                            c = 4 * (s->left_ref_ctx[row7] == 1);
(??)                        }
(??)                    } else {
(??)                        c = 2;
(??)                    }
(??)                    bit = vp56_rac_get_prob(&s->c, s->prob.p.single_ref[c][1]);
(??)                    s->counts.single_ref[c][1][bit]++;
(??)                    b->ref[0] = 1 + bit;
(??)                }
(??)            }
(??)        }
(??)
(??)        if (b->bs <= BS_8x8) {
(??)            if (s->segmentation.feat[b->seg_id].skip_enabled) {
(??)                b->mode[0] = b->mode[1] = b->mode[2] = b->mode[3] = ZEROMV;
(??)            } else {
(??)                static const uint8_t off[10] = {
(??)                    3, 0, 0, 1, 0, 0, 0, 0, 0, 0
(??)                };
(??)
(??)                // FIXME this needs to use the LUT tables from find_ref_mvs
(??)                // because not all are -1,0/0,-1
(??)                int c = inter_mode_ctx_lut[s->above_mode_ctx[col + off[b->bs]]]
(??)                                          [s->left_mode_ctx[row7 + off[b->bs]]];
(??)
(??)                b->mode[0] = vp8_rac_get_tree(&s->c, vp9_inter_mode_tree,
(??)                                              s->prob.p.mv_mode[c]);
(??)                b->mode[1] = b->mode[2] = b->mode[3] = b->mode[0];
(??)                s->counts.mv_mode[c][b->mode[0] - 10]++;
(??)            }
(??)        }
(??)
(??)        if (s->filtermode == FILTER_SWITCHABLE) {
(??)            int c;
(??)
(??)            if (have_a && s->above_mode_ctx[col] >= NEARESTMV) {
(??)                if (have_l && s->left_mode_ctx[row7] >= NEARESTMV) {
(??)                    c = s->above_filter_ctx[col] == s->left_filter_ctx[row7] ?
(??)                        s->left_filter_ctx[row7] : 3;
(??)                } else {
(??)                    c = s->above_filter_ctx[col];
(??)                }
(??)            } else if (have_l && s->left_mode_ctx[row7] >= NEARESTMV) {
(??)                c = s->left_filter_ctx[row7];
(??)            } else {
(??)                c = 3;
(??)            }
(??)
(??)            filter_id = vp8_rac_get_tree(&s->c, vp9_filter_tree,
(??)                                         s->prob.p.filter[c]);
(??)            s->counts.filter[c][filter_id]++;
(??)            b->filter = vp9_filter_lut[filter_id];
(??)        } else {
(??)            b->filter = s->filtermode;
(??)        }
(??)
(??)        if (b->bs > BS_8x8) {
(??)            int c = inter_mode_ctx_lut[s->above_mode_ctx[col]][s->left_mode_ctx[row7]];
(??)
(??)            b->mode[0] = vp8_rac_get_tree(&s->c, vp9_inter_mode_tree,
(??)                                          s->prob.p.mv_mode[c]);
(??)            s->counts.mv_mode[c][b->mode[0] - 10]++;
(??)            fill_mv(s, b->mv[0], b->mode[0], 0);
(??)
(??)            if (b->bs != BS_8x4) {
(??)                b->mode[1] = vp8_rac_get_tree(&s->c, vp9_inter_mode_tree,
(??)                                              s->prob.p.mv_mode[c]);
(??)                s->counts.mv_mode[c][b->mode[1] - 10]++;
(??)                fill_mv(s, b->mv[1], b->mode[1], 1);
(??)            } else {
(??)                b->mode[1] = b->mode[0];
(??)                AV_COPY32(&b->mv[1][0], &b->mv[0][0]);
(??)                AV_COPY32(&b->mv[1][1], &b->mv[0][1]);
(??)            }
(??)
(??)            if (b->bs != BS_4x8) {
(??)                b->mode[2] = vp8_rac_get_tree(&s->c, vp9_inter_mode_tree,
(??)                                              s->prob.p.mv_mode[c]);
(??)                s->counts.mv_mode[c][b->mode[2] - 10]++;
(??)                fill_mv(s, b->mv[2], b->mode[2], 2);
(??)
(??)                if (b->bs != BS_8x4) {
(??)                    b->mode[3] = vp8_rac_get_tree(&s->c, vp9_inter_mode_tree,
(??)                                                  s->prob.p.mv_mode[c]);
(??)                    s->counts.mv_mode[c][b->mode[3] - 10]++;
(??)                    fill_mv(s, b->mv[3], b->mode[3], 3);
(??)                } else {
(??)                    b->mode[3] = b->mode[2];
(??)                    AV_COPY32(&b->mv[3][0], &b->mv[2][0]);
(??)                    AV_COPY32(&b->mv[3][1], &b->mv[2][1]);
(??)                }
(??)            } else {
(??)                b->mode[2] = b->mode[0];
(??)                AV_COPY32(&b->mv[2][0], &b->mv[0][0]);
(??)                AV_COPY32(&b->mv[2][1], &b->mv[0][1]);
(??)                b->mode[3] = b->mode[1];
(??)                AV_COPY32(&b->mv[3][0], &b->mv[1][0]);
(??)                AV_COPY32(&b->mv[3][1], &b->mv[1][1]);
(??)            }
(??)        } else {
(??)            fill_mv(s, b->mv[0], b->mode[0], -1);
(??)            AV_COPY32(&b->mv[1][0], &b->mv[0][0]);
(??)            AV_COPY32(&b->mv[2][0], &b->mv[0][0]);
(??)            AV_COPY32(&b->mv[3][0], &b->mv[0][0]);
(??)            AV_COPY32(&b->mv[1][1], &b->mv[0][1]);
(??)            AV_COPY32(&b->mv[2][1], &b->mv[0][1]);
(??)            AV_COPY32(&b->mv[3][1], &b->mv[0][1]);
(??)        }
(??)
(??)        vref = b->ref[b->comp ? s->signbias[s->varcompref[0]] : 0];
(??)    }
(??)
(??)#if HAVE_FAST_64BIT
(??)#define SPLAT_CTX(var, val, n) \
(??)    switch (n) { \
(??)    case 1:  var = val;                                    break; \
(??)    case 2:  AV_WN16A(&var, val *             0x0101);     break; \
(??)    case 4:  AV_WN32A(&var, val *         0x01010101);     break; \
(??)    case 8:  AV_WN64A(&var, val * 0x0101010101010101ULL);  break; \
(??)    case 16: { \
(??)        uint64_t v64 = val * 0x0101010101010101ULL; \
(??)        AV_WN64A(              &var,     v64); \
(??)        AV_WN64A(&((uint8_t *) &var)[8], v64); \
(??)        break; \
(??)    } \
(??)    }
(??)#else
(??)#define SPLAT_CTX(var, val, n) \
(??)    switch (n) { \
(??)    case 1:  var = val;                         break; \
(??)    case 2:  AV_WN16A(&var, val *     0x0101);  break; \
(??)    case 4:  AV_WN32A(&var, val * 0x01010101);  break; \
(??)    case 8: { \
(??)        uint32_t v32 = val * 0x01010101; \
(??)        AV_WN32A(              &var,     v32); \
(??)        AV_WN32A(&((uint8_t *) &var)[4], v32); \
(??)        break; \
(??)    } \
(??)    case 16: { \
(??)        uint32_t v32 = val * 0x01010101; \
(??)        AV_WN32A(              &var,      v32); \
(??)        AV_WN32A(&((uint8_t *) &var)[4],  v32); \
(??)        AV_WN32A(&((uint8_t *) &var)[8],  v32); \
(??)        AV_WN32A(&((uint8_t *) &var)[12], v32); \
(??)        break; \
(??)    } \
(??)    }
(??)#endif
(??)
(??)    switch (bwh_tab[1][b->bs][0]) {
(??)#define SET_CTXS(dir, off, n) \
(??)    do { \
(??)        SPLAT_CTX(s->dir##_skip_ctx[off],      b->skip,          n); \
(??)        SPLAT_CTX(s->dir##_txfm_ctx[off],      b->tx,            n); \
(??)        SPLAT_CTX(s->dir##_partition_ctx[off], dir##_ctx[b->bs], n); \
(??)        if (!s->keyframe && !s->intraonly) { \
(??)            SPLAT_CTX(s->dir##_intra_ctx[off], b->intra,   n); \
(??)            SPLAT_CTX(s->dir##_comp_ctx[off],  b->comp,    n); \
(??)            SPLAT_CTX(s->dir##_mode_ctx[off],  b->mode[3], n); \
(??)            if (!b->intra) { \
(??)                SPLAT_CTX(s->dir##_ref_ctx[off], vref, n); \
(??)                if (s->filtermode == FILTER_SWITCHABLE) { \
(??)                    SPLAT_CTX(s->dir##_filter_ctx[off], filter_id, n); \
(??)                } \
(??)            } \
(??)        } \
(??)    } while (0)
(??)    case 1: SET_CTXS(above, col, 1); break;
(??)    case 2: SET_CTXS(above, col, 2); break;
(??)    case 4: SET_CTXS(above, col, 4); break;
(??)    case 8: SET_CTXS(above, col, 8); break;
(??)    }
(??)    switch (bwh_tab[1][b->bs][1]) {
(??)    case 1: SET_CTXS(left, row7, 1); break;
(??)    case 2: SET_CTXS(left, row7, 2); break;
(??)    case 4: SET_CTXS(left, row7, 4); break;
(??)    case 8: SET_CTXS(left, row7, 8); break;
(??)    }
(??)#undef SPLAT_CTX
(??)#undef SET_CTXS
(??)
(??)    if (!s->keyframe && !s->intraonly) {
(??)        if (b->bs > BS_8x8) {
(??)            int mv0 = AV_RN32A(&b->mv[3][0]), mv1 = AV_RN32A(&b->mv[3][1]);
(??)
(??)            AV_COPY32(&s->left_mv_ctx[row7 * 2 + 0][0], &b->mv[1][0]);
(??)            AV_COPY32(&s->left_mv_ctx[row7 * 2 + 0][1], &b->mv[1][1]);
(??)            AV_WN32A(&s->left_mv_ctx[row7 * 2 + 1][0], mv0);
(??)            AV_WN32A(&s->left_mv_ctx[row7 * 2 + 1][1], mv1);
(??)            AV_COPY32(&s->above_mv_ctx[col * 2 + 0][0], &b->mv[2][0]);
(??)            AV_COPY32(&s->above_mv_ctx[col * 2 + 0][1], &b->mv[2][1]);
(??)            AV_WN32A(&s->above_mv_ctx[col * 2 + 1][0], mv0);
(??)            AV_WN32A(&s->above_mv_ctx[col * 2 + 1][1], mv1);
(??)        } else {
(??)            int n, mv0 = AV_RN32A(&b->mv[3][0]), mv1 = AV_RN32A(&b->mv[3][1]);
(??)
(??)            for (n = 0; n < w4 * 2; n++) {
(??)                AV_WN32A(&s->above_mv_ctx[col * 2 + n][0], mv0);
(??)                AV_WN32A(&s->above_mv_ctx[col * 2 + n][1], mv1);
(??)            }
(??)            for (n = 0; n < h4 * 2; n++) {
(??)                AV_WN32A(&s->left_mv_ctx[row7 * 2 + n][0], mv0);
(??)                AV_WN32A(&s->left_mv_ctx[row7 * 2 + n][1], mv1);
(??)            }
(??)        }
(??)    }
(??)
(??)    // FIXME kinda ugly
(??)    for (y = 0; y < h4; y++) {
(??)        int x, o = (row + y) * s->sb_cols * 8 + col;
(??)        struct VP9mvrefPair *mv = &s->frames[CUR_FRAME].mv[o];
(??)
(??)        if (b->intra) {
(??)            for (x = 0; x < w4; x++) {
(??)                mv[x].ref[0] =
(??)                mv[x].ref[1] = -1;
(??)            }
(??)        } else if (b->comp) {
(??)            for (x = 0; x < w4; x++) {
(??)                mv[x].ref[0] = b->ref[0];
(??)                mv[x].ref[1] = b->ref[1];
(??)                AV_COPY32(&mv[x].mv[0], &b->mv[3][0]);
(??)                AV_COPY32(&mv[x].mv[1], &b->mv[3][1]);
(??)            }
(??)        } else {
(??)            for (x = 0; x < w4; x++) {
(??)                mv[x].ref[0] = b->ref[0];
(??)                mv[x].ref[1] = -1;
(??)                AV_COPY32(&mv[x].mv[0], &b->mv[3][0]);
(??)            }
(??)        }
(??)    }
(??)}
(??)
(??)// FIXME merge cnt/eob arguments?
(??)static av_always_inline int
(??)decode_coeffs_b_generic(VP56RangeCoder *c, int16_t *coef, int n_coeffs,
(??)                        int is_tx32x32, unsigned (*cnt)[6][3],
(??)                        unsigned (*eob)[6][2], uint8_t (*p)[6][11],
(??)                        int nnz, const int16_t *scan, const int16_t (*nb)[2],
(??)                        const int16_t *band_counts, const int16_t *qmul)
(??){
(??)    int i = 0, band = 0, band_left = band_counts[band];
(??)    uint8_t *tp = p[0][nnz];
(??)    uint8_t cache[1024];
(??)
(??)    do {
(??)        int val, rc;
(??)
(??)        val = vp56_rac_get_prob_branchy(c, tp[0]); // eob
(??)        eob[band][nnz][val]++;
(??)        if (!val)
(??)            break;
(??)
(??)    skip_eob:
(??)        if (!vp56_rac_get_prob_branchy(c, tp[1])) { // zero
(??)            cnt[band][nnz][0]++;
(??)            if (!--band_left)
(??)                band_left = band_counts[++band];
(??)            cache[scan[i]] = 0;
(??)            nnz = (1 + cache[nb[i][0]] + cache[nb[i][1]]) >> 1;
(??)            tp = p[band][nnz];
(??)            if (++i == n_coeffs)
(??)                break; //invalid input; blocks should end with EOB
(??)            goto skip_eob;
(??)        }
(??)
(??)        rc = scan[i];
(??)        if (!vp56_rac_get_prob_branchy(c, tp[2])) { // one
(??)            cnt[band][nnz][1]++;
(??)            val = 1;
(??)            cache[rc] = 1;
(??)        } else {
(??)            // fill in p[3-10] (model fill) - only once per frame for each pos
(??)            if (!tp[3])
(??)                memcpy(&tp[3], vp9_model_pareto8[tp[2]], 8);
(??)
(??)            cnt[band][nnz][2]++;
(??)            if (!vp56_rac_get_prob_branchy(c, tp[3])) { // 2, 3, 4
(??)                if (!vp56_rac_get_prob_branchy(c, tp[4])) {
(??)                    cache[rc] = val = 2;
(??)                } else {
(??)                    val = 3 + vp56_rac_get_prob(c, tp[5]);
(??)                    cache[rc] = 3;
(??)                }
(??)            } else if (!vp56_rac_get_prob_branchy(c, tp[6])) { // cat1/2
(??)                cache[rc] = 4;
(??)                if (!vp56_rac_get_prob_branchy(c, tp[7])) {
(??)                    val = 5 + vp56_rac_get_prob(c, 159);
(??)                } else {
(??)                    val  = 7 + (vp56_rac_get_prob(c, 165) << 1);
(??)                    val +=      vp56_rac_get_prob(c, 145);
(??)                }
(??)            } else { // cat 3-6
(??)                cache[rc] = 5;
(??)                if (!vp56_rac_get_prob_branchy(c, tp[8])) {
(??)                    if (!vp56_rac_get_prob_branchy(c, tp[9])) {
(??)                        val  = 11 + (vp56_rac_get_prob(c, 173) << 2);
(??)                        val +=      (vp56_rac_get_prob(c, 148) << 1);
(??)                        val +=       vp56_rac_get_prob(c, 140);
(??)                    } else {
(??)                        val  = 19 + (vp56_rac_get_prob(c, 176) << 3);
(??)                        val +=      (vp56_rac_get_prob(c, 155) << 2);
(??)                        val +=      (vp56_rac_get_prob(c, 140) << 1);
(??)                        val +=       vp56_rac_get_prob(c, 135);
(??)                    }
(??)                } else if (!vp56_rac_get_prob_branchy(c, tp[10])) {
(??)                    val  = 35 + (vp56_rac_get_prob(c, 180) << 4);
(??)                    val +=      (vp56_rac_get_prob(c, 157) << 3);
(??)                    val +=      (vp56_rac_get_prob(c, 141) << 2);
(??)                    val +=      (vp56_rac_get_prob(c, 134) << 1);
(??)                    val +=       vp56_rac_get_prob(c, 130);
(??)                } else {
(??)                    val  = 67 + (vp56_rac_get_prob(c, 254) << 13);
(??)                    val +=      (vp56_rac_get_prob(c, 254) << 12);
(??)                    val +=      (vp56_rac_get_prob(c, 254) << 11);
(??)                    val +=      (vp56_rac_get_prob(c, 252) << 10);
(??)                    val +=      (vp56_rac_get_prob(c, 249) << 9);
(??)                    val +=      (vp56_rac_get_prob(c, 243) << 8);
(??)                    val +=      (vp56_rac_get_prob(c, 230) << 7);
(??)                    val +=      (vp56_rac_get_prob(c, 196) << 6);
(??)                    val +=      (vp56_rac_get_prob(c, 177) << 5);
(??)                    val +=      (vp56_rac_get_prob(c, 153) << 4);
(??)                    val +=      (vp56_rac_get_prob(c, 140) << 3);
(??)                    val +=      (vp56_rac_get_prob(c, 133) << 2);
(??)                    val +=      (vp56_rac_get_prob(c, 130) << 1);
(??)                    val +=       vp56_rac_get_prob(c, 129);
(??)                }
(??)            }
(??)        }
(??)        if (!--band_left)
(??)            band_left = band_counts[++band];
(??)        if (is_tx32x32)
(??)            coef[rc] = ((vp8_rac_get(c) ? -val : val) * qmul[!!i]) / 2;
(??)        else
(??)            coef[rc] = (vp8_rac_get(c) ? -val : val) * qmul[!!i];
(??)        nnz = (1 + cache[nb[i][0]] + cache[nb[i][1]]) >> 1;
(??)        tp = p[band][nnz];
(??)    } while (++i < n_coeffs);
(??)
(??)    return i;
(??)}
(??)
(??)static int decode_coeffs_b(VP56RangeCoder *c, int16_t *coef, int n_coeffs,
(??)                           unsigned (*cnt)[6][3], unsigned (*eob)[6][2],
(??)                           uint8_t (*p)[6][11], int nnz, const int16_t *scan,
(??)                           const int16_t (*nb)[2], const int16_t *band_counts,
(??)                           const int16_t *qmul)
(??){
(??)    return decode_coeffs_b_generic(c, coef, n_coeffs, 0, cnt, eob, p,
(??)                                   nnz, scan, nb, band_counts, qmul);
(??)}
(??)
(??)static int decode_coeffs_b32(VP56RangeCoder *c, int16_t *coef, int n_coeffs,
(??)                             unsigned (*cnt)[6][3], unsigned (*eob)[6][2],
(??)                             uint8_t (*p)[6][11], int nnz, const int16_t *scan,
(??)                             const int16_t (*nb)[2], const int16_t *band_counts,
(??)                             const int16_t *qmul)
(??){
(??)    return decode_coeffs_b_generic(c, coef, n_coeffs, 1, cnt, eob, p,
(??)                                   nnz, scan, nb, band_counts, qmul);
(??)}
(??)
(??)static void decode_coeffs(AVCodecContext *ctx)
(??){
(??)    VP9Context *s = ctx->priv_data;
(??)    VP9Block *b = s->b;
(??)    int row = s->row, col = s->col;
(??)    uint8_t (*p)[6][11] = s->prob.coef[b->tx][0 /* y */][!b->intra];
(??)    unsigned (*c)[6][3] = s->counts.coef[b->tx][0 /* y */][!b->intra];
(??)    unsigned (*e)[6][2] = s->counts.eob[b->tx][0 /* y */][!b->intra];
(??)    int w4 = bwh_tab[1][b->bs][0] << 1, h4 = bwh_tab[1][b->bs][1] << 1;
(??)    int end_x = FFMIN(2 * (s->cols - col), w4);
(??)    int end_y = FFMIN(2 * (s->rows - row), h4);
(??)    int n, pl, x, y, res;
(??)    int16_t (*qmul)[2] = s->segmentation.feat[b->seg_id].qmul;
(??)    int tx = 4 * s->lossless + b->tx;
(??)    const int16_t * const *yscans = vp9_scans[tx];
(??)    const int16_t (* const *ynbs)[2] = vp9_scans_nb[tx];
(??)    const int16_t *uvscan = vp9_scans[b->uvtx][DCT_DCT];
(??)    const int16_t (*uvnb)[2] = vp9_scans_nb[b->uvtx][DCT_DCT];
(??)    uint8_t *a = &s->above_y_nnz_ctx[col * 2];
(??)    uint8_t *l = &s->left_y_nnz_ctx[(row & 7) << 1];
(??)    static const int16_t band_counts[4][8] = {
(??)        { 1, 2, 3, 4,  3,   16 - 13 },
(??)        { 1, 2, 3, 4, 11,   64 - 21 },
(??)        { 1, 2, 3, 4, 11,  256 - 21 },
(??)        { 1, 2, 3, 4, 11, 1024 - 21 },
(??)    };
(??)    const int16_t *y_band_counts = band_counts[b->tx];
(??)    const int16_t *uv_band_counts = band_counts[b->uvtx];
(??)
(??)#define MERGE(la, end, step, rd) \
(??)    for (n = 0; n < end; n += step) \
(??)        la[n] = !!rd(&la[n])
(??)#define MERGE_CTX(step, rd) \
(??)    do { \
(??)        MERGE(l, end_y, step, rd); \
(??)        MERGE(a, end_x, step, rd); \
(??)    } while (0)
(??)
(??)#define DECODE_Y_COEF_LOOP(step, mode_index, v) \
(??)    for (n = 0, y = 0; y < end_y; y += step) { \
(??)        for (x = 0; x < end_x; x += step, n += step * step) { \
(??)            enum TxfmType txtp = vp9_intra_txfm_type[b->mode[mode_index]]; \
(??)            res = decode_coeffs_b##v(&s->c, s->block + 16 * n, 16 * step * step, \
(??)                                     c, e, p, a[x] + l[y], yscans[txtp], \
(??)                                     ynbs[txtp], y_band_counts, qmul[0]); \
(??)            a[x] = l[y] = !!res; \
(??)            if (step >= 4) { \
(??)                AV_WN16A(&s->eob[n], res); \
(??)            } else { \
(??)                s->eob[n] = res; \
(??)            } \
(??)        } \
(??)    }
(??)
(??)#define SPLAT(la, end, step, cond) \
(??)    if (step == 2) { \
(??)        for (n = 1; n < end; n += step) \
(??)            la[n] = la[n - 1]; \
(??)    } else if (step == 4) { \
(??)        if (cond) { \
(??)            for (n = 0; n < end; n += step) \
(??)                AV_WN32A(&la[n], la[n] * 0x01010101); \
(??)        } else { \
(??)            for (n = 0; n < end; n += step) \
(??)                memset(&la[n + 1], la[n], FFMIN(end - n - 1, 3)); \
(??)        } \
(??)    } else /* step == 8 */ { \
(??)        if (cond) { \
(??)            if (HAVE_FAST_64BIT) { \
(??)                for (n = 0; n < end; n += step) \
(??)                    AV_WN64A(&la[n], la[n] * 0x0101010101010101ULL); \
(??)            } else { \
(??)                for (n = 0; n < end; n += step) { \
(??)                    uint32_t v32 = la[n] * 0x01010101; \
(??)                    AV_WN32A(&la[n],     v32); \
(??)                    AV_WN32A(&la[n + 4], v32); \
(??)                } \
(??)            } \
(??)        } else { \
(??)            for (n = 0; n < end; n += step) \
(??)                memset(&la[n + 1], la[n], FFMIN(end - n - 1, 7)); \
(??)        } \
(??)    }
(??)#define SPLAT_CTX(step) \
(??)    do { \
(??)        SPLAT(a, end_x, step, end_x == w4); \
(??)        SPLAT(l, end_y, step, end_y == h4); \
(??)    } while (0)
(??)
(??)    /* y tokens */
(??)    switch (b->tx) {
(??)    case TX_4X4:
(??)        DECODE_Y_COEF_LOOP(1, b->bs > BS_8x8 ? n : 0,);
(??)        break;
(??)    case TX_8X8:
(??)        MERGE_CTX(2, AV_RN16A);
(??)        DECODE_Y_COEF_LOOP(2, 0,);
(??)        SPLAT_CTX(2);
(??)        break;
(??)    case TX_16X16:
(??)        MERGE_CTX(4, AV_RN32A);
(??)        DECODE_Y_COEF_LOOP(4, 0,);
(??)        SPLAT_CTX(4);
(??)        break;
(??)    case TX_32X32:
(??)        MERGE_CTX(8, AV_RN64A);
(??)        DECODE_Y_COEF_LOOP(8, 0, 32);
(??)        SPLAT_CTX(8);
(??)        break;
(??)    }
(??)
(??)#define DECODE_UV_COEF_LOOP(step) \
(??)    for (n = 0, y = 0; y < end_y; y += step) { \
(??)        for (x = 0; x < end_x; x += step, n += step * step) { \
(??)            res = decode_coeffs_b(&s->c, s->uvblock[pl] + 16 * n, \
(??)                                  16 * step * step, c, e, p, a[x] + l[y], \
(??)                                  uvscan, uvnb, uv_band_counts, qmul[1]); \
(??)            a[x] = l[y] = !!res; \
(??)            if (step >= 4) { \
(??)                AV_WN16A(&s->uveob[pl][n], res); \
(??)            } else { \
(??)                s->uveob[pl][n] = res; \
(??)            } \
(??)        } \
(??)    }
(??)
(??)    p = s->prob.coef[b->uvtx][1 /* uv */][!b->intra];
(??)    c = s->counts.coef[b->uvtx][1 /* uv */][!b->intra];
(??)    e = s->counts.eob[b->uvtx][1 /* uv */][!b->intra];
(??)    w4 >>= 1;
(??)    h4 >>= 1;
(??)    end_x >>= 1;
(??)    end_y >>= 1;
(??)    for (pl = 0; pl < 2; pl++) {
(??)        a = &s->above_uv_nnz_ctx[pl][col];
(??)        l = &s->left_uv_nnz_ctx[pl][row & 7];
(??)        switch (b->uvtx) {
(??)        case TX_4X4:
(??)            DECODE_UV_COEF_LOOP(1);
(??)            break;
(??)        case TX_8X8:
(??)            MERGE_CTX(2, AV_RN16A);
(??)            DECODE_UV_COEF_LOOP(2);
(??)            SPLAT_CTX(2);
(??)            break;
(??)        case TX_16X16:
(??)            MERGE_CTX(4, AV_RN32A);
(??)            DECODE_UV_COEF_LOOP(4);
(??)            SPLAT_CTX(4);
(??)            break;
(??)        case TX_32X32:
(??)            MERGE_CTX(8, AV_RN64A);
(??)            // a 64x64 (max) uv block can ever only contain 1 tx32x32 block
(??)            // so there is no need to loop
(??)            res = decode_coeffs_b32(&s->c, s->uvblock[pl],
(??)                                    1024, c, e, p, a[0] + l[0],
(??)                                    uvscan, uvnb, uv_band_counts, qmul[1]);
(??)            a[0] = l[0] = !!res;
(??)            AV_WN16A(&s->uveob[pl][0], res);
(??)            SPLAT_CTX(8);
(??)            break;
(??)        }
(??)    }
(??)}
(??)
(??)static av_always_inline int check_intra_mode(VP9Context *s, int mode, uint8_t **a,
(??)                                             uint8_t *dst_edge, ptrdiff_t stride_edge,
(??)                                             uint8_t *dst_inner, ptrdiff_t stride_inner,
(??)                                             uint8_t *l, int col, int x, int w,
(??)                                             int row, int y, enum TxfmMode tx,
(??)                                             int p)
(??){
(??)    int have_top = row > 0 || y > 0;
(??)    int have_left = col > s->tiling.tile_col_start || x > 0;
(??)    int have_right = x < w - 1;
(??)    static const uint8_t mode_conv[10][2 /* have_left */][2 /* have_top */] = {
(??)        [VERT_PRED]            = { { DC_127_PRED,          VERT_PRED },
(??)                                   { DC_127_PRED,          VERT_PRED } },
(??)        [HOR_PRED]             = { { DC_129_PRED,          DC_129_PRED },
(??)                                   { HOR_PRED,             HOR_PRED } },
(??)        [DC_PRED]              = { { DC_128_PRED,          TOP_DC_PRED },
(??)                                   { LEFT_DC_PRED,         DC_PRED } },
(??)        [DIAG_DOWN_LEFT_PRED]  = { { DC_127_PRED,          DIAG_DOWN_LEFT_PRED },
(??)                                   { DC_127_PRED,          DIAG_DOWN_LEFT_PRED } },
(??)        [DIAG_DOWN_RIGHT_PRED] = { { DIAG_DOWN_RIGHT_PRED, DIAG_DOWN_RIGHT_PRED },
(??)                                   { DIAG_DOWN_RIGHT_PRED, DIAG_DOWN_RIGHT_PRED } },
(??)        [VERT_RIGHT_PRED]      = { { VERT_RIGHT_PRED,      VERT_RIGHT_PRED },
(??)                                   { VERT_RIGHT_PRED,      VERT_RIGHT_PRED } },
(??)        [HOR_DOWN_PRED]        = { { HOR_DOWN_PRED,        HOR_DOWN_PRED },
(??)                                   { HOR_DOWN_PRED,        HOR_DOWN_PRED } },
(??)        [VERT_LEFT_PRED]       = { { DC_127_PRED,          VERT_LEFT_PRED },
(??)                                   { DC_127_PRED,          VERT_LEFT_PRED } },
(??)        [HOR_UP_PRED]          = { { DC_129_PRED,          DC_129_PRED },
(??)                                   { HOR_UP_PRED,          HOR_UP_PRED } },
(??)        [TM_VP8_PRED]          = { { DC_129_PRED,          VERT_PRED },
(??)                                   { HOR_PRED,             TM_VP8_PRED } },
(??)    };
(??)    static const struct {
(??)        uint8_t needs_left:1;
(??)        uint8_t needs_top:1;
(??)        uint8_t needs_topleft:1;
(??)        uint8_t needs_topright:1;
(??)        uint8_t invert_left:1;
(??)    } edges[N_INTRA_PRED_MODES] = {
(??)        [VERT_PRED]            = { .needs_top  = 1 },
(??)        [HOR_PRED]             = { .needs_left = 1 },
(??)        [DC_PRED]              = { .needs_top  = 1, .needs_left = 1 },
(??)        [DIAG_DOWN_LEFT_PRED]  = { .needs_top  = 1, .needs_topright = 1 },
(??)        [DIAG_DOWN_RIGHT_PRED] = { .needs_left = 1, .needs_top = 1, .needs_topleft = 1 },
(??)        [VERT_RIGHT_PRED]      = { .needs_left = 1, .needs_top = 1, .needs_topleft = 1 },
(??)        [HOR_DOWN_PRED]        = { .needs_left = 1, .needs_top = 1, .needs_topleft = 1 },
(??)        [VERT_LEFT_PRED]       = { .needs_top  = 1, .needs_topright = 1 },
(??)        [HOR_UP_PRED]          = { .needs_left = 1, .invert_left = 1 },
(??)        [TM_VP8_PRED]          = { .needs_left = 1, .needs_top = 1, .needs_topleft = 1 },
(??)        [LEFT_DC_PRED]         = { .needs_left = 1 },
(??)        [TOP_DC_PRED]          = { .needs_top  = 1 },
(??)        [DC_128_PRED]          = { 0 },
(??)        [DC_127_PRED]          = { 0 },
(??)        [DC_129_PRED]          = { 0 }
(??)    };
(??)
(??)    av_assert2(mode >= 0 && mode < 10);
(??)    mode = mode_conv[mode][have_left][have_top];
(??)    if (edges[mode].needs_top) {
(??)        uint8_t *top, *topleft;
(??)        int n_px_need = 4 << tx, n_px_have = (((s->cols - col) << !p) - x) * 4;
(??)        int n_px_need_tr = 0;
(??)
(??)        if (tx == TX_4X4 && edges[mode].needs_topright && have_right)
(??)            n_px_need_tr = 4;
(??)
(??)        // if top of sb64-row, use s->intra_pred_data[] instead of
(??)        // dst[-stride] for intra prediction (it contains pre- instead of
(??)        // post-loopfilter data)
(??)        if (have_top) {
(??)            top = !(row & 7) && !y ?
(??)                s->intra_pred_data[p] + col * (8 >> !!p) + x * 4 :
(??)                y == 0 ? &dst_edge[-stride_edge] : &dst_inner[-stride_inner];
(??)            if (have_left)
(??)                topleft = !(row & 7) && !y ?
(??)                    s->intra_pred_data[p] + col * (8 >> !!p) + x * 4 :
(??)                    y == 0 || x == 0 ? &dst_edge[-stride_edge] :
(??)                    &dst_inner[-stride_inner];
(??)        }
(??)
(??)        if (have_top &&
(??)            (!edges[mode].needs_topleft || (have_left && top == topleft)) &&
(??)            (tx != TX_4X4 || !edges[mode].needs_topright || have_right) &&
(??)            n_px_need + n_px_need_tr <= n_px_have) {
(??)            *a = top;
(??)        } else {
(??)            if (have_top) {
(??)                if (n_px_need <= n_px_have) {
(??)                    memcpy(*a, top, n_px_need);
(??)                } else {
(??)                    memcpy(*a, top, n_px_have);
(??)                    memset(&(*a)[n_px_have], (*a)[n_px_have - 1],
(??)                           n_px_need - n_px_have);
(??)                }
(??)            } else {
(??)                memset(*a, 127, n_px_need);
(??)            }
(??)            if (edges[mode].needs_topleft) {
(??)                if (have_left && have_top) {
(??)                    (*a)[-1] = topleft[-1];
(??)                } else {
(??)                    (*a)[-1] = have_top ? 129 : 127;
(??)                }
(??)            }
(??)            if (tx == TX_4X4 && edges[mode].needs_topright) {
(??)                if (have_top && have_right &&
(??)                    n_px_need + n_px_need_tr <= n_px_have) {
(??)                    memcpy(&(*a)[4], &top[4], 4);
(??)                } else {
(??)                    memset(&(*a)[4], (*a)[3], 4);
(??)                }
(??)            }
(??)        }
(??)    }
(??)    if (edges[mode].needs_left) {
(??)        if (have_left) {
(??)            int n_px_need = 4 << tx, i, n_px_have = (((s->rows - row) << !p) - y) * 4;
(??)            uint8_t *dst = x == 0 ? dst_edge : dst_inner;
(??)            ptrdiff_t stride = x == 0 ? stride_edge : stride_inner;
(??)
(??)            if (edges[mode].invert_left) {
(??)                if (n_px_need <= n_px_have) {
(??)                    for (i = 0; i < n_px_need; i++)
(??)                        l[i] = dst[i * stride - 1];
(??)                } else {
(??)                    for (i = 0; i < n_px_have; i++)
(??)                        l[i] = dst[i * stride - 1];
(??)                    memset(&l[n_px_have], l[n_px_have - 1], n_px_need - n_px_have);
(??)                }
(??)            } else {
(??)                if (n_px_need <= n_px_have) {
(??)                    for (i = 0; i < n_px_need; i++)
(??)                        l[n_px_need - 1 - i] = dst[i * stride - 1];
(??)                } else {
(??)                    for (i = 0; i < n_px_have; i++)
(??)                        l[n_px_need - 1 - i] = dst[i * stride - 1];
(??)                    memset(l, l[n_px_need - n_px_have], n_px_need - n_px_have);
(??)                }
(??)            }
(??)        } else {
(??)            memset(l, 129, 4 << tx);
(??)        }
(??)    }
(??)
(??)    return mode;
(??)}
(??)
(??)static void intra_recon(AVCodecContext *ctx, ptrdiff_t y_off, ptrdiff_t uv_off)
(??){
(??)    VP9Context *s = ctx->priv_data;
(??)    VP9Block *b = s->b;
(??)    int row = s->row, col = s->col;
(??)    int w4 = bwh_tab[1][b->bs][0] << 1, step1d = 1 << b->tx, n;
(??)    int h4 = bwh_tab[1][b->bs][1] << 1, x, y, step = 1 << (b->tx * 2);
(??)    int end_x = FFMIN(2 * (s->cols - col), w4);
(??)    int end_y = FFMIN(2 * (s->rows - row), h4);
(??)    int tx = 4 * s->lossless + b->tx, uvtx = b->uvtx + 4 * s->lossless;
(??)    int uvstep1d = 1 << b->uvtx, p;
(??)    uint8_t *dst = s->dst[0], *dst_r = s->frames[CUR_FRAME].tf.f->data[0] + y_off;
(??)    LOCAL_ALIGNED_32(uint8_t, a_buf, [64]);
(??)    LOCAL_ALIGNED_32(uint8_t, l, [32]);
(??)
(??)    for (n = 0, y = 0; y < end_y; y += step1d) {
(??)        uint8_t *ptr = dst, *ptr_r = dst_r;
(??)        for (x = 0; x < end_x; x += step1d, ptr += 4 * step1d,
(??)                               ptr_r += 4 * step1d, n += step) {
(??)            int mode = b->mode[b->bs > BS_8x8 && b->tx == TX_4X4 ?
(??)                               y * 2 + x : 0];
(??)            uint8_t *a = &a_buf[32];
(??)            enum TxfmType txtp = vp9_intra_txfm_type[mode];
(??)            int eob = b->skip ? 0 : b->tx > TX_8X8 ? AV_RN16A(&s->eob[n]) : s->eob[n];
(??)
(??)            mode = check_intra_mode(s, mode, &a, ptr_r,
(??)                                    s->frames[CUR_FRAME].tf.f->linesize[0],
(??)                                    ptr, s->y_stride, l,
(??)                                    col, x, w4, row, y, b->tx, 0);
(??)            s->dsp.intra_pred[b->tx][mode](ptr, s->y_stride, l, a);
(??)            if (eob)
(??)                s->dsp.itxfm_add[tx][txtp](ptr, s->y_stride,
(??)                                           s->block + 16 * n, eob);
(??)        }
(??)        dst_r += 4 * step1d * s->frames[CUR_FRAME].tf.f->linesize[0];
(??)        dst   += 4 * step1d * s->y_stride;
(??)    }
(??)
(??)    // U/V
(??)    w4 >>= 1;
(??)    end_x >>= 1;
(??)    end_y >>= 1;
(??)    step = 1 << (b->uvtx * 2);
(??)    for (p = 0; p < 2; p++) {
(??)        dst   = s->dst[1 + p];
(??)        dst_r = s->frames[CUR_FRAME].tf.f->data[1 + p] + uv_off;
(??)        for (n = 0, y = 0; y < end_y; y += uvstep1d) {
(??)            uint8_t *ptr = dst, *ptr_r = dst_r;
(??)            for (x = 0; x < end_x; x += uvstep1d, ptr += 4 * uvstep1d,
(??)                                   ptr_r += 4 * uvstep1d, n += step) {
(??)                int mode = b->uvmode;
(??)                uint8_t *a = &a_buf[16];
(??)                int eob = b->skip ? 0 : b->uvtx > TX_8X8 ? AV_RN16A(&s->uveob[p][n]) : s->uveob[p][n];
(??)
(??)                mode = check_intra_mode(s, mode, &a, ptr_r,
(??)                                        s->frames[CUR_FRAME].tf.f->linesize[1],
(??)                                        ptr, s->uv_stride, l,
(??)                                        col, x, w4, row, y, b->uvtx, p + 1);
(??)                s->dsp.intra_pred[b->uvtx][mode](ptr, s->uv_stride, l, a);
(??)                if (eob)
(??)                    s->dsp.itxfm_add[uvtx][DCT_DCT](ptr, s->uv_stride,
(??)                                                    s->uvblock[p] + 16 * n, eob);
(??)            }
(??)            dst_r += 4 * uvstep1d * s->frames[CUR_FRAME].tf.f->linesize[1];
(??)            dst   += 4 * uvstep1d * s->uv_stride;
(??)        }
(??)    }
(??)}
(??)
(??)static av_always_inline void mc_luma_dir(VP9Context *s, vp9_mc_func (*mc)[2],
(??)                                         uint8_t *dst, ptrdiff_t dst_stride,
(??)                                         const uint8_t *ref, ptrdiff_t ref_stride,
(??)                                         ThreadFrame *ref_frame,
(??)                                         ptrdiff_t y, ptrdiff_t x, const VP56mv *mv,
(??)                                         int bw, int bh, int w, int h)
(??){
(??)    int mx = mv->x, my = mv->y, th;
(??)
(??)    y += my >> 3;
(??)    x += mx >> 3;
(??)    ref += y * ref_stride + x;
(??)    mx &= 7;
(??)    my &= 7;
(??)    // FIXME bilinear filter only needs 0/1 pixels, not 3/4
(??)    // we use +7 because the last 7 pixels of each sbrow can be changed in
(??)    // the longest loopfilter of the next sbrow
(??)    th = (y + bh + 4 * !!my + 7) >> 6;
(??)    ff_thread_await_progress(ref_frame, FFMAX(th, 0), 0);
(??)    if (x < !!mx * 3 || y < !!my * 3 ||
(??)        x + !!mx * 4 > w - bw || y + !!my * 4 > h - bh) {
(??)        s->vdsp.emulated_edge_mc(s->edge_emu_buffer,
(??)                                 ref - !!my * 3 * ref_stride - !!mx * 3,
(??)                                 80, ref_stride,
(??)                                 bw + !!mx * 7, bh + !!my * 7,
(??)                                 x - !!mx * 3, y - !!my * 3, w, h);
(??)        ref = s->edge_emu_buffer + !!my * 3 * 80 + !!mx * 3;
(??)        ref_stride = 80;
(??)    }
(??)    mc[!!mx][!!my](dst, dst_stride, ref, ref_stride, bh, mx << 1, my << 1);
(??)}
(??)
(??)static av_always_inline void mc_chroma_dir(VP9Context *s, vp9_mc_func (*mc)[2],
(??)                                           uint8_t *dst_u, uint8_t *dst_v,
(??)                                           ptrdiff_t dst_stride,
(??)                                           const uint8_t *ref_u, ptrdiff_t src_stride_u,
(??)                                           const uint8_t *ref_v, ptrdiff_t src_stride_v,
(??)                                           ThreadFrame *ref_frame,
(??)                                           ptrdiff_t y, ptrdiff_t x, const VP56mv *mv,
(??)                                           int bw, int bh, int w, int h)
(??){
(??)    int mx = mv->x, my = mv->y, th;
(??)
(??)    y += my >> 4;
(??)    x += mx >> 4;
(??)    ref_u += y * src_stride_u + x;
(??)    ref_v += y * src_stride_v + x;
(??)    mx &= 15;
(??)    my &= 15;
(??)    // FIXME bilinear filter only needs 0/1 pixels, not 3/4
(??)    // we use +7 because the last 7 pixels of each sbrow can be changed in
(??)    // the longest loopfilter of the next sbrow
(??)    th = (y + bh + 4 * !!my + 7) >> 5;
(??)    ff_thread_await_progress(ref_frame, FFMAX(th, 0), 0);
(??)    if (x < !!mx * 3 || y < !!my * 3 ||
(??)        x + !!mx * 4 > w - bw || y + !!my * 4 > h - bh) {
(??)        s->vdsp.emulated_edge_mc(s->edge_emu_buffer,
(??)                                 ref_u - !!my * 3 * src_stride_u - !!mx * 3,
(??)                                 80, src_stride_u,
(??)                                 bw + !!mx * 7, bh + !!my * 7,
(??)                                 x - !!mx * 3, y - !!my * 3, w, h);
(??)        ref_u = s->edge_emu_buffer + !!my * 3 * 80 + !!mx * 3;
(??)        mc[!!mx][!!my](dst_u, dst_stride, ref_u, 80, bh, mx, my);
(??)
(??)        s->vdsp.emulated_edge_mc(s->edge_emu_buffer,
(??)                                 ref_v - !!my * 3 * src_stride_v - !!mx * 3,
(??)                                 80, src_stride_v,
(??)                                 bw + !!mx * 7, bh + !!my * 7,
(??)                                 x - !!mx * 3, y - !!my * 3, w, h);
(??)        ref_v = s->edge_emu_buffer + !!my * 3 * 80 + !!mx * 3;
(??)        mc[!!mx][!!my](dst_v, dst_stride, ref_v, 80, bh, mx, my);
(??)    } else {
(??)        mc[!!mx][!!my](dst_u, dst_stride, ref_u, src_stride_u, bh, mx, my);
(??)        mc[!!mx][!!my](dst_v, dst_stride, ref_v, src_stride_v, bh, mx, my);
(??)    }
(??)}
(??)
(??)static void inter_recon(AVCodecContext *ctx)
(??){
(??)    static const uint8_t bwlog_tab[2][N_BS_SIZES] = {
(??)        { 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4 },
(??)        { 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 4 },
(??)    };
(??)    VP9Context *s = ctx->priv_data;
(??)    VP9Block *b = s->b;
(??)    int row = s->row, col = s->col;
(??)    ThreadFrame *tref1 = &s->refs[s->refidx[b->ref[0]]], *tref2;
(??)    AVFrame *ref1 = tref1->f, *ref2;
(??)    int w1 = ref1->width, h1 = ref1->height, w2, h2;
(??)    ptrdiff_t ls_y = s->y_stride, ls_uv = s->uv_stride;
(??)
(??)    if (b->comp) {
(??)        tref2 = &s->refs[s->refidx[b->ref[1]]];
(??)        ref2 = tref2->f;
(??)        w2 = ref2->width;
(??)        h2 = ref2->height;
(??)    }
(??)
(??)    // y inter pred
(??)    if (b->bs > BS_8x8) {
(??)        if (b->bs == BS_8x4) {
(??)            mc_luma_dir(s, s->dsp.mc[3][b->filter][0], s->dst[0], ls_y,
(??)                        ref1->data[0], ref1->linesize[0], tref1,
(??)                        row << 3, col << 3, &b->mv[0][0], 8, 4, w1, h1);
(??)            mc_luma_dir(s, s->dsp.mc[3][b->filter][0],
(??)                        s->dst[0] + 4 * ls_y, ls_y,
(??)                        ref1->data[0], ref1->linesize[0], tref1,
(??)                        (row << 3) + 4, col << 3, &b->mv[2][0], 8, 4, w1, h1);
(??)
(??)            if (b->comp) {
(??)                mc_luma_dir(s, s->dsp.mc[3][b->filter][1], s->dst[0], ls_y,
(??)                            ref2->data[0], ref2->linesize[0], tref2,
(??)                            row << 3, col << 3, &b->mv[0][1], 8, 4, w2, h2);
(??)                mc_luma_dir(s, s->dsp.mc[3][b->filter][1],
(??)                            s->dst[0] + 4 * ls_y, ls_y,
(??)                            ref2->data[0], ref2->linesize[0], tref2,
(??)                            (row << 3) + 4, col << 3, &b->mv[2][1], 8, 4, w2, h2);
(??)            }
(??)        } else if (b->bs == BS_4x8) {
(??)            mc_luma_dir(s, s->dsp.mc[4][b->filter][0], s->dst[0], ls_y,
(??)                        ref1->data[0], ref1->linesize[0], tref1,
(??)                        row << 3, col << 3, &b->mv[0][0], 4, 8, w1, h1);
(??)            mc_luma_dir(s, s->dsp.mc[4][b->filter][0], s->dst[0] + 4, ls_y,
(??)                        ref1->data[0], ref1->linesize[0], tref1,
(??)                        row << 3, (col << 3) + 4, &b->mv[1][0], 4, 8, w1, h1);
(??)
(??)            if (b->comp) {
(??)                mc_luma_dir(s, s->dsp.mc[4][b->filter][1], s->dst[0], ls_y,
(??)                            ref2->data[0], ref2->linesize[0], tref2,
(??)                            row << 3, col << 3, &b->mv[0][1], 4, 8, w2, h2);
(??)                mc_luma_dir(s, s->dsp.mc[4][b->filter][1], s->dst[0] + 4, ls_y,
(??)                            ref2->data[0], ref2->linesize[0], tref2,
(??)                            row << 3, (col << 3) + 4, &b->mv[1][1], 4, 8, w2, h2);
(??)            }
(??)        } else {
(??)            av_assert2(b->bs == BS_4x4);
(??)
(??)            // FIXME if two horizontally adjacent blocks have the same MV,
(??)            // do a w8 instead of a w4 call
(??)            mc_luma_dir(s, s->dsp.mc[4][b->filter][0], s->dst[0], ls_y,
(??)                        ref1->data[0], ref1->linesize[0], tref1,
(??)                        row << 3, col << 3, &b->mv[0][0], 4, 4, w1, h1);
(??)            mc_luma_dir(s, s->dsp.mc[4][b->filter][0], s->dst[0] + 4, ls_y,
(??)                        ref1->data[0], ref1->linesize[0], tref1,
(??)                        row << 3, (col << 3) + 4, &b->mv[1][0], 4, 4, w1, h1);
(??)            mc_luma_dir(s, s->dsp.mc[4][b->filter][0],
(??)                        s->dst[0] + 4 * ls_y, ls_y,
(??)                        ref1->data[0], ref1->linesize[0], tref1,
(??)                        (row << 3) + 4, col << 3, &b->mv[2][0], 4, 4, w1, h1);
(??)            mc_luma_dir(s, s->dsp.mc[4][b->filter][0],
(??)                        s->dst[0] + 4 * ls_y + 4, ls_y,
(??)                        ref1->data[0], ref1->linesize[0], tref1,
(??)                        (row << 3) + 4, (col << 3) + 4, &b->mv[3][0], 4, 4, w1, h1);
(??)
(??)            if (b->comp) {
(??)                mc_luma_dir(s, s->dsp.mc[4][b->filter][1], s->dst[0], ls_y,
(??)                            ref2->data[0], ref2->linesize[0], tref2,
(??)                            row << 3, col << 3, &b->mv[0][1], 4, 4, w2, h2);
(??)                mc_luma_dir(s, s->dsp.mc[4][b->filter][1], s->dst[0] + 4, ls_y,
(??)                            ref2->data[0], ref2->linesize[0], tref2,
(??)                            row << 3, (col << 3) + 4, &b->mv[1][1], 4, 4, w2, h2);
(??)                mc_luma_dir(s, s->dsp.mc[4][b->filter][1],
(??)                            s->dst[0] + 4 * ls_y, ls_y,
(??)                            ref2->data[0], ref2->linesize[0], tref2,
(??)                            (row << 3) + 4, col << 3, &b->mv[2][1], 4, 4, w2, h2);
(??)                mc_luma_dir(s, s->dsp.mc[4][b->filter][1],
(??)                            s->dst[0] + 4 * ls_y + 4, ls_y,
(??)                            ref2->data[0], ref2->linesize[0], tref2,
(??)                            (row << 3) + 4, (col << 3) + 4, &b->mv[3][1], 4, 4, w2, h2);
(??)            }
(??)        }
(??)    } else {
(??)        int bwl = bwlog_tab[0][b->bs];
(??)        int bw = bwh_tab[0][b->bs][0] * 4, bh = bwh_tab[0][b->bs][1] * 4;
(??)
(??)        mc_luma_dir(s, s->dsp.mc[bwl][b->filter][0], s->dst[0], ls_y,
(??)                    ref1->data[0], ref1->linesize[0], tref1,
(??)                    row << 3, col << 3, &b->mv[0][0],bw, bh, w1, h1);
(??)
(??)        if (b->comp)
(??)            mc_luma_dir(s, s->dsp.mc[bwl][b->filter][1], s->dst[0], ls_y,
(??)                        ref2->data[0], ref2->linesize[0], tref2,
(??)                        row << 3, col << 3, &b->mv[0][1], bw, bh, w2, h2);
(??)    }
(??)
(??)    // uv inter pred
(??)    {
(??)        int bwl = bwlog_tab[1][b->bs];
(??)        int bw = bwh_tab[1][b->bs][0] * 4, bh = bwh_tab[1][b->bs][1] * 4;
(??)        VP56mv mvuv;
(??)
(??)        w1 = (w1 + 1) >> 1;
(??)        h1 = (h1 + 1) >> 1;
(??)        if (b->comp) {
(??)            w2 = (w2 + 1) >> 1;
(??)            h2 = (h2 + 1) >> 1;
(??)        }
(??)        if (b->bs > BS_8x8) {
(??)            mvuv.x = ROUNDED_DIV(b->mv[0][0].x + b->mv[1][0].x + b->mv[2][0].x + b->mv[3][0].x, 4);
(??)            mvuv.y = ROUNDED_DIV(b->mv[0][0].y + b->mv[1][0].y + b->mv[2][0].y + b->mv[3][0].y, 4);
(??)        } else {
(??)            mvuv = b->mv[0][0];
(??)        }
(??)
(??)        mc_chroma_dir(s, s->dsp.mc[bwl][b->filter][0],
(??)                      s->dst[1], s->dst[2], ls_uv,
(??)                      ref1->data[1], ref1->linesize[1],
(??)                      ref1->data[2], ref1->linesize[2], tref1,
(??)                      row << 2, col << 2, &mvuv, bw, bh, w1, h1);
(??)
(??)        if (b->comp) {
(??)            if (b->bs > BS_8x8) {
(??)                mvuv.x = ROUNDED_DIV(b->mv[0][1].x + b->mv[1][1].x + b->mv[2][1].x + b->mv[3][1].x, 4);
(??)                mvuv.y = ROUNDED_DIV(b->mv[0][1].y + b->mv[1][1].y + b->mv[2][1].y + b->mv[3][1].y, 4);
(??)            } else {
(??)                mvuv = b->mv[0][1];
(??)            }
(??)            mc_chroma_dir(s, s->dsp.mc[bwl][b->filter][1],
(??)                          s->dst[1], s->dst[2], ls_uv,
(??)                          ref2->data[1], ref2->linesize[1],
(??)                          ref2->data[2], ref2->linesize[2], tref2,
(??)                          row << 2, col << 2, &mvuv, bw, bh, w2, h2);
(??)        }
(??)    }
(??)
(??)    if (!b->skip) {
(??)        /* mostly copied intra_reconn() */
(??)
(??)        int w4 = bwh_tab[1][b->bs][0] << 1, step1d = 1 << b->tx, n;
(??)        int h4 = bwh_tab[1][b->bs][1] << 1, x, y, step = 1 << (b->tx * 2);
(??)        int end_x = FFMIN(2 * (s->cols - col), w4);
(??)        int end_y = FFMIN(2 * (s->rows - row), h4);
(??)        int tx = 4 * s->lossless + b->tx, uvtx = b->uvtx + 4 * s->lossless;
(??)        int uvstep1d = 1 << b->uvtx, p;
(??)        uint8_t *dst = s->dst[0];
(??)
(??)        // y itxfm add
(??)        for (n = 0, y = 0; y < end_y; y += step1d) {
(??)            uint8_t *ptr = dst;
(??)            for (x = 0; x < end_x; x += step1d, ptr += 4 * step1d, n += step) {
(??)                int eob = b->tx > TX_8X8 ? AV_RN16A(&s->eob[n]) : s->eob[n];
(??)
(??)                if (eob)
(??)                    s->dsp.itxfm_add[tx][DCT_DCT](ptr, s->y_stride,
(??)                                                  s->block + 16 * n, eob);
(??)            }
(??)            dst += 4 * s->y_stride * step1d;
(??)        }
(??)
(??)        // uv itxfm add
(??)        end_x >>= 1;
(??)        end_y >>= 1;
(??)        step = 1 << (b->uvtx * 2);
(??)        for (p = 0; p < 2; p++) {
(??)            dst = s->dst[p + 1];
(??)            for (n = 0, y = 0; y < end_y; y += uvstep1d) {
(??)                uint8_t *ptr = dst;
(??)                for (x = 0; x < end_x; x += uvstep1d, ptr += 4 * uvstep1d, n += step) {
(??)                    int eob = b->uvtx > TX_8X8 ? AV_RN16A(&s->uveob[p][n]) : s->uveob[p][n];
(??)
(??)                    if (eob)
(??)                        s->dsp.itxfm_add[uvtx][DCT_DCT](ptr, s->uv_stride,
(??)                                                        s->uvblock[p] + 16 * n, eob);
(??)                }
(??)                dst += 4 * uvstep1d * s->uv_stride;
(??)            }
(??)        }
(??)    }
(??)}
(??)
(??)static av_always_inline void mask_edges(struct VP9Filter *lflvl, int is_uv,
(??)                                        int row_and_7, int col_and_7,
(??)                                        int w, int h, int col_end, int row_end,
(??)                                        enum TxfmMode tx, int skip_inter)
(??){
(??)    // FIXME I'm pretty sure all loops can be replaced by a single LUT if
(??)    // we make VP9Filter.mask uint64_t (i.e. row/col all single variable)
(??)    // and make the LUT 5-indexed (bl, bp, is_uv, tx and row/col), and then
(??)    // use row_and_7/col_and_7 as shifts (1*col_and_7+8*row_and_7)
(??)
(??)    // the intended behaviour of the vp9 loopfilter is to work on 8-pixel
(??)    // edges. This means that for UV, we work on two subsampled blocks at
(??)    // a time, and we only use the topleft block's mode information to set
(??)    // things like block strength. Thus, for any block size smaller than
(??)    // 16x16, ignore the odd portion of the block.
(??)    if (tx == TX_4X4 && is_uv) {
(??)        if (h == 1) {
(??)            if (row_and_7 & 1)
(??)                return;
(??)            if (!row_end)
(??)                h += 1;
(??)        }
(??)        if (w == 1) {
(??)            if (col_and_7 & 1)
(??)                return;
(??)            if (!col_end)
(??)                w += 1;
(??)        }
(??)    }
(??)
(??)    if (tx == TX_4X4 && !skip_inter) {
(??)        int t = 1 << col_and_7, m_col = (t << w) - t, y;
(??)        int m_col_odd = (t << (w - 1)) - t;
(??)
(??)        // on 32-px edges, use the 8-px wide loopfilter; else, use 4-px wide
(??)        if (is_uv) {
(??)            int m_row_8 = m_col & 0x01, m_row_4 = m_col - m_row_8;
(??)
(??)            for (y = row_and_7; y < h + row_and_7; y++) {
(??)                int col_mask_id = 2 - !(y & 7);
(??)
(??)                lflvl->mask[is_uv][0][y][1] |= m_row_8;
(??)                lflvl->mask[is_uv][0][y][2] |= m_row_4;
(??)                // for odd lines, if the odd col is not being filtered,
(??)                // skip odd row also:
(??)                // .---. <-- a
(??)                // |   |
(??)                // |___| <-- b
(??)                // ^   ^
(??)                // c   d
(??)                //
(??)                // if a/c are even row/col and b/d are odd, and d is skipped,
(??)                // e.g. right edge of size-66x66.webm, then skip b also (bug)
(??)                if ((col_end & 1) && (y & 1)) {
(??)                    lflvl->mask[is_uv][1][y][col_mask_id] |= m_col_odd;
(??)                } else {
(??)                    lflvl->mask[is_uv][1][y][col_mask_id] |= m_col;
(??)                }
(??)            }
(??)        } else {
(??)            int m_row_8 = m_col & 0x11, m_row_4 = m_col - m_row_8;
(??)
(??)            for (y = row_and_7; y < h + row_and_7; y++) {
(??)                int col_mask_id = 2 - !(y & 3);
(??)
(??)                lflvl->mask[is_uv][0][y][1] |= m_row_8; // row edge
(??)                lflvl->mask[is_uv][0][y][2] |= m_row_4;
(??)                lflvl->mask[is_uv][1][y][col_mask_id] |= m_col; // col edge
(??)                lflvl->mask[is_uv][0][y][3] |= m_col;
(??)                lflvl->mask[is_uv][1][y][3] |= m_col;
(??)            }
(??)        }
(??)    } else {
(??)        int y, t = 1 << col_and_7, m_col = (t << w) - t;
(??)
(??)        if (!skip_inter) {
(??)            int mask_id = (tx == TX_8X8);
(??)            int l2 = tx + is_uv - 1, step1d = 1 << l2;
(??)            static const unsigned masks[4] = { 0xff, 0x55, 0x11, 0x01 };
(??)            int m_row = m_col & masks[l2];
(??)
(??)            // at odd UV col/row edges tx16/tx32 loopfilter edges, force
(??)            // 8wd loopfilter to prevent going off the visible edge.
(??)            if (is_uv && tx > TX_8X8 && (w ^ (w - 1)) == 1) {
(??)                int m_row_16 = ((t << (w - 1)) - t) & masks[l2];
(??)                int m_row_8 = m_row - m_row_16;
(??)
(??)                for (y = row_and_7; y < h + row_and_7; y++) {
(??)                    lflvl->mask[is_uv][0][y][0] |= m_row_16;
(??)                    lflvl->mask[is_uv][0][y][1] |= m_row_8;
(??)                }
(??)            } else {
(??)                for (y = row_and_7; y < h + row_and_7; y++)
(??)                    lflvl->mask[is_uv][0][y][mask_id] |= m_row;
(??)            }
(??)
(??)            if (is_uv && tx > TX_8X8 && (h ^ (h - 1)) == 1) {
(??)                for (y = row_and_7; y < h + row_and_7 - 1; y += step1d)
(??)                    lflvl->mask[is_uv][1][y][0] |= m_col;
(??)                if (y - row_and_7 == h - 1)
(??)                    lflvl->mask[is_uv][1][y][1] |= m_col;
(??)            } else {
(??)                for (y = row_and_7; y < h + row_and_7; y += step1d)
(??)                    lflvl->mask[is_uv][1][y][mask_id] |= m_col;
(??)            }
(??)        } else if (tx != TX_4X4) {
(??)            int mask_id;
(??)
(??)            mask_id = (tx == TX_8X8) || (is_uv && h == 1);
(??)            lflvl->mask[is_uv][1][row_and_7][mask_id] |= m_col;
(??)            mask_id = (tx == TX_8X8) || (is_uv && w == 1);
(??)            for (y = row_and_7; y < h + row_and_7; y++)
(??)                lflvl->mask[is_uv][0][y][mask_id] |= t;
(??)        } else if (is_uv) {
(??)            int t8 = t & 0x01, t4 = t - t8;
(??)
(??)            for (y = row_and_7; y < h + row_and_7; y++) {
(??)                lflvl->mask[is_uv][0][y][2] |= t4;
(??)                lflvl->mask[is_uv][0][y][1] |= t8;
(??)            }
(??)            lflvl->mask[is_uv][1][row_and_7][2 - !(row_and_7 & 7)] |= m_col;
(??)        } else {
(??)            int t8 = t & 0x11, t4 = t - t8;
(??)
(??)            for (y = row_and_7; y < h + row_and_7; y++) {
(??)                lflvl->mask[is_uv][0][y][2] |= t4;
(??)                lflvl->mask[is_uv][0][y][1] |= t8;
(??)            }
(??)            lflvl->mask[is_uv][1][row_and_7][2 - !(row_and_7 & 3)] |= m_col;
(??)        }
(??)    }
(??)}
(??)
(??)static void decode_b(AVCodecContext *ctx, int row, int col,
(??)                     struct VP9Filter *lflvl, ptrdiff_t yoff, ptrdiff_t uvoff,
(??)                     enum BlockLevel bl, enum BlockPartition bp)
(??){
(??)    VP9Context *s = ctx->priv_data;
(??)    VP9Block *b = s->b;
(??)    enum BlockSize bs = bl * 3 + bp;
(??)    int w4 = bwh_tab[1][bs][0], h4 = bwh_tab[1][bs][1], lvl;
(??)    int emu[2];
(??)    AVFrame *f = s->frames[CUR_FRAME].tf.f;
(??)
(??)    s->row = row;
(??)    s->row7 = row & 7;
(??)    s->col = col;
(??)    s->col7 = col & 7;
(??)    s->min_mv.x = -(128 + col * 64);
(??)    s->min_mv.y = -(128 + row * 64);
(??)    s->max_mv.x = 128 + (s->cols - col - w4) * 64;
(??)    s->max_mv.y = 128 + (s->rows - row - h4) * 64;
(??)    if (s->pass < 2) {
(??)        b->bs = bs;
(??)        b->bl = bl;
(??)        b->bp = bp;
(??)        decode_mode(ctx);
(??)        b->uvtx = b->tx - (w4 * 2 == (1 << b->tx) || h4 * 2 == (1 << b->tx));
(??)
(??)        if (!b->skip) {
(??)            decode_coeffs(ctx);
(??)        } else {
(??)            int row7 = s->row7;
(??)
(??)#define SPLAT_ZERO_CTX(v, n) \
(??)    switch (n) { \
(??)    case 1:  v = 0;          break; \
(??)    case 2:  AV_ZERO16(&v);  break; \
(??)    case 4:  AV_ZERO32(&v);  break; \
(??)    case 8:  AV_ZERO64(&v);  break; \
(??)    case 16: AV_ZERO128(&v); break; \
(??)    }
(??)#define SPLAT_ZERO_YUV(dir, var, off, n) \
(??)    do { \
(??)        SPLAT_ZERO_CTX(s->dir##_y_##var[off * 2], n * 2); \
(??)        SPLAT_ZERO_CTX(s->dir##_uv_##var[0][off], n); \
(??)        SPLAT_ZERO_CTX(s->dir##_uv_##var[1][off], n); \
(??)    } while (0)
(??)
(??)            switch (w4) {
(??)            case 1: SPLAT_ZERO_YUV(above, nnz_ctx, col, 1); break;
(??)            case 2: SPLAT_ZERO_YUV(above, nnz_ctx, col, 2); break;
(??)            case 4: SPLAT_ZERO_YUV(above, nnz_ctx, col, 4); break;
(??)            case 8: SPLAT_ZERO_YUV(above, nnz_ctx, col, 8); break;
(??)            }
(??)            switch (h4) {
(??)            case 1: SPLAT_ZERO_YUV(left, nnz_ctx, row7, 1); break;
(??)            case 2: SPLAT_ZERO_YUV(left, nnz_ctx, row7, 2); break;
(??)            case 4: SPLAT_ZERO_YUV(left, nnz_ctx, row7, 4); break;
(??)            case 8: SPLAT_ZERO_YUV(left, nnz_ctx, row7, 8); break;
(??)            }
(??)        }
(??)        if (s->pass == 1) {
(??)            s->b++;
(??)            s->block += w4 * h4 * 64;
(??)            s->uvblock[0] += w4 * h4 * 16;
(??)            s->uvblock[1] += w4 * h4 * 16;
(??)            s->eob += 4 * w4 * h4;
(??)            s->uveob[0] += w4 * h4;
(??)            s->uveob[1] += w4 * h4;
(??)
(??)            return;
(??)        }
(??)    }
(??)
(??)    // emulated overhangs if the stride of the target buffer can't hold. This
(??)    // allows to support emu-edge and so on even if we have large block
(??)    // overhangs
(??)    emu[0] = (col + w4) * 8 > f->linesize[0] ||
(??)             (row + h4) > s->rows;
(??)    emu[1] = (col + w4) * 4 > f->linesize[1] ||
(??)             (row + h4) > s->rows;
(??)    if (emu[0]) {
(??)        s->dst[0] = s->tmp_y;
(??)        s->y_stride = 64;
(??)    } else {
(??)        s->dst[0] = f->data[0] + yoff;
(??)        s->y_stride = f->linesize[0];
(??)    }
(??)    if (emu[1]) {
(??)        s->dst[1] = s->tmp_uv[0];
(??)        s->dst[2] = s->tmp_uv[1];
(??)        s->uv_stride = 32;
(??)    } else {
(??)        s->dst[1] = f->data[1] + uvoff;
(??)        s->dst[2] = f->data[2] + uvoff;
(??)        s->uv_stride = f->linesize[1];
(??)    }
(??)    if (b->intra) {
(??)        intra_recon(ctx, yoff, uvoff);
(??)    } else {
(??)        inter_recon(ctx);
(??)    }
(??)    if (emu[0]) {
(??)        int w = FFMIN(s->cols - col, w4) * 8, h = FFMIN(s->rows - row, h4) * 8, n, o = 0;
(??)
(??)        for (n = 0; o < w; n++) {
(??)            int bw = 64 >> n;
(??)
(??)            av_assert2(n <= 4);
(??)            if (w & bw) {
(??)                s->dsp.mc[n][0][0][0][0](f->data[0] + yoff + o, f->linesize[0],
(??)                                         s->tmp_y + o, 64, h, 0, 0);
(??)                o += bw;
(??)            }
(??)        }
(??)    }
(??)    if (emu[1]) {
(??)        int w = FFMIN(s->cols - col, w4) * 4, h = FFMIN(s->rows - row, h4) * 4, n, o = 0;
(??)
(??)        for (n = 1; o < w; n++) {
(??)            int bw = 64 >> n;
(??)
(??)            av_assert2(n <= 4);
(??)            if (w & bw) {
(??)                s->dsp.mc[n][0][0][0][0](f->data[1] + uvoff + o, f->linesize[1],
(??)                                         s->tmp_uv[0] + o, 32, h, 0, 0);
(??)                s->dsp.mc[n][0][0][0][0](f->data[2] + uvoff + o, f->linesize[2],
(??)                                         s->tmp_uv[1] + o, 32, h, 0, 0);
(??)                o += bw;
(??)            }
(??)        }
(??)    }
(??)
(??)    // pick filter level and find edges to apply filter to
(??)    if (s->filter.level &&
(??)        (lvl = s->segmentation.feat[b->seg_id].lflvl[b->intra ? 0 : b->ref[0] + 1]
(??)                                                    [b->mode[3] != ZEROMV]) > 0) {
(??)        int x_end = FFMIN(s->cols - col, w4), y_end = FFMIN(s->rows - row, h4);
(??)        int skip_inter = !b->intra && b->skip, col7 = s->col7, row7 = s->row7;
(??)
(??)        setctx_2d(&lflvl->level[row7 * 8 + col7], w4, h4, 8, lvl);
(??)        mask_edges(lflvl, 0, row7, col7, x_end, y_end, 0, 0, b->tx, skip_inter);
(??)        mask_edges(lflvl, 1, row7, col7, x_end, y_end,
(??)                   s->cols & 1 && col + w4 >= s->cols ? s->cols & 7 : 0,
(??)                   s->rows & 1 && row + h4 >= s->rows ? s->rows & 7 : 0,
(??)                   b->uvtx, skip_inter);
(??)
(??)        if (!s->filter.lim_lut[lvl]) {
(??)            int sharp = s->filter.sharpness;
(??)            int limit = lvl;
(??)
(??)            if (sharp > 0) {
(??)                limit >>= (sharp + 3) >> 2;
(??)                limit = FFMIN(limit, 9 - sharp);
(??)            }
(??)            limit = FFMAX(limit, 1);
(??)
(??)            s->filter.lim_lut[lvl] = limit;
(??)            s->filter.mblim_lut[lvl] = 2 * (lvl + 2) + limit;
(??)        }
(??)    }
(??)
(??)    if (s->pass == 2) {
(??)        s->b++;
(??)        s->block += w4 * h4 * 64;
(??)        s->uvblock[0] += w4 * h4 * 16;
(??)        s->uvblock[1] += w4 * h4 * 16;
(??)        s->eob += 4 * w4 * h4;
(??)        s->uveob[0] += w4 * h4;
(??)        s->uveob[1] += w4 * h4;
(??)    }
(??)}
(??)
(??)static void decode_sb(AVCodecContext *ctx, int row, int col, struct VP9Filter *lflvl,
                      ptrdiff_t yoff, ptrdiff_t uvoff, enum BlockLevel bl)
{
    const VP9Context *s = td->s;
    int c = ((s->above_partition_ctx[col] >> (3 - bl)) & 1) |
            (((td->left_partition_ctx[row & 0x7] >> (3 - bl)) & 1) << 1);
    const uint8_t *p = s->s.h.keyframe || s->s.h.intraonly ? ff_vp9_default_kf_partition_probs[bl][c] :
                                                     s->prob.p.partition[bl][c];
    enum BlockPartition bp;
    ptrdiff_t hbs = 4 >> bl;
    AVFrame *f = s->s.frames[CUR_FRAME].tf.f;
    ptrdiff_t y_stride = f->linesize[0], uv_stride = f->linesize[1];
    int bytesperpixel = s->bytesperpixel;

    if (bl == BL_8X8) {
        bp = vp8_rac_get_tree(td->c, ff_vp9_partition_tree, p);
        ff_vp9_decode_block(td, row, col, lflvl, yoff, uvoff, bl, bp);
    } else if (col + hbs < s->cols) { // FIXME why not <=?
        if (row + hbs < s->rows) { // FIXME why not <=?
            bp = vp8_rac_get_tree(td->c, ff_vp9_partition_tree, p);
            switch (bp) {
            case PARTITION_NONE:
                ff_vp9_decode_block(td, row, col, lflvl, yoff, uvoff, bl, bp);
                break;
            case PARTITION_H:
                ff_vp9_decode_block(td, row, col, lflvl, yoff, uvoff, bl, bp);
                yoff  += hbs * 8 * y_stride;
                uvoff += hbs * 8 * uv_stride >> s->ss_v;
                ff_vp9_decode_block(td, row + hbs, col, lflvl, yoff, uvoff, bl, bp);
                break;
            case PARTITION_V:
                ff_vp9_decode_block(td, row, col, lflvl, yoff, uvoff, bl, bp);
                yoff  += hbs * 8 * bytesperpixel;
                uvoff += hbs * 8 * bytesperpixel >> s->ss_h;
                ff_vp9_decode_block(td, row, col + hbs, lflvl, yoff, uvoff, bl, bp);
                break;
            case PARTITION_SPLIT:
                decode_sb(td, row, col, lflvl, yoff, uvoff, bl + 1);
                decode_sb(td, row, col + hbs, lflvl,
                          yoff + 8 * hbs * bytesperpixel,
                          uvoff + (8 * hbs * bytesperpixel >> s->ss_h), bl + 1);
                yoff  += hbs * 8 * y_stride;
                uvoff += hbs * 8 * uv_stride >> s->ss_v;
                decode_sb(td, row + hbs, col, lflvl, yoff, uvoff, bl + 1);
                decode_sb(td, row + hbs, col + hbs, lflvl,
                          yoff + 8 * hbs * bytesperpixel,
                          uvoff + (8 * hbs * bytesperpixel >> s->ss_h), bl + 1);
                break;
            default:
                av_assert0(0);
            }
        } else if (vp56_rac_get_prob_branchy(td->c, p[1])) {
            bp = PARTITION_SPLIT;
            decode_sb(td, row, col, lflvl, yoff, uvoff, bl + 1);
            decode_sb(td, row, col + hbs, lflvl,
                      yoff + 8 * hbs * bytesperpixel,
                      uvoff + (8 * hbs * bytesperpixel >> s->ss_h), bl + 1);
        } else {
            bp = PARTITION_H;
            ff_vp9_decode_block(td, row, col, lflvl, yoff, uvoff, bl, bp);
        }
    } else if (row + hbs < s->rows) { // FIXME why not <=?
        if (vp56_rac_get_prob_branchy(td->c, p[2])) {
            bp = PARTITION_SPLIT;
            decode_sb(td, row, col, lflvl, yoff, uvoff, bl + 1);
            yoff  += hbs * 8 * y_stride;
            uvoff += hbs * 8 * uv_stride >> s->ss_v;
            decode_sb(td, row + hbs, col, lflvl, yoff, uvoff, bl + 1);
        } else {
            bp = PARTITION_V;
            ff_vp9_decode_block(td, row, col, lflvl, yoff, uvoff, bl, bp);
        }
    } else {
        bp = PARTITION_SPLIT;
        decode_sb(td, row, col, lflvl, yoff, uvoff, bl + 1);
    }
    td->counts.partition[bl][c][bp]++;
}

static void decode_sb_mem(VP9TileData *td, int row, int col, VP9Filter *lflvl,
                          ptrdiff_t yoff, ptrdiff_t uvoff, enum BlockLevel bl)
{
    const VP9Context *s = td->s;
    VP9Block *b = td->b;
    ptrdiff_t hbs = 4 >> bl;
    AVFrame *f = s->s.frames[CUR_FRAME].tf.f;
    ptrdiff_t y_stride = f->linesize[0], uv_stride = f->linesize[1];
    int bytesperpixel = s->bytesperpixel;

    if (bl == BL_8X8) {
        av_assert2(b->bl == BL_8X8);
        ff_vp9_decode_block(td, row, col, lflvl, yoff, uvoff, b->bl, b->bp);
    } else if (td->b->bl == bl) {
        ff_vp9_decode_block(td, row, col, lflvl, yoff, uvoff, b->bl, b->bp);
        if (b->bp == PARTITION_H && row + hbs < s->rows) {
            yoff  += hbs * 8 * y_stride;
            uvoff += hbs * 8 * uv_stride >> s->ss_v;
            ff_vp9_decode_block(td, row + hbs, col, lflvl, yoff, uvoff, b->bl, b->bp);
        } else if (b->bp == PARTITION_V && col + hbs < s->cols) {
            yoff  += hbs * 8 * bytesperpixel;
            uvoff += hbs * 8 * bytesperpixel >> s->ss_h;
            ff_vp9_decode_block(td, row, col + hbs, lflvl, yoff, uvoff, b->bl, b->bp);
        }
    } else {
        decode_sb_mem(td, row, col, lflvl, yoff, uvoff, bl + 1);
        if (col + hbs < s->cols) { // FIXME why not <=?
            if (row + hbs < s->rows) {
                decode_sb_mem(td, row, col + hbs, lflvl, yoff + 8 * hbs * bytesperpixel,
                              uvoff + (8 * hbs * bytesperpixel >> s->ss_h), bl + 1);
                yoff  += hbs * 8 * y_stride;
                uvoff += hbs * 8 * uv_stride >> s->ss_v;
                decode_sb_mem(td, row + hbs, col, lflvl, yoff, uvoff, bl + 1);
                decode_sb_mem(td, row + hbs, col + hbs, lflvl,
                              yoff + 8 * hbs * bytesperpixel,
                              uvoff + (8 * hbs * bytesperpixel >> s->ss_h), bl + 1);
            } else {
                yoff  += hbs * 8 * bytesperpixel;
                uvoff += hbs * 8 * bytesperpixel >> s->ss_h;
                decode_sb_mem(td, row, col + hbs, lflvl, yoff, uvoff, bl + 1);
            }
        } else if (row + hbs < s->rows) {
            yoff  += hbs * 8 * y_stride;
            uvoff += hbs * 8 * uv_stride >> s->ss_v;
            decode_sb_mem(td, row + hbs, col, lflvl, yoff, uvoff, bl + 1);
        }
    }
}

static void set_tile_offset(int *start, int *end, int idx, int log2_n, int n)
{
    int sb_start = ( idx      * n) >> log2_n;
    int sb_end   = ((idx + 1) * n) >> log2_n;
    *start = FFMIN(sb_start, n) << 3;
    *end   = FFMIN(sb_end,   n) << 3;
}

static void free_buffers(VP9Context *s)
{
    int i;

    av_freep(&s->intra_pred_data[0]);
    for (i = 0; i < s->active_tile_cols; i++) {
        av_freep(&s->td[i].b_base);
        av_freep(&s->td[i].block_base);
    }
}

static av_cold int vp9_decode_free(AVCodecContext *avctx)
{
    VP9Context *s = avctx->priv_data;
    int i;

    for (i = 0; i < 3; i++) {
        if (s->s.frames[i].tf.f->buf[0])
            vp9_frame_unref(avctx, &s->s.frames[i]);
        av_frame_free(&s->s.frames[i].tf.f);
    }
    for (i = 0; i < 8; i++) {
        if (s->s.refs[i].f->buf[0])
            ff_thread_release_buffer(avctx, &s->s.refs[i]);
        av_frame_free(&s->s.refs[i].f);
        if (s->next_refs[i].f->buf[0])
            ff_thread_release_buffer(avctx, &s->next_refs[i]);
        av_frame_free(&s->next_refs[i].f);
    }

    free_buffers(s);
    vp9_free_entries(avctx);
    av_freep(&s->td);
    return 0;
}

static int decode_tiles(AVCodecContext *avctx,
                        const uint8_t *data, int size)
{
    VP9Context *s = avctx->priv_data;
    VP9TileData *td = &s->td[0];
    int row, col, tile_row, tile_col, ret;
    int bytesperpixel;
    int tile_row_start, tile_row_end, tile_col_start, tile_col_end;
    AVFrame *f;
    ptrdiff_t yoff, uvoff, ls_y, ls_uv;

    f = s->s.frames[CUR_FRAME].tf.f;
    ls_y = f->linesize[0];
    ls_uv =f->linesize[1];
    bytesperpixel = s->bytesperpixel;

    yoff = uvoff = 0;
    for (tile_row = 0; tile_row < s->s.h.tiling.tile_rows; tile_row++) {
        set_tile_offset(&tile_row_start, &tile_row_end,
                        tile_row, s->s.h.tiling.log2_tile_rows, s->sb_rows);

        for (tile_col = 0; tile_col < s->s.h.tiling.tile_cols; tile_col++) {
            int64_t tile_size;

            if (tile_col == s->s.h.tiling.tile_cols - 1 &&
                tile_row == s->s.h.tiling.tile_rows - 1) {
                tile_size = size;
            } else {
                tile_size = AV_RB32(data);
                data += 4;
                size -= 4;
            }
            if (tile_size > size) {
                ff_thread_report_progress(&s->s.frames[CUR_FRAME].tf, INT_MAX, 0);
                return AVERROR_INVALIDDATA;
            }
            ret = ff_vp56_init_range_decoder(&td->c_b[tile_col], data, tile_size);
            if (ret < 0)
                return ret;
            if (vp56_rac_get_prob_branchy(&td->c_b[tile_col], 128)) { // marker bit
                ff_thread_report_progress(&s->s.frames[CUR_FRAME].tf, INT_MAX, 0);
                return AVERROR_INVALIDDATA;
            }
            data += tile_size;
            size -= tile_size;
        }

        for (row = tile_row_start; row < tile_row_end;
             row += 8, yoff += ls_y * 64, uvoff += ls_uv * 64 >> s->ss_v) {
            VP9Filter *lflvl_ptr = s->lflvl;
            ptrdiff_t yoff2 = yoff, uvoff2 = uvoff;

            for (tile_col = 0; tile_col < s->s.h.tiling.tile_cols; tile_col++) {
                set_tile_offset(&tile_col_start, &tile_col_end,
                                tile_col, s->s.h.tiling.log2_tile_cols, s->sb_cols);
                td->tile_col_start = tile_col_start;
                if (s->pass != 2) {
                    memset(td->left_partition_ctx, 0, 8);
                    memset(td->left_skip_ctx, 0, 8);
                    if (s->s.h.keyframe || s->s.h.intraonly) {
                        memset(td->left_mode_ctx, DC_PRED, 16);
                    } else {
                        memset(td->left_mode_ctx, NEARESTMV, 8);
                    }
                    memset(td->left_y_nnz_ctx, 0, 16);
                    memset(td->left_uv_nnz_ctx, 0, 32);
                    memset(td->left_segpred_ctx, 0, 8);

                    td->c = &td->c_b[tile_col];
                }

                for (col = tile_col_start;
                     col < tile_col_end;
                     col += 8, yoff2 += 64 * bytesperpixel,
                     uvoff2 += 64 * bytesperpixel >> s->ss_h, lflvl_ptr++) {
                    // FIXME integrate with lf code (i.e. zero after each
                    // use, similar to invtxfm coefficients, or similar)
                    if (s->pass != 1) {
                        memset(lflvl_ptr->mask, 0, sizeof(lflvl_ptr->mask));
                    }

                    if (s->pass == 2) {
                        decode_sb_mem(td, row, col, lflvl_ptr,
                                      yoff2, uvoff2, BL_64X64);
                    } else {
                        decode_sb(td, row, col, lflvl_ptr,
                                  yoff2, uvoff2, BL_64X64);
                    }
                }
            }

            if (s->pass == 1)
                continue;

            // backup pre-loopfilter reconstruction data for intra
            // prediction of next row of sb64s
            if (row + 8 < s->rows) {
                memcpy(s->intra_pred_data[0],
                       f->data[0] + yoff + 63 * ls_y,
                       8 * s->cols * bytesperpixel);
                memcpy(s->intra_pred_data[1],
                       f->data[1] + uvoff + ((64 >> s->ss_v) - 1) * ls_uv,
                       8 * s->cols * bytesperpixel >> s->ss_h);
                memcpy(s->intra_pred_data[2],
                       f->data[2] + uvoff + ((64 >> s->ss_v) - 1) * ls_uv,
                       8 * s->cols * bytesperpixel >> s->ss_h);
            }

            // loopfilter one row
            if (s->s.h.filter.level) {
                yoff2 = yoff;
                uvoff2 = uvoff;
                lflvl_ptr = s->lflvl;
                for (col = 0; col < s->cols;
                     col += 8, yoff2 += 64 * bytesperpixel,
                     uvoff2 += 64 * bytesperpixel >> s->ss_h, lflvl_ptr++) {
                    ff_vp9_loopfilter_sb(avctx, lflvl_ptr, row, col,
                                         yoff2, uvoff2);
                }
            }

            // FIXME maybe we can make this more finegrained by running the
            // loopfilter per-block instead of after each sbrow
            // In fact that would also make intra pred left preparation easier?
            ff_thread_report_progress(&s->s.frames[CUR_FRAME].tf, row >> 3, 0);
        }
    }
    return 0;
}

#if HAVE_THREADS
static av_always_inline
int decode_tiles_mt(AVCodecContext *avctx, void *tdata, int jobnr,
                              int threadnr)
{
    VP9Context *s = avctx->priv_data;
    VP9TileData *td = &s->td[jobnr];
    ptrdiff_t uvoff, yoff, ls_y, ls_uv;
    int bytesperpixel = s->bytesperpixel, row, col, tile_row;
    unsigned tile_cols_len;
    int tile_row_start, tile_row_end, tile_col_start, tile_col_end;
    VP9Filter *lflvl_ptr_base;
    AVFrame *f;

    f = s->s.frames[CUR_FRAME].tf.f;
    ls_y = f->linesize[0];
    ls_uv =f->linesize[1];

    set_tile_offset(&tile_col_start, &tile_col_end,
                    jobnr, s->s.h.tiling.log2_tile_cols, s->sb_cols);
    td->tile_col_start  = tile_col_start;
    uvoff = (64 * bytesperpixel >> s->ss_h)*(tile_col_start >> 3);
    yoff = (64 * bytesperpixel)*(tile_col_start >> 3);
    lflvl_ptr_base = s->lflvl+(tile_col_start >> 3);

    for (tile_row = 0; tile_row < s->s.h.tiling.tile_rows; tile_row++) {
        set_tile_offset(&tile_row_start, &tile_row_end,
                        tile_row, s->s.h.tiling.log2_tile_rows, s->sb_rows);

        td->c = &td->c_b[tile_row];
        for (row = tile_row_start; row < tile_row_end;
             row += 8, yoff += ls_y * 64, uvoff += ls_uv * 64 >> s->ss_v) {
            ptrdiff_t yoff2 = yoff, uvoff2 = uvoff;
            VP9Filter *lflvl_ptr = lflvl_ptr_base+s->sb_cols*(row >> 3);

            memset(td->left_partition_ctx, 0, 8);
            memset(td->left_skip_ctx, 0, 8);
            if (s->s.h.keyframe || s->s.h.intraonly) {
                memset(td->left_mode_ctx, DC_PRED, 16);
            } else {
                memset(td->left_mode_ctx, NEARESTMV, 8);
            }
            memset(td->left_y_nnz_ctx, 0, 16);
            memset(td->left_uv_nnz_ctx, 0, 32);
            memset(td->left_segpred_ctx, 0, 8);

            for (col = tile_col_start;
                 col < tile_col_end;
                 col += 8, yoff2 += 64 * bytesperpixel,
                 uvoff2 += 64 * bytesperpixel >> s->ss_h, lflvl_ptr++) {
                // FIXME integrate with lf code (i.e. zero after each
                // use, similar to invtxfm coefficients, or similar)
                memset(lflvl_ptr->mask, 0, sizeof(lflvl_ptr->mask));
                decode_sb(td, row, col, lflvl_ptr,
                            yoff2, uvoff2, BL_64X64);
            }

            // backup pre-loopfilter reconstruction data for intra
            // prediction of next row of sb64s
            tile_cols_len = tile_col_end - tile_col_start;
            if (row + 8 < s->rows) {
                memcpy(s->intra_pred_data[0] + (tile_col_start * 8 * bytesperpixel),
                       f->data[0] + yoff + 63 * ls_y,
                       8 * tile_cols_len * bytesperpixel);
                memcpy(s->intra_pred_data[1] + (tile_col_start * 8 * bytesperpixel >> s->ss_h),
                       f->data[1] + uvoff + ((64 >> s->ss_v) - 1) * ls_uv,
                       8 * tile_cols_len * bytesperpixel >> s->ss_h);
                memcpy(s->intra_pred_data[2] + (tile_col_start * 8 * bytesperpixel >> s->ss_h),
                       f->data[2] + uvoff + ((64 >> s->ss_v) - 1) * ls_uv,
                       8 * tile_cols_len * bytesperpixel >> s->ss_h);
            }

            vp9_report_tile_progress(s, row >> 3, 1);
        }
    }
    return 0;
}

static av_always_inline
int loopfilter_proc(AVCodecContext *avctx)
{
    VP9Context *s = avctx->priv_data;
    ptrdiff_t uvoff, yoff, ls_y, ls_uv;
    VP9Filter *lflvl_ptr;
    int bytesperpixel = s->bytesperpixel, col, i;
    AVFrame *f;

    f = s->s.frames[CUR_FRAME].tf.f;
    ls_y = f->linesize[0];
    ls_uv =f->linesize[1];

    for (i = 0; i < s->sb_rows; i++) {
        vp9_await_tile_progress(s, i, s->s.h.tiling.tile_cols);

        if (s->s.h.filter.level) {
            yoff = (ls_y * 64)*i;
            uvoff =  (ls_uv * 64 >> s->ss_v)*i;
            lflvl_ptr = s->lflvl+s->sb_cols*i;
            for (col = 0; col < s->cols;
                 col += 8, yoff += 64 * bytesperpixel,
                 uvoff += 64 * bytesperpixel >> s->ss_h, lflvl_ptr++) {
                ff_vp9_loopfilter_sb(avctx, lflvl_ptr, i << 3, col,
                                     yoff, uvoff);
            }
        }
    }
    return 0;
}
#endif

static int vp9_decode_frame(AVCodecContext *avctx, void *frame,
                            int *got_frame, AVPacket *pkt)
{
    const uint8_t *data = pkt->data;
    int size = pkt->size;
    VP9Context *s = avctx->priv_data;
    int ret, i, j, ref;
    int retain_segmap_ref = s->s.frames[REF_FRAME_SEGMAP].segmentation_map &&
                            (!s->s.h.segmentation.enabled || !s->s.h.segmentation.update_map);
    AVFrame *f;

    if ((ret = decode_frame_header(avctx, data, size, &ref)) < 0) {
        return ret;
    } else if (ret == 0) {
        if (!s->s.refs[ref].f->buf[0]) {
            av_log(avctx, AV_LOG_ERROR, "Requested reference %d not available\n", ref);
            return AVERROR_INVALIDDATA;
        }
        if ((ret = av_frame_ref(frame, s->s.refs[ref].f)) < 0)
            return ret;
        ((AVFrame *)frame)->pts = pkt->pts;
#if FF_API_PKT_PTS
FF_DISABLE_DEPRECATION_WARNINGS
        ((AVFrame *)frame)->pkt_pts = pkt->pts;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
        ((AVFrame *)frame)->pkt_dts = pkt->dts;
        for (i = 0; i < 8; i++) {
            if (s->next_refs[i].f->buf[0])
                ff_thread_release_buffer(avctx, &s->next_refs[i]);
            if (s->s.refs[i].f->buf[0] &&
                (ret = ff_thread_ref_frame(&s->next_refs[i], &s->s.refs[i])) < 0)
                return ret;
        }
        *got_frame = 1;
        return pkt->size;
    }
    data += ret;
    size -= ret;

    if (!retain_segmap_ref || s->s.h.keyframe || s->s.h.intraonly) {
        if (s->s.frames[REF_FRAME_SEGMAP].tf.f->buf[0])
            vp9_frame_unref(avctx, &s->s.frames[REF_FRAME_SEGMAP]);
        if (!s->s.h.keyframe && !s->s.h.intraonly && !s->s.h.errorres && s->s.frames[CUR_FRAME].tf.f->buf[0] &&
            (ret = vp9_frame_ref(avctx, &s->s.frames[REF_FRAME_SEGMAP], &s->s.frames[CUR_FRAME])) < 0)
            return ret;
    }
    if (s->s.frames[REF_FRAME_MVPAIR].tf.f->buf[0])
        vp9_frame_unref(avctx, &s->s.frames[REF_FRAME_MVPAIR]);
    if (!s->s.h.intraonly && !s->s.h.keyframe && !s->s.h.errorres && s->s.frames[CUR_FRAME].tf.f->buf[0] &&
        (ret = vp9_frame_ref(avctx, &s->s.frames[REF_FRAME_MVPAIR], &s->s.frames[CUR_FRAME])) < 0)
        return ret;
    if (s->s.frames[CUR_FRAME].tf.f->buf[0])
        vp9_frame_unref(avctx, &s->s.frames[CUR_FRAME]);
    if ((ret = vp9_frame_alloc(avctx, &s->s.frames[CUR_FRAME])) < 0)
        return ret;
    f = s->s.frames[CUR_FRAME].tf.f;
    f->key_frame = s->s.h.keyframe;
    f->pict_type = (s->s.h.keyframe || s->s.h.intraonly) ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

    if (s->s.frames[REF_FRAME_SEGMAP].tf.f->buf[0] &&
        (s->s.frames[REF_FRAME_MVPAIR].tf.f->width  != s->s.frames[CUR_FRAME].tf.f->width ||
         s->s.frames[REF_FRAME_MVPAIR].tf.f->height != s->s.frames[CUR_FRAME].tf.f->height)) {
        vp9_frame_unref(avctx, &s->s.frames[REF_FRAME_SEGMAP]);
    }

    // ref frame setup
    for (i = 0; i < 8; i++) {
        if (s->next_refs[i].f->buf[0])
            ff_thread_release_buffer(avctx, &s->next_refs[i]);
        if (s->s.h.refreshrefmask & (1 << i)) {
            ret = ff_thread_ref_frame(&s->next_refs[i], &s->s.frames[CUR_FRAME].tf);
        } else if (s->s.refs[i].f->buf[0]) {
            ret = ff_thread_ref_frame(&s->next_refs[i], &s->s.refs[i]);
        }
        if (ret < 0)
            return ret;
    }

    if (avctx->hwaccel) {
        ret = avctx->hwaccel->start_frame(avctx, NULL, 0);
        if (ret < 0)
            return ret;
        ret = avctx->hwaccel->decode_slice(avctx, pkt->data, pkt->size);
        if (ret < 0)
            return ret;
        ret = avctx->hwaccel->end_frame(avctx);
        if (ret < 0)
            return ret;
        goto finish;
    }

    // main tile decode loop
    memset(s->above_partition_ctx, 0, s->cols);
    memset(s->above_skip_ctx, 0, s->cols);
    if (s->s.h.keyframe || s->s.h.intraonly) {
        memset(s->above_mode_ctx, DC_PRED, s->cols * 2);
    } else {
        memset(s->above_mode_ctx, NEARESTMV, s->cols);
    }
    memset(s->above_y_nnz_ctx, 0, s->sb_cols * 16);
    memset(s->above_uv_nnz_ctx[0], 0, s->sb_cols * 16 >> s->ss_h);
    memset(s->above_uv_nnz_ctx[1], 0, s->sb_cols * 16 >> s->ss_h);
    memset(s->above_segpred_ctx, 0, s->cols);
    s->pass = s->s.frames[CUR_FRAME].uses_2pass =
        avctx->active_thread_type == FF_THREAD_FRAME && s->s.h.refreshctx && !s->s.h.parallelmode;
    if ((ret = update_block_buffers(avctx)) < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to allocate block buffers\n");
        return ret;
    }
    if (s->s.h.refreshctx && s->s.h.parallelmode) {
        int j, k, l, m;

        for (i = 0; i < 4; i++) {
            for (j = 0; j < 2; j++)
                for (k = 0; k < 2; k++)
                    for (l = 0; l < 6; l++)
                        for (m = 0; m < 6; m++)
                            memcpy(s->prob_ctx[s->s.h.framectxid].coef[i][j][k][l][m],
                                   s->prob.coef[i][j][k][l][m], 3);
            if (s->s.h.txfmmode == i)
                break;
        }
        s->prob_ctx[s->s.h.framectxid].p = s->prob.p;
        ff_thread_finish_setup(avctx);
    } else if (!s->s.h.refreshctx) {
        ff_thread_finish_setup(avctx);
    }

#if HAVE_THREADS
    if (avctx->active_thread_type & FF_THREAD_SLICE) {
        for (i = 0; i < s->sb_rows; i++)
            atomic_store(&s->entries[i], 0);
    }
#endif

    do {
        for (i = 0; i < s->active_tile_cols; i++) {
            s->td[i].b = s->td[i].b_base;
            s->td[i].block = s->td[i].block_base;
            s->td[i].uvblock[0] = s->td[i].uvblock_base[0];
            s->td[i].uvblock[1] = s->td[i].uvblock_base[1];
            s->td[i].eob = s->td[i].eob_base;
            s->td[i].uveob[0] = s->td[i].uveob_base[0];
            s->td[i].uveob[1] = s->td[i].uveob_base[1];
        }

#if HAVE_THREADS
        if (avctx->active_thread_type == FF_THREAD_SLICE) {
            int tile_row, tile_col;

            av_assert1(!s->pass);

            for (tile_row = 0; tile_row < s->s.h.tiling.tile_rows; tile_row++) {
                for (tile_col = 0; tile_col < s->s.h.tiling.tile_cols; tile_col++) {
                    int64_t tile_size;

                    if (tile_col == s->s.h.tiling.tile_cols - 1 &&
                        tile_row == s->s.h.tiling.tile_rows - 1) {
                        tile_size = size;
                    } else {
                        tile_size = AV_RB32(data);
                        data += 4;
                        size -= 4;
                    }
                    if (tile_size > size)
                        return AVERROR_INVALIDDATA;
                    ret = ff_vp56_init_range_decoder(&s->td[tile_col].c_b[tile_row], data, tile_size);
                    if (ret < 0)
                        return ret;
                    if (vp56_rac_get_prob_branchy(&s->td[tile_col].c_b[tile_row], 128)) // marker bit
                        return AVERROR_INVALIDDATA;
                    data += tile_size;
                    size -= tile_size;
                }
            }

            ff_slice_thread_execute_with_mainfunc(avctx, decode_tiles_mt, loopfilter_proc, s->td, NULL, s->s.h.tiling.tile_cols);
        } else
#endif
        {
            ret = decode_tiles(avctx, data, size);
            if (ret < 0) {
                ff_thread_report_progress(&s->s.frames[CUR_FRAME].tf, INT_MAX, 0);
                return ret;
            }
        }

        // Sum all counts fields into td[0].counts for tile threading
        if (avctx->active_thread_type == FF_THREAD_SLICE)
            for (i = 1; i < s->s.h.tiling.tile_cols; i++)
                for (j = 0; j < sizeof(s->td[i].counts) / sizeof(unsigned); j++)
                    ((unsigned *)&s->td[0].counts)[j] += ((unsigned *)&s->td[i].counts)[j];

        if (s->pass < 2 && s->s.h.refreshctx && !s->s.h.parallelmode) {
            ff_vp9_adapt_probs(s);
            ff_thread_finish_setup(avctx);
        }
    } while (s->pass++ == 1);
    ff_thread_report_progress(&s->s.frames[CUR_FRAME].tf, INT_MAX, 0);

finish:
    // ref frame setup
    for (i = 0; i < 8; i++) {
        if (s->s.refs[i].f->buf[0])
            ff_thread_release_buffer(avctx, &s->s.refs[i]);
        if (s->next_refs[i].f->buf[0] &&
            (ret = ff_thread_ref_frame(&s->s.refs[i], &s->next_refs[i])) < 0)
            return ret;
    }

    if (!s->s.h.invisible) {
        if ((ret = av_frame_ref(frame, s->s.frames[CUR_FRAME].tf.f)) < 0)
            return ret;
        *got_frame = 1;
    }

    return pkt->size;
}

static void vp9_decode_flush(AVCodecContext *avctx)
{
    VP9Context *s = avctx->priv_data;
    int i;

    for (i = 0; i < 3; i++)
        vp9_frame_unref(avctx, &s->s.frames[i]);
    for (i = 0; i < 8; i++)
        ff_thread_release_buffer(avctx, &s->s.refs[i]);
}

static int init_frames(AVCodecContext *avctx)
{
    VP9Context *s = avctx->priv_data;
    int i;

    for (i = 0; i < 3; i++) {
        s->s.frames[i].tf.f = av_frame_alloc();
        if (!s->s.frames[i].tf.f) {
            vp9_decode_free(avctx);
            av_log(avctx, AV_LOG_ERROR, "Failed to allocate frame buffer %d\n", i);
            return AVERROR(ENOMEM);
        }
    }
    for (i = 0; i < 8; i++) {
        s->s.refs[i].f = av_frame_alloc();
        s->next_refs[i].f = av_frame_alloc();
        if (!s->s.refs[i].f || !s->next_refs[i].f) {
            vp9_decode_free(avctx);
            av_log(avctx, AV_LOG_ERROR, "Failed to allocate frame buffer %d\n", i);
            return AVERROR(ENOMEM);
        }
    }

    return 0;
}

static av_cold int vp9_decode_init(AVCodecContext *avctx)
{
    VP9Context *s = avctx->priv_data;

    avctx->internal->allocate_progress = 1;
    s->last_bpp = 0;
    s->s.h.filter.sharpness = -1;

    return init_frames(avctx);
}

#if HAVE_THREADS
static av_cold int vp9_decode_init_thread_copy(AVCodecContext *avctx)
{
    return init_frames(avctx);
}

static int vp9_decode_update_thread_context(AVCodecContext *dst, const AVCodecContext *src)
{
    int i, ret;
    VP9Context *s = dst->priv_data, *ssrc = src->priv_data;

    for (i = 0; i < 3; i++) {
        if (s->s.frames[i].tf.f->buf[0])
            vp9_frame_unref(dst, &s->s.frames[i]);
        if (ssrc->s.frames[i].tf.f->buf[0]) {
            if ((ret = vp9_frame_ref(dst, &s->s.frames[i], &ssrc->s.frames[i])) < 0)
                return ret;
        }
    }
    for (i = 0; i < 8; i++) {
        if (s->s.refs[i].f->buf[0])
            ff_thread_release_buffer(dst, &s->s.refs[i]);
        if (ssrc->next_refs[i].f->buf[0]) {
            if ((ret = ff_thread_ref_frame(&s->s.refs[i], &ssrc->next_refs[i])) < 0)
                return ret;
        }
    }

    s->s.h.invisible = ssrc->s.h.invisible;
    s->s.h.keyframe = ssrc->s.h.keyframe;
    s->s.h.intraonly = ssrc->s.h.intraonly;
    s->ss_v = ssrc->ss_v;
    s->ss_h = ssrc->ss_h;
    s->s.h.segmentation.enabled = ssrc->s.h.segmentation.enabled;
    s->s.h.segmentation.update_map = ssrc->s.h.segmentation.update_map;
    s->s.h.segmentation.absolute_vals = ssrc->s.h.segmentation.absolute_vals;
    s->bytesperpixel = ssrc->bytesperpixel;
    s->gf_fmt = ssrc->gf_fmt;
    s->w = ssrc->w;
    s->h = ssrc->h;
    s->s.h.bpp = ssrc->s.h.bpp;
    s->bpp_index = ssrc->bpp_index;
    s->pix_fmt = ssrc->pix_fmt;
    memcpy(&s->prob_ctx, &ssrc->prob_ctx, sizeof(s->prob_ctx));
    memcpy(&s->s.h.lf_delta, &ssrc->s.h.lf_delta, sizeof(s->s.h.lf_delta));
    memcpy(&s->s.h.segmentation.feat, &ssrc->s.h.segmentation.feat,
           sizeof(s->s.h.segmentation.feat));

    return 0;
}
#endif

AVCodec ff_vp9_decoder = {
    .name                  = "vp9",
    .long_name             = NULL_IF_CONFIG_SMALL("Google VP9"),
    .type                  = AVMEDIA_TYPE_VIDEO,
    .id                    = AV_CODEC_ID_VP9,
    .priv_data_size        = sizeof(VP9Context),
    .init                  = vp9_decode_init,
    .close                 = vp9_decode_free,
    .decode                = vp9_decode_frame,
    .capabilities          = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_FRAME_THREADS | AV_CODEC_CAP_SLICE_THREADS,
    .caps_internal         = FF_CODEC_CAP_SLICE_THREAD_HAS_MF,
    .flush                 = vp9_decode_flush,
    .init_thread_copy      = ONLY_IF_THREADS_ENABLED(vp9_decode_init_thread_copy),
    .update_thread_context = ONLY_IF_THREADS_ENABLED(vp9_decode_update_thread_context),
    .profiles              = NULL_IF_CONFIG_SMALL(ff_vp9_profiles),
};
