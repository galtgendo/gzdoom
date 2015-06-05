/*
 * Copyright (c) 2014 Clément Bœsch
 *
 * This file is part of FFmpeg.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * @file
 * hqx magnification filters (hq2x, hq3x, hq4x)
 *
 * Originally designed by Maxim Stephin.
 *
 * @see http://en.wikipedia.org/wiki/Hqx
 * @see http://web.archive.org/web/20131114143602/http://www.hiend3d.com/hq3x.html
 */

#include <stdlib.h>
#include "hqx.h"

uint32_t rgbtoyuv[1<<24];

static inline uint32_t rgb2yuv(uint32_t c)
{
    return rgbtoyuv[c & 0xffffff];
}

static inline int yuv_diff(uint32_t yuv1, uint32_t yuv2)
{
#define YMASK 0xff0000
#define UMASK 0x00ff00
#define VMASK 0x0000ff
    return abs((yuv1 & YMASK) - (yuv2 & YMASK)) > (48 << 16) ||
           abs((yuv1 & UMASK) - (yuv2 & UMASK)) > ( 7 <<  8) ||
           abs((yuv1 & VMASK) - (yuv2 & VMASK)) > ( 6 <<  0);
}

/* (c1*w1 + c2*w2) >> s */
static inline uint32_t interp_2px(uint32_t c1, int w1, uint32_t c2, int w2, int s)
{
    return (((((c1 & 0xff000000) >> 24) * w1 + ((c2 & 0xff000000) >> 24) * w2) << (24-s)) & 0xff000000) +
           ((((c1 & 0x0000ff00) * w1 + (c2 & 0x0000ff00) * w2) >> s) & 0x0000ff00) +
           ((((c1 & 0x00ff00ff) * w1 + (c2 & 0x00ff00ff) * w2) >> s) & 0x00ff00ff);
}

/* (c1*w1 + c2*w2 + c3*w3) >> s */
static inline uint32_t interp_3px(uint32_t c1, int w1, uint32_t c2, int w2, uint32_t c3, int w3, int s)
{
    return (((((c1 & 0xff000000) >> 24) * w1 + ((c2 & 0xff000000) >> 24) * w2 + ((c3 & 0xff000000) >> 24) * w3) << (24-s)) & 0xff000000) +
           ((((c1 & 0x0000ff00) * w1 + (c2 & 0x0000ff00) * w2 + (c3 & 0x0000ff00) * w3) >>  s) & 0x0000ff00) +
           ((((c1 & 0x00ff00ff) * w1 + (c2 & 0x00ff00ff) * w2 + (c3 & 0x00ff00ff) * w3) >>  s) & 0x00ff00ff);
}

/* m is the mask of diff with the center pixel that matters in the pattern, and
 * r is the expected result (bit set to 1 if there is difference with the
 * center, 0 otherwise) */
#define P(m, r) ((k_shuffled & (m)) == (r))

/* adjust 012345678 to 01235678: the mask doesn't contain the (null) diff
 * between the center/current pixel and itself */
#define DROP4(z) ((z) > 4 ? (z)-1 : (z))

/* shuffle the input mask: move bit n (4-adjusted) to position stored in p<n> */
#define SHF(x, rot, n) (((x) >> ((rot) ? 7-DROP4(n) : DROP4(n)) & 1) << DROP4(p##n))

/* used to check if there is YUV difference between 2 pixels */
#define WDIFF(c1, c2) yuv_diff(rgb2yuv(c1), rgb2yuv(c2))

/* bootstrap template for every interpolation code. It defines the shuffled
 * masks and surrounding pixels. The rot flag is used to indicate if it's a
 * rotation; its basic effect is to shuffle k using p8..p0 instead of p0..p8 */
#define INTERP_BOOTSTRAP(rot)                                           \
    const int k_shuffled = SHF(k,rot,0) | SHF(k,rot,1) | SHF(k,rot,2)   \
                         | SHF(k,rot,3) |       0      | SHF(k,rot,5)   \
                         | SHF(k,rot,6) | SHF(k,rot,7) | SHF(k,rot,8);  \
                                                                        \
    const uint32_t w0 = w[p0], w1 = w[p1],                              \
                   w3 = w[p3], w4 = w[p4], w5 = w[p5],                  \
                               w7 = w[p7]

/* Assuming p0..p8 is mapped to pixels 0..8, this function interpolates the
 * top-left pixel in the total of the 2x2 pixels to interpolates. The function
 * is also used for the 3 other pixels */
static inline uint32_t hq2x_interp_1x1(int k, const uint32_t *w,
                                       int p0, int p1, int p2,
                                       int p3, int p4, int p5,
                                       int p6, int p7, int p8)
{
    INTERP_BOOTSTRAP(0);

    if ((P(0xbf,0x37) || P(0xdb,0x13)) && WDIFF(w1, w5))
        return interp_2px(w4, 3, w3, 1, 2);
    if ((P(0xdb,0x49) || P(0xef,0x6d)) && WDIFF(w7, w3))
        return interp_2px(w4, 3, w1, 1, 2);
    if ((P(0x0b,0x0b) || P(0xfe,0x4a) || P(0xfe,0x1a)) && WDIFF(w3, w1))
        return w4;
    if ((P(0x6f,0x2a) || P(0x5b,0x0a) || P(0xbf,0x3a) || P(0xdf,0x5a) ||
         P(0x9f,0x8a) || P(0xcf,0x8a) || P(0xef,0x4e) || P(0x3f,0x0e) ||
         P(0xfb,0x5a) || P(0xbb,0x8a) || P(0x7f,0x5a) || P(0xaf,0x8a) ||
         P(0xeb,0x8a)) && WDIFF(w3, w1))
        return interp_2px(w4, 3, w0, 1, 2);
    if (P(0x0b,0x08))
        return interp_3px(w4, 2, w0, 1, w1, 1, 2);
    if (P(0x0b,0x02))
        return interp_3px(w4, 2, w0, 1, w3, 1, 2);
    if (P(0x2f,0x2f))
        return interp_3px(w4, 14, w3, 1, w1, 1, 4);
    if (P(0xbf,0x37) || P(0xdb,0x13))
        return interp_3px(w4, 5, w1, 2, w3, 1, 3);
    if (P(0xdb,0x49) || P(0xef,0x6d))
        return interp_3px(w4, 5, w3, 2, w1, 1, 3);
    if (P(0x1b,0x03) || P(0x4f,0x43) || P(0x8b,0x83) || P(0x6b,0x43))
        return interp_2px(w4, 3, w3, 1, 2);
    if (P(0x4b,0x09) || P(0x8b,0x89) || P(0x1f,0x19) || P(0x3b,0x19))
        return interp_2px(w4, 3, w1, 1, 2);
    if (P(0x7e,0x2a) || P(0xef,0xab) || P(0xbf,0x8f) || P(0x7e,0x0e))
        return interp_3px(w4, 2, w3, 3, w1, 3, 3);
    if (P(0xfb,0x6a) || P(0x6f,0x6e) || P(0x3f,0x3e) || P(0xfb,0xfa) ||
        P(0xdf,0xde) || P(0xdf,0x1e))
        return interp_2px(w4, 3, w0, 1, 2);
    if (P(0x0a,0x00) || P(0x4f,0x4b) || P(0x9f,0x1b) || P(0x2f,0x0b) ||
        P(0xbe,0x0a) || P(0xee,0x0a) || P(0x7e,0x0a) || P(0xeb,0x4b) ||
        P(0x3b,0x1b))
        return interp_3px(w4, 2, w3, 1, w1, 1, 2);
    return interp_3px(w4, 6, w3, 1, w1, 1, 3);
}

/* Assuming p0..p8 is mapped to pixels 0..8, this function interpolates the
 * top-left and top-center pixel in the total of the 3x3 pixels to
 * interpolates. The function is also used for the 3 other couples of pixels
 * defining the outline. The center pixel is not defined through this function,
 * since it's just the same as the original value. */
static inline void hq3x_interp_2x1(uint32_t *dst, int dst_linesize, int k,
                                   const uint32_t *w, int pos00, int pos01,
                                   int p0, int p1, int p2,
                                   int p3, int p4, int p5,
                                   int p6, int p7, int p8,
                                   int rotate)
{
    INTERP_BOOTSTRAP(rotate);

    uint32_t *dst00 = &dst[dst_linesize*(pos00>>1) + (pos00&1)];
    uint32_t *dst01 = &dst[dst_linesize*(pos01>>1) + (pos01&1)];

    if ((P(0xdb,0x49) || P(0xef,0x6d)) && WDIFF(w7, w3))
        *dst00 = interp_2px(w4, 3, w1, 1, 2);
    else if ((P(0xbf,0x37) || P(0xdb,0x13)) && WDIFF(w1, w5))
        *dst00 = interp_2px(w4, 3, w3, 1, 2);
    else if ((P(0x0b,0x0b) || P(0xfe,0x4a) || P(0xfe,0x1a)) && WDIFF(w3, w1))
        *dst00 = w4;
    else if ((P(0x6f,0x2a) || P(0x5b,0x0a) || P(0xbf,0x3a) || P(0xdf,0x5a) ||
              P(0x9f,0x8a) || P(0xcf,0x8a) || P(0xef,0x4e) || P(0x3f,0x0e) ||
              P(0xfb,0x5a) || P(0xbb,0x8a) || P(0x7f,0x5a) || P(0xaf,0x8a) ||
              P(0xeb,0x8a)) && WDIFF(w3, w1))
        *dst00 = interp_2px(w4, 3, w0, 1, 2);
    else if (P(0x4b,0x09) || P(0x8b,0x89) || P(0x1f,0x19) || P(0x3b,0x19))
        *dst00 = interp_2px(w4, 3, w1, 1, 2);
    else if (P(0x1b,0x03) || P(0x4f,0x43) || P(0x8b,0x83) || P(0x6b,0x43))
        *dst00 = interp_2px(w4, 3, w3, 1, 2);
    else if (P(0x7e,0x2a) || P(0xef,0xab) || P(0xbf,0x8f) || P(0x7e,0x0e))
        *dst00 = interp_2px(w3, 1, w1, 1, 1);
    else if (P(0x4f,0x4b) || P(0x9f,0x1b) || P(0x2f,0x0b) || P(0xbe,0x0a) ||
             P(0xee,0x0a) || P(0x7e,0x0a) || P(0xeb,0x4b) || P(0x3b,0x1b))
        *dst00 = interp_3px(w4, 2, w3, 7, w1, 7, 4);
    else if (P(0x0b,0x08) || P(0xf9,0x68) || P(0xf3,0x62) || P(0x6d,0x6c) ||
             P(0x67,0x66) || P(0x3d,0x3c) || P(0x37,0x36) || P(0xf9,0xf8) ||
             P(0xdd,0xdc) || P(0xf3,0xf2) || P(0xd7,0xd6) || P(0xdd,0x1c) ||
             P(0xd7,0x16) || P(0x0b,0x02))
        *dst00 = interp_2px(w4, 3, w0, 1, 2);
    else
        *dst00 = interp_3px(w4, 2, w3, 1, w1, 1, 2);

    if ((P(0xfe,0xde) || P(0x9e,0x16) || P(0xda,0x12) || P(0x17,0x16) ||
         P(0x5b,0x12) || P(0xbb,0x12)) && WDIFF(w1, w5))
        *dst01 = w4;
    else if ((P(0x0f,0x0b) || P(0x5e,0x0a) || P(0xfb,0x7b) || P(0x3b,0x0b) ||
              P(0xbe,0x0a) || P(0x7a,0x0a)) && WDIFF(w3, w1))
        *dst01 = w4;
    else if (P(0xbf,0x8f) || P(0x7e,0x0e) || P(0xbf,0x37) || P(0xdb,0x13))
        *dst01 = interp_2px(w1, 3, w4, 1, 2);
    else if (P(0x02,0x00) || P(0x7c,0x28) || P(0xed,0xa9) || P(0xf5,0xb4) ||
             P(0xd9,0x90))
        *dst01 = interp_2px(w4, 3, w1, 1, 2);
    else if (P(0x4f,0x4b) || P(0xfb,0x7b) || P(0xfe,0x7e) || P(0x9f,0x1b) ||
             P(0x2f,0x0b) || P(0xbe,0x0a) || P(0x7e,0x0a) || P(0xfb,0x4b) ||
             P(0xfb,0xdb) || P(0xfe,0xde) || P(0xfe,0x56) || P(0x57,0x56) ||
             P(0x97,0x16) || P(0x3f,0x1e) || P(0xdb,0x12) || P(0xbb,0x12))
        *dst01 = interp_2px(w4, 7, w1, 1, 3);
    else
        *dst01 = w4;
}

/* Assuming p0..p8 is mapped to pixels 0..8, this function interpolates the
 * top-left block of 2x2 pixels in the total of the 4x4 pixels (or 4 blocks) to
 * interpolates. The function is also used for the 3 other blocks of 2x2
 * pixels. */
static inline void hq4x_interp_2x2(uint32_t *dst, int dst_linesize,
                                   int k, const uint32_t *w,
                                   int pos00, int pos01,
                                   int pos10, int pos11,
                                   int p0, int p1, int p2,
                                   int p3, int p4, int p5,
                                   int p6, int p7, int p8)
{
    INTERP_BOOTSTRAP(0);

    uint32_t *dst00 = &dst[dst_linesize*(pos00>>1) + (pos00&1)];
    uint32_t *dst01 = &dst[dst_linesize*(pos01>>1) + (pos01&1)];
    uint32_t *dst10 = &dst[dst_linesize*(pos10>>1) + (pos10&1)];
    uint32_t *dst11 = &dst[dst_linesize*(pos11>>1) + (pos11&1)];

    const int cond00 = (P(0xbf,0x37) || P(0xdb,0x13)) && WDIFF(w1, w5);
    const int cond01 = (P(0xdb,0x49) || P(0xef,0x6d)) && WDIFF(w7, w3);
    const int cond02 = (P(0x6f,0x2a) || P(0x5b,0x0a) || P(0xbf,0x3a) ||
                        P(0xdf,0x5a) || P(0x9f,0x8a) || P(0xcf,0x8a) ||
                        P(0xef,0x4e) || P(0x3f,0x0e) || P(0xfb,0x5a) ||
                        P(0xbb,0x8a) || P(0x7f,0x5a) || P(0xaf,0x8a) ||
                        P(0xeb,0x8a)) && WDIFF(w3, w1);
    const int cond03 = P(0xdb,0x49) || P(0xef,0x6d);
    const int cond04 = P(0xbf,0x37) || P(0xdb,0x13);
    const int cond05 = P(0x1b,0x03) || P(0x4f,0x43) || P(0x8b,0x83) ||
                       P(0x6b,0x43);
    const int cond06 = P(0x4b,0x09) || P(0x8b,0x89) || P(0x1f,0x19) ||
                       P(0x3b,0x19);
    const int cond07 = P(0x0b,0x08) || P(0xf9,0x68) || P(0xf3,0x62) ||
                       P(0x6d,0x6c) || P(0x67,0x66) || P(0x3d,0x3c) ||
                       P(0x37,0x36) || P(0xf9,0xf8) || P(0xdd,0xdc) ||
                       P(0xf3,0xf2) || P(0xd7,0xd6) || P(0xdd,0x1c) ||
                       P(0xd7,0x16) || P(0x0b,0x02);
    const int cond08 = (P(0x0f,0x0b) || P(0x2b,0x0b) || P(0xfe,0x4a) ||
                        P(0xfe,0x1a)) && WDIFF(w3, w1);
    const int cond09 = P(0x2f,0x2f);
    const int cond10 = P(0x0a,0x00);
    const int cond11 = P(0x0b,0x09);
    const int cond12 = P(0x7e,0x2a) || P(0xef,0xab);
    const int cond13 = P(0xbf,0x8f) || P(0x7e,0x0e);
    const int cond14 = P(0x4f,0x4b) || P(0x9f,0x1b) || P(0x2f,0x0b) ||
                       P(0xbe,0x0a) || P(0xee,0x0a) || P(0x7e,0x0a) ||
                       P(0xeb,0x4b) || P(0x3b,0x1b);
    const int cond15 = P(0x0b,0x03);

    if (cond00)
        *dst00 = interp_2px(w4, 5, w3, 3, 3);
    else if (cond01)
        *dst00 = interp_2px(w4, 5, w1, 3, 3);
    else if ((P(0x0b,0x0b) || P(0xfe,0x4a) || P(0xfe,0x1a)) && WDIFF(w3, w1))
        *dst00 = w4;
    else if (cond02)
        *dst00 = interp_2px(w4, 5, w0, 3, 3);
    else if (cond03)
        *dst00 = interp_2px(w4, 3, w3, 1, 2);
    else if (cond04)
        *dst00 = interp_2px(w4, 3, w1, 1, 2);
    else if (cond05)
        *dst00 = interp_2px(w4, 5, w3, 3, 3);
    else if (cond06)
        *dst00 = interp_2px(w4, 5, w1, 3, 3);
    else if (P(0x0f,0x0b) || P(0x5e,0x0a) || P(0x2b,0x0b) || P(0xbe,0x0a) ||
             P(0x7a,0x0a) || P(0xee,0x0a))
        *dst00 = interp_2px(w1, 1, w3, 1, 1);
    else if (cond07)
        *dst00 = interp_2px(w4, 5, w0, 3, 3);
    else
        *dst00 = interp_3px(w4, 2, w1, 1, w3, 1, 2);

    if (cond00)
        *dst01 = interp_2px(w4, 7, w3, 1, 3);
    else if (cond08)
        *dst01 = w4;
    else if (cond02)
        *dst01 = interp_2px(w4, 3, w0, 1, 2);
    else if (cond09)
        *dst01 = w4;
    else if (cond10)
        *dst01 = interp_3px(w4, 5, w1, 2, w3, 1, 3);
    else if (P(0x0b,0x08))
        *dst01 = interp_3px(w4, 5, w1, 2, w0, 1, 3);
    else if (cond11)
        *dst01 = interp_2px(w4, 5, w1, 3, 3);
    else if (cond04)
        *dst01 = interp_2px(w1, 3, w4, 1, 2);
    else if (cond12)
        *dst01 = interp_3px(w1, 2, w4, 1, w3, 1, 2);
    else if (cond13)
        *dst01 = interp_2px(w1, 5, w3, 3, 3);
    else if (cond05)
        *dst01 = interp_2px(w4, 7, w3, 1, 3);
    else if (P(0xf3,0x62) || P(0x67,0x66) || P(0x37,0x36) || P(0xf3,0xf2) ||
             P(0xd7,0xd6) || P(0xd7,0x16) || P(0x0b,0x02))
        *dst01 = interp_2px(w4, 3, w0, 1, 2);
    else if (cond14)
        *dst01 = interp_2px(w1, 1, w4, 1, 1);
    else
        *dst01 = interp_2px(w4, 3, w1, 1, 2);

    if (cond01)
        *dst10 = interp_2px(w4, 7, w1, 1, 3);
    else if (cond08)
        *dst10 = w4;
    else if (cond02)
        *dst10 = interp_2px(w4, 3, w0, 1, 2);
    else if (cond09)
        *dst10 = w4;
    else if (cond10)
        *dst10 = interp_3px(w4, 5, w3, 2, w1, 1, 3);
    else if (P(0x0b,0x02))
        *dst10 = interp_3px(w4, 5, w3, 2, w0, 1, 3);
    else if (cond15)
        *dst10 = interp_2px(w4, 5, w3, 3, 3);
    else if (cond03)
        *dst10 = interp_2px(w3, 3, w4, 1, 2);
    else if (cond13)
        *dst10 = interp_3px(w3, 2, w4, 1, w1, 1, 2);
    else if (cond12)
        *dst10 = interp_2px(w3, 5, w1, 3, 3);
    else if (cond06)
        *dst10 = interp_2px(w4, 7, w1, 1, 3);
    else if (P(0x0b,0x08) || P(0xf9,0x68) || P(0x6d,0x6c) || P(0x3d,0x3c) ||
             P(0xf9,0xf8) || P(0xdd,0xdc) || P(0xdd,0x1c))
        *dst10 = interp_2px(w4, 3, w0, 1, 2);
    else if (cond14)
        *dst10 = interp_2px(w3, 1, w4, 1, 1);
    else
        *dst10 = interp_2px(w4, 3, w3, 1, 2);

    if ((P(0x7f,0x2b) || P(0xef,0xab) || P(0xbf,0x8f) || P(0x7f,0x0f)) &&
         WDIFF(w3, w1))
        *dst11 = w4;
    else if (cond02)
        *dst11 = interp_2px(w4, 7, w0, 1, 3);
    else if (cond15)
        *dst11 = interp_2px(w4, 7, w3, 1, 3);
    else if (cond11)
        *dst11 = interp_2px(w4, 7, w1, 1, 3);
    else if (P(0x0a,0x00) || P(0x7e,0x2a) || P(0xef,0xab) || P(0xbf,0x8f) ||
             P(0x7e,0x0e))
        *dst11 = interp_3px(w4, 6, w3, 1, w1, 1, 3);
    else if (cond07)
        *dst11 = interp_2px(w4, 7, w0, 1, 3);
    else
        *dst11 = w4;
}

static void hqx_filter(unsigned *pIn, unsigned *pOut,
                       int width, int height, int n)
{
    int x, y;
    //const int slice_start = (height *  jobnr   ) / nb_jobs;
    //const int slice_end   = (height * (jobnr+1)) / nb_jobs;
    uint32_t       *dst32 = pOut;
    const uint32_t *src32 =  pIn;

    const int dst32_linesize = width * n;
    const int src32_linesize = width;

    for (y = 0; y < height; y++) {
        const int prevline = y > 0          ? -src32_linesize : 0;
        const int nextline = y < height - 1 ?  src32_linesize : 0;

        for (x = 0; x < width; x++) {
            uint32_t yuv1, yuv2;
            const int prevcol = x > 0         ? -1 : 0;
            const int nextcol = x < width - 1 ?  1 : 0;
            int pattern = 0, flag = 1, k;
            const uint32_t w[3*3] = {
                src32[prevcol + prevline], src32[prevline], src32[prevline + nextcol],
                src32[prevcol           ], src32[       0], src32[           nextcol],
                src32[prevcol + nextline], src32[nextline], src32[nextline + nextcol]
            };

            yuv1 = rgb2yuv(w[4]);

            for (k = 0; k < 9; k++) {
                if (k == 4)
                    continue;
                if (w[k] != w[4]) {
                    yuv2 = rgb2yuv(w[k]);
                    if (yuv_diff(yuv1, yuv2))
                        pattern |= flag;
                }
                flag <<= 1;
            }

            if (n == 2) {
                dst32[dst32_linesize*0 + 0] = hq2x_interp_1x1(pattern, w, 0,1,2,3,4,5,6,7,8);  // 00
                dst32[dst32_linesize*0 + 1] = hq2x_interp_1x1(pattern, w, 2,1,0,5,4,3,8,7,6);  // 01 (vert mirrored)
                dst32[dst32_linesize*1 + 0] = hq2x_interp_1x1(pattern, w, 6,7,8,3,4,5,0,1,2);  // 10 (horiz mirrored)
                dst32[dst32_linesize*1 + 1] = hq2x_interp_1x1(pattern, w, 8,7,6,5,4,3,2,1,0);  // 11 (center mirrored)
            } else if (n == 3) {
                hq3x_interp_2x1(dst32,                        dst32_linesize, pattern, w, 0,1, 0,1,2,3,4,5,6,7,8, 0);  // 00 01
                hq3x_interp_2x1(dst32 + 1,                    dst32_linesize, pattern, w, 1,3, 2,5,8,1,4,7,0,3,6, 1);  // 02 12 (rotated to the right)
                hq3x_interp_2x1(dst32 + 1*dst32_linesize,     dst32_linesize, pattern, w, 2,0, 6,3,0,7,4,1,8,5,2, 1);  // 20 10 (rotated to the left)
                hq3x_interp_2x1(dst32 + 1*dst32_linesize + 1, dst32_linesize, pattern, w, 3,2, 8,7,6,5,4,3,2,1,0, 0);  // 22 21 (center mirrored)
                dst32[dst32_linesize + 1] = w[4];                                                                           // 11
            } else if (n == 4) {
                hq4x_interp_2x2(dst32,                        dst32_linesize, pattern, w, 0,1,2,3, 0,1,2,3,4,5,6,7,8); // 00 01 10 11
                hq4x_interp_2x2(dst32 + 2,                    dst32_linesize, pattern, w, 1,0,3,2, 2,1,0,5,4,3,8,7,6); // 02 03 12 13 (vert mirrored)
                hq4x_interp_2x2(dst32 + 2*dst32_linesize,     dst32_linesize, pattern, w, 2,3,0,1, 6,7,8,3,4,5,0,1,2); // 20 21 30 31 (horiz mirrored)
                hq4x_interp_2x2(dst32 + 2*dst32_linesize + 2, dst32_linesize, pattern, w, 3,2,1,0, 8,7,6,5,4,3,2,1,0); // 22 23 32 33 (center mirrored)
            } else {
                return;
            }

            src32 += 1;
            dst32 += n;
        }
        dst32 += (n-1)*n*width;
    }
}

#define HQX_FUNC(size) \
void hq##size##x_32(unsigned *pIn, unsigned *pOut, \
                    int width, int height) \
{ \
    hqx_filter((pIn), (pOut), (width), (height), (size)); \
}

HQX_FUNC(2)
HQX_FUNC(3)
HQX_FUNC(4)

void hqxInit(void)
{
    uint32_t c;
    for (c = 0; c < (1 << 24); c++) {
        const uint32_t r = c >> 16 & 0xff;
        const uint32_t g = c >>  8 & 0xff;
        const uint32_t b = c       & 0xff;
        const uint32_t y = (uint32_t)( 0.299*r + 0.587*g + 0.114*b);
        const uint32_t u = (uint32_t)(-0.169*r - 0.331*g +   0.5*b) + 128;
        const uint32_t v = (uint32_t)(   0.5*r - 0.419*g - 0.081*b) + 128;
        rgbtoyuv[c] = (y << 16) + (u << 8) + v;
    }

}
