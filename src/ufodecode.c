
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "ufodecode.h"
#include "ufodecode-private.h"
#include "config.h"

#ifdef HAVE_SSE
#include <xmmintrin.h>
#endif

#define IPECAMERA_NUM_CHANNELS 16 /**< Number of channels per row */
#define IPECAMERA_PIXELS_PER_CHANNEL 128 /**< Number of pixels per channel */
#define IPECAMERA_WIDTH (IPECAMERA_NUM_CHANNELS * IPECAMERA_PIXELS_PER_CHANNEL) /**< Total pixel width of row */


/**
 * Check if value matches expected input.
 */
#define CHECK_VALUE(value, expected) \
    if (value != expected) { \
        fprintf(stderr, "<%s:%i> 0x%x != 0x%x\n", __FILE__, __LINE__, value, expected); \
        err = 1; \
    }

/**
 * Check that flag evaluates to non-zero.
 */
#define CHECK_FLAG(flag, check, ...) \
    if (!(check)) { \
        fprintf(stderr, "<%s:%i> Unexpected value 0x%x of " flag "\n", __FILE__, __LINE__,  __VA_ARGS__); \
        err = 1; \
    }


/**
 * \brief Setup a new decoder instance
 *
 * \param height Number of rows that are expected in the data stream
 * \param raw The data stream from the camera or NULL if set later with
 * ufo_decoder_set_raw_data.
 * \param num_bytes Size of the data stream buffer in bytes
 *
 * \return A new decoder instance that can be used to iterate over the frames
 * using ufo_decoder_get_next_frame.
 */
ufo_decoder ufo_decoder_new(uint32_t height, uint32_t width, uint32_t *raw, size_t num_bytes)
{
    if (width%IPECAMERA_PIXELS_PER_CHANNEL)
	return NULL;

    ufo_decoder decoder = malloc(sizeof(struct ufo_decoder_t));
    if (decoder == NULL)
        return NULL;

    decoder->width = width;
    decoder->height = height;
    ufo_decoder_set_raw_data(decoder, raw, num_bytes);
    return decoder;
}


/**
 * \brief Release decoder instance
 *
 * \param decoder An ufo_decoder instance
 */
void ufo_decoder_free(ufo_decoder decoder)
{
    free(decoder);
}


/**
 * \brief Set raw data stream
 *
 * \param decoder An ufo_decoder instance
 * \param raw Raw data stream
 * \param num_bytes Size of data stream buffer in bytes
 */
void ufo_decoder_set_raw_data(ufo_decoder decoder, uint32_t *raw, size_t num_bytes)
{
    decoder->raw = raw;
    decoder->num_bytes = num_bytes;
    decoder->current_pos = 0;
}

static int ufo_decode_frame(ufo_decoder decoder, uint16_t *pixel_buffer, uint16_t *cmask, uint32_t *raw, size_t num_words, int *offset)
{
    static int channel_order[IPECAMERA_NUM_CHANNELS] = { 15, 13, 14, 12, 10, 8, 11, 7, 9, 6, 5, 2, 4, 3, 0, 1 };
    static int channel_size = (2 + IPECAMERA_PIXELS_PER_CHANNEL / 3);

    int info;
    int num_rows = decoder->height;
    int num_cols = decoder->width;
    size_t c;
    const size_t cpl = (num_cols / IPECAMERA_PIXELS_PER_CHANNEL);
    const size_t cpi = num_rows * cpl;
    int row = 0;
    int channel = 0;
    int pixels = 0;
    int pos = 0;
    uint32_t data;
    const int bytes = channel_size - 1;

#ifdef HAVE_SSE
    __m128i mask = _mm_set_epi32(0x3FF, 0x3FF, 0x3FF, 0x3FF);
    __m128i packed;
    __m128i tmp1, tmp2;
    uint32_t result[4] __attribute__ ((aligned (16))) = {0};
#endif

    if (cpi * channel_size > num_words) 
	return EILSEQ;

    for (c = 0; c < cpi; c++) {
        info = raw[0];
        row = (info >> 4) & 0x7FF;
	channel = info & 0x0F;
        pixels = (info >> 20) & 0xFF;

	if ((row > num_rows)||(channel > cpl)||(pixels>IPECAMERA_PIXELS_PER_CHANNEL))
	    return EILSEQ;
	
	if (cmask) cmask[row] |= (1<<channel);

        channel = channel_order[channel];

        int base = row * IPECAMERA_WIDTH + channel * IPECAMERA_PIXELS_PER_CHANNEL;

#ifdef DEBUG
        int err = 0;
        int header = (info >> 30) & 0x03;   // 2 bits
        const int bpp = (info >> 16) & 0x0F;      // 4 bits
        CHECK_FLAG("raw header magick", header == 2, header);
        CHECK_FLAG("pixel size, only 10 bits are supported", bpp == 10, bpp);
        CHECK_FLAG("channel, limited by %i output channels", channel < IPECAMERA_NUM_CHANNELS, channel, IPECAMERA_NUM_CHANNELS);
#endif

        /* "Correct" missing pixel */
        if ((row < 2) && (pixels == (IPECAMERA_PIXELS_PER_CHANNEL - 1))) {
            pixel_buffer[base] = 0;
            /* base++; */
        }
#ifdef DEBUG
        else 
            CHECK_FLAG("number of pixels, %i is expected", pixels == IPECAMERA_PIXELS_PER_CHANNEL, pixels, IPECAMERA_PIXELS_PER_CHANNEL);
#endif

#ifdef HAVE_SSE
        for (int i = 1 ; i < bytes-4; i += 4, base += 12) {
            packed = _mm_set_epi32(raw[i], raw[i+1], raw[i+2], raw[i+3]);

            tmp1 = _mm_srli_epi32(packed, 20);
            tmp2 = _mm_and_si128(tmp1, mask);
            _mm_storeu_si128((__m128i*) result, tmp2);
            pixel_buffer[base] = result[0];
            pixel_buffer[base+3] = result[1];
            pixel_buffer[base+6] = result[2];
            pixel_buffer[base+9] = result[3];

            tmp1 = _mm_srli_epi32(packed, 10);
            tmp2 = _mm_and_si128(tmp1, mask);
            _mm_storeu_si128((__m128i*) result, tmp2);
            pixel_buffer[base+1] = result[0];
            pixel_buffer[base+4] = result[1];
            pixel_buffer[base+7] = result[2];
            pixel_buffer[base+10] = result[3];

            tmp1 = _mm_and_si128(packed, mask);
            _mm_storeu_si128((__m128i*) result, tmp1);
            pixel_buffer[base+2] = result[0];
            pixel_buffer[base+5] = result[1];
            pixel_buffer[base+8] = result[2];
            pixel_buffer[base+11] = result[3];
        }
        
        /* Compute last pixels the usual way */
        for (int i = bytes-4; i < bytes; i++) {
            data = raw[i];
            pixel_buffer[base++] = (data >> 20) & 0x3FF;
            pixel_buffer[base++] = (data >> 10) & 0x3FF;
            pixel_buffer[base++] = data & 0x3FF;
        }
#else
        for (int i = 1 ; i < bytes; i++) {
            data = raw[i];
#ifdef DEBUG
            header = (data >> 30) & 0x03;   
            CHECK_FLAG("raw data magick", header == 3, header);
            if (err) 
                return err;
#endif
            pixel_buffer[base++] = (data >> 20) & 0x3FF;
            pixel_buffer[base++] = (data >> 10) & 0x3FF;
            pixel_buffer[base++] = data & 0x3FF;
        }
#endif

        data = raw[bytes];
#ifdef DEBUG
        header = (data >> 30) & 0x03;
        CHECK_FLAG("raw data magick", header == 3, header);
        CHECK_FLAG("raw footer magick", (data & 0x3FF) == 0x55, (data & 0x3FF));
        if (err) 
            return err;
#endif

        int ppw = pixels >> 6;
        for (int j = 0; j < ppw; j++)
            pixel_buffer[base++] = (data >> (10 * (ppw - j))) & 0x3FF;

        pos += channel_size;
        raw += channel_size;
    }

    *offset = pos;
    return 0;
}


/**
 * \brief Deinterlace by interpolating between two rows
 *
 * \param in Input frame
 * \param out Destination of interpolated frame
 * \param width Width of frame in pixels
 * \param height Height of frame in pixels
 */
void ufo_deinterlace_interpolate(const uint16_t *in, uint16_t *out, int width, int height)
{
    const size_t row_size_bytes = width * sizeof(uint16_t);

    for (int row = 0; row < height; row++) {
        /* Copy one line */
        memcpy(out, in + row*width, row_size_bytes);
        out += width;

        /* Interpolate between source row and row+1 */ 
        for (int x = 0; x < width; x++) {
            out[x] = (int) (0.5 * in[row*width + x] + 0.5 * in[(row+1)*width + x]);
        }
        out += width;
    }

    /* Copy last row */
    memcpy(out, in + width * (height - 1), row_size_bytes);
}


/**
 * \brief Deinterlace by "weaving" the rows of two frames
 *
 * \param in1 First frame
 * \param in2 Second frame
 * \param out Destination of weaved frame
 * \param width Width of frame in pixels
 * \param height Height of frame in pixels
 */
void ufo_deinterlace_weave(const uint16_t *in1, const uint16_t *in2, uint16_t *out, int width, int height)
{
    const size_t row_size_bytes = width * sizeof(uint16_t);
    for (int row = 0; row < height; row++) {
        memcpy(out, in1 + row*width, row_size_bytes); 
        out += width;
        memcpy(out, in2 + row*width, row_size_bytes); 
        out += width;
    }
}


/**
 * \brief Iterate and decode next frame
 *
 * This function tries to decode the next frame in the currently set raw data
 * stream. 
 *
 * \param decoder An ufo_decoder instance
 * \param pixels If pointer with NULL content is passed, a new buffer is
 * allocated otherwise, this user-supplied buffer is used.
 * \param frame_number Frame number as reported in the header
 * \param time_stamp Time stamp of the frame as reported in the header
 * \paran cmask Change-mask
 *
 * \return 0 in case of no error, ENOSR if end of stream was reached, ENOMEM if
 * NULL was passed but no memory could be allocated, EILSEQ if data stream is
 * corrupt and EFAULT if pixels is a NULL-pointer.
 */
int ufo_decoder_get_next_frame(ufo_decoder decoder, uint16_t **pixels, uint32_t *frame_number, uint32_t *time_stamp, uint16_t *cmask)
{

    uint32_t *raw = decoder->raw;
    int err = 0;
    size_t pos = decoder->current_pos;
    int advance;
    const size_t num_words = decoder->num_bytes / 4;

    if (pixels == NULL)
        return EFAULT;

    if (pos >= num_words)
        return ENOSR; 

    if (num_words < 16)
        return EILSEQ;

    if (*pixels == NULL) {
        *pixels = (uint16_t *) malloc(IPECAMERA_WIDTH * decoder->height * sizeof(uint16_t));
        if (*pixels == NULL)
            return ENOMEM;
    }

#ifdef DEBUG
    CHECK_VALUE(raw[pos++], 0x51111111);
    CHECK_VALUE(raw[pos++], 0x52222222);
    CHECK_VALUE(raw[pos++], 0x53333333);
    CHECK_VALUE(raw[pos++], 0x54444444);
    CHECK_VALUE(raw[pos++], 0x55555555);
    CHECK_VALUE(raw[pos++], 0x56666666);
    CHECK_VALUE(raw[pos] >> 28, 0x5);
    *frame_number = raw[pos++] & 0xF0000000;
    CHECK_VALUE(raw[pos] >> 28, 0x5);
    *time_stamp = raw[pos++] & 0xF0000000;
    if (err)
        return EILSEQ;
#else
    *frame_number = raw[pos + 6] & 0xF0000000;
    *time_stamp = raw[pos + 7] & 0xF0000000;
    pos += 8;
#endif

    err = ufo_decode_frame(decoder, *pixels, cmask, raw + pos, num_words - pos - 8, &advance);
    if (err)
        return EILSEQ;

    pos += advance;

#ifdef DEBUG
    CHECK_VALUE(raw[pos++], 0x0AAAAAAA);
    CHECK_VALUE(raw[pos++], 0x0BBBBBBB);
    CHECK_VALUE(raw[pos++], 0x0CCCCCCC);
    CHECK_VALUE(raw[pos++], 0x0DDDDDDD);
    CHECK_VALUE(raw[pos++], 0x0EEEEEEE);
    CHECK_VALUE(raw[pos++], 0x0FFFFFFF);
    CHECK_VALUE(raw[pos++], 0x00000000);
    CHECK_VALUE(raw[pos++], 0x01111111);
#else
    pos += 8;
#endif

    /* if bytes left and we see fill bytes, skip them */
    if (((pos + 2) < num_words) && ((raw[pos] == 0x0) && (raw[pos+1] == 0x1111111))) {
        pos += 2;
        while ((pos < num_words) && ((raw[pos] == 0x89abcdef) || (raw[pos] == 0x1234567)))
            pos++;
    }

    decoder->current_pos = pos;
    return 0;
}

