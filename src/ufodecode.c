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

#define CHECKS

#define IPECAMERA_NUM_ROWS 1088
#define IPECAMERA_NUM_CHANNELS 16 /**< Number of channels per row */
#define IPECAMERA_PIXELS_PER_CHANNEL 128 /**< Number of pixels per channel */
#define IPECAMERA_WIDTH (IPECAMERA_NUM_CHANNELS * IPECAMERA_PIXELS_PER_CHANNEL) /**< Total pixel width of row */

typedef struct {
    unsigned int pixel_number : 8;
    unsigned int row_number : 12;
    unsigned int pixel_size : 4;
    unsigned int magic : 8;
} payload_header_v5;


/**
 * Check if value matches expected input.
 */
#ifdef DEBUG
# define CHECK_VALUE(value, expected) \
    if (value != expected) { \
        fprintf(stderr, "<%s:%i> 0x%x != 0x%x\n", __FILE__, __LINE__, value, expected); \
        err = 1; \
    }
#else
# define CHECK_VALUE(value, expected) \
    if (value != expected) { \
        err = 1; \
    }
#endif

/**
 * Check that flag evaluates to non-zero.
 */
#ifdef DEBUG
# define CHECK_FLAG(flag, check, ...) \
    if (!(check)) { \
        fprintf(stderr, "<%s:%i> Unexpected value 0x%x of " flag "\n", __FILE__, __LINE__,  __VA_ARGS__); \
        err = 1; \
    }
#else
# define CHECK_FLAG(flag, check, ...) \
    if (!(check)) { \
        err = 1; \
    }
#endif

/**
 * \brief Setup a new decoder instance
 *
 * \param height Number of rows that are expected in the data stream. Set this
 *      smaller 0 to let the decoder figure out the number of rows.
 * \param raw The data stream from the camera or NULL if set later with
 * ufo_decoder_set_raw_data.
 * \param num_bytes Size of the data stream buffer in bytes
 *
 * \return A new decoder instance that can be used to iterate over the frames
 * using ufo_decoder_get_next_frame.
 */
ufo_decoder ufo_decoder_new(int32_t height, uint32_t width, uint32_t *raw, size_t num_bytes)
{
    if (width % IPECAMERA_PIXELS_PER_CHANNEL)
        return NULL;

    ufo_decoder decoder = malloc(sizeof(struct ufo_decoder_t));

    if (decoder == NULL)
        return NULL;

    decoder->width = width;
    decoder->height = height;
    decoder->old_time_stamp = 0;
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

static int ufo_decode_frame_channels_v0(ufo_decoder decoder, 
        uint16_t *pixel_buffer, uint16_t *cmask, uint32_t *raw, 
        size_t num_words, size_t *offset)
{
    static int channel_order[IPECAMERA_NUM_CHANNELS] = { 15, 13, 14, 12, 10, 8, 11, 7, 9, 6, 5, 2, 4, 3, 0, 1 };
    static int channel_size = (2 + IPECAMERA_PIXELS_PER_CHANNEL / 3);

    const int bytes = channel_size - 1;
    const int num_rows = decoder->height;
    const size_t cpl = (decoder->width / IPECAMERA_PIXELS_PER_CHANNEL);
    const size_t cpi = num_rows * cpl;
    int pos = 0;
    uint32_t data;

#if defined(HAVE_SSE) && !defined(DEBUG)
    __m128i mask = _mm_set_epi32(0x3FF, 0x3FF, 0x3FF, 0x3FF);
    __m128i packed;
    __m128i tmp1, tmp2;
    uint32_t result[4] __attribute__ ((aligned (16))) = {0};
#endif

    if (cpi * channel_size > num_words) {
#ifdef DEBUG
        fprintf(stderr, "Not enough data to decode frame, expected %lu bytes, but received %lu\n", cpi * channel_size * sizeof(uint32_t), num_words * sizeof(uint32_t));
#endif
        return EILSEQ;
    }

    for (size_t c = 0; c < cpi; c++) {
        const int info = raw[0];
        int row = (info >> 4) & 0x7FF;
        int channel = info & 0x0F;
        int pixels = (info >> 20) & 0xFF;

#ifdef CHECKS
        int err = 0;
        int header = (info >> 30) & 0x03;
        const int bpp = (info >> 16) & 0x0F;
        CHECK_FLAG("raw header magick", header == 2, header);
        CHECK_FLAG("row number, only %i rows requested", row < num_rows, row, num_rows);
        CHECK_FLAG("pixel size, only 10 bits are supported", bpp == 10, bpp);
        CHECK_FLAG("channel, limited by %zu output channels", channel < cpl, channel, cpl);
        CHECK_FLAG("channel (line %i), duplicate entry",
                (!cmask) || (cmask[row] & (1<<channel_order[channel])) == 0,
                channel_order[channel], row);
#endif

        if ((row > num_rows) || (channel > cpl) || (pixels > IPECAMERA_PIXELS_PER_CHANNEL))
            return EILSEQ;

        channel = channel_order[channel];
        int base = row * IPECAMERA_WIDTH + channel * IPECAMERA_PIXELS_PER_CHANNEL;

        if (cmask) 
            cmask[row] |= (1 << channel);

        /* "Correct" missing pixel */
        if ((row < 2) && (pixels == (IPECAMERA_PIXELS_PER_CHANNEL - 1))) {
            pixel_buffer[base] = 0;
            /* base++; */
        }
#ifdef CHECKS
        else 
            CHECK_FLAG("number of pixels, %i is expected", pixels == IPECAMERA_PIXELS_PER_CHANNEL, pixels, IPECAMERA_PIXELS_PER_CHANNEL);
#endif

#if defined(HAVE_SSE) && !defined(DEBUG)
        for (int i = 1 ; i < bytes-4; i += 4, base += 12) {
            packed = _mm_set_epi32(raw[i], raw[i+1], raw[i+2], raw[i+3]);

            tmp1 = _mm_srli_epi32(packed, 20);
            tmp2 = _mm_and_si128(tmp1, mask);
            _mm_storeu_si128((__m128i*) result, tmp2);

            pixel_buffer[base+0] = result[3];
            pixel_buffer[base+3] = result[2];
            pixel_buffer[base+6] = result[1];
            pixel_buffer[base+9] = result[0];

            tmp1 = _mm_srli_epi32(packed, 10);
            tmp2 = _mm_and_si128(tmp1, mask);
            _mm_storeu_si128((__m128i*) result, tmp2);
            pixel_buffer[base+1] = result[3];
            pixel_buffer[base+4] = result[2];
            pixel_buffer[base+7] = result[1];
            pixel_buffer[base+10] = result[0];

            tmp1 = _mm_and_si128(packed, mask);
            _mm_storeu_si128((__m128i*) result, tmp1);
            pixel_buffer[base+2] = result[3];
            pixel_buffer[base+5] = result[2];
            pixel_buffer[base+8] = result[1];
            pixel_buffer[base+11] = result[0];
        }

        /* Compute last pixels by hand */
        data = raw[41];
        pixel_buffer[base++] = (data >> 20) & 0x3FF;
        pixel_buffer[base++] = (data >> 10) & 0x3FF;
        pixel_buffer[base++] = data & 0x3FF;

        data = raw[42];
        pixel_buffer[base++] = (data >> 20) & 0x3FF;
        pixel_buffer[base++] = (data >> 10) & 0x3FF;
        pixel_buffer[base++] = data & 0x3FF;
#else
        for (int i = 1 ; i < bytes; i++) {
            data = raw[i];
#ifdef DEBUG
            header = (data >> 30) & 0x03;   
            CHECK_FLAG("raw data magic", header == 3, header);
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
        CHECK_FLAG("raw data magic", header == 3, header);
        CHECK_FLAG("raw footer magic", (data & 0x3FF) == 0x55, (data & 0x3FF));
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

static int ufo_decode_frame_channels_v4(ufo_decoder decoder, 
        uint16_t *pixel_buffer, uint16_t *cmask, uint32_t *raw, 
        size_t num_words, size_t num_rows, size_t *offset)
{
    static const int channel_order[IPECAMERA_NUM_CHANNELS] = { 15, 13, 14, 12, 10, 8, 11, 7, 9, 6, 5, 2, 4, 3, 0, 1 };
    static const int channel_size = (2 + IPECAMERA_PIXELS_PER_CHANNEL / 3);

    const int bytes = channel_size - 1;
    const size_t channels_per_row = (decoder->width / IPECAMERA_PIXELS_PER_CHANNEL);
    const size_t cpi = num_rows * channels_per_row;
    int pos = 0;
    uint32_t data;

#if defined(HAVE_SSE) && !defined(DEBUG)
    __m128i mask = _mm_set_epi32(0x3FF, 0x3FF, 0x3FF, 0x3FF);
    __m128i packed;
    __m128i tmp1, tmp2;
    uint32_t result[4] __attribute__ ((aligned (16))) = {0};
#endif

    if (cpi * channel_size > num_words) {
#ifdef DEBUG
        fprintf(stderr, "Not enough data to decode frame, expected %lu bytes, but received %lu\n", cpi * channel_size * sizeof(uint32_t), num_words * sizeof(uint32_t));
#endif
        return EILSEQ;
    }

    for (size_t c = 0; c < cpi; c++) {
        const int info = raw[0];
        int row = (info >> 4) & 0x7FF;
        int channel = info & 0x0F;
        int pixels = (info >> 20) & 0xFF;

#ifdef DEBUG
        int err = 0;
        int header = (info >> 30) & 0x03;
        const int bpp = (info >> 16) & 0x0F;
        CHECK_FLAG("raw header magick", header == 2, header);

        /* XXX: rows are numbered absolutely so this becomes unnecessary */
        /* CHECK_FLAG("row number, only %i rows requested", row < num_rows, row, num_rows); */

        CHECK_FLAG("pixel size, only 10 bits are supported", bpp == 10, bpp);
        CHECK_FLAG("channel, limited by %zu output channels", channel < channels_per_row, channel, channels_per_row);
        CHECK_FLAG("channel (line %i), duplicate entry",
                (!cmask) || (cmask[row] & (1 << channel_order[channel])) == 0,
                channel_order[channel], row);
#endif

        if ((channel > channels_per_row) || (pixels > IPECAMERA_PIXELS_PER_CHANNEL))
            return EILSEQ;

        channel = channel_order[channel];
        int base = row * IPECAMERA_WIDTH + channel * IPECAMERA_PIXELS_PER_CHANNEL;

        if (cmask) 
            cmask[row] |= (1 << channel);

        /* "Correct" missing pixel */
        if ((row < 2) && (pixels == (IPECAMERA_PIXELS_PER_CHANNEL - 1))) {
            pixel_buffer[base] = 0;
            /* base++; */
        }
/* #ifdef DEBUG */
/*         else */ 
/*             CHECK_FLAG("number of pixels, %i is expected", pixels == IPECAMERA_PIXELS_PER_CHANNEL, pixels, IPECAMERA_PIXELS_PER_CHANNEL); */
/* #endif */

#if defined(HAVE_SSE) && !defined(DEBUG)
        for (int i = 1 ; i < bytes-4; i += 4, base += 12) {
            packed = _mm_set_epi32(raw[i], raw[i+1], raw[i+2], raw[i+3]);

            tmp1 = _mm_srli_epi32(packed, 20);
            tmp2 = _mm_and_si128(tmp1, mask);
            _mm_storeu_si128((__m128i*) result, tmp2);

            pixel_buffer[base+0] = result[3];
            pixel_buffer[base+3] = result[2];
            pixel_buffer[base+6] = result[1];
            pixel_buffer[base+9] = result[0];

            tmp1 = _mm_srli_epi32(packed, 10);
            tmp2 = _mm_and_si128(tmp1, mask);
            _mm_storeu_si128((__m128i*) result, tmp2);
            pixel_buffer[base+1] = result[3];
            pixel_buffer[base+4] = result[2];
            pixel_buffer[base+7] = result[1];
            pixel_buffer[base+10] = result[0];

            tmp1 = _mm_and_si128(packed, mask);
            _mm_storeu_si128((__m128i*) result, tmp1);
            pixel_buffer[base+2] = result[3];
            pixel_buffer[base+5] = result[2];
            pixel_buffer[base+8] = result[1];
            pixel_buffer[base+11] = result[0];
        }

        /* Compute last pixels by hand */
        data = raw[41];
        pixel_buffer[base++] = (data >> 20) & 0x3FF;
        pixel_buffer[base++] = (data >> 10) & 0x3FF;
        pixel_buffer[base++] = data & 0x3FF;

        data = raw[42];
        pixel_buffer[base++] = (data >> 20) & 0x3FF;
        pixel_buffer[base++] = (data >> 10) & 0x3FF;
        pixel_buffer[base++] = data & 0x3FF;
#else
        for (int i = 1 ; i < bytes; i++) {
            data = raw[i];
#ifdef DEBUG
            header = (data >> 30) & 0x03;   
            CHECK_FLAG("raw data magic", header == 3, header);
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
        CHECK_FLAG("raw data magic", header == 3, header);
        CHECK_FLAG("raw footer magic", (data & 0x3FF) == 0x55, (data & 0x3FF));
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

static int ufo_decode_frame_channels_v5(ufo_decoder decoder, 
        uint16_t *pixel_buffer, uint16_t *cmask, uint32_t *raw, 
        size_t num_words, size_t num_rows, size_t *offset)
{
    size_t base = 0, index = 0;

    for (int row = 0; row < num_rows; row++) {
        for (int pix = 0; pix < 128; pix++) {
            payload_header_v5 *header = (payload_header_v5 *) &raw[base];

            if (header->row_number > num_rows) {
                fprintf(stderr, "Error: row_number in header is %i instead of %i\n", 
                        header->row_number, row); 
                return 1;
            }

            if (header->pixel_number > 128) {
                fprintf(stderr, "Error: pixel_number in header is %i instead of %i\n", 
                        header->pixel_number, pix); 
                return 1;
            }

            index = header->row_number * IPECAMERA_WIDTH + header->pixel_number;
            base += 3;

            pixel_buffer[index + 15*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base] >> 22);
            pixel_buffer[index + 13*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base] >> 12);
            pixel_buffer[index + 14*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base] >> 2);
            pixel_buffer[index + 12*IPECAMERA_PIXELS_PER_CHANNEL] = ((0x3 & raw[base]) << 8) | (0x3ff & (raw[base+1] >> 24));
            pixel_buffer[index + 10*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+1] >> 14);
            pixel_buffer[index +  8*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+1] >> 4);
            pixel_buffer[index + 11*IPECAMERA_PIXELS_PER_CHANNEL] = ((0xf & raw[base+1]) << 6) | (0x3ff & (raw[base+2] >> 26));
            pixel_buffer[index +  7*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+2] >> 16);
            pixel_buffer[index +  9*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+2] >> 6);
            pixel_buffer[index +  6*IPECAMERA_PIXELS_PER_CHANNEL] = ((0x3f & raw[base+2]) << 4) | (0x3ff & (raw[base+3] >> 28));
            pixel_buffer[index +  5*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+3] >> 18);
            pixel_buffer[index +  2*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+3] >> 8);
            pixel_buffer[index +  4*IPECAMERA_PIXELS_PER_CHANNEL] = ((0xff & raw[base+3]) << 2) | (0x3ff & (raw[base+4] >> 30));
            pixel_buffer[index +  3*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+4] >> 20);
            pixel_buffer[index +  0*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+4] >> 10);
            pixel_buffer[index +  1*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & raw[base+4];
            base += 5;
        }

        if (row != 0)
            base += 8;
    }

    *offset = base;
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
        for (int x = 0; x < width; x++)
            out[x] = (int) (0.5 * in[row*width + x] + 0.5 * in[(row+1)*width + x]);

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
 * \brief Decodes frame
 *
 * This function tries to decode the supplied data
 *
 * \param decoder An ufo_decoder instance
 * \param raw Raw data stream
 * \param num_bytes Size of data stream buffer in bytes
 * \param pixels If pointer with NULL content is passed, a new buffer is
 * allocated otherwise, this user-supplied buffer is used.
 * \param frame_number Frame number as reported in the header
 * \param time_stamp Time stamp of the frame as reported in the header
 * \paran cmask Change-mask
 *
 * \return number of decoded bytes or 0 in case of error
 */
size_t ufo_decoder_decode_frame(ufo_decoder decoder, uint32_t *raw, 
        size_t num_bytes, uint16_t *pixels, 
        uint32_t *num_rows, uint32_t *frame_number, uint32_t *time_stamp, 
        uint16_t *cmask)
{
    int err = 0;
    size_t pos = 0;
    size_t advance = 0;
    const size_t num_words = num_bytes / 4;

    if ((pixels == NULL) || (num_words < 16))
        return 0;

    size_t rows_per_frame = decoder->height;
    const int version = (raw[pos+6] >> 24) & 0xF;

#ifdef DEBUG
    CHECK_VALUE(raw[pos++], 0x51111111);
    CHECK_VALUE(raw[pos++], 0x52222222);
    CHECK_VALUE(raw[pos++], 0x53333333);
    CHECK_VALUE(raw[pos++], 0x54444444);
    CHECK_VALUE(raw[pos++], 0x55555555);

    switch (version) {
        case 0:
            CHECK_VALUE(raw[pos++], 0x56666666);
            CHECK_VALUE(raw[pos] >> 28, 0x5);
            *frame_number = raw[pos++] & 0xFFFFFFF;
            CHECK_VALUE(raw[pos] >> 28, 0x5);
            *time_stamp = raw[pos++] & 0xFFFFFFF;
            break;

        case 4:
        case 5:
            CHECK_VALUE(raw[pos] >> 28, 0x5);
            rows_per_frame = raw[pos] & 0x7FF;
            pos++;
            *frame_number = raw[pos++] & 0x1FFFFFF;
            CHECK_VALUE(raw[pos] >> 24, 0x50);
            *time_stamp = raw[pos++] & 0xFFFFFF;
            break;

        default:
            fprintf(stderr, "Unsupported data format detected\n");
            return 0;
    }

    if (err) 
        return 0;
#else
    switch (version) {
        case 0:
            *frame_number = raw[pos + 6] & 0xFFFFFFF;
            *time_stamp = raw[pos + 7] & 0xFFFFFFF;
            break;
        case 4:
        case 5:
            *frame_number = raw[pos + 6] & 0x1FFFFFF;
            *time_stamp = raw[pos + 7] & 0xFFFFFF;
            break;
        default:
            fprintf(stderr, "Unsupported data format detected\n");
            return 0;
    }

    pos += 8;
#endif

    *num_rows = rows_per_frame;

    switch (version) {
        case 0:
            err = ufo_decode_frame_channels_v0(decoder, pixels, cmask, raw + pos, num_words - pos - 8, &advance);
            break;
        case 4:
            err = ufo_decode_frame_channels_v4(decoder, pixels, cmask, raw + pos, num_words - pos - 8, rows_per_frame, &advance);
            break;
        case 5:
            err = ufo_decode_frame_channels_v5(decoder, pixels, cmask, raw + pos, num_words - pos - 8, rows_per_frame, &advance);
            break;
        default:
            break;
    }

    if (err)
        return 0;

    pos += advance;

#ifdef CHECKS
    CHECK_VALUE(raw[pos++], 0x0AAAAAAA);
    /* CHECK_VALUE(raw[pos++], 0x0BBBBBBB); */
    pos++;
    pos++; /* 0x840dffff expected */
    pos++; /* 0x0f001001 expected */
    pos++; /* 0x28000111 explains problems if status2 is wrong */
    pos++;
    /* CHECK_VALUE(raw[pos++], 0x0FFFFFFF); */
    CHECK_VALUE(raw[pos++], 0x00000000);
    CHECK_VALUE(raw[pos++], 0x01111111);

    if (err) 
        return 0;
#else
    pos += 8;
#endif

    return pos;
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
 * \param num_rows Number of actual decoded rows
 * \param frame_number Frame number as reported in the header
 * \param time_stamp Time stamp of the frame as reported in the header
 * \paran cmask Change-mask
 *
 * \return 0 in case of no error, EIO if end of stream was reached, ENOMEM if
 * NULL was passed but no memory could be allocated, EILSEQ if data stream is
 * corrupt and EFAULT if pixels is a NULL-pointer.
 */
int ufo_decoder_get_next_frame(ufo_decoder decoder, uint16_t **pixels, 
        uint32_t *num_rows, uint32_t *frame_number, uint32_t *time_stamp, 
        uint16_t *cmask)
{
    uint32_t *raw = decoder->raw;
    size_t pos = decoder->current_pos;
    size_t advance;
    const size_t num_words = decoder->num_bytes / 4;

    if (pixels == NULL)
        return 0;

    if (pos >= num_words)
        return EIO; 

    if (num_words < 16)
        return EILSEQ;

    if (*pixels == NULL) {
        *pixels = (uint16_t *) malloc(IPECAMERA_WIDTH * decoder->height * sizeof(uint16_t));
        if (*pixels == NULL)
            return ENOMEM;
    }

    while ((pos < num_words) && (raw[pos] != 0x51111111))
        pos++;

    advance = ufo_decoder_decode_frame(decoder, raw + pos, decoder->num_bytes -
            pos, *pixels, num_rows, frame_number, time_stamp, cmask);

    /*
     * On error, advance is 0 but we have to advance at least a bit to net get
     * caught in an infinite loop when trying to decode subsequent frames.
     */
    pos += advance == 0 ? 1 : advance;

    /* if bytes left and we see fill bytes, skip them */
    if (((pos + 2) < num_words) && ((raw[pos] == 0x0) && (raw[pos+1] == 0x1111111))) {
        pos += 2;
        while ((pos < num_words) && ((raw[pos] == 0x89abcdef) || (raw[pos] == 0x1234567)))
            pos++;
    }

    decoder->current_pos = pos;

    if (!advance)
        return EILSEQ;

    return 0;
}

