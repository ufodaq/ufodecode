
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "libipe.h"
#include "libipe-private.h"
#include "config.h"

#define IPECAMERA_NUM_CHANNELS 16
#define IPECAMERA_PIXELS_PER_CHANNEL 128
#define IPECAMERA_WIDTH (IPECAMERA_NUM_CHANNELS * IPECAMERA_PIXELS_PER_CHANNEL)


#define CHECK_VALUE(value, expected) \
    if (value != expected) { \
        fprintf(stderr, "<%s:%i> 0x%x != 0x%x\n", __FILE__, __LINE__, value, expected); \
        err = 1; \
    }

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
 * ipe_decoder_set_raw_data.
 * \param num_bytes Size of the data stream buffer in bytes
 *
 * \return A new decoder instance that can be used to iterate over the frames
 * using ipe_decoder_get_next_frame.
 */
ipe_decoder ipe_decoder_new(uint32_t height, uint32_t *raw, size_t num_bytes)
{
    ipe_decoder decoder = malloc(sizeof(struct ipe_decoder_t));
    if (decoder == NULL)
        return NULL;

    decoder->height = height;
    ipe_decoder_set_raw_data(decoder, raw, num_bytes);
    return decoder;
}


/**
 * \brief Release decoder instance
 *
 * \param decoder An ipe_decoder instance
 */
void ipe_decoder_free(ipe_decoder decoder)
{
    free(decoder);
}


/**
 * \brief Set raw data stream
 *
 * \param decoder An ipe_decoder instance
 * \param raw Raw data stream
 * \param num_bytes Size of data stream buffer in bytes
 */
void ipe_decoder_set_raw_data(ipe_decoder decoder, uint32_t *raw, size_t num_bytes)
{
    decoder->raw = raw;
    decoder->num_bytes = num_bytes;
    decoder->current_pos = 0;
}


static int ipe_decode_frame(uint16_t *pixel_buffer, uint32_t *raw, int num_rows, int *offset)
{
    static int channel_order[IPECAMERA_NUM_CHANNELS] = { 15, 13, 14, 12, 10, 8, 11, 7, 9, 6, 5, 2, 4, 3, 0, 1 };

    int info;
    int row = 0;
    int channel = 0;
    int pos = 0;
    uint32_t data;

    do {
        info = raw[0];
        channel = info & 0x0F;
        row = (info >> 4) & 0x7FF;
        int pixels = (info >> 20) & 0xFF;

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
            base++;
        } 
#ifdef DEBUG
        else 
            CHECK_FLAG("number of pixels, %i is expected", pixels == IPECAMERA_PIXELS_PER_CHANNEL, pixels, IPECAMERA_PIXELS_PER_CHANNEL);
#endif

        int bytes = 43;
        /* bytes = pixels / 3; */
        /* int ppw = pixels - bytes * 3; */
        /* if (ppw) */ 
        /*     ++bytes; */

        for (int i = 1; i < bytes; i++) {
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

        pos += bytes + 1;
        raw += bytes + 1;
    } while ((row < (num_rows - 1)) || (channel != 1));

    *offset = pos;
    return 0;
}


/**
 * \brief Deinterlace by interpolating between two rows
 *
 * \param in Input frame
 * \param out Destination of interpolated frame
 * \param width Width of frame in pixels
 * \param heigh Height of frame in pixels
 */
void ipe_deinterlace_interpolate(const uint16_t *in, uint16_t *out, int width, int height)
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
 * \param heigh Height of frame in pixels
 */
void ipe_deinterlace_weave(const uint16_t *in1, const uint16_t *in2, uint16_t *out, int width, int height)
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
 * \param decoder An ipe_decoder instance
 * \param pixels If pointer with NULL content is passed, a new buffer is
 * allocated otherwise, this user-supplied buffer is used.
 * \param frame_number Frame number as reported in the header
 * \param time_stamp Time stamp of the frame as reported in the header
 *
 * \return 0 in case of no error, ENOSR if end of stream was reached, ENOMEM if
 * NULL was passed but no memory could be allocated, EILSEQ if data stream is
 * corrupt and EFAULT if pixels is a NULL-pointer.
 */
int ipe_decoder_get_next_frame(ipe_decoder decoder, uint16_t **pixels, uint32_t *frame_number, uint32_t *time_stamp)
{

    uint32_t *raw = decoder->raw;
    int err = 0;
    int pos = decoder->current_pos;
    int advance;
    const int num_words = decoder->num_bytes / 4;

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
    pos += 8;
#endif

    err = ipe_decode_frame(*pixels, raw + pos, decoder->height, &advance);
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
    if ((raw[pos] == 0x0) && (raw[pos+1] == 0x1111111)) {
        pos += 2;
        while ((pos < num_words) && ((raw[pos] == 0x89abcdef) || (raw[pos] == 0x1234567)))
            pos++;
    }

    decoder->current_pos = pos;
    return 0;
}

