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

#define IPECAMERA_NUM_ROWS              1088
#define IPECAMERA_NUM_CHANNELS          16      /**< Number of channels per row */
#define IPECAMERA_PIXELS_PER_CHANNEL    128     /**< Number of pixels per channel */
#define IPECAMERA_WIDTH (IPECAMERA_NUM_CHANNELS * IPECAMERA_PIXELS_PER_CHANNEL) /**< Total pixel width of row */

#define IPECAMERA_WIDTH_20MP            5120

#define IPECAMERA_MODE_16_CHAN_IO	0
#define IPECAMERA_MODE_4_CHAN_IO	2

#define IPECAMERA_MODE_12_BIT_ADC	2
#define IPECAMERA_MODE_11_BIT_ADC	1
#define IPECAMERA_MODE_10_BIT_ADC	0

typedef struct {
    unsigned no_ext_header : 1;
    unsigned version: 3;
    unsigned ones : 24;
    unsigned five: 4;
} pre_header_t;

typedef struct {
    uint32_t magic_2;
    uint32_t magic_3;
    uint32_t magic_4;
    uint32_t magic_5;
    unsigned n_rows : 11;
    unsigned n_skipped_rows : 7;
    unsigned cmosis_start_address : 10;
    unsigned five_1 : 4;
    unsigned frame_number : 24;
    unsigned dataformat_version : 4;
    unsigned five_2 : 4;
    unsigned timestamp : 24;
    unsigned zero_1 : 2;
    unsigned output_mode : 2;
    unsigned zero_2 : 2;
    unsigned adc_resolution : 2;
} header_v5_t;

typedef struct {
    uint32_t magic_2;
    uint32_t magic_3;
    uint32_t magic_4;
    unsigned cmosis_start_address : 16;
    unsigned output_mode : 4;
    unsigned adc_resolution : 4;
    unsigned five_1 : 4;
    unsigned n_rows : 16;
    unsigned n_skipped_rows : 12;
    unsigned five_2 : 4;
    unsigned frame_number : 24;
    unsigned dataformat_version : 4;
    unsigned five_3 : 4;
    unsigned timestamp : 28;
    unsigned five_4 : 4;
} header_v6_t;

typedef struct {
    unsigned pixel_number : 8;
    unsigned row_number : 12;
    unsigned pixel_size : 4;
    unsigned magic : 8;
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
UfoDecoder *
ufo_decoder_new (int32_t height, uint32_t width, uint32_t *raw, size_t num_bytes)
{
    if (width % IPECAMERA_PIXELS_PER_CHANNEL)
        return NULL;

    UfoDecoder *decoder = malloc (sizeof(UfoDecoder));

    if (decoder == NULL)
        return NULL;

    decoder->width = width;
    decoder->height = height;
    ufo_decoder_set_raw_data (decoder, raw, num_bytes);
    return decoder;
}

/**
 * \brief Release decoder instance
 *
 * \param decoder An UfoDecoder instance
 */
void
ufo_decoder_free (UfoDecoder *decoder)
{
    free (decoder);
}

/**
 * \brief Set raw data stream
 *
 * \param decoder An UfoDecoder instance
 * \param raw Raw data stream
 * \param num_bytes Size of data stream buffer in bytes
 */
void
ufo_decoder_set_raw_data (UfoDecoder *decoder, uint32_t *raw, size_t num_bytes)
{
    decoder->raw = raw;
    decoder->num_bytes = num_bytes;
    decoder->current_pos = 0;
}

static size_t
ufo_decode_frame_channels_v5 (UfoDecoder *decoder, uint16_t *pixel_buffer, uint32_t *raw, size_t num_rows, uint8_t output_mode)
{
    payload_header_v5 *header;
    size_t base = 0, index = 0;

    if (output_mode == IPECAMERA_MODE_4_CHAN_IO) {
        size_t off = 0;

        while (raw[base] != 0xAAAAAAA) {
            header = (payload_header_v5 *) &raw[base];
            index = header->row_number * IPECAMERA_WIDTH + header->pixel_number;

            /* Skip header + one zero-filled words */
            base += 2;

            if ((header->magic != 0xe0) && (header->magic != 0xc0)) {
                pixel_buffer[index +  (0+off)*IPECAMERA_PIXELS_PER_CHANNEL] = 0xfff & (raw[base+5] >> 12);
                pixel_buffer[index +  (4+off)*IPECAMERA_PIXELS_PER_CHANNEL] = 0xfff & (raw[base+4] >> 4);
                pixel_buffer[index +  (8+off)*IPECAMERA_PIXELS_PER_CHANNEL] = ((0xf & raw[base+1]) << 8) | (raw[base+2] >> 24);
                pixel_buffer[index + (12+off)*IPECAMERA_PIXELS_PER_CHANNEL] = 0xfff & (raw[base+1] >> 16);
            }
            else {
                off++;

                if (header->magic == 0xc0)
                    off = 0;
            }

            base += 6;
        }
    }
    else {
        while (raw[base] != 0xAAAAAAA) {
            header = (payload_header_v5 *) &raw[base];
            index = header->row_number * IPECAMERA_WIDTH + header->pixel_number;

            /* Skip header + two zero-filled words */
            base += 2;

            if (header->magic != 0xc0) {
                pixel_buffer[index + 15*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base] >> 20);
                pixel_buffer[index + 13*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base] >> 8);
                pixel_buffer[index + 14*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (((0xff & raw[base]) << 4) | (raw[base+1] >> 28));
                pixel_buffer[index + 12*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+1] >> 16);
                pixel_buffer[index + 10*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+1] >> 4);
                pixel_buffer[index +  8*IPECAMERA_PIXELS_PER_CHANNEL] = ((0x3 & raw[base+1]) << 8) | (raw[base+2] >> 24);
                pixel_buffer[index + 11*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+2] >> 12);
                pixel_buffer[index +  7*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & raw[base+2];
                pixel_buffer[index +  9*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+3] >> 20);
                pixel_buffer[index +  6*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+3] >> 8);
                pixel_buffer[index +  5*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (((0xff & raw[base+3]) << 4) | (raw[base+4] >> 28));
                pixel_buffer[index +  2*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+4] >> 16);
                pixel_buffer[index +  4*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+4] >> 4);
                pixel_buffer[index +  3*IPECAMERA_PIXELS_PER_CHANNEL] = ((0x3 & raw[base+4]) << 8) | (raw[base+5] >> 24);
                pixel_buffer[index +  0*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & (raw[base+5] >> 12);
                pixel_buffer[index +  1*IPECAMERA_PIXELS_PER_CHANNEL] = 0x3ff & raw[base+5];
            }

            base += 6;
        }
    }

    return base;
}

static size_t
ufo_decode_frame_channels_v6 (UfoDecoder *decoder, uint16_t *pixel_buffer, uint32_t *raw, size_t num_rows)
{
    size_t base = 0;
    size_t index = 0;
    const size_t space = 640;

#ifdef HAVE_SSE
    const __m64 mask_fff = _mm_set_pi32 (0xfff, 0xfff);
    __m64 mm_r;
#endif

    while (raw[base] != 0xAAAAAAA) {
        const size_t row_number = raw[base] & 0xfff;
        const size_t pixel_number = (raw[base + 1] >> 16) & 0xfff;

        base += 2;
        index = row_number * IPECAMERA_WIDTH_20MP + pixel_number;

#ifdef HAVE_SSE
        const __m64 src1 = _mm_set_pi32 (raw[base], raw[base + 3]);
        const __m64 src2 = _mm_set_pi32 (raw[base + 1], raw[base + 4]);
        const __m64 src3 = _mm_set_pi32 (raw[base + 2], raw[base + 5]);

#define store(i) \
        pixel_buffer[index + i * space] = ((uint32_t *) &mm_r)[0]; \
        pixel_buffer[index + IPECAMERA_WIDTH_20MP + i * space] = ((uint32_t *) &mm_r)[1];

        mm_r = _mm_srli_pi32 (src1, 20);
        store(0);

        mm_r = _mm_and_si64 (_mm_srli_pi32 (src1, 8), mask_fff);
        store(1);

        mm_r = _mm_or_si64 (_mm_and_si64 (_mm_slli_pi32 (src1, 4), mask_fff), _mm_srli_pi32 (src2, 28));
        store(2);

        mm_r = _mm_and_si64 (_mm_srli_pi32 (src2, 16), mask_fff);
        store(3);

        mm_r = _mm_and_si64 (_mm_srli_pi32 (src2, 4), mask_fff);
        store(4);

        mm_r = _mm_or_si64 (_mm_and_si64 (_mm_slli_pi32 (src2, 8), mask_fff), _mm_srli_pi32 (src3, 24));
        store(5);

        mm_r = _mm_and_si64 (_mm_srli_pi32 (src3, 12), mask_fff);
        store(6);

        mm_r = _mm_and_si64 (src3, mask_fff);
        store(7);
#else
        pixel_buffer[index + 0 * space] = (raw[base] >> 20);
        pixel_buffer[index + 1 * space] = (raw[base] >> 8) & 0xfff;
        pixel_buffer[index + 2 * space] = ((raw[base] << 4) & 0xfff) | (raw[base + 1] >> 28);
        pixel_buffer[index + 3 * space] = (raw[base + 1] >> 16) & 0xfff;
        pixel_buffer[index + 4 * space] = (raw[base + 1] >> 4) & 0xfff;
        pixel_buffer[index + 5 * space] = ((raw[base + 1] << 8) & 0xfff) | (raw[base + 2] >> 24);
        pixel_buffer[index + 6 * space] = (raw[base + 2] >> 12) & 0xfff;
        pixel_buffer[index + 7 * space] = raw[base + 2] & 0xfff;

        index += IPECAMERA_WIDTH_20MP;
        pixel_buffer[index + 0 * space] = (raw[base + 3] >> 20);
        pixel_buffer[index + 1 * space] = (raw[base + 3] >> 8) & 0xfff;
        pixel_buffer[index + 2 * space] = ((raw[base + 3] << 4) & 0xfff) | (raw[base + 4] >> 28);
        pixel_buffer[index + 3 * space] = (raw[base + 4] >> 16) & 0xfff;
        pixel_buffer[index + 4 * space] = (raw[base + 4] >> 4) & 0xfff;
        pixel_buffer[index + 5 * space] = ((raw[base + 4] << 8) & 0xfff) | (raw[base + 5] >> 24);
        pixel_buffer[index + 6 * space] = (raw[base + 5] >> 12) & 0xfff;
        pixel_buffer[index + 7 * space] = (raw[base + 5] & 0xfff);
#endif

        base += 6;
    }

    return base;
}

/**
 * \brief Deinterlace by interpolating between two rows
 *
 * \param in Input frame
 * \param out Destination of interpolated frame
 * \param width Width of frame in pixels
 * \param height Height of frame in pixels
 */
void
ufo_deinterlace_interpolate (const uint16_t *in, uint16_t *out, int width, int height)
{
    const size_t row_size_bytes = width * sizeof(uint16_t);

    for (int row = 0; row < height; row++) {
        /* Copy one line */
        memcpy (out, in + row*width, row_size_bytes);
        out += width;

        /* Interpolate between source row and row+1 */
        for (int x = 0; x < width; x++)
            out[x] = (int) (0.5 * in[row*width + x] + 0.5 * in[(row+1)*width + x]);

        out += width;
    }

    /* Copy last row */
    memcpy (out, in + width * (height - 1), row_size_bytes);
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
void
ufo_deinterlace_weave (const uint16_t *in1, const uint16_t *in2, uint16_t *out, int width, int height)
{
    const size_t row_size_bytes = width * sizeof(uint16_t);
    for (int row = 0; row < height; row++) {
        memcpy (out, in1 + row*width, row_size_bytes);
        out += width;
        memcpy (out, in2 + row*width, row_size_bytes);
        out += width;
    }
}

/**
 * \brief Decodes frame
 *
 * This function tries to decode the supplied data
 *
 * \param decoder An UfoDecoder instance
 * \param raw Raw data stream
 * \param num_bytes Size of data stream buffer in bytes
 * \param pixels If pointer with NULL content is passed, a new buffer is
 * allocated otherwise, this user-supplied buffer is used.
 * \param frame_number Frame number as reported in the header
 * \param time_stamp Time stamp of the frame as reported in the header
 *
 * \return number of decoded bytes or 0 in case of error
 */
size_t
ufo_decoder_decode_frame (UfoDecoder *decoder, uint32_t *raw, size_t num_bytes, uint16_t *pixels, UfoDecoderMeta *meta)
{
    int err = 0;
    size_t pos = 0;
    size_t advance = 0;
    const size_t num_words = num_bytes / 4;
    size_t rows_per_frame = decoder->height;
    const pre_header_t *pre_header;

    if ((pixels == NULL) || (num_words < 16))
        return 0;

    pre_header = (pre_header_t *) raw;

    CHECK_VALUE (pre_header->five, 0x5);
    CHECK_VALUE (pre_header->ones, 0x111111);

    const int header_version = pre_header->version + 5;    /* it starts with 0 */
    int dataformat_version = 5;      /* will overwrite for header_version >= 6 */

    switch (header_version) {
        case 5:
            {
                const header_v5_t *header = (header_v5_t *) &raw[pos + 1];

                CHECK_VALUE (header->magic_2, 0x52222222);
                CHECK_VALUE (header->magic_3, 0x53333333);
                CHECK_VALUE (header->magic_4, 0x54444444);
                CHECK_VALUE (header->magic_5, 0x55555555);

                CHECK_VALUE (header->five_1, 0x5);
                CHECK_VALUE (header->five_2, 0x5);

                meta->time_stamp = header->timestamp;
                meta->cmosis_start_address = header->cmosis_start_address;
                meta->frame_number = header->frame_number;
                meta->n_rows = header->n_rows;
                meta->n_skipped_rows = header->n_skipped_rows;
                break;
            }

        case 6:
            {
                const header_v6_t *header = (header_v6_t *) &raw[pos + 1];
                CHECK_VALUE (header->magic_2, 0x52222222);
                CHECK_VALUE (header->magic_3, 0x53333333);
                CHECK_VALUE (header->magic_4, 0x54444444);

                dataformat_version = header->dataformat_version;

                meta->output_mode = header->output_mode;
                meta->adc_resolution = header->adc_resolution;
                meta->time_stamp = header->timestamp;
                meta->cmosis_start_address = header->cmosis_start_address;
                meta->frame_number = header->frame_number;
                meta->n_rows = header->n_rows;
                meta->n_skipped_rows = header->n_skipped_rows;

                break;
            }

        default:
            fprintf (stderr, "Unsupported header version %i\n", header_version);
    }


#ifdef DEBUG
    if ((meta->output_mode != IPECAMERA_MODE_4_CHAN_IO) && (meta->output_mode != IPECAMERA_MODE_16_CHAN_IO)) {
        fprintf (stderr, "Output mode 0x%x is not supported\n", meta->output_mode);
        return EILSEQ;
    }

    if (err) {
        fprintf (stderr, "Corrupt data:");

        for (int i = 0; i < pos; i++) {
            if ((i % 8) == 0)
                fprintf (stderr, "\n");

            fprintf (stderr, " %#08x", raw[i]);
        }

        fprintf (stderr, "\n");
        return 0;
    }
#endif

    pos += 8;

    switch (dataformat_version) {
        case 5:
            advance = ufo_decode_frame_channels_v5 (decoder, pixels, raw + pos, rows_per_frame, meta->output_mode);
            break;

        case 6:
            advance = ufo_decode_frame_channels_v6 (decoder, pixels, raw + pos, rows_per_frame);
            break;

        default:
            fprintf (stderr, "Data format version %i unsupported\n", dataformat_version);
    }

    if (err)
        return 0;

    pos += advance;

    CHECK_VALUE(raw[pos], 0x0AAAAAAA);
    pos++;

    meta->status1.bits = raw[pos++];
    meta->status2.bits = raw[pos++];
    meta->status3.bits = raw[pos++];
    pos += 2;

    CHECK_VALUE(raw[pos], 0x00000000);
    pos++;

    CHECK_VALUE(raw[pos], 0x01111111);
    pos++;

    if (err)
        return 0;

    return pos;
}

/**
 * \brief Iterate and decode next frame
 *
 * This function tries to decode the next frame in the currently set raw data
 * stream.
 *
 * \param decoder An UfoDecoder instance
 * \param pixels If pointer with NULL content is passed, a new buffer is
 * allocated otherwise, this user-supplied buffer is used.
 * \param num_rows Number of actual decoded rows
 * \param frame_number Frame number as reported in the header
 * \param time_stamp Time stamp of the frame as reported in the header
 *
 * \return 0 in case of no error, EIO if end of stream was reached, ENOMEM if
 * NULL was passed but no memory could be allocated, EILSEQ if data stream is
 * corrupt and EFAULT if pixels is a NULL-pointer.
 */
int
ufo_decoder_get_next_frame (UfoDecoder *decoder, uint16_t **pixels, UfoDecoderMeta *meta)
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
        *pixels = (uint16_t *) malloc (IPECAMERA_WIDTH * decoder->height * sizeof(uint16_t));

        if (*pixels == NULL)
            return ENOMEM;
    }

    while ((pos < num_words) &&
           ((raw[pos] & 0xFFFFFFF0) != 0x51111110)) /* we can only match the first part */
        pos++;

    advance = ufo_decoder_decode_frame (decoder, raw + pos, decoder->num_bytes - pos, *pixels, meta);

    /*
     * On error, advance is 0 but we have to advance at least a bit to net get
     * caught in an infinite loop when trying to decode subsequent frames.
     */
    pos += advance == 0 ? 1 : advance;

    /* if bytes left and we see fill bytes, skip them */
    if (((pos + 2) < num_words) && ((raw[pos] == 0x0) && ((raw[pos+1] == 0x1111111) || raw[pos+1] == 0x0))) {
        pos += 2;
        while ((pos < num_words) &&
               ((raw[pos] == 0x89abcdef) || (raw[pos] == 0x1234567) ||
                (raw[pos] == 0x0) || (raw[pos] == 0xdeadbeef) || (0x98badcfe)))     /* new filling ... */ {
            pos++;
        }
    }

    decoder->current_pos = pos;

    if (!advance)
        return EILSEQ;

    return 0;
}

/**
 * \brief Convert Bayer pattern to RGB
 *
 * Convert Bayer pattern to RGB via bilinear interpolation.
 *
 * \param in 16 bit input data in Bayer pattern format
 * \param out Location for 24 bit output data in RGB format. At
 * least width x height x 3 bytes must be allocated.
 * \param width Width of a frame
 * \param height Height of a frame
 */
void
ufo_convert_bayer_to_rgb (const uint16_t *in, uint8_t *out, int width, int height)
{
    /* According to the CMV docs, the pattern starts at (0,0) with
     *
     *   R G
     *   G B
     */

#define BY(x,y) in[(x) + width * (y)]
#define R(x,y) out[0 + 3 * ((x) + width * (y))]
#define G(x,y) out[1 + 3 * ((x) + width * (y))]
#define B(x,y) out[2 + 3 * ((x) + width * (y))]

    double scale;
    uint16_t max = 0;

    for (int i = 0; i < width * height; i++) {
        if (max < in[i])
            max = in[i];
    }

    scale = 255. / max;

    for (int i = 1; i < width - 1; i += 2) {
        for (int j = 1; j < height - 1; j += 2) {
            /* Top left */
            R(i + 0, j + 0) = ((uint32_t) BY(i - 1, j - 1) +
                               (uint32_t) BY(i + 1, j - 1) +
                               (uint32_t) BY(i - 1, j + 1) +
                               (uint32_t) BY(i + 1, j + 1)) / 4 * scale;
            G(i + 0, j + 0) = ((uint32_t) BY(i - 1, j + 0) +
                               (uint32_t) BY(i + 0, j - 1) +
                               (uint32_t) BY(i + 1, j + 0) +
                               (uint32_t) BY(i + 0, j + 1)) / 4 * scale;
            B(i + 0, j + 0) = BY(i + 0, j + 0) * scale;

            /* Top right */
            R(i + 1, j + 0) = ((uint32_t) BY(i + 1, j - 1) +
                               (uint32_t) BY(i + 1, j + 1)) / 2 * scale;
            G(i + 1, j + 0) = BY(i + 1, j + 0) * scale;
            B(i + 1, j + 0) = ((uint32_t) BY(i + 0, j + 0) +
                               (uint32_t) BY(i + 2, j + 0)) / 2 * scale;

            /* Lower left */
            R(i + 0, j + 1) = ((uint32_t) BY(i - 1, j + 0) +
                               (uint32_t) BY(i + 1, j + 1)) / 2 * scale;
            G(i + 0, j + 1) = BY(i + 0, j + 1) * scale;
            B(i + 0, j + 1) = ((uint32_t) BY(i + 0, j + 0) +
                               (uint32_t) BY(i + 0, j + 2)) / 2 * scale;

            /* Lower right */
            R(i + 1, j + 1) = BY(i + 1, j + 1) * scale;
            G(i + 1, j + 1) = ((uint32_t) BY(i + 1, j + 0) +
                               (uint32_t) BY(i + 0, j + 1) +
                               (uint32_t) BY(i + 2, j + 1) +
                               (uint32_t) BY(i + 2, j + 1)) / 4 * scale;
            B(i + 1, j + 1) = ((uint32_t) BY(i + 0, j + 0) +
                               (uint32_t) BY(i + 2, j + 0) +
                               (uint32_t) BY(i + 0, j + 2) +
                               (uint32_t) BY(i + 2, j + 2)) / 4 * scale;
        }
    }

#undef R
#undef G
#undef B
}
