#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

#define IPECAMERA_MAX_LINES 1088
#define IPECAMERA_MAX_CHANNELS 16
#define IPECAMERA_PIXELS_PER_CHANNEL 128
#define IPECAMERA_WIDTH (IPECAMERA_MAX_CHANNELS * IPECAMERA_PIXELS_PER_CHANNEL)
#define IPECAMERA_HEIGHT 1088

int ipecamera_channel_order[IPECAMERA_MAX_CHANNELS] = { 15, 13, 14, 12, 10, 8, 11, 7, 9, 6, 5, 2, 4, 3, 0, 1 };

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


static int ipecamera_get_payload(uint16_t *pixel_buffer, 
        int line_req, size_t size, 
        uint32_t *payload,
        int *advance, int *increase_line)
{
    int ppw;
    int err = 0;

    uint32_t info = payload[0];
    int channel = info & 0x0F;          // 4 bits
    int line = (info >> 4) & 0x7FF;     // 11 bits
    int bpp = (info >> 16) & 0x0F;      // 4 bits
    int pixels = (info >> 20) & 0xFF;   // 8 bits
    int header = (info >> 30) & 0x03;   // 2 bits

    int bytes, pix, pix_offset = 0;
    uint32_t data;

    *increase_line = channel == 15 ? 1 : 0;
    channel = ipecamera_channel_order[channel];

    CHECK_FLAG("payload header magick", header == 2, header);
    CHECK_FLAG("pixel size, only 10 bits are supported", bpp == 10, bpp);
    CHECK_FLAG("row number, should be %i", line == line_req, line, line_req);
    CHECK_FLAG("channel, limited by %i output channels", channel < IPECAMERA_MAX_CHANNELS, channel, IPECAMERA_MAX_CHANNELS);

    if ((line < 2) && (pixels == (IPECAMERA_PIXELS_PER_CHANNEL - 1))) {
        pix_offset = 1;
        pixel_buffer[channel*IPECAMERA_PIXELS_PER_CHANNEL] = 0;
    } 
    else 
        CHECK_FLAG("number of pixels, %i is expected", pixels == IPECAMERA_PIXELS_PER_CHANNEL, pixels, IPECAMERA_PIXELS_PER_CHANNEL);

    bytes = pixels / 3;
    ppw = pixels - bytes * 3;
    if (ppw) 
        ++bytes;

    CHECK_FLAG("payload data bytes, at least %i are expected", bytes < size, (unsigned int) size, bytes);

    if (err) 
        return err;

    pix = pix_offset;
    for (int i = 1; i < bytes; i++) {
        data = payload[i];
        header = (data >> 30) & 0x03;   

        CHECK_FLAG("payload data magick", header == 3, header);
        if (err) 
            return err;

        pixel_buffer[channel*IPECAMERA_PIXELS_PER_CHANNEL + pix++] = (data >> 20) & 0x3FF;
        pixel_buffer[channel*IPECAMERA_PIXELS_PER_CHANNEL + pix++] = (data >> 10) & 0x3FF;
        pixel_buffer[channel*IPECAMERA_PIXELS_PER_CHANNEL + pix++] = data & 0x3FF;
    }

    data = payload[bytes];
    header = (data >> 30) & 0x03;

    CHECK_FLAG("payload data magick", header == 3, header);
    CHECK_FLAG("payload footer magick", (data&0x3FF) == 0x55, (data&0x3FF));
    if ((err) && ((data & 0x3FF) != 0x55)) {
        err = 0; 
        int counter = 0;
        printf(" -> %x\n", data);
        while ((data & 0x3FF) != 0x55) {
            bytes++; 
            data = payload[bytes];
            counter++;
        }
        printf("skipped %i bytes: found %x\n", counter, data);
    }
    if (err) 
        return err;

    ppw = pixels % 3;
    assert(ppw < 3);

    for (int j = 0; j < ppw; j++, pix++)
        pixel_buffer[channel*IPECAMERA_PIXELS_PER_CHANNEL + pix] = (data >> (10 * (ppw - j))) & 0x3FF;

    *advance = bytes + 1;
    return 0;
}

static int ipecamera_parse_image(int first_line, int num_lines, 
        size_t size, uint32_t *linebuf, int *frames_decoded) 
{
    int err = 0, current_frame = 0;
    int pos = 0, advance, increase_line;

    if (size < 16) {
        fprintf(stderr, "The payload is tool small, we should have at least 8 header dwords and 8 footer.");
        return 1;
    }

    FILE *fp = fopen("image.dec.raw", "wb");
    uint16_t *pixel_buffer = (uint16_t *) malloc(num_lines * IPECAMERA_WIDTH * sizeof(uint16_t));

    printf("Decoding");
    while (size > 0) {
        printf(".");
        int line = first_line;
        CHECK_VALUE(linebuf[pos++], 0x51111111);
        CHECK_VALUE(linebuf[pos++], 0x52222222);
        CHECK_VALUE(linebuf[pos++], 0x53333333);
        CHECK_VALUE(linebuf[pos++], 0x54444444);
        CHECK_VALUE(linebuf[pos++], 0x55555555);
        CHECK_VALUE(linebuf[pos++], 0x56666666);
        CHECK_VALUE(linebuf[pos++], 0x57777777);
        CHECK_VALUE(linebuf[pos++], 0x58888888);
        if (err) 
            return err;

        size -= 16;

        while ((size > 0) && (line < (first_line + num_lines))) {
            err = ipecamera_get_payload(pixel_buffer + line * IPECAMERA_WIDTH,
                    line - first_line, size, linebuf + pos, &advance, &increase_line);
            if (err) 
                return err;

            pos += advance;
            size -= advance;
            line += increase_line;
        }

        CHECK_FLAG("lines read, we expect to read exactly %i lines", line == (first_line + num_lines), line - first_line, num_lines);

        CHECK_VALUE(linebuf[pos++], 0x0AAAAAAA);
        CHECK_VALUE(linebuf[pos++], 0x0BBBBBBB);
        CHECK_VALUE(linebuf[pos++], 0x0CCCCCCC);
        CHECK_VALUE(linebuf[pos++], 0x0DDDDDDD);
        CHECK_VALUE(linebuf[pos++], 0x0EEEEEEE);
        CHECK_VALUE(linebuf[pos++], 0x0FFFFFFF);
        CHECK_VALUE(linebuf[pos++], 0x00000000);
        CHECK_VALUE(linebuf[pos++], 0x01111111);

        fwrite(pixel_buffer, IPECAMERA_WIDTH * num_lines, sizeof(uint16_t), fp);

        if (linebuf[pos] == 0)
            break;

        current_frame++;
    }
    fclose(fp);
    free(pixel_buffer);
    printf("\n");
    *frames_decoded = current_frame + 1;
    return err;
}

int main(int argc, char const* argv[])
{
    if (argc < 3) {
        fprintf(stderr, "Usage: ipedec <filename> <number of lines per frame>\n");
        return 1;
    }

    FILE *fp = fopen(argv[1], "rb"); 
    if (fp == NULL) {
        fprintf(stderr, "Couldn't open file %s", argv[1]);
        return 1;
    }

    const int lines_per_frame = atoi(argv[2]);

    fseek(fp, 0, SEEK_END);
    const size_t length = ftell(fp);
    rewind(fp);

    char *buffer = (char *) malloc(length);
    if (buffer == NULL) {
        fclose(fp);
        return 1;
    }

    size_t buffer_length = fread(buffer, 1, length, fp);
    fclose(fp);
    if (buffer_length != length) {
        free(buffer);
        return 1;
    }

    int num_frames;
    if (ipecamera_parse_image(0, lines_per_frame, length, (uint32_t *) buffer, &num_frames))
        fprintf(stderr, "Couldn't parse image\n");

    printf("Decoded %i frames.\n", num_frames);
    free(buffer);
    return 0;
}

