#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <getopt.h>

#define IPE_DEFAULT_WIDTH 2048

char *strdup(const char *str)
{
    int n = strlen(str) + 1;
    char *dup = malloc(n);
    if (dup)
        strcpy(dup, str);
    return dup;
}

int process_simple_frame(uint16_t *frame_in, uint16_t *frame_out, int width, int height)
{
    const size_t row_size_bytes = width * sizeof(uint16_t);

    for (int row = 0; row < height; row++) {
        /* Copy one line */
        memcpy(frame_out, frame_in + row*width, row_size_bytes);
        frame_out += width;

        /* Interpolate between source row and row+1 */ 
        for (int x = 0; x < width; x++) {
            frame_out[x] = (int) (0.5 * frame_in[row*width + x] + 0.5 * frame_in[(row+1)*width + x]);
        }
        frame_out += width;
    }

    /* Copy last row */
    memcpy(frame_out, frame_in + width * (height - 1), row_size_bytes);
    return 0;
}

int process_file(const char *filename, int width, int height, int num_lines_skipped)
{
    FILE *fp = fopen(filename, "rb");
    if (fp == NULL) {
        return EIO; 
    }
    fseek(fp, 0, SEEK_END);
    const size_t file_size = ftell(fp);
    rewind(fp);

    uint16_t *frame_in_buffer = (uint16_t *) malloc(file_size);
    if (frame_in_buffer == NULL) {
        fclose(fp);
        return ENOMEM;
    }

    size_t buffer_length = fread(frame_in_buffer, 1, file_size, fp);
    fclose(fp);
    if (buffer_length != file_size) {
        free(frame_in_buffer);
        return EIO;
    }

    uint16_t *frame_out_buffer = (uint16_t *) malloc(file_size * 2);
    if (frame_out_buffer == NULL) {
        fclose(fp);
        return ENOMEM;
    }

    const int num_frames = file_size / (width * height * sizeof(uint16_t));
    printf("de-interlacing %i frames...\n", num_frames);
    for (int frame = 0; frame < num_frames; frame++) {
        process_simple_frame(
                frame_in_buffer + frame * (width * height),
                frame_out_buffer + frame * (width * height * 2),
                width, height);
    }

    fp = fopen("result.raw", "wb");
    fwrite(frame_out_buffer, 1, file_size * 2, fp);
    fclose(fp);

    free(frame_in_buffer);
    free(frame_out_buffer);
    return 0;
}

int main(int argc, char const* argv[])
{
    static struct option long_options[] = {
        { "width", 1, 0, 'w' },
        { "interlaced-height", 1, 0, 'i' },
        { "target-height", 1, 0, 't' },
        { "file", 1, 0, 'f' },
        { NULL, 0, NULL, 0 }
    }; 

    int c, option_index = 0, errnum;
    int width = IPE_DEFAULT_WIDTH;
    int interlaced_height = -1;
    int target_height = -1;
    char *file_name = NULL;
    char *end;

    while ((c = getopt_long(argc, (char *const *) argv, "w:i:t:f:", long_options, &option_index)) != -1) {
        switch (c) {
            case 'w': 
                width = (int) strtol(optarg, &end, 10);
                break;
            case 'i':
                interlaced_height = (int) strtol(optarg, &end, 10);
                break;
            case 't':
                target_height = (int) strtol(optarg, &end, 10);
                break;
            case 'f':
                file_name = strdup( (char *) optarg);
                break;
        } 
    }

    if (interlaced_height == -1 || file_name == NULL) {
        printf("Usage: deinterlace --interlaced-height=[number] --target-height=[number] --file=[name]\n");
        exit(EXIT_FAILURE);
    }

    if (target_height == -1)
        target_height = interlaced_height * 2;

    if ((errnum = process_file(file_name, width, interlaced_height, (target_height / interlaced_height) - 1)))
        printf("Error occured: %s\n", strerror(errnum));

    free(file_name);
    return 0;
}
