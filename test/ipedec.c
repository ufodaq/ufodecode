#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include <getopt.h>
#include <ufodecode.h>


static int read_raw_file(const char *filename, char **buffer, size_t *length)
{
    FILE *fp = fopen(filename, "rb"); 
    if (fp == NULL)
        return ENOENT;

    fseek(fp, 0, SEEK_END);
    *length = ftell(fp);
    rewind(fp);

    *buffer = (char *) malloc(*length);

    if (*buffer == NULL) {
        fclose(fp);
        return ENOMEM;
    }

    size_t buffer_length = fread(*buffer, 1, *length, fp);
    fclose(fp);
    if (buffer_length != *length) {
        free(*buffer);
        return ERANGE;
    }
    return 0;
}

static void usage(void)
{
    printf("usage: ipedec [--num-rows=ROWS] [--clear-frame] FILE [FILE ...]\n\
Options:\n\
  -h, --help         Show this help message and exit\n\
  -r, --num-rows=N   N rows that are contained in the file\n\
  -c, --clear-frame  Clear the frame for each iteration\n");
}

static void process_file(const char *filename, int rows, int clear_frame)
{
    char *buffer = NULL;
    size_t num_bytes = 0;
    int err = 0;
    uint16_t *pixels = (uint16_t *) malloc(2048 * 1088 * sizeof(uint16_t));
    uint32_t num_rows, frame_number, time_stamp;
    int num_frames = 0;
    struct timeval start, end;
    long seconds = 0L, useconds = 0L;
    int error = read_raw_file(filename, &buffer, &num_bytes);

    if (error) {
        fprintf(stderr, "Error reading %s: %s\n", filename, strerror(error));
        return;
    }

    ufo_decoder decoder = ufo_decoder_new(rows, 2048, (uint32_t *) buffer, num_bytes);

    if (!decoder) {
        fprintf(stderr, "Failed to initialize decoder\n");
        return;
    }

    char output_name[256];
    snprintf(output_name, 256, "%s.raw", filename);
    FILE *fp = fopen(output_name, "wb");

    if (!fp) {
        fprintf(stderr, "Failed to open file for writing\n");
        return;
    }

    while (err != EIO) {
        if (clear_frame)
            memset(pixels, 0, 2048 * 1088 * sizeof(uint16_t));

        gettimeofday(&start, NULL);
        err = ufo_decoder_get_next_frame(decoder, &pixels, &num_rows, &frame_number, &time_stamp, NULL);
        gettimeofday(&end, NULL);

        if (!err) {
            num_frames++;
            seconds += end.tv_sec - start.tv_sec;
            useconds += end.tv_usec - start.tv_usec;
            fwrite(pixels, sizeof(uint16_t), 2048 * 1088, fp);
        }
        else if (err != EIO)
            fprintf(stderr, "Failed to decode frame %i\n", num_frames); 
    }

    fclose(fp);

    float mtime = seconds * 1000.0 + useconds / 1000.0;
    printf("Decoded %i frames in %.5fms\n", num_frames, mtime);

    free(pixels);
    ufo_decoder_free(decoder);
    free(buffer);
}

int main(int argc, char const* argv[])
{
    int getopt_ret, index;

    static struct option long_options[] = {
        { "num-rows", required_argument, 0, 'r' },
        { "clear-frame", no_argument, 0, 'c' },
        { "help", no_argument, 0, 'h' },
        { 0, 0, 0, 0 }
    };

    int clear_frame = 0;
    int rows = 1088;

    if (argc == 1) {
        printf("ipedec: no input files\n");
        return 0;
    }

    while ((getopt_ret = getopt_long(argc, (char *const *) argv, "r:ch", long_options, &index)) != -1) {
        switch (getopt_ret) {
            case 'r': 
                rows = atoi(optarg);
                break;
            case 'c':
                clear_frame = 1;
                break;
            case 'h':
                usage();
                return 0;
            default:
                break;
        } 
    }

    while (optind < argc)
        process_file(argv[optind++], rows, clear_frame);

    return 0;
}

