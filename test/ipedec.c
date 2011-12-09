#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <ufodecode.h>


int read_raw_file(const char *filename, char **buffer, size_t *length)
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
        free(buffer);
        return ERANGE;
    }
    return 0;
}


int main(int argc, char const* argv[])
{
    if (argc < 3) {
        fprintf(stderr, "Usage: ipedec <filename> <number of lines per frame>\n");
        return EXIT_FAILURE;
    }

    char *buffer = NULL;
    size_t num_bytes = 0;
    int error = read_raw_file(argv[1], &buffer, &num_bytes);
    if (error) {
        printf("file reading error: %s\n", strerror(error));
        return EXIT_FAILURE;
    }

    const int rows = atoi(argv[2]);

    ufo_decoder decoder = ufo_decoder_new(rows, 2048, (uint32_t *) buffer, num_bytes);
    int err = 0;
    uint16_t *pixels = (uint16_t *) malloc(2048 * rows * sizeof(uint16_t));
    uint32_t frame_number, time_stamp;
    int num_frames = 0;
    struct timeval start, end;
    long seconds = 0L, useconds = 0L;

    if (!decoder) {
	fprintf(stderr, "Failed to initialize decoder\n");
	return EXIT_FAILURE;
    }

    FILE *fp = fopen("test.raw", "wb");
    if (!fp) {
	fprintf(stderr, "Failed to open file for writting\n");
	return EXIT_FAILURE;
    }

    while (!err) {
        gettimeofday(&start, NULL);
        err = ufo_decoder_get_next_frame(decoder, &pixels, &frame_number, &time_stamp, NULL);
        gettimeofday(&end, NULL);

        if (!err) {
            num_frames++;
            seconds += end.tv_sec - start.tv_sec;
            useconds += end.tv_usec - start.tv_usec;
            fwrite(pixels, sizeof(uint16_t), 2048 * 1088, fp);
        }
    }
    fclose(fp);

    float mtime = seconds * 1000.0 + useconds / 1000.0;
    printf("Decoded %i frames in %.5fms\n", num_frames, mtime);

    free(pixels);
    ufo_decoder_free(decoder);
    free(buffer);

    return 0;
}

