#ifndef LIB_UFODECODE_H
#define LIB_UFODECODE_H

#include <inttypes.h>

typedef struct ufo_decoder_t *ufo_decoder;

#ifdef __cplusplus
extern "C" {
#endif


ufo_decoder ufo_decoder_new(uint32_t height, uint32_t width, uint32_t *raw, size_t num_bytes);
void ufo_decoder_free(ufo_decoder decoder);
size_t ufo_decoder_decode_frame(ufo_decoder decoder, uint32_t *raw, size_t num_bytes, uint16_t *pixels, uint32_t *frame_number, uint32_t *time_stamp, uint16_t *cmask);
void ufo_decoder_set_raw_data(ufo_decoder decoder, uint32_t *raw, size_t num_bytes);
int ufo_decoder_get_next_frame(ufo_decoder decoder, uint16_t **pixels, uint32_t *frame_number, uint32_t *time_stamp, uint16_t *cmask);

void ufo_deinterlace_interpolate(const uint16_t *frame_in, uint16_t *frame_out, int width, int height);
void ufo_deinterlace_weave(const uint16_t *in1, const uint16_t *in2, uint16_t *out, int width, int height);


#ifdef __cplusplus
}
#endif

#endif

