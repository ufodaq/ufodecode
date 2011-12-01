#ifndef LIB_IPE_H
#define LIB_IPE_H

#include <inttypes.h>

typedef struct ipe_decoder_t *ipe_decoder;

#ifdef __cplusplus
extern "C" {
#endif


ipe_decoder ipe_decoder_new(uint32_t height, uint32_t *raw, size_t num_bytes);
void ipe_decoder_free(ipe_decoder decoder);
void ipe_decoder_set_raw_data(ipe_decoder decoder, uint32_t *raw, size_t num_bytes);
int ipe_decoder_get_next_frame(ipe_decoder decoder, uint16_t **pixels, uint32_t *frame_number, uint32_t *time_stamp);

void ipe_deinterlace_interpolate(const uint16_t *frame_in, uint16_t *frame_out, int width, int height);
void ipe_deinterlace_weave(const uint16_t *in1, const uint16_t *in2, uint16_t *out, int width, int height);


#ifdef __cplusplus
}
#endif

#endif

