#ifndef LIB_IPE_PRIVATE_H
#define LIB_IPE_PRIVATE_H

#include <stdbool.h>

struct ipe_decoder_t {
    uint32_t height;
    uint32_t *raw;
    size_t num_bytes; 
    uint32_t current_pos;
};


#endif

