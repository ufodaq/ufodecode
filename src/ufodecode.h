#ifndef LIB_UFODECODE_H
#define LIB_UFODECODE_H

#include <inttypes.h>

typedef struct _UfoDecoder UfoDecoder;

typedef struct {
    unsigned    data_lock:16;
    unsigned    control_lock:1;
    unsigned    pixel_full:1;
    unsigned    fsm_daq:4;
    unsigned    dummy2:4;
    unsigned    fsm_master_readout:4;
    unsigned    dummy1:2;
} UfoDecoderStatus1;

typedef struct {
    unsigned    ddr_fifo_empty:1;
    unsigned    ddr_fifo_full:1;
    unsigned    ddr_fifo_write_count:8;
    unsigned    dummy:2;
    unsigned    data_fifo_empty:1;
    unsigned    data_fifo_full:1;
    unsigned    data_fifo_read_count:10;
    unsigned    error_status:4;   /* What the heck? */
    unsigned    busy_interl:1;
    unsigned    busy_ddr:1;
    unsigned    busy_or:1;
    unsigned    end_of_frames:1;
} UfoDecoderStatus2;

typedef struct {
    unsigned    ddr_arbiter:4;
    unsigned    ddr_write:4;
    unsigned    ddr_read:4;
    unsigned    pixel_counter:7;
    unsigned    row_counter:11;
    unsigned    dummy:2;
} UfoDecoderStatus3;

typedef struct {
    uint32_t        frame_number;
    uint32_t        time_stamp;
    uint32_t        n_rows;
    uint8_t         n_skipped_rows;
    uint16_t        cmosis_start_address;
    union {
        uint32_t            bits;
        UfoDecoderStatus1   desc;
    }                       status1;
    union {
        uint32_t            bits; 
        UfoDecoderStatus2   desc;
    }                       status2;
    union {
        uint32_t            bits; 
        UfoDecoderStatus3   desc;
    }                       status3;
} UfoDecoderMeta;

#ifdef __cplusplus
extern "C" {
#endif

UfoDecoder *ufo_decoder_new             (int32_t         height, 
                                         uint32_t        width, 
                                         uint32_t       *raw, 
                                         size_t          num_bytes);
void        ufo_decoder_free            (UfoDecoder     *decoder);
size_t      ufo_decoder_decode_frame    (UfoDecoder     *decoder, 
                                         uint32_t       *raw, 
                                         size_t          num_bytes, 
                                         uint16_t       *pixels, 
                                         UfoDecoderMeta *meta);
void        ufo_decoder_set_raw_data    (UfoDecoder     *decoder,
                                         uint32_t       *raw,
                                         size_t          num_bytes);
int         ufo_decoder_get_next_frame  (UfoDecoder     *decoder, 
                                         uint16_t      **pixels, 
                                         UfoDecoderMeta *meta_data);
void        ufo_deinterlace_interpolate (const uint16_t *frame_in, 
                                         uint16_t       *frame_out, 
                                         int             width, 
                                         int             height);
void        ufo_deinterlace_weave       (const uint16_t *in1,
                                         const uint16_t *in2,
                                         uint16_t       *out, 
                                         int             width, 
                                         int             height);

#ifdef __cplusplus
}
#endif

#endif

