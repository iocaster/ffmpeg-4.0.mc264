/*
 * AAC encoder
 * Copyright (C) 2019 YongTae Kim
 *
 */

#ifndef AVCODEC_MCAACENC_H
#define AVCODEC_MCAACENC_H

/**
 * AAC encoder context
 */
typedef struct AACEncContext {
#if 0
    AVClass *av_class;
    AACEncOptions options;                       ///< encoding options
    PutBitContext pb;
    FFTContext mdct1024;                         ///< long (1024 samples) frame transform context
    FFTContext mdct128;                          ///< short (128 samples) frame transform context
    AVFloatDSPContext *fdsp;
    AACPCEInfo pce;                              ///< PCE data, if needed
#endif
    float *planar_samples[16];                   ///< saved preprocessed input

    int profile;                                 ///< copied from avctx
#if 0
    int needs_pce;                               ///< flag for non-standard layout
    LPCContext lpc;                              ///< used by TNS
    int samplerate_index;                        ///< MPEG-4 samplerate index
    int channels;                                ///< channel count
    const uint8_t *reorder_map;                  ///< lavc to aac reorder map
    const uint8_t *chan_map;                     ///< channel configuration map

    ChannelElement *cpe;                         ///< channel elements
    FFPsyContext psy;
    struct FFPsyPreprocessContext* psypp;
    const AACCoefficientsEncoder *coder;
    int cur_channel;                             ///< current channel for coder context
    int random_state;
    float lambda;
    int last_frame_pb_count;                     ///< number of bits for the previous frame
    float lambda_sum;                            ///< sum(lambda), for Qvg reporting
    int lambda_count;                            ///< count(lambda), for Qvg reporting
    enum RawDataBlockType cur_type;              ///< channel group type cur_channel belongs to

    AudioFrameQueue afq;
    DECLARE_ALIGNED(16, int,   qcoefs)[96];      ///< quantized coefficients
    DECLARE_ALIGNED(32, float, scoefs)[1024];    ///< scaled coefficients

    uint16_t quantize_band_cost_cache_generation;
    AACQuantizeBandCostCacheEntry quantize_band_cost_cache[256][128]; ///< memoization area for quantize_band_cost

    void (*abs_pow34)(float *out, const float *in, const int size);
    void (*quant_bands)(int *out, const float *in, const float *scaled,
                        int size, int is_signed, int maxval, const float Q34,
                        const float rounding);
#endif

    struct {
        float *samples;
    } buffer;
} AACEncContext;

#if 0
void ff_aac_dsp_init_x86(AACEncContext *s);
void ff_aac_coder_init_mips(AACEncContext *c);
void ff_quantize_band_cost_cache_init(struct AACEncContext *s);
#endif


#endif /* AVCODEC_MCAACENC_H */
