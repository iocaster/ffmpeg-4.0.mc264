/*
 * AAC encoder
 * Copyright (C) 2019 YongTae Kim
 *
 */

#include "libavutil/opt.h"
#include "avcodec.h"
#include "internal.h"

//#include "mcaac.h"

extern int sendAACParameterToJavaEncoder( int samplerate, int bitrate, int numchannels  ); 					//ffmpeg_jni.c
extern int sendPCMFrameToJavaEncoder( void *data, int length, uint64_t pts ); 								//ffmpeg_jni.c
extern int getEncodedFrameAAC( void **data, int *length, int64_t *presentationTimeUs, int32_t *b_keyframe );//ffmpeg_jni.c
extern int mediacodecAAC_release(void);																		//ffmpeg_jni.c

typedef struct mediacodecAACContext {
	AVClass	*class;
	//float fps;
	//uint64_t frameIdx;
    int64_t pts;
	uint8_t b_close;
	//struct timeval startTime;
} mediacodecAACContext;

static av_cold int mcaac_encode_init(AVCodecContext *avctx)
{
    mediacodecAACContext *myCtx = avctx->priv_data;

	int ret = 0;

	myCtx->pts = 0;
	myCtx->b_close = 0;

	//PCM bufferSize : int bufferSize = ((20ms * 44100) / 1000) * sizeof(short) * chanels;
    //avctx->frame_size = ((20 * avctx->sample_rate) / 1000) * sizeof(short) * avctx->channels;
    avctx->frame_size = ((20 * avctx->sample_rate) / 1000);

	av_log(avctx, AV_LOG_DEBUG, "mcaac_encode_init() :   avctx->sample_rate = %d\n", avctx->sample_rate );
	av_log(avctx, AV_LOG_DEBUG, "mcaac_encode_init() :   avctx->bit_rate = %"PRId64"\n", avctx->bit_rate );
	av_log(avctx, AV_LOG_DEBUG, "mcaac_encode_init() :   avctx->channels = %d\n", avctx->channels );
	av_log(avctx, AV_LOG_DEBUG, "mcaac_encode_init() :   avctx->frame_size = %d\n", avctx->frame_size );

	//sendAACParameterToJavaEncoder( int samplerate, int bitrate, int numchannels );
	sendAACParameterToJavaEncoder( avctx->sample_rate, avctx->bit_rate, avctx->channels /*, avctx->sample_fmt*/ );

	return ret;
}


/*
 * Copy input samples.
 * Channels are reordered from libavcodec's default order to AAC order.
 */
#if 0
static void copy_input_samples(AACEncContext *s, const AVFrame *frame)
{
    int ch;
    int end = 2048 + (frame ? frame->nb_samples : 0);
    const uint8_t *channel_map = s->reorder_map;

    /* copy and remap input samples */
    for (ch = 0; ch < s->channels; ch++) {
        /* copy last 1024 samples of previous frame to the start of the current frame */
        memcpy(&s->planar_samples[ch][1024], &s->planar_samples[ch][2048], 1024 * sizeof(s->planar_samples[0][0]));

        /* copy new samples and zero any remaining samples */
        if (frame) {
            memcpy(&s->planar_samples[ch][2048],
                   frame->extended_data[channel_map[ch]],
                   frame->nb_samples * sizeof(s->planar_samples[0][0]));
        }
        memset(&s->planar_samples[ch][end], 0,
               (3072 - end) * sizeof(s->planar_samples[0][0]));
    }
}
#endif

static int getSampleSize( AVCodecContext *avctx )
{
		int sample_size = 2;
		switch( avctx->sample_fmt ) {
		/* interleaved type */
    	case AV_SAMPLE_FMT_NONE :
			break;
    	case AV_SAMPLE_FMT_U8 :
			sample_size = sizeof(char);
			break;
    	case AV_SAMPLE_FMT_S16 :
			sample_size = sizeof(short);
			break;
    	case AV_SAMPLE_FMT_S32 :
			sample_size = sizeof(int32_t);
			break;
    	case AV_SAMPLE_FMT_FLT :
			sample_size = sizeof(float);
			break;
    	case AV_SAMPLE_FMT_DBL :
			sample_size = sizeof(double);
			break;

		/* plannar type */
    	case AV_SAMPLE_FMT_U8P :
			sample_size = sizeof(char);
			break;
    	case AV_SAMPLE_FMT_S16P :
			sample_size = sizeof(short);
			break;
    	case AV_SAMPLE_FMT_S32P :
			sample_size = sizeof(int32_t);
			break;
    	case AV_SAMPLE_FMT_FLTP :
			sample_size = sizeof(float);
			break;
    	case AV_SAMPLE_FMT_DBLP :
			sample_size = sizeof(double);
			break;
    	case AV_SAMPLE_FMT_S64 :
			sample_size = sizeof(int64_t);
			break;
    	case AV_SAMPLE_FMT_S64P :
			sample_size = sizeof(int64_t);
			break;
		}
	return sample_size;
}

static int mcaac_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                            const AVFrame *frame, int *got_packet_ptr)
{
    mediacodecAACContext *myCtx = avctx->priv_data;

	unsigned char *encData;
	int encLen;
	int64_t presentationTimeUs;
	int32_t b_keyframe;
	int ret = 0;

	*got_packet_ptr = 0;

	av_log(avctx, AV_LOG_DEBUG, "mcaac_encode_frame() : Enter ...\n");
	av_log(avctx, AV_LOG_DEBUG, "mcaac_encode_frame() :   avctx->sample_fmt = %d\n", avctx->sample_fmt );
	av_log(avctx, AV_LOG_DEBUG, "mcaac_encode_frame() :   frame->pts = %"PRId64"\n", frame->pts );	//by avctx->frame_size : 3840
	av_log(avctx, AV_LOG_DEBUG, "mcaac_encode_frame() :   frame->linesize[0] = %d\n", frame->linesize[0] );	//15360 = 3840 x 4

	if( frame ) {
		if( avctx->sample_fmt != AV_SAMPLE_FMT_S16 ) {
			av_log(avctx, AV_LOG_ERROR, "  I can handle only AV_SAMPLE_FMT_S16 !!!\n");
		}

		int sample_size = getSampleSize( avctx );

		#if 1
		//unsigned char *pNewData = convPlannerToInterleaved(frame);
		//sendPCMFrameToJavaEncoder( void *data, int length, uint64_t pts );
		sendPCMFrameToJavaEncoder(frame->extended_data[0], frame->linesize[0], frame->pts );
		//sendPCMFrameToJavaEncoder(frame->extended_data[0], avctx->frame_size, frame->pts );
		//free( pNewData );
		#else
		int loopCnt = frame->linesize[0]/avctx->frame_size;
		for( int i=0; i<loopCnt; i++ ) {
			sendPCMFrameToJavaEncoder(frame->extended_data[0]+(avctx->frame_size*i), avctx->frame_size, frame->pts );
		}
		#endif
	}
	#if 0
	if( getEncodedFrameAAC(&encData, &encLen, &presentationTimeUs, &b_keyframe) >= 0 ) {
		if ((ret = ff_alloc_packet2(avctx, avpkt, encLen, 0)) < 0)
        	return ret;

		memcpy( avpkt->data, encData, encLen );
		//avpkt->size = encLen;
		free( encData );

		#if 0	//fps 0.03 on ffplay
		//avpkt->pts = (int64_t)((double)presentationTimeUs * (double)avctx->time_base.den / 1000.0);
		//avpkt->pts = 99000 + (int64_t)((double)presentationTimeUs * (double)1000000 / (double)avctx->time_base.den);	//ffplay(O), VLC(X)
		//avpkt->pts = presentationTimeUs;	//ffplay(O), VLC(O)
		avpkt->pts = (int64_t)( (double)presentationTimeUs / (double)(1000000/(double)myCtx->fps) );
		//avpkt->pts = (int64_t)( (double)presentationTimeUs / (double)(90000/(double)myCtx->fps) );
		//avpkt->pts = frame->pts;
		//avpkt->pts = myCtx->pts;
		#else 	//fps 0.01 on ffplay
		AVRational bq; bq.num = 1; bq.den = 1000000; //AV_TIME_BASE;
		AVRational cq; cq.num = 1; cq.den = 90000;
		//avpkt->pts = av_rescale_q( presentationTimeUs, bq, cq );
		avpkt->pts = av_rescale_q( presentationTimeUs, bq, cq );	//3000, 6000, 9000
		av_log(avctx, AV_LOG_DEBUG, "  after av_rescale_q() : pavpkt->pts = %"PRId64"\n", avpkt->pts );
		//avpkt->pts = av_rescale_q( presentationTimeUs, bq, cq ) / (90000/(int)myCtx->fps);	//3000/3000, 6000/3000, 9000/3000
		//avpkt->pts = av_rescale_q_rnd( 1, avctx->time_base, cq, AV_ROUND_NEAR_INF|AV_ROUND_PASS_MINMAX );
		#endif
		avpkt->dts = avpkt->pts;

    	*got_packet_ptr = 1;

		av_log(avctx, AV_LOG_DEBUG, "  after getEncodedFrameAAC() data=%p encLen=%d presentationTimeUs=%" PRId64 " avctx->time_base.num|den=%d|%d avpkt->pts=%" PRId64 " frame->pts=%"PRId64" b_keyframe=%d\n", avpkt->data, encLen, presentationTimeUs, avctx->time_base.num, avctx->time_base.den, avpkt->pts, frame->pts, b_keyframe );

	}
	#else
	int ptIdx = 0;
	unsigned char *pt[4*2];
	int tmpEncLen[4*2];
	int totalLen = 0;
	int64_t pTimeUs[4*2];
	while( getEncodedFrameAAC(&encData, &encLen, &presentationTimeUs, &b_keyframe) >= 0 ) {
		pt[ptIdx] = encData;
		tmpEncLen[ptIdx] = encLen;
		totalLen += encLen;
		pTimeUs[ptIdx] = presentationTimeUs;
		ptIdx++;
	}
	av_log(avctx, AV_LOG_DEBUG, "  after the loop of getEncodedFrameAAC() : ptIdx = %d, totalLen = %d\n", ptIdx, totalLen );

	if( totalLen > 0 ) {
		unsigned char *newEncData = malloc( totalLen );
		int offset = 0;
		for( int i=0; i<ptIdx; i++ ) {
			memcpy( newEncData + offset, pt[i], tmpEncLen[i] );
			free( pt[i] );
			offset += tmpEncLen[i];
		}

		if ((ret = ff_alloc_packet2(avctx, avpkt, totalLen, 0)) < 0)
        	return ret;
		memcpy( avpkt->data, newEncData, totalLen );
		free(newEncData);

		AVRational bq; bq.num = 1; bq.den = 1000000; //AV_TIME_BASE;
		AVRational cq; cq.num = 1; cq.den = avctx->sample_rate; //90000;
		avpkt->pts = av_rescale_q( pTimeUs[ptIdx-1], bq, cq );
		//avpkt->pts = myCtx->pts;
		av_log(avctx, AV_LOG_DEBUG, "  after av_rescale_q() : pTimeUs[%d] = %"PRId64" pavpkt->pts = %"PRId64"\n", ptIdx-1, pTimeUs[ptIdx-1], avpkt->pts );
		avpkt->dts = avpkt->pts;

    	*got_packet_ptr = 1;

		av_log(avctx, AV_LOG_DEBUG, "  after getEncodedFrameAAC() data=%p totalLen=%d presentationTimeUs=%" PRId64 " avctx->time_base.num|den=%d|%d avpkt->pts=%" PRId64 " frame->pts=%"PRId64" b_keyframe=%d\n", avpkt->data, totalLen, presentationTimeUs, avctx->time_base.num, avctx->time_base.den, avpkt->pts, frame->pts, b_keyframe );
	}
	#endif

	if( frame )
		myCtx->pts = frame->pts;
    return 0;
}

static unsigned char * convertToS16( const AVFrame *frame )
{
	return NULL;
}


#if 0
static unsigned char * convPlannerToInterleaved( AVFrame *frame )
{
	unsigned char *pNewData = malloc( sizeof(float) * frame->channels * frame->nb_samples );
	for( int s=0; s < frame->nb_samples; s++ ) {
		for( int c=0; c < frame->channels; c++ ) {
			//memcpy( 
		}
	}
}
#endif

static av_cold int mcaac_encode_close(AVCodecContext *avctx)
{
    mediacodecAACContext *myCtx = avctx->priv_data;

	av_log(avctx, AV_LOG_DEBUG, "mcaac_encode_close() : Enter ...\n" );

/*
    av_log(avctx, AV_LOG_INFO, "Qavg: %.3f\n", s->lambda_sum / s->lambda_count);

    ff_mdct_end(&s->mdct1024);
    ff_mdct_end(&s->mdct128);
    ff_psy_end(&s->psy);
    ff_lpc_end(&s->lpc);
    if (s->psypp)
        ff_psy_preprocess_end(s->psypp);
    av_freep(&s->buffer.samples);
    av_freep(&s->cpe);
    av_freep(&s->fdsp);
    ff_af_queue_close(&s->afq);
*/
	mediacodecAAC_release();	//call ffmpeg_jni.c :: mediacodecAAC_release()
    return 0;
}

static const AVClass aacenc_class = {
    .class_name = "AAC encoder",
    .item_name  = av_default_item_name,
    //.option     = aacenc_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

static const AVCodecDefault aac_encode_defaults[] = {
    { "b", "0" },
    { NULL }
};

AVCodec ff_mcaac_encoder = {
    .name           = "mcaac",
    .long_name      = NULL_IF_CONFIG_SMALL("AAC (Advanced Audio Coding)"),
    .type           = AVMEDIA_TYPE_AUDIO,
    .id             = AV_CODEC_ID_AAC,
    .priv_data_size = sizeof(mediacodecAACContext),
    .init           = mcaac_encode_init,
    .encode2        = mcaac_encode_frame,
    .close          = mcaac_encode_close,
#if 0
    .defaults       = aac_encode_defaults,
    .supported_samplerates = mpeg4audio_sample_rates,
    .caps_internal  = FF_CODEC_CAP_INIT_THREADSAFE,
    .capabilities   = AV_CODEC_CAP_SMALL_LAST_FRAME | AV_CODEC_CAP_DELAY,
    .sample_fmts    = (const enum AVSampleFormat[]){ AV_SAMPLE_FMT_FLTP,
                                                     AV_SAMPLE_FMT_NONE },
#else
    .sample_fmts    = (const enum AVSampleFormat[]){ AV_SAMPLE_FMT_S16,
                                                     AV_SAMPLE_FMT_NONE },
#endif
    .priv_class     = &aacenc_class,
};
