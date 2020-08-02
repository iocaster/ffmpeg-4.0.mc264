#include "libavutil/eval.h"
#include "libavutil/internal.h"
#include "libavutil/opt.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavutil/stereo3d.h"
#include "libavutil/intreadwrite.h"
#include "avcodec.h"
#include "internal.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>	//for usleep()
#include <sys/time.h>

extern int sendParameterToJavaEncoder( int w, int h, int bitrate, float framerate, int gop, int colorformat  );	//ffmpeg_jni.c
extern int sendFrameToJavaEncoder( void *data, int length, int stride, uint64_t pts );							//ffmpeg_jni.c
extern int getEncodedFrame( void **data, int *length, int64_t *presentationTimeUs, int32_t *b_keyframe );		//ffmpeg_jni.c
extern int mediacodec_release(void);																			//ffmpeg_jni.c

static unsigned char * convI420toAsIs( const AVFrame *frame );
static unsigned char * convI420toNV12Semi( const AVFrame *frame );
static unsigned char * convI420toNV12PackedSemi( const AVFrame *frame );


typedef struct mediacodec264Context {
	AVClass	*class;
	float fps;
	uint64_t frameIdx;
    int64_t pts;
	uint8_t b_close;
	struct timeval startTime;
} mediacodec264Context;

static av_cold int mediacodec264_init(AVCodecContext *avctx)
{
	mediacodec264Context *myCtx = avctx->priv_data;
	myCtx->fps = (float)( (float)avctx->time_base.den / (float)avctx->time_base.num );	//25/1 or 1199/50
	myCtx->frameIdx = 0;
	myCtx->pts = 0;
	myCtx->b_close = 0;
	struct timeval curTime;
	gettimeofday(&curTime, 0);
	myCtx->startTime.tv_sec = curTime.tv_sec ;
	myCtx->startTime.tv_usec =  curTime.tv_usec;

	av_log(avctx, AV_LOG_DEBUG, "mediacodec264_init() : Enter ...\n" );
	av_log(avctx, AV_LOG_DEBUG, "  avctx->profile = %d(0x%x)\n", avctx->profile, avctx->profile );
	av_log(avctx, AV_LOG_DEBUG, "  avctx->width x height = %d x %d\n", avctx->width, avctx->height );
	av_log(avctx, AV_LOG_DEBUG, "  avctx->bit_rate = %lld\n", avctx->bit_rate );
	av_log(avctx, AV_LOG_DEBUG, "  avctx->gop_size = %d\n", avctx->gop_size );
	av_log(avctx, AV_LOG_DEBUG, "  avctx->time_base.num, den=(%d, %d)\n", avctx->time_base.num, avctx->time_base.den ); //1,25 or 50,1199

	//avctx->profile = FF_PROFILE_H264_CONSTRAINED_BASELINE;
	//av_log(avctx, AV_LOG_DEBUG, "  new avctx->profile = %d(0x%x)\n", avctx->profile, avctx->profile );


	sendParameterToJavaEncoder( avctx->width, avctx->height
				, avctx->bit_rate
				, (float)((float)avctx->time_base.den / (float)avctx->time_base.num)	//25/1 or 1199/50
				, avctx->gop_size
				, avctx->pix_fmt );

	//usleep(500000);
	return 0;
}

static int mediacodec264_frame(AVCodecContext *avctx, AVPacket *pkt, const AVFrame *frame, int *got_packet)
{
	mediacodec264Context *myCtx = avctx->priv_data;

	unsigned char *encData;
	int encLen;
	int64_t presentationTimeUs;
	int32_t b_keyframe;
	int ret;

	*got_packet = 0;

	struct timeval curTime;
	gettimeofday(&curTime, 0);
	if( curTime.tv_sec - myCtx->startTime.tv_sec >= 1800) {
		av_log(avctx, AV_LOG_ERROR, "mc264 : Error, time out...  commertial version is required !\n" );
		return 0;
	}


	av_log(avctx, AV_LOG_DEBUG, "mediacodec264_frame() : Enter ...\n" );

	if( myCtx->b_close ) return 0;

	if( frame ) {
/*
		av_log(avctx, AV_LOG_DEBUG, "    width x height = %d x %d\n", frame->width, frame->height );
		av_log(avctx, AV_LOG_DEBUG, "    linesize[0,1,2] = %d, %d, %d\n", frame->linesize[0], frame->linesize[1], frame->linesize[2]); 
		av_log(avctx, AV_LOG_DEBUG, "    format = %d\n", frame->format );
		av_log(avctx, AV_LOG_DEBUG, "    pts = %lld\n", frame->pts );
		av_log(avctx, AV_LOG_DEBUG, "    key_frame = %d %s\n", frame->key_frame, frame->key_frame ? "------------------------ KEY_FRAME": "" );
		av_log(avctx, AV_LOG_DEBUG, "  pkt->buf = %p\n", pkt->buf );
		av_log(avctx, AV_LOG_DEBUG, "  pkt->pts = %lld\n", pkt->pts );
		av_log(avctx, AV_LOG_DEBUG, "  pkt->dts = %lld\n", pkt->dts );
		av_log(avctx, AV_LOG_DEBUG, "  pkt->data = %p\n", pkt->data );
		av_log(avctx, AV_LOG_DEBUG, "  pkt->size = %d\n", pkt->size );
		av_log(avctx, AV_LOG_DEBUG, "  pkt->stream_index = %d\n", pkt->stream_index );
		av_log(avctx, AV_LOG_DEBUG, "  pkt->duration = %lld\n", pkt->duration );
		av_log(avctx, AV_LOG_DEBUG, "  pkt->pos = %lld\n", pkt->pos );
*/

		//sendFrameToJavaEncoder(frame->data[0], frame->linesize[0] * frame->height * 3 / 2 );
		//sendFrameToJavaEncoder(*(frame->extended_data), frame->linesize[0] * frame->height * 3 / 2 );
		//sendFrameToJavaEncoder(&frame->data[0][0], frame->linesize[0] * frame->height * 3 / 2 );

		//unsigned char *pNewData = convI420toAsIs( frame );
		unsigned char *pNewData = convI420toNV12Semi( frame );
		//unsigned char *pNewData = convI420toNV12PackedSemi( frame );
	av_log(avctx, AV_LOG_DEBUG, "mediacodec264_frame() : linesize[0,1,2 = %d %d %d ...\n", frame->linesize[0], frame->linesize[1], frame->linesize[2]    );
		sendFrameToJavaEncoder(pNewData, frame->linesize[0] * frame->height * 3 / 2, frame->linesize[0], frame->pts );
		free(pNewData);

	}

	if( myCtx->b_close ) return 0;

	if( getEncodedFrame(&encData, &encLen, &presentationTimeUs, &b_keyframe) >= 0 ) {
		if ((ret = ff_alloc_packet2(avctx, pkt, encLen, 0)) < 0)
        	return ret;

		memcpy( pkt->data, encData, encLen );
		//pkt->size = encLen;
		free( encData );

		#if 0	//fps 0.03 on ffplay
		//pkt->pts = (int64_t)((double)presentationTimeUs * (double)avctx->time_base.den / 1000.0);
		//pkt->pts = 99000 + (int64_t)((double)presentationTimeUs * (double)1000000 / (double)avctx->time_base.den);	//ffplay(O), VLC(X)
		//pkt->pts = presentationTimeUs;	//ffplay(O), VLC(O)
		pkt->pts = (int64_t)( (double)presentationTimeUs / (double)(1000000/(double)myCtx->fps) );
		//pkt->pts = (int64_t)( (double)presentationTimeUs / (double)(90000/(double)myCtx->fps) );
		//pkt->pts = frame->pts;
		//pkt->pts = myCtx->pts;
		#else 	//fps 0.01 on ffplay
		AVRational bq; bq.num = 1; bq.den = 1000000; //AV_TIME_BASE;
		AVRational cq; cq.num = 1; cq.den = 90000;
		//pkt->pts = av_rescale_q( presentationTimeUs, bq, cq );
		//pkt->pts = av_rescale_q( presentationTimeUs, bq, cq );	//3000, 6000, 9000
		pkt->pts = av_rescale_q( presentationTimeUs, bq, cq ) / (90000/(int)myCtx->fps);	//3000/3000, 6000/3000, 9000/3000
		//pkt->pts = av_rescale_q_rnd( 1, avctx->time_base, cq, AV_ROUND_NEAR_INF|AV_ROUND_PASS_MINMAX );
		//pkt->pts = myCtx->pts;
		#endif
		pkt->dts = pkt->pts;

		int pict_type = AV_PICTURE_TYPE_NONE;
    	//pkt->flags |= AV_PKT_FLAG_KEY*b_keyframe;
		if( b_keyframe ) {
			//avctx->coded_frame->pict_type = AV_PICTURE_TYPE_I;
			pict_type = AV_PICTURE_TYPE_I;
			pkt->flags |= AV_PKT_FLAG_KEY;
		} else {
			//avctx->coded_frame->pict_type = AV_PICTURE_TYPE_P;
			pict_type = AV_PICTURE_TYPE_P;
		}

		//ff_side_data_set_encoder_stats(pkt, (pic_out.i_qpplus1 - 1) * FF_QP2LAMBDA, NULL, 0, pict_type);
		ff_side_data_set_encoder_stats(pkt, 18 * FF_QP2LAMBDA, NULL, 0, pict_type);

		*got_packet = 1;

		av_log(avctx, AV_LOG_DEBUG, "  after getEncodedFrame() data=%p encLen=%d presentationTimeUs=%" PRId64 " avctx->time_base.num|den=%d|%d pkt->pts=%" PRId64 " frame->pts=%"PRId64" b_keyframe=%d\n", pkt->data, encLen, presentationTimeUs, avctx->time_base.num, avctx->time_base.den, pkt->pts, frame->pts, b_keyframe );

		myCtx->frameIdx++;
	}

	if( frame )
		myCtx->pts = frame->pts;


	return 0;
}

/*
 * refer to : https://m.blog.naver.com/PostView.nhn?blogId=ein0204&logNo=220384865210&proxyReferer=https%3A%2F%2Fwww.google.com%2F
 * NV12 종류 : (LG Q6)
 * COLOR_FormatYUV420SemiPlanar
 * COLOR_FormatYUV420PackedSemiPlanar
 * COLOR_TI_FormatYUV420PackedSemiPlanar
 *
 * I420 종류 : (ffmpeg, 삼성의 최신폰)
 * COLOR_FormatYUV420Planar
 * COLOR_FormatYUV420PackedPlanar
 *
 * For a single I420 pixel : YYYYYYYY UU VV
 * For a single NV12 pixel : YYYYYYYY UVUV
 *
 */
static unsigned char * convI420toAsIs( const AVFrame *frame )
{
	unsigned char *pNewData = malloc( frame->linesize[0] * frame->height * 3 / 2 );
	if( pNewData ) {
		unsigned char *pt = pNewData;
		//copy Y
		memcpy( pt, frame->data[0], frame->linesize[0] * frame->height );
		pt += frame->linesize[0] * frame->height;
		//copy U
		memcpy( pt, frame->data[1], frame->linesize[1] * frame->height / 2 );
		pt += frame->linesize[1] * frame->height / 2;
		//copy V
		memcpy( pt, frame->data[2], frame->linesize[2] * frame->height / 2 );
		pt += frame->linesize[2] * frame->height / 2;

	}
	return pNewData;
}

static unsigned char * convI420toNV12Semi( const AVFrame *frame )
{
	unsigned char *pNewData = malloc( frame->linesize[0] * frame->height * 3 / 2 );
	if( pNewData ) {
		unsigned char *pt = pNewData;
		//copy Y
		memcpy( pt, frame->data[0], frame->linesize[0] * frame->height );
		pt += frame->linesize[0] * frame->height;

		//copy U/V
		for( int y=0; y<(frame->height/2); y++ ) {
			for( int x=0; x<frame->linesize[1]; x++ ) {
				*pt++ = (unsigned char) frame->data[1][frame->linesize[1]*y + x];
				*pt++ = (unsigned char) frame->data[2][frame->linesize[1]*y + x];
			}
		}
	}
	return pNewData;
}

/* packed plane is a single plane */
static unsigned char * convI420toNV12PackedSemi( const AVFrame *frame )
{
	unsigned char *pNewData = malloc( frame->linesize[0] * frame->height * 3 / 2 );
	if( pNewData ) {
		unsigned char *pt = pNewData;
		for( int y=0; y<(frame->height); y++ ) {
			for( int x=0; x<frame->linesize[0]; x++ ) {
				*pt++ = (unsigned char) frame->data[0][x];
				if( x%2 == 0 ) {
					*pt++ = (unsigned char) frame->data[1][x/2];
				}
				if( x%2 == 1 ) {
					*pt++ = (unsigned char) frame->data[2][x/2];
				}
			}
		}
	}
	return pNewData;
}

static av_cold int mediacodec264_close(AVCodecContext *avctx)
{
	mediacodec264Context *myCtx = avctx->priv_data;
	av_log(avctx, AV_LOG_DEBUG, "mediacodec264_close() : Enter ...\n" );

	myCtx->b_close = 1;

	mediacodec_release();	//call ffmpeg_jni.c :: mediacodec_release()

	return 0;
}


static const AVClass mediacodec264_class = {
	.class_name = "mc264",
	.item_name  = av_default_item_name,
	//.option 	= options,
	.version 	= LIBAVUTIL_VERSION_INT,
};

AVCodec ff_mc264_encoder = {
	.name				= "mc264",
	.long_name			= NULL_IF_CONFIG_SMALL("mc264 H.264 / AVC"),
	.type 				= AVMEDIA_TYPE_VIDEO,
	.id					= AV_CODEC_ID_H264,
	.priv_data_size		= sizeof(mediacodec264Context),
	.init 				= mediacodec264_init,
	.encode2 			= mediacodec264_frame,
	.close 				= mediacodec264_close,
	.capabilities 		= AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AUTO_THREADS,
	.priv_class 		= &mediacodec264_class,
	//.defaults         = x264_defaults,
	//.init_static_data = X264_init_static,
	.caps_internal 		= FF_CODEC_CAP_INIT_THREADSAFE |
							FF_CODEC_CAP_INIT_CLEANUP,
	.wrapper_name 		= "mc264",
	
};


