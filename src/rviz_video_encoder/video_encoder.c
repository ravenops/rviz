#include <stdint.h>
#include <time.h>

#include <libswscale/swscale.h>
#include <libavcodec/avcodec.h>
#include <libavutil/common.h>
#include <libavformat/avformat.h>
#include <libavutil/timestamp.h>
#include <libavutil/imgutils.h>

#include "xxhash.h"
#include "video_encoder.h"

struct _video_encoder {
    VideoEncodeParams params;
    int out_width,out_height;

    // video encoding
    AVCodecContext *cc;
    // output muxing
    AVFormatContext *oc;

    struct SwsContext *sws;

    AVFrame *frame_raw,*frame_scaled;

    AVPacket pbuf;

    AVStream *st;

    uint32_t num_frames,skipped_frames;

    // for keyed capture
    long long unsigned last_frame_hash;

    // frame rate stats
    time_t stat_last_frame;
    time_t stat_first_frame;
    time_t stat_interval_frame;
    uint64_t stat_frame_count;
    uint16_t stat_interval_period;
};

VideoEncoder* video_encoder_init(VideoEncodeParams params){

    VideoEncoder *enc = (VideoEncoder*) malloc(sizeof(VideoEncoder));
    if ( enc == NULL )
    {
        fprintf(stderr,"could not allocate VideoEncoder");
        return NULL;
    }
    memset(enc,0,sizeof(VideoEncoder));
    // RVN:TODO: hardcoded for now
    enc->stat_interval_period = 16;
    params.inFmt = AV_PIX_FMT_RGB32;

    enc->params = params;
    if (enc->params.width % 8 || enc->params.height % 8){
        fprintf(stderr,"%dx%d, dimensions not divisible by 8!\n",enc->params.width,enc->params.height);
        return NULL;
    }
    if (enc->params.width > enc->params.maxWidth) {
        printf("[video encoder] WARN max width exceeded, scaling down to max width %dpx\n",enc->params.maxWidth);
        float aspectRatio = (float) enc->params.width / (float) enc->params.height;
        enc->out_width = enc->params.maxWidth;
        enc->out_height = (int)( (float) enc->out_width / aspectRatio);
    }else{
        enc->out_width = enc->params.width;
        enc->out_height = enc->params.height;
    }
    enc->out_width -= enc->out_width % 8;
    enc->out_height -= enc->out_height % 8;

    printf("[video encoder init] %d x %d --> %d x %d @ %f FPS\n",
	   enc->params.width,enc->params.height,
	   enc->out_width,enc->out_height,
	   (double) enc->params.fpsNum / (double) enc->params.fpsDen);

    // init scaled input buffer frame
    enc->frame_scaled = av_frame_alloc();
    enc->frame_scaled->width = enc->out_width;
    enc->frame_scaled->height = enc->out_height;
    enc->frame_scaled->format = AV_PIX_FMT_YUV420P;

    enc->frame_raw = av_frame_alloc();
    enc->frame_raw->width = enc->params.width;
    enc->frame_raw->height = enc->params.height;
    enc->frame_raw->format = enc->params.inFmt;
    int ret;
    ret = av_frame_get_buffer(enc->frame_scaled,32);
    if (ret < 0) {
        fprintf(stderr, "Could not allocate scaled frame data: %s\n",av_err2str(ret));
        return NULL;
    }
    ret = av_frame_get_buffer(enc->frame_raw,32);
    if (ret < 0) {
        fprintf(stderr, "Could not allocate raw frame data: %s\n",av_err2str(ret));
        return NULL;
    }

    // set up sws context
    enc->sws = sws_getContext(enc->params.width, enc->params.height, enc->params.inFmt,
			      enc->out_width, enc->out_height, AV_PIX_FMT_YUV420P,
			      SWS_FAST_BILINEAR, NULL, NULL, NULL);

    enum AVCodecID codec_id = AV_CODEC_ID_H264;

    AVCodec *codec = avcodec_find_encoder_by_name(enc->params.h264Encoder);
    if (!codec){
        fprintf(stderr,"Could not find h264 codec: %d %s\n",codec_id,enc->params.h264Encoder);
        return NULL;
    }

    // output format context
    ret = avformat_alloc_output_context2(&enc->oc,NULL,NULL,enc->params.output_path);
    if (!enc->oc){
        fprintf(stderr,"could not alloc output context: %s\n", av_err2str(ret));
        return NULL;
    }
    ret = avio_open(&enc->oc->pb, enc->params.output_path, AVIO_FLAG_WRITE);
    if (ret < 0) {
        fprintf(stderr, "could not open output file '%s': %s\n", enc->params.output_path, av_err2str(ret));
        return NULL;
    }

    enc->st = avformat_new_stream(enc->oc,NULL);
    enc->st->id = enc->oc->nb_streams-1;

    enc->cc = avcodec_alloc_context3(codec);
    if(!enc->cc){
        fprintf(stderr,"failed to allocate context for codec: %d\n",codec_id);
        return NULL;
    }

    // set video encoder params
    enc->cc->codec_id = codec_id;
    enc->cc->width = enc->out_width;
    enc->cc->height = enc->out_height;
    enc->st->time_base = (AVRational){enc->params.fpsDen,enc->params.fpsNum};
    enc->cc->time_base = enc->st->time_base;
    enc->cc->pix_fmt = enc->frame_scaled->format;

    // RVN:TODO this is good neighboor mode. probably should have drag race mode
    //enc->cc->thread_count = 1;

    if (enc->oc->oformat->flags & AVFMT_GLOBALHEADER)
        enc->cc->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    // open the codec
    AVDictionary *opt = NULL;
    av_dict_copy(&opt, NULL, 0);

    if(strcmp(enc->params.h264Encoder,"libx264") == 0){
        //crf
        char crf_str[5];
        sprintf(crf_str,"%.1f",enc->params.crf);
        av_dict_set(&opt, "crf", crf_str, 0);

        //preset
        av_dict_set(&opt, "preset", enc->params.preset, 0);
        av_dict_set(&opt, "profile", "baseline", 0);

    }else if(strcmp(enc->params.h264Encoder,"h264_nvenc") == 0){
        // top level
        //av_dict_set_int(&opt,"2pass",1,0);
        av_dict_set(&opt,"preset","lossless",0);
        av_dict_set(&opt,"profile","high",0);

        /* // rate control */
        /* /\* av_dict_set(&opt,"rc","vbr",0); *\/ */
        av_dict_set_int(&opt,"rc-lookahead",64,0);


        //fancies
        av_dict_set_int(&opt,"spatial-aq",1,0);
        av_dict_set_int(&opt,"temporal-aq",1,0);
        av_dict_set_int(&opt,"aq-strength",15,0);

        //don't need
        av_dict_set_int(&opt,"no-scenecut",1,0);
    }else{
        fprintf(stderr,"Unsupported encoder name: %s\n",enc->params.h264Encoder);
        return NULL;
    }

    /* open the codec */
    ret = avcodec_open2(enc->cc, codec, &opt);
    av_dict_free(&opt);
    if (ret < 0) {
        fprintf(stderr, "Could not open video codec: %s\n", av_err2str(ret));
        return NULL;
    }

    // copy stream params to muxer
    ret = avcodec_parameters_from_context(enc->st->codecpar, enc->cc);
    if (ret < 0) {
        fprintf(stderr, "Could not copy the stream parameters: %s\n",av_err2str(ret));
        return NULL;
    }

    AVDictionary *mp4opt = NULL;
    av_dict_copy(&mp4opt,NULL,0);

    //faststart (enable second pass to put MOOV atom at beginning of file
    //allow playback to start before file download is complete, streaming)
    av_dict_set(&mp4opt, "movflags","faststart", 0);

    fprintf(stdout,"Writing header\n");
    ret = avformat_write_header(enc->oc,&mp4opt);
    if (ret < 0){
        fprintf(stderr," error writing output file header: %s\n",av_err2str(ret));
        return NULL;
    }
    av_dict_free(&mp4opt);

    // init bookeeping
    enc->num_frames = 0;

    fprintf(stdout,"[video encoder] initialized!\n");
    return enc;

}

static int write_frame(AVFormatContext *fmt_ctx, const AVRational *time_base, AVStream *st, AVPacket *pkt)
{
    av_packet_rescale_ts(pkt, *time_base, st->time_base);
    pkt->stream_index = st->index;

    return av_interleaved_write_frame(fmt_ctx, pkt);
}

static int video_encoder_send_frame(VideoEncoder *enc, AVFrame *frame){
    int ret;
    ret = avcodec_send_frame(enc->cc,frame);
    if (ret < 0){
        fprintf(stderr, "error encoding video frame: %s\n", av_err2str(ret));
        return ret;
    }

    while (1)
    {
        av_init_packet(&enc->pbuf);
        enc->pbuf.data = NULL;
        enc->pbuf.size = 0;

        ret = avcodec_receive_packet(enc->cc,&enc->pbuf);
        if (ret < 0){
            if (ret == AVERROR_EOF) {
                fprintf(stdout,"[video encoder] EOF\n");
                break;
            }
            if (ret == AVERROR(EAGAIN)){
                break;
            }

            fprintf(stderr,"error receiving packet from encoder: %s\n",av_err2str(ret));
            return ret;
        }
        ret = write_frame(enc->oc, &enc->cc->time_base, enc->st, &enc->pbuf);
        if (ret < 0) {
            fprintf(stderr, "error while writing video frame to transport stream: %s\n", av_err2str(ret));
            return ret;
        }
        av_packet_unref(&enc->pbuf);
    }
    return 0;
}

int video_encoder_encode_frame(VideoEncoder *enc, const uint8_t *pixels){
    // this is (the only!!!) current system time for the duration of this function
    time_t now = time(NULL);

    // begin atomic book-keeping update (if ever parallelized)
    enc->stat_last_frame = now;
    if (enc->stat_frame_count){
        // print frame-rate stats

        if(enc->stat_frame_count % enc->stat_interval_period == 0){

            // summary printout
            double ifps = difftime(enc->stat_last_frame,enc->stat_interval_frame);
            double cfps = difftime(enc->stat_last_frame,enc->stat_first_frame);

            if(ifps > 0.0){
                ifps = ((double) enc->stat_interval_period) / ifps;
            }else{
                fprintf(stderr,"rviz_ve: non-positive time elapsed in interval %ld -> %ld\n",enc->stat_first_frame,enc->stat_interval_frame);
            }

            if(cfps > 0.0){
                cfps = ((double) enc->stat_frame_count) / cfps;
            }else{
                fprintf(stderr,"rviz_ve: non-positive time elapsed overall %ld -> %ld\n",enc->stat_first_frame,enc->stat_last_frame);
            }

            const char* pfx = " ";
            if ( enc->params.keyed ){
                pfx = " [keyed] ";
            }
            //            fprintf(stdout,"rviz_ve:%s[ %lu ] FPS: interval= %.4f Hz  cum= %.4f Hz\n",pfx,enc->stat_frame_count, ifps,cfps);
            //fflush(stdout);
            //fflush(stderr);
            // stat state
            enc->stat_interval_frame = now;
        }
    }else{
        // first frame
        enc->stat_first_frame = now;
        enc->stat_interval_frame = now;
    }

    enc->stat_frame_count+=1;
    // end atomic book-keeping update

    int bytesFilled = av_image_fill_arrays(enc->frame_raw->data,
                                           enc->frame_raw->linesize,
                                           pixels,
                                           enc->frame_raw->format,
                                           enc->frame_raw->width,enc->frame_raw->height,32);
    if(bytesFilled < 0){
       fprintf(stderr,"Could not fill raw picture bytes: %s\n",av_err2str(bytesFilled));
       return -1;
    }

    if (enc->params.keyed){
        unsigned long long const frame_hash = XXH64((void*) pixels, bytesFilled, 0);
        if (enc->last_frame_hash == frame_hash){
            // no change since last frame. skip
            enc->skipped_frames++;
            return 0;
        }

        enc->last_frame_hash = frame_hash;
    }

    enc->frame_scaled->pts = enc->num_frames++;
    int ret = sws_scale(enc->sws,
              (const uint8_t * const *) enc->frame_raw->data,
	      enc->frame_raw->linesize,
              0,
              enc->params.height,
              enc->frame_scaled->data,
              enc->frame_scaled->linesize
              );
    if (ret < 0){
        fprintf(stderr,"failed to scale frame: %s\n",av_err2str(ret));
        return ret;
    }
    return video_encoder_send_frame(enc,enc->frame_scaled);
}

void video_encoder_destroy(VideoEncoder *enc)
{
    if(enc){
        fprintf(stdout,"Encode finished!: %d frames encoded, %d frames skipped\n",enc->num_frames, enc->skipped_frames);
	if(enc->sws){
            sws_freeContext(enc->sws);
        }
        if (enc->cc){
            // flush codec
            video_encoder_send_frame(enc,NULL);
            avcodec_free_context(&enc->cc);
        }
        if (enc->frame_scaled){
            av_frame_free(&enc->frame_scaled);
        }
        if (enc->frame_raw){
            av_frame_free(&enc->frame_raw);
        }

        if(enc->oc){
            av_write_trailer(enc->oc);
            if (enc->oc->pb){
                avio_closep(&enc->oc->pb);
            }
            avformat_free_context(enc->oc);
        }
	free(enc);
    }
}
