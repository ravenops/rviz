#ifndef RVIZ_VIDEO_ENCODER_H
#define RVIZ_VIDEO_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif
struct _video_encoder;

typedef struct _video_encoder VideoEncoder;

typedef struct {
    int width;
    int height;
    int fpsNum;
    int fpsDen;
    int maxWidth;
    int keyed;

    // constant rate factor metric
    double crf;

    // speed/quality tradeoff [slowest ... fastest]
    const char* preset;

    // AVPixelFormat enum. Is hardcoded (for now) to RGB32, as this will always be what Ogre gives us.
    int inFmt;

    // path to output file: ie: /a/b/c/captured.mp4
    // will auto-detect container format based on file extension.
    const char* output_path;

    // name of libav codec to use
    const char* h264Encoder;

} VideoEncodeParams;

VideoEncoder* video_encoder_init(VideoEncodeParams args);
int video_encoder_encode_frame(VideoEncoder *enc, const uint8_t *pixels);
void video_encoder_destroy(VideoEncoder *enc);


#ifdef __cplusplus
}
#endif

#endif /* RVIZ_VIDEO_ENCODER_H */
