
// Need to warp ffmpeg headers into extern "C"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}
#include <iostream>
#include <string>
int main() {
  const AVCodec *codec;
  AVCodecContext *c = NULL;

  AVFrame *frame;
  AVPacket *pkt;

  std::string codec_name = "h264_nvenc";
  codec = avcodec_find_encoder_by_name(codec_name.c_str());
  if (!codec) {
    fprintf(stderr, "Codec '%s' not found\n", codec_name.c_str());
    exit(1);
  }
}