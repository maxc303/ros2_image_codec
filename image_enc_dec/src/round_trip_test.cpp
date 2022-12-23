#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>

#include <iostream>
int main() {
  const AVCodec *codec;
  AVCodecContext *c = NULL;

  AVFrame *frame;
  AVPacket *pkt;
}