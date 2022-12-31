
// Need to warp ffmpeg headers into extern "C"
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

#include <boost/program_options.hpp>

#include "NvEncoder/NvEncoderCuda.h"
#include "NvEncoderCLIOptions.h"
// #include <filesystem>
// #include <iostream>
// #include <opencv4/opencv2/opencv.hpp>
// #include <opencv4/opencv2/quality.hpp>
// #include <string>

// namespace po = boost::program_options;
int main(int argc, char** argv) {
  CUcontext cuContext = NULL;
  CUdevice cuDevice = 0;
  int iGpu = 0;

  // cuDeviceGet(&cuDevice, iGpu);
  // char szDeviceName[80];
  // ck(cuDeviceGetName(szDeviceName, sizeof(szDeviceName), cuDevice));
  // std::cout << "GPU in use: " << szDeviceName << std::endl;
  // ck(cuCtxCreate(&cuContext, 0, cuDevice));
  // NV_ENC_BUFFER_FORMAT eFormat = NV_ENC_BUFFER_FORMAT_IYUV;

  // NvEncoderCuda enc(cuContext, 1600, 900, eFormat, 0);

  return 0;
}