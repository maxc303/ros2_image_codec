
// Need to warp ffmpeg headers into extern "C"
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

#include <boost/program_options.hpp>
#include <filesystem>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/quality.hpp>
#include <string>

#include "NvDecoder/NvDecoder.h"
#include "NvEncoder/NvEncoderCuda.h"
#include "NvEncoderCLIOptions.h"

namespace po = boost::program_options;
simplelogger::Logger* logger =
    simplelogger::LoggerFactory::CreateConsoleLogger();

int main(int argc, char** argv) {
  spdlog::cfg::load_env_levels();

  // Parse test infomration
  po::options_description desc(
      "FFmpeg API encoding and decoding round trip test. The program will try "
      "to encode the images from input directory and decode the packets back "
      "to images. Quality Loss of each image will be calculated. If an output "
      "directory is provided, the decoded image will be save to that "
      "directory.");
  desc.add_options()("help", "produce help message")  //
      ("input_dir", po::value<std::string>(),
       "input images dir. Only .png and .jpg images will be used.")  //
      ("output_dir", po::value<std::string>(), "output images dir")  //
      ("encoder", po::value<std::string>()->default_value("libx264"),
       "FFmpeg encoder name")  //
      ("decoder", po::value<std::string>()->default_value("h264"),
       "FFmpeg decoder name")  //
      ("num_images", po::value<int>()->default_value(10),
       "number of images to be used in this test")  //
      ("bit_rate", po::value<int>()->default_value(5000000),
       "encoder bit rate")  //
      ("gop_size", po::value<int>()->default_value(10),
       "Encoder Group of Pictures size.  Emit one intra frame in each group.");

  po::variables_map boost_args;
  po::store(po::parse_command_line(argc, argv, desc), boost_args);
  po::notify(boost_args);

  if (boost_args.count("help") != 0 || boost_args.count("input_dir") == 0) {
    std::cout << desc << std::endl;
    return 1;
  }

  std::filesystem::path output_dir;
  bool save_output = false;
  if (boost_args.count("output_dir") == 0) {
    std::cout << "output_dir is not provided. No decoded image will be saved."
              << std::endl;
  } else {
    output_dir = boost_args["output_dir"].as<std::string>();
    save_output = true;
  }

  // Parse input
  std::filesystem::path input_dir = boost_args["input_dir"].as<std::string>();
  int max_num_images = boost_args["num_images"].as<int>();
  std::string encoder_name = boost_args["encoder"].as<std::string>();
  std::string decoder_name = boost_args["decoder"].as<std::string>();
  int bit_rate = boost_args["bit_rate"].as<int>();
  int gop_size = boost_args["gop_size"].as<int>();
  spdlog::info("Using encoder:{} and decoder:{}.", encoder_name, decoder_name);

  // Get the image path for the test.
  std::vector<std::filesystem::path> image_paths;
  for (const auto& entry : std::filesystem::directory_iterator(input_dir)) {
    if (entry.path().extension() == ".jpg" ||
        entry.path().extension() == ".png") {
      image_paths.push_back(entry.path());
    }
    if (image_paths.size() >= max_num_images) {
      break;
    }
  }
  if (image_paths.size() < 1) {
    spdlog::error("No image is found in the provided path: {}",
                  input_dir.string());
    return 1;
  }

  // Sort the path alphabetically
  std::sort(image_paths.begin(), image_paths.end());
  spdlog::info("Test image path (num = {}): ", image_paths.size());
  for (int idx = 0; idx < image_paths.size(); idx++) {
    spdlog::info("{}", image_paths[idx].string());
  }

  cv::Mat init_image = imread(image_paths[0], cv::IMREAD_COLOR);
  int img_width = init_image.size().width;
  int img_height = init_image.size().height;

  //========== ========== ========== ========== ==========
  // Encoding
  //========== ========== ========== ========== ==========

  int iGpu = 0;
  ck(cuInit(0));
  int nGpu = 0;
  ck(cuDeviceGetCount(&nGpu));
  if (iGpu < 0 || iGpu >= nGpu) {
    std::cout << "GPU ordinal out of range. Should be within [" << 0 << ", "
              << nGpu - 1 << "]" << std::endl;
    return 1;
  }
  CUdevice cuDevice = 0;
  ck(cuDeviceGet(&cuDevice, iGpu));
  char szDeviceName[80];
  ck(cuDeviceGetName(szDeviceName, sizeof(szDeviceName), cuDevice));
  std::cout << "GPU in use: " << szDeviceName << std::endl;
  CUcontext cuContext = NULL;
  ck(cuCtxCreate(&cuContext, 0, cuDevice));

  NvEncoderCuda encoder(
      cuContext, img_width, img_height, NV_ENC_BUFFER_FORMAT_IYUV,
      0);  // Set nExtraOutputDelay to 0 to make sure we can get one in one out

  NV_ENC_INITIALIZE_PARAMS initializeParams = {NV_ENC_INITIALIZE_PARAMS_VER};
  NV_ENC_CONFIG encodeConfig = {NV_ENC_CONFIG_VER};
  encodeConfig.rcParams.zeroReorderDelay = 1;

  initializeParams.encodeConfig = &encodeConfig;
  initializeParams.enablePTD = 0;

  encoder.CreateDefaultEncoderParams(
      &initializeParams, NV_ENC_CODEC_H264_GUID, NV_ENC_PRESET_P4_GUID,
      NV_ENC_TUNING_INFO_LOW_LATENCY);  // NV_ENC_TUNING_INFO_ULTRA_LOW_LATENCY

  encodeConfig.gopLength = NVENC_INFINITE_GOPLENGTH;
  encodeConfig.frameIntervalP = 1;
  encodeConfig.encodeCodecConfig.h264Config.idrPeriod =
      NVENC_INFINITE_GOPLENGTH;

  encodeConfig.rcParams.rateControlMode = NV_ENC_PARAMS_RC_CBR;
  encodeConfig.rcParams.multiPass = NV_ENC_TWO_PASS_FULL_RESOLUTION;
  encodeConfig.rcParams.averageBitRate = 10000000;
  encodeConfig.rcParams.vbvBufferSize =
      (encodeConfig.rcParams.averageBitRate * initializeParams.frameRateDen /
       initializeParams.frameRateNum) *
      5;
  encodeConfig.rcParams.maxBitRate = encodeConfig.rcParams.averageBitRate;
  encodeConfig.rcParams.vbvInitialDelay = encodeConfig.rcParams.vbvBufferSize;

  initializeParams.bufferFormat = NV_ENC_BUFFER_FORMAT_IYUV;
  NvEncoderInitParam encodeCLIOptions;

  encodeCLIOptions.SetInitParams(&initializeParams, NV_ENC_BUFFER_FORMAT_IYUV);

  encoder.CreateEncoder(&initializeParams);
  int nFrameSize = encoder.GetFrameSize();
  std::unique_ptr<uint8_t[]> pHostFrame(new uint8_t[nFrameSize]);
  std::vector<std::vector<uint8_t>> encoded_packets;
  std::vector<std::vector<uint8_t>> vPacket;

  // Params for one frame
  NV_ENC_PIC_PARAMS picParams = {NV_ENC_PIC_PARAMS_VER};
  picParams.encodePicFlags = 0;

  //
  // Main Image Encoding loop
  //
  std::vector<cv::Mat> original_bgr_images;

  for (const auto& path : image_paths) {
    cv::Mat img = imread(path, cv::IMREAD_COLOR);

    if (img.empty()) {
      spdlog::error("Could not read the image: path = {}", path.string());
      return 1;
    }

    original_bgr_images.push_back(img);

    cv::Mat img_iyuv;
    cv::cvtColor(img, img_iyuv, cv::COLOR_BGR2YUV_I420);

    const NvEncInputFrame* encoderInputFrame = encoder.GetNextInputFrame();
    NvEncoderCuda::CopyToDeviceFrame(
        cuContext, img_iyuv.data, 0, (CUdeviceptr)encoderInputFrame->inputPtr,
        (int)encoderInputFrame->pitch, encoder.GetEncodeWidth(),
        encoder.GetEncodeHeight(), CU_MEMORYTYPE_HOST,
        encoderInputFrame->bufferFormat, encoderInputFrame->chromaOffsets,
        encoderInputFrame->numChromaPlanes);

    encoder.EncodeFrame(vPacket, &picParams);
    if (vPacket.size() == 0) {
      spdlog::error("Didn't get input packet from the encoded frame");
    } else if (vPacket.size() > 1) {
      spdlog::error("Got more than 1 packet from one input frame.");
      return 1;
    } else {
      spdlog::info("Got packet size = {} bytes", vPacket.back().size());
      encoded_packets.push_back(std::move(vPacket.back()));
    }
  }

  int total_packet_size = 0;
  for (const auto& pkt_data : encoded_packets) {
    total_packet_size += pkt_data.size();
  }

  int bgr_data_size = 3 * img_width * img_height;
  int total_bgr_data_size = encoded_packets.size() * bgr_data_size;
  double compression_ratio = static_cast<double>(total_packet_size) /
                             static_cast<double>(total_bgr_data_size);
  spdlog::info(
      "Number of Encoded packets = {} , Total packet size = {} bytes, Total "
      "Raw BGR(8bit) size = {}, Compression Ratio = {}. ",
      encoded_packets.size(), total_packet_size, total_bgr_data_size,
      compression_ratio);

  encoder.EndEncode(vPacket);
  encoder.DestroyEncoder();

  NvDecoder decoder(cuContext, false, cudaVideoCodec_H264, true, false, NULL,
                    NULL, false, 0, 0, 1000, true);

  int pkt_idx = 0;
  double mse_r = 0.0;
  double mse_g = 0.0;
  double mse_b = 0.0;
  double psnr_r = 0.0;
  double psnr_g = 0.0;
  double psnr_b = 0.0;
  int nFrameReturned = 0;
  int64_t timestamp = 0;
  for (auto& pkt_data : encoded_packets) {
    nFrameReturned = 0;
    nFrameReturned = decoder.Decode(pkt_data.data(), pkt_data.size(),
                                    CUVID_PKT_ENDOFPICTURE, pkt_idx);

    if (nFrameReturned == 0) {
      spdlog::error("No frame returned from decoder.");
      return 1;
    } else if (nFrameReturned > 1) {
      spdlog::error("More than one frame is returned from decoder.");
      return 1;
    }
    cv::Mat img_iyuv(img_height * 3 / 2, img_width, CV_8UC1);

    auto decoded_frame_data = decoder.GetFrame(&timestamp);
    auto decoded_frame_size = decoder.GetFrameSize();

    // std::memcpy(img_iyuv.data, decoded_frame_data, decoded_frame_size);

    // cuvid output format is nv12
    int y_ch_size = img_width * img_height;
    int uv_ch_size = y_ch_size / 2;

    std::memcpy(img_iyuv.data, decoded_frame_data, y_ch_size);
    std::memcpy(img_iyuv.data + y_ch_size, decoded_frame_data + y_ch_size,
                uv_ch_size);

    spdlog::info("Frame idx {}, frame size = {}", pkt_idx,
                 decoder.GetFrameSize());

    cv::Mat image_bgr(img_height, img_width, CV_8UC3, 3);
    cv::cvtColor(img_iyuv, image_bgr, cv::COLOR_YUV2BGR_NV12);

    auto mse = cv::quality::QualityMSE::compute(original_bgr_images[pkt_idx],
                                                image_bgr, cv::noArray());
    auto psnr = cv::quality::QualityPSNR::compute(original_bgr_images[pkt_idx],
                                                  image_bgr, cv::noArray());

    mse_b += mse[0];
    mse_g += mse[1];
    mse_r += mse[2];

    psnr_b += psnr[0];
    psnr_g += psnr[1];
    psnr_r += psnr[2];

    if (save_output) {
      auto output_path = output_dir / image_paths[pkt_idx].filename();
      cv::imwrite(output_path, image_bgr);
      // spdlog::info("Saved Image to :{}", output_path.string());
    }

    pkt_idx++;
  }

  mse_b /= encoded_packets.size();
  mse_g /= encoded_packets.size();
  mse_r /= encoded_packets.size();

  psnr_b /= encoded_packets.size();
  psnr_g /= encoded_packets.size();
  psnr_r /= encoded_packets.size();

  spdlog::info("Average Image quality scores:");
  spdlog::info("BGR MSE (Lower is better) = [{}, {}, {}]", mse_b, mse_g, mse_r);
  spdlog::info("BGR PSNR (Should be ~30-50dB, higher is better) = [{}, {}, {}]",
               psnr_b, psnr_g, psnr_r);

  return 0;
}