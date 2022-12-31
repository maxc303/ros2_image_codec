
// Need to warp ffmpeg headers into extern "C"
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

#include <boost/program_options.hpp>
#include <filesystem>
#include <iostream>

#include "nvEncodeAPI.h"
#include "nvcuvid.h"
// #include <opencv4/opencv2/opencv.hpp>
// #include <opencv4/opencv2/quality.hpp>
// #include <string>

// namespace po = boost::program_options;

/**
 * @brief Exception class for error reporting from NvEncodeAPI calls.
 */
class NVENCException : public std::exception {
 public:
  NVENCException(const std::string& errorStr, const NVENCSTATUS errorCode)
      : m_errorString(errorStr), m_errorCode(errorCode) {}

  virtual ~NVENCException() throw() {}
  virtual const char* what() const throw() { return m_errorString.c_str(); }
  NVENCSTATUS getErrorCode() const { return m_errorCode; }
  const std::string& getErrorString() const { return m_errorString; }
  static NVENCException makeNVENCException(const std::string& errorStr,
                                           const NVENCSTATUS errorCode,
                                           const std::string& functionName,
                                           const std::string& fileName,
                                           int lineNo);

 private:
  std::string m_errorString;
  NVENCSTATUS m_errorCode;
};

inline NVENCException NVENCException::makeNVENCException(
    const std::string& errorStr, const NVENCSTATUS errorCode,
    const std::string& functionName, const std::string& fileName, int lineNo) {
  std::ostringstream errorLog;
  errorLog << functionName << " : " << errorStr << " at " << fileName << ":"
           << lineNo << std::endl;
  NVENCException exception(errorLog.str(), errorCode);
  return exception;
}

#define NVENC_THROW_ERROR(errorStr, errorCode)                  \
  do {                                                          \
    throw NVENCException::makeNVENCException(                   \
        errorStr, errorCode, __FUNCTION__, __FILE__, __LINE__); \
  } while (0)

#define NVENC_API_CALL(nvencAPI)                                        \
  do {                                                                  \
    NVENCSTATUS errorCode = nvencAPI;                                   \
    if (errorCode != NV_ENC_SUCCESS) {                                  \
      std::ostringstream errorLog;                                      \
      errorLog << #nvencAPI << " returned error " << errorCode;         \
      throw NVENCException::makeNVENCException(                         \
          errorLog.str(), errorCode, __FUNCTION__, __FILE__, __LINE__); \
    }                                                                   \
  } while (0)

inline bool check(int e, int iLine, const char* szFile) {
  if (e < 0) {
    std::cout << "General error " << e << " at line " << iLine << " in file "
              << szFile;
    return false;
  }
  return true;
}

#define ck(call) check(call, __LINE__, __FILE__)
namespace po = boost::program_options;
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

  //
  // Init Encoder
  //
  ck(cuInit(0));
  int nGpu = 0;
  int iCase = 0;
  int iGpu = 0;
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

  uint32_t version = 0;
  uint32_t currentVersion =
      (NVENCAPI_MAJOR_VERSION << 4) | NVENCAPI_MINOR_VERSION;
  NVENC_API_CALL(NvEncodeAPIGetMaxSupportedVersion(&version));
  if (currentVersion > version) {
    NVENC_THROW_ERROR(
        "Current Driver Version does not support this NvEncodeAPI version, "
        "please upgrade driver",
        NV_ENC_ERR_INVALID_VERSION);
  }

  NV_ENCODE_API_FUNCTION_LIST m_nvenc;
  m_nvenc = {NV_ENCODE_API_FUNCTION_LIST_VER};
  int32_t m_nEncoderBuffer = 0;

  NVENC_API_CALL(NvEncodeAPICreateInstance(&m_nvenc));
  if (!m_nvenc.nvEncOpenEncodeSession) {
    m_nEncoderBuffer = 0;
    NVENC_THROW_ERROR("EncodeAPI not found", NV_ENC_ERR_NO_ENCODE_DEVICE);
  }

  NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS encodeSessionExParams = {
      NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS_VER};
  void* m_pDevice;
  NV_ENC_DEVICE_TYPE m_eDeviceType;
  encodeSessionExParams.device = cuContext;
  encodeSessionExParams.deviceType = NV_ENC_DEVICE_TYPE_CUDA;
  encodeSessionExParams.apiVersion = NVENCAPI_VERSION;
  void* hEncoder = NULL;
  NVENC_API_CALL(
      m_nvenc.nvEncOpenEncodeSessionEx(&encodeSessionExParams, &hEncoder));

  NV_ENC_INITIALIZE_PARAMS initializeParams = {NV_ENC_INITIALIZE_PARAMS_VER};
  NV_ENC_CONFIG encodeConfig = {NV_ENC_CONFIG_VER};
  initializeParams.encodeConfig = &encodeConfig;

  int image_width = 1600;
  int image_height = 900;
  initializeParams.encodeGUID =
      NV_ENC_CODEC_H264_GUID;  // NV_ENC_CODEC_HEVC_GUID
  initializeParams.presetGUID = NV_ENC_PRESET_P4_GUID;  // Default
  initializeParams.encodeWidth = image_width;
  initializeParams.encodeHeight = image_height;
  initializeParams.darWidth = image_width;
  initializeParams.darHeight = image_height;
  initializeParams.frameRateNum = 10;
  initializeParams.frameRateDen = 1;
  initializeParams.enablePTD = 1;
  initializeParams.reportSliceOffsets = 0;
  initializeParams.enableSubFrameWrite = 0;
  initializeParams.maxEncodeHeight = image_height;
  initializeParams.maxEncodeWidth = image_width;
  initializeParams.enableMEOnlyMode = 0;
  initializeParams.enableOutputInVidmem = 1;

  initializeParams.tuningInfo = NV_ENC_TUNING_INFO_LOW_LATENCY;
  // initializeParams.tuningInfo = NV_ENC_TUNING_INFO_ULTRA_LOW_LATENCY;

  NV_ENC_PRESET_CONFIG presetConfig = {NV_ENC_PRESET_CONFIG_VER,
                                       {NV_ENC_CONFIG_VER}};
  m_nvenc.nvEncGetEncodePresetConfigEx(
      hEncoder, initializeParams.encodeGUID, initializeParams.presetGUID,
      initializeParams.tuningInfo, &presetConfig);
  memcpy(initializeParams.encodeConfig, &presetConfig.presetCfg,
         sizeof(NV_ENC_CONFIG));

  initializeParams.encodeConfig->gopLength = NVENC_INFINITE_GOPLENGTH;
  initializeParams.encodeConfig->frameIntervalP = 1;
  initializeParams.encodeConfig->encodeCodecConfig.h264Config.idrPeriod =
      NVENC_INFINITE_GOPLENGTH;

  initializeParams.encodeConfig->rcParams.rateControlMode =
      NV_ENC_PARAMS_RC_CBR;
  initializeParams.encodeConfig->rcParams.multiPass =
      NV_ENC_TWO_PASS_FULL_RESOLUTION;
  initializeParams.encodeConfig->rcParams.averageBitRate =
      (static_cast<unsigned int>(5.0f * initializeParams.encodeWidth *
                                 initializeParams.encodeHeight) /
       (1280 * 720)) *
      100000;
  initializeParams.encodeConfig->rcParams.vbvBufferSize =
      (encodeConfig.rcParams.averageBitRate * initializeParams.frameRateDen /
       initializeParams.frameRateNum) *
      5;
  initializeParams.encodeConfig->rcParams.maxBitRate =
      encodeConfig.rcParams.averageBitRate;
  initializeParams.encodeConfig->rcParams.vbvInitialDelay =
      encodeConfig.rcParams.vbvBufferSize;

  initializeParams.bufferFormat = NV_ENC_BUFFER_FORMAT_IYUV;

  NVENC_API_CALL(m_nvenc.nvEncInitializeEncoder(hEncoder, &initializeParams));

  if (!hEncoder) {
    return 1;
  }

  m_nvenc.nvEncDestroyEncoder(hEncoder);
  return 0;
}