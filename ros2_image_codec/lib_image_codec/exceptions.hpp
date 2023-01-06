#ifndef LIB_IMAGE_CODEC__ENCODER_EXCEPTION_HPP
#define LIB_IMAGE_CODEC__ENCODER_EXCEPTION_HPP

extern "C" {
#include <libavcodec/avcodec.h>
}

#include <exception>
#include <string>

#define CHECK_LIBAV_ERROR(x)                                                  \
  {                                                                           \
    int status = x;                                                           \
    if (status < 0) {                                                         \
      throw image_codec::LibavException(                                      \
          std::string("libav Error '") + image_codec::av_err2string(status) + \
          std::string("' executing function:\n " #x) +                        \
          std::string("\n at " __FILE__ ":") + std::to_string(__LINE__));     \
    }                                                                         \
  };

namespace image_codec {

inline std::string av_err2string(int errnum) {
  char str[AV_ERROR_MAX_STRING_SIZE];
  return av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum);
}

class LibavException : public std::exception {
 public:
  explicit LibavException(const std::string& msg) : msg_(msg) {}

  const char* what() const noexcept override { return msg_.c_str(); }

 private:
  std::string msg_;
};

class EncoderException : public std::exception {
 public:
  explicit EncoderException(const std::string& msg) : msg_(msg) {}

  const char* what() const noexcept override { return msg_.c_str(); }

 private:
  std::string msg_;
};
}  // namespace image_codec

#endif  // LIB_IMAGE_CODEC__ENCODER_EXCEPTION_HPP
