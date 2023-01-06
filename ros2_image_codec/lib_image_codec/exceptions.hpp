#ifndef LIB_IMAGE_CODEC__ENCODER_EXCEPTION_HPP
#define LIB_IMAGE_CODEC__ENCODER_EXCEPTION_HPP

#include <exception>
#include <string>

namespace image_codec {

std::string av_err2string(int errnum) {
  char str[AV_ERROR_MAX_STRING_SIZE];
  return av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum);
}

class EncoderException : public std::exception {
 public:
  explicit EncoderException(const std::string& msg) : msg_(msg) {}

  const char* what() const noexcept override { return msg_.c_str(); }

 private:
  std::string msg_;
};
}  // namespace image_codec

#endif  // LIB_IMAGE_CODEC__ENCODER_EXCEPTION_HPP
