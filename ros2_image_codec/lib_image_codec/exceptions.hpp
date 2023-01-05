#ifndef IMAGE_CODEC__ENCODER_EXCEPTION_HPP
#define IMAGE_CODEC__ENCODER_EXCEPTION_HPP

#include <exception>
#include <string>

namespace image_codec {

class EncoderException : public std::exception {
 public:
  explicit EncoderException(const std::string& msg) : msg_(msg) {}

  const char* what() const noexcept override { return msg_.c_str(); }

 private:
  std::string msg_;
};
}  // namespace image_codec

#endif  // IMAGE_CODEC__ENCODER_EXCEPTION_HPP
