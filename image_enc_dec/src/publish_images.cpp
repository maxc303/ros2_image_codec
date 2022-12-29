#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.h>
#include <spdlog/spdlog.h>

#include <boost/program_options.hpp>
#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

namespace po = boost::program_options;

struct ImagePublisherParams {
  std::filesystem::path input_dir;
  std::string topic_name;
  double framerate;
  std::string frame_id;
  std::string encoding;
};

class ImagePublisherNode : public rclcpp::Node {
 public:
  ImagePublisherNode(ImagePublisherParams params)
      : Node("Image_publisher"), params_(params) {
    // Get the image path for the test.
    for (const auto& entry :
         std::filesystem::directory_iterator(params.input_dir)) {
      if (entry.path().extension() == ".jpg" ||
          entry.path().extension() == ".png") {
        image_paths_.push_back(entry.path());
      }
    }
    if (image_paths_.size() < 1) {
      spdlog::error("No image is found in the provided path: {}",
                    params.input_dir.string());
    }
    // Sort the path alphabetically
    std::sort(image_paths_.begin(), image_paths_.end());
    frame_time_gap = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / params.framerate));

    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(params.topic_name, 10);
  }

  void publish_images() {
    spdlog::info("Start publishing images.");
    auto t_start = std::chrono::high_resolution_clock::now();
    auto t_last_publish = std::chrono::high_resolution_clock::now();
    int idx = 1;

    for (const auto& path : image_paths_) {
      if (idx % 10 == 0) {
        spdlog::info("Publishing [{}/{}].", idx, image_paths_.size());
      }
      idx++;

      cv::Mat bgr_img = cv::imread(path, cv::IMREAD_COLOR);

      sensor_msgs::msg::Image::SharedPtr msg =
          cv_bridge::CvImage(std_msgs::msg::Header(), params_.encoding, bgr_img)
              .toImageMsg();

      msg->header.frame_id = params_.frame_id;
      auto t_now = std::chrono::high_resolution_clock::now();
      if ((t_now - t_last_publish).count() > frame_time_gap.count()) {
        spdlog::warn(
            "Time from last publish [{}] is greater than target time gap [{}]",
            (t_now - t_last_publish).count(), frame_time_gap.count());
      }
      auto t_sleep = frame_time_gap - (t_now - t_last_publish);

      // Sleep to control frame rate
      std::this_thread::sleep_for(t_sleep);

      t_now = std::chrono::high_resolution_clock::now();
      t_last_publish = t_now;
      msg->header.stamp.nanosec = (t_now - t_start).count() % 1000000000;
      msg->header.stamp.sec = (t_now - t_start).count() / 1000000000;

      spdlog::debug("Timestamp: {}", (t_now - t_start).count());
      publisher_->publish(*msg);
      spdlog::debug("Published image: {}", path.string());
    }
  }

 private:
  ImagePublisherParams params_;
  std::vector<std::filesystem::path> image_paths_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::chrono::nanoseconds frame_time_gap;
};

int main(int argc, char** argv) {
  po::options_description desc(
      "Publish all the images in a directoy as a stream of ROS2 message");
  desc.add_options()("help", "produce help message")  //
      ("input_dir", po::value<std::string>(),
       "input images dir. Only .png and .jpg images will be used.")   //
      ("topic_name", po::value<std::string>(), "Output topic name.")  //
      ("framerate", po::value<double>()->default_value(10.0),
       "Publish framerate.")  //
      ("frame_id", po::value<std::string>()->default_value("map"),
       "Image message frame id.")  //
      ("encoding", po::value<std::string>()->default_value("bgr8"),
       "Encoding of the Image message");
  po::variables_map boost_args;
  po::store(po::parse_command_line(argc, argv, desc), boost_args);
  po::notify(boost_args);

  if (boost_args.count("help") != 0 || boost_args.count("input_dir") == 0 ||
      boost_args.count("topic_name") == 0) {
    std::cout << desc << std::endl;
    return 1;
  }

  ImagePublisherParams node_params;
  node_params.input_dir = boost_args["input_dir"].as<std::string>();
  node_params.topic_name = boost_args["topic_name"].as<std::string>();
  node_params.framerate = boost_args["framerate"].as<double>();
  node_params.frame_id = boost_args["frame_id"].as<std::string>();
  node_params.encoding = boost_args["encoding"].as<std::string>();

  rclcpp::init(argc, argv);
  ImagePublisherNode node(node_params);
  node.publish_images();

  return 0;
}
