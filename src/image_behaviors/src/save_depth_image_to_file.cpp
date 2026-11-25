#include "image_behaviors/save_depth_image_to_file.hpp"

// Jazzy ships cv_bridge.hpp; Humble ships only cv_bridge.h.
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <moveit_pro_common/utils/filesystem_utils.hpp>

#include <filesystem>

namespace
{
constexpr auto kPortIDFilePath = "file_path";
constexpr auto kPortIDFilePrefix = "file_prefix";
constexpr auto kPortIDImage = "image";
}  // namespace

namespace image_behaviors
{

SaveDepthImageToFile::SaveDepthImageToFile(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList SaveDepthImageToFile::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDFilePath, "~/.config/moveit_pro/saved_behavior_data",
                               "Directory to save the depth image in."),
    BT::InputPort<std::string>(kPortIDFilePrefix, "depth", "Prefix for the output filename."),
    BT::InputPort<sensor_msgs::msg::Image>(kPortIDImage, "{depth_image}", "Depth image (16UC1 or 32FC1)."),
  };
}

BT::KeyValueVector SaveDepthImageToFile::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Perception - 2D Image" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Save a depth image to a 16-bit PNG file. Unlike SaveImageToFile, this preserves "
             "the original depth encoding instead of converting to BGR8." } };
}

tl::expected<bool, std::string> SaveDepthImageToFile::doWork()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIDFilePath),
                                                              getInput<std::string>(kPortIDFilePrefix),
                                                              getInput<sensor_msgs::msg::Image>(kPortIDImage));

  if (!ports.has_value())
  {
    return tl::make_unexpected("Missing input port: " + ports.error());
  }

  const auto& [file_path, file_prefix, image_msg] = ports.value();

  const auto maybe_path = moveit_pro::common::filesystem_utils::expandHomeFilePath(file_path);
  if (!maybe_path)
  {
    return tl::make_unexpected(maybe_path.error());
  }

  std::filesystem::path dir(maybe_path.value());
  if (!std::filesystem::exists(dir))
  {
    if (!std::filesystem::create_directories(dir))
    {
      return tl::make_unexpected("Cannot create directory: " + maybe_path.value());
    }
  }

  std::string full_filename = (dir / file_prefix).string() + "_" +
                              moveit_pro::common::filesystem_utils::createTimestampFromCurrentTime() + ".png";

  // Convert to CV image, preserving the original encoding.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // Use toCvCopy with passthrough to preserve the original depth encoding.
    cv_ptr = cv_bridge::toCvCopy(image_msg);
  }
  catch (const cv_bridge::Exception& e)
  {
    return tl::make_unexpected("cv_bridge conversion failed: " + std::string(e.what()));
  }

  // For 32FC1, convert to 16UC1 (millimeters) so PNG can store it losslessly.
  cv::Mat output_image;
  if (cv_ptr->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    // Assume meters, convert to millimeters as 16-bit unsigned.
    cv_ptr->image.convertTo(output_image, CV_16UC1, 1000.0);
  }
  else
  {
    output_image = cv_ptr->image;
  }

  try
  {
    if (cv::imwrite(full_filename, output_image))
    {
      getBehaviorContext()->logger->publishInfoMessage(name(), "Saved depth image to: " + full_filename);
      return true;
    }
    return tl::make_unexpected("Failed to write depth image to: " + full_filename);
  }
  catch (const cv::Exception& e)
  {
    return tl::make_unexpected("Image write exception: " + std::string(e.what()));
  }
}

}  // namespace image_behaviors
