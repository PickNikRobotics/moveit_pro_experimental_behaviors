// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "image_behaviors/rotate_and_crop_object_image.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>

// Jazzy ships cv_bridge.hpp; Humble ships only cv_bridge.h.
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <tf2_ros/buffer.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <moveit_pro_common/utils/filesystem_utils.hpp>
#include <moveit_studio_vision_msgs/msg/mask2_d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace image_behaviors
{
namespace
{
constexpr auto kPortIDImage = "image";
constexpr auto kPortIDMask = "mask";
constexpr auto kPortIDObjectPose = "object_pose";
constexpr auto kPortIDCameraInfo = "camera_info";
constexpr auto kPortIDPaddingFactor = "padding_factor";
constexpr auto kPortIDSaveImageDir = "save_image_dir";
constexpr auto kPortIDRotatedImage = "rotated_image";

constexpr double kDefaultPaddingFactor = 1.5;
// Step along the object +Z axis for the second projected point: long enough to be above pixel
// noise, short enough to stay in front of the camera.
constexpr double kZAxisProjectionLengthMeters = 0.05;
// Below this projected length, +Z is too close to the optical axis to have a direction in the image.
constexpr double kMinProjectedPixelMagnitude = 1.0;
// Smallest crop side, so that a tiny mask still gives a usable image.
constexpr int kMinCropSizePixels = 32;

// Millisecond timestamp for file names, so that two saves in one second do not overwrite each other.
[[nodiscard]] std::string makeTimestampSuffix()
{
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::system_clock::to_time_t(now);
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
  std::tm tm{};
  localtime_r(&seconds, &tm);
  std::ostringstream os;
  os << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_" << std::setw(3) << std::setfill('0') << millis;
  return os.str();
}
}  // namespace

RotateAndCropObjectImage::RotateAndCropObjectImage(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList RotateAndCropObjectImage::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::Image>(kPortIDImage, "{image}", "Color image that shows the object."),
    BT::InputPort<moveit_studio_vision_msgs::msg::Mask2D>(
        kPortIDMask, "{mask2d}",
        "Mask of the object in the image. x and y are the top-left corner of the mask in the image, and pixels is "
        "the mask sub-image. Non-zero pixels are kept; the others become black."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
        kPortIDObjectPose, "{object_pose}", "Pose of the object. Its +Z axis is the axis that points up in the output."),
    BT::InputPort<sensor_msgs::msg::CameraInfo>(kPortIDCameraInfo, "{camera_info}",
                                                "Intrinsics of the camera that took the image. Its frame_id is the "
                                                "camera optical frame."),
    BT::InputPort<double>(kPortIDPaddingFactor, kDefaultPaddingFactor,
                          "The crop side is the larger side of the mask bounding box times this factor, so the "
                          "rotation does not cut the corners of the object."),
    BT::InputPort<std::string>(kPortIDSaveImageDir, "",
                               "If not empty, the output image is also saved to this directory as a timestamped "
                               "JPEG. A leading '~' is your home directory. Empty means do not save."),
    BT::OutputPort<sensor_msgs::msg::Image>(kPortIDRotatedImage, "{rotated_image}",
                                            "Cropped, masked and rotated BGR8 image, with the object +Z axis pointing "
                                            "up."),
  };
}

BT::KeyValueVector RotateAndCropObjectImage::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Perception - 2D Image" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Crops one object out of an image by its mask, and rotates the crop so that the +Z axis of the object "
             "frame points up. Use it to give a model a view of the object in a known orientation." } };
}

BT::NodeStatus RotateAndCropObjectImage::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<sensor_msgs::msg::Image>(kPortIDImage), getInput<moveit_studio_vision_msgs::msg::Mask2D>(kPortIDMask),
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDObjectPose),
      getInput<sensor_msgs::msg::CameraInfo>(kPortIDCameraInfo), getInput<double>(kPortIDPaddingFactor));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Missing required input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [image, mask, object_pose, camera_info, padding_factor] = ports.value();
  const std::string camera_frame = camera_info.header.frame_id;
  if (camera_frame.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "camera_info.header.frame_id is empty.");
    return BT::NodeStatus::FAILURE;
  }
  if (!(padding_factor > 0.0))
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "padding_factor must be greater than zero.");
    return BT::NodeStatus::FAILURE;
  }

  // Decode the image, and paste the mask sub-image into a full-size mask at its offset.
  cv_bridge::CvImagePtr color_ptr;
  cv_bridge::CvImagePtr mask_sub_ptr;
  try
  {
    color_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    mask_sub_ptr = cv_bridge::toCvCopy(mask.pixels, sensor_msgs::image_encodings::MONO8);
  }
  catch (const cv_bridge::Exception& e)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), std::string("Image conversion failed: ") + e.what());
    return BT::NodeStatus::FAILURE;
  }

  cv::Mat full_mask = cv::Mat::zeros(color_ptr->image.rows, color_ptr->image.cols, CV_8UC1);
  const cv::Rect mask_roi(mask.x, mask.y, mask_sub_ptr->image.cols, mask_sub_ptr->image.rows);
  const cv::Rect clamped = mask_roi & cv::Rect(0, 0, full_mask.cols, full_mask.rows);
  if (clamped.area() == 0)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "The mask is fully outside the image.");
    return BT::NodeStatus::FAILURE;
  }
  const cv::Rect source(clamped.x - mask_roi.x, clamped.y - mask_roi.y, clamped.width, clamped.height);
  mask_sub_ptr->image(source).copyTo(full_mask(clamped));

  // Object pose in the camera optical frame.
  Eigen::Isometry3d camera_to_object;
  try
  {
    const auto transform = getBehaviorContext()->transform_buffer_ptr->lookupTransform(
        camera_frame, object_pose.header.frame_id, tf2::TimePointZero);
    Eigen::Isometry3d frame_to_object;
    tf2::fromMsg(object_pose.pose, frame_to_object);
    camera_to_object = tf2::transformToEigen(transform) * frame_to_object;
  }
  catch (const std::exception& e)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Cannot transform the object pose into the camera frame: " + std::string(e.what()));
    return BT::NodeStatus::FAILURE;
  }

  // Project the object origin and a point along its +Z axis into the image.
  const Eigen::Vector3d origin = camera_to_object.translation();
  const Eigen::Vector3d along_z = origin + kZAxisProjectionLengthMeters * camera_to_object.linear().col(2);
  if (origin.z() <= 0.0 || along_z.z() <= 0.0)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "The object is behind the camera.");
    return BT::NodeStatus::FAILURE;
  }
  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4];
  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];
  const cv::Point2f pixel_origin(static_cast<float>(fx * origin.x() / origin.z() + cx),
                                 static_cast<float>(fy * origin.y() / origin.z() + cy));
  const cv::Point2f pixel_along(static_cast<float>(fx * along_z.x() / along_z.z() + cx),
                                static_cast<float>(fy * along_z.y() / along_z.z() + cy));

  const double du = pixel_along.x - pixel_origin.x;
  const double dv = pixel_along.y - pixel_origin.y;
  if (std::hypot(du, dv) < kMinProjectedPixelMagnitude)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "The object +Z axis points almost along the camera axis, so it has no direction in the image.");
    return BT::NodeStatus::FAILURE;
  }
  // OpenCV turns counter-clockwise for a positive angle; this angle maps (du, dv) to image-up.
  const double rotation_degrees = std::atan2(du, -dv) * 180.0 / M_PI;

  cv::Mat masked_color = cv::Mat::zeros(color_ptr->image.size(), color_ptr->image.type());
  color_ptr->image.copyTo(masked_color, full_mask);

  const cv::Rect mask_bbox = cv::boundingRect(full_mask);
  if (mask_bbox.area() == 0)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "The mask has no non-zero pixels.");
    return BT::NodeStatus::FAILURE;
  }
  int crop_size =
      std::max(static_cast<int>(std::max(mask_bbox.width, mask_bbox.height) * padding_factor), kMinCropSizePixels);
  // An even size puts the projected origin exactly at the center of the crop.
  crop_size += crop_size % 2;

  // getRectSubPix takes a sub-pixel center and repeats the border pixels, so a crop at the image edge is safe.
  cv::Mat crop;
  cv::getRectSubPix(masked_color, cv::Size(crop_size, crop_size), pixel_origin, crop);
  const cv::Point2f crop_center(static_cast<float>(crop_size) / 2.0F, static_cast<float>(crop_size) / 2.0F);
  cv::Mat rotated;
  cv::warpAffine(crop, rotated, cv::getRotationMatrix2D(crop_center, rotation_degrees, 1.0),
                 cv::Size(crop_size, crop_size), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

  sensor_msgs::msg::Image output;
  cv_bridge::CvImage(image.header, sensor_msgs::image_encodings::BGR8, rotated).toImageMsg(output);
  setOutput(kPortIDRotatedImage, output);

  // A failed debug save only warns: the output is already set.
  if (const std::string save_dir = getInput<std::string>(kPortIDSaveImageDir).value_or(""); !save_dir.empty())
  {
    const auto expanded = moveit_pro::common::filesystem_utils::expandHomeFilePath(save_dir);
    std::error_code ec;
    if (expanded.has_value())
    {
      std::filesystem::create_directories(expanded.value(), ec);
    }
    const std::string path =
        expanded.has_value() ? expanded.value() + "/rotated_object_" + makeTimestampSuffix() + ".jpg" : save_dir;
    if (!expanded.has_value() || ec || !cv::imwrite(path, rotated))
    {
      getBehaviorContext()->logger->publishWarnMessage(name(), "Could not save the image to '" + path + "'.");
    }
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace image_behaviors
