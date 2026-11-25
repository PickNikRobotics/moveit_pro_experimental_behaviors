# Agent comments

Detailed comments by package-relative file and line. Keep entries short.

## include/point_cloud_behaviors/clear_point_cloud_timestamp.hpp

- line 25 (`ClearPointCloudTimestamp`): Mechanism: the updater looks up each collision link transform at the cloud
  header stamp (PlanningSceneMonitor::getShapeTransformCache, `canTransform(..., stamp)` with zero wait). A link
  whose transform is not in the TF buffer at that stamp is skipped without an error; the shape mask logs "Missing
  transform for shape ..." and masks nothing for it. A cloud pushed long after capture trips this: the TF buffer
  holds only a short history, fixed links still resolve from /tf_static, moving links do not. A zero stamp is right
  for static world-frame clouds pushed while the robot is still, not for live sensor data.

## include/point_cloud_behaviors/count_points_in_cloud.hpp

- line 31 (`CountPointsInCloud`): Counting rules: dense depth clouds carry a NaN for every pixel with no depth plus
  some near-origin artifacts, so `width * height` overstates the geometry; the NaN and near-zero rejection is the
  one `PclNnFilter` and `ConvexHullFillPointCloud` use, so counts at different pipeline stages compare. An empty
  cloud is SUCCESS because "is anything in this volume" needs zero as an answer; to tell "nothing there" from
  "nothing seen", also count the uncropped cloud. Two outputs per count: BT.CPP fixes an entry type at tree
  creation, and its script language has no number-to-string conversion.

## include/point_cloud_behaviors/crop_point_cloud_to_camera_frustum.hpp

- line 37 (`CropPointCloudToCameraFrustum`): Why the range limits are planes along the optical Z axis and not
  spheres: a radial band cuts the image corners at a different depth than the center, which biases a plane that is
  later fitted through the kept points. The pixel margin exists because a stereo camera computes the least reliable
  depth at the image edge. The test runs in the camera optical frame, but the kept points stay in the frame of the
  input cloud. Only little-endian FLOAT32 x, y and z fields are read; row padding is handled through row_step.

## include/point_cloud_behaviors/poisson_reconstruct.hpp

- line 33 (`PoissonReconstruct`): Why trim: the trim removes the closed underside and webbing that Poisson guesses
  where there is no data, so the result follows the real surface instead of ballooning, which is what a convex hull
  or MLS dilation does.

## src/camera_frustum.hpp

- line 18 (`CameraIntrinsics`): A second copy of this struct is in `image_behaviors/src/pixel_ray.hpp`,
  on purpose, so that each folder stays copyable on its own. Change both if you change one.

## src/count_points_in_cloud.cpp

- line 42 (`kMinVoxelSize`): kMinVoxelSize: well below any grid a caller would ask for, and it keeps the floored
  voxel indices far inside int64_t for clouds of room size. Without a floor, a tiny or subnormal size sends
  `std::floor(p.x / voxel_size)` out of int64_t range and the cast is undefined behaviour, so a config typo would
  become UB instead of an error.

## test/test_behavior_index.cpp

- One JSON field per line allows the comparison to ignore C++ type text. All IDs, metadata and
  other port fields must match. A `null` default means no default; `""` means an empty string.

## CMakeLists.txt

- Before finding PCL dependencies, create the C MPI target if it is missing. Ubuntu VTK
  references it through PCL. The target stays in this independent package's CMake scope.
- Link only the PCL modules used by these Behaviors rather than every PCL module.
