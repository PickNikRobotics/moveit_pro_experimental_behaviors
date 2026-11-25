# Agent comments

Detailed comments by package-relative file and line. Keep entries short.

## include/image_behaviors/intersect_pixel_rays_with_plane.hpp

- line 39 (`IntersectPixelRaysWithPlane`): What it trades: it needs a plane, for example from a plane fit or a known
  surface, and the result is right only for features that lie on that plane. The plane can have any orientation. The
  pinhole model does not remove lens distortion, so use a rectified image. A ray that meets the plane behind the
  camera is rejected, because it would otherwise come back as a target behind the camera.

## include/image_behaviors/rotate_and_crop_object_image.hpp

- line 37 (`RotateAndCropObjectImage`): Steps. Transform the object pose into the camera optical frame (latest TF).
  Project the object origin and a point a short step along its +Z axis with the pinhole intrinsics. The angle
  `atan2(du, -dv)` turns that projected direction to image-up (OpenCV turns counter-clockwise for a positive angle).
  Paste the mask sub-image into a full-size mask at its x, y offset and black out the other pixels. Cut a square
  crop, centered on the projected origin, whose side is the larger mask bounding-box side times padding_factor (with
  a minimum size, rounded to even), then rotate it about its center. It fails when +Z points almost along the
  optical axis, because the projected direction is then too short to define an angle.

## src/pixel_ray.hpp

- line 20 (`CameraIntrinsics`): A second copy of this struct is in
  `point_cloud_behaviors/src/camera_frustum.hpp`. The copy is on purpose: a folder must not include the
  headers of another folder, so that each folder stays copyable on its own. Change both if you change one.

## test/test_behavior_index.cpp

- One JSON field per line allows the comparison to ignore C++ type text. All IDs, metadata and
  other port fields must match. A `null` default means no default; `""` means an empty string.
