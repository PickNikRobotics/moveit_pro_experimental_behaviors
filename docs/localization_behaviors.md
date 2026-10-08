# Localization Behaviors

These Behaviors help an Objective check and refine where a mobile robot thinks it is on a 2D map, before
it plans or moves. They fill gaps in MoveIt Pro 10.0 and 10.1, which have no Behavior to read a pose
estimate with its uncertainty, to score a pose against a laser scan, or to call a `std_srvs/Empty`
service by name.

Source: `src/localization_behaviors/` and `include/experimental_behaviors/localization_behaviors/`.
Tests: `test/localization_behaviors/`.

## Concepts in plain words

- **Localizer.** A node (for example an AMCL-style particle filter) that estimates the robot's pose on a
  known map. It usually publishes that estimate as a `geometry_msgs/PoseWithCovarianceStamped`.
- **Pose covariance.** A 6x6 table that says how sure the localizer is. The diagonal holds the variance
  of x, y, z, roll, pitch and yaw. The square root of a variance is a standard deviation, in metres for
  x and y and in radians for yaw. Small values mean a confident estimate.
- **Occupancy grid.** The map, as a `nav_msgs/OccupancyGrid`: a grid of cells, each 0 (free) to 100
  (occupied), or -1 (unknown). These Behaviors treat unknown cells as free.
- **Laser scan.** A `sensor_msgs/LaserScan`: a fan of beams from a 2D lidar. Each beam has a bearing and
  a measured range. The point where a beam stops is its endpoint, and it should lie on a wall.
- **Scan match.** Put the scan at a candidate robot pose, and check how many beam endpoints land on
  occupied cells. If the pose is right, most do. The distance from an endpoint to the nearest occupied
  cell is its **residual**. An endpoint with a residual below a limit is an **inlier**. The share of
  inliers is the **inlier fraction**: near 1 for a good pose, low for a wrong one.
- **Latched topic.** A publisher with transient local durability keeps its last message for late
  subscribers. Maps and last pose estimates are often latched. These Behaviors match their subscription
  to the publishers, so they read a latched message at once.

## Behaviors

### GetPoseWithCovarianceFromTopic

Takes the next `geometry_msgs/PoseWithCovarianceStamped` from a topic. Outputs the pose and the standard
deviations of its x, y and yaw. Use it to read the localizer's estimate and gate on its confidence, for
example with a `Script` condition on `x_stddev`.

| Data Port Name      | Port Type | Object Type                     | Default          | Description                        |
| ------------------- | --------- | ------------------------------- | ---------------- | ---------------------------------- |
| topic_name          | input     | std::string                     |                  | Topic to read                      |
| message_timeout_sec | input     | double                          | `5.0`            | Seconds to wait for a message      |
| pose_stamped        | output    | geometry_msgs::msg::PoseStamped | `{pose_stamped}` | The pose, without covariance       |
| x_stddev            | output    | double                          | `{x_stddev}`     | Standard deviation of x (m)        |
| y_stddev            | output    | double                          | `{y_stddev}`     | Standard deviation of y (m)        |
| yaw_stddev          | output    | double                          | `{yaw_stddev}`   | Standard deviation of yaw (rad)    |

- Fails if no message arrives in time, if the timeout is negative or not finite, or if a variance it
  reads is negative or not finite.

### ScanMatchResidualFromTopics

Scores a robot pose by how well the latest laser scan, placed at that pose, fits the map. It reads the
scan and the map from their topics, so it needs no other Behavior to fetch them.

| Data Port Name      | Port Type | Object Type                     | Default             | Description                                          |
| ------------------- | --------- | ------------------------------- | ------------------- | ---------------------------------------------------- |
| pose                | input     | geometry_msgs::msg::PoseStamped |                     | Robot pose to score, in the map's frame              |
| robot_frame_id      | input     | std::string                     | `base_link`         | Frame the pose places; its TF offset to the scan frame is used |
| scan_topic          | input     | std::string                     | `/scan`             | `sensor_msgs/LaserScan` topic                        |
| map_topic           | input     | std::string                     | `/map`              | `nav_msgs/OccupancyGrid` topic                       |
| inlier_distance     | input     | double                          |                     | Largest residual that counts as an inlier (m)        |
| min_inlier_fraction | input     | double                          |                     | Smallest inlier fraction, 0 to 1, that passes        |
| message_timeout_sec | input     | double                          | `5.0`               | Seconds to wait for each of the scan and the map     |
| inlier_fraction     | output    | double                          | `{inlier_fraction}` | Share of used beams that are inliers                 |
| median_residual     | output    | double                          | `{median_residual}` | Median residual of the used beams (m)                |
| beams_used          | output    | int                             | `{beams_used}`      | Number of beams scored                               |

- Beams with a NaN, infinite, too short or max-range return are skipped. An endpoint off the map is an
  outlier.
- The TF offset from `robot_frame_id` to the scan frame is the latest one; the robot is assumed still.
- Cells at or above the usual map-server occupied threshold count as walls.
- The outputs are set before the pass/fail check, so a failed run can still be logged.
- Fails if the inlier fraction is below `min_inlier_fraction`, the pose is not finite or not in the
  map's frame, an input is out of range, the TF offset is missing, no beam is usable, or the scan or map
  does not arrive in time.

### CallEmptyServiceByName

Calls a `std_srvs/srv/Empty` service by name, for example a localizer's "spread the particles" or
"request no-motion update" service. An Empty response has no success field, so any response is success.

| Data Port Name                    | Port Type | Object Type | Default | Description                                         |
| --------------------------------- | --------- | ----------- | ------- | --------------------------------------------------- |
| service_name                      | input     | std::string |         | Service to call                                     |
| response_timeout                  | input     | double      | `3.0`   | Seconds to wait for the response; negative waits forever |
| wait_for_server_available_timeout | input     | double      | `3.0`   | Seconds to wait for the server to appear            |

- Fails if the server does not appear or the response does not arrive in time.

## Relation to newer MoveIt Pro releases

MoveIt Pro `main` (after 10.1.1) adds `CallEmptyService` and a `ScanMatchResidual` that takes the scan
and the map as input ports (fetched with `GetLaserScan` and `GetOccupancyGrid`). The IDs here differ on
purpose, so this package still loads on a MoveIt Pro release that includes those Behaviors.

## Example

Check the localizer's estimate against the scan, and fail early when it is poor:

```xml
<Action ID="GetPoseWithCovarianceFromTopic" topic_name="/amcl_pose" message_timeout_sec="5.0"
        pose_stamped="{estimate}" x_stddev="{x_stddev}" y_stddev="{y_stddev}" yaw_stddev="{yaw_stddev}"/>
<Action ID="ScanMatchResidualFromTopics" pose="{estimate}" robot_frame_id="base_link" scan_topic="/scan"
        map_topic="/map" inlier_distance="0.1" min_inlier_fraction="0.7" message_timeout_sec="5.0"
        inlier_fraction="{inlier_fraction}" median_residual="{median_residual}" beams_used="{beams_used}"/>
```
