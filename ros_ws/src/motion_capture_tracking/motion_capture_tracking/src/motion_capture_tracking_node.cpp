#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include <cmath>
#include <deque>

#include <fstream>
#include <iomanip>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>

// Motion Capture
#include <libmotioncapture/motioncapture.h>

// Rigid Body tracker
#include <librigidbodytracker/rigid_body_tracker.h>
#include <librigidbodytracker/cloudlog.hpp>

void logWarn(rclcpp::Logger logger, const std::string &msg)
{
  RCLCPP_WARN(logger, "%s", msg.c_str());
}

std::set<std::string> extract_names(
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
    const std::string &pattern)
{
  std::set<std::string> result;
  for (const auto &i : parameter_overrides)
  {
    if (i.first.find(pattern) == 0)
    {
      size_t start = pattern.size() + 1;
      size_t end = i.first.find(".", start);
      result.insert(i.first.substr(start, end - start));
    }
  }
  return result;
}

std::vector<double> get_vec(const rclcpp::ParameterValue &param_value)
{
  if (param_value.get_type() == rclcpp::PARAMETER_INTEGER_ARRAY)
  {
    const auto int_vec = param_value.get<std::vector<int64_t>>();
    std::vector<double> result;
    for (int v : int_vec)
    {
      result.push_back(v);
    }
    return result;
  }
  return param_value.get<std::vector<double>>();
}

Eigen::Vector3f quaternionToEuler(const Eigen::Quaternionf &q)
{
  Eigen::Vector3f euler;
  float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

  // Roll
  float sinr_cosp = 2 * (qw * qx + qy * qz);
  float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  euler.x() = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch
  float sinp = 2 * (qw * qy - qz * qx);
  if (std::abs(sinp) >= 1)
    euler.y() = std::copysign(M_PI / 2, sinp);
  else
    euler.y() = std::asin(sinp);

  //// ANOTHER IMPLEMENTATION VERSION
  // float sinp = std::sqrt(1 + 2 * (qw * qy - qx * qz));
  // float cosp = std::sqrt(1 - 2 * (qw * qy - qx * qz));
  // euler.y()= 2 * std::atan2(sinp, cosp) - M_PI / 2;

  // Yaw
  float siny_cosp = 2 * (qw * qz + qx * qy);
  float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  euler.z() = std::atan2(siny_cosp, cosy_cosp);

  return euler;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motion_capture_tracking_node");
  node->declare_parameter<std::string>("type", "vicon");
  node->declare_parameter<std::string>("hostname", "localhost");
  node->declare_parameter<std::string>("topics.poses.qos.mode", "none");
  node->declare_parameter<double>("topics.poses.qos.deadline", 100.0);
  node->declare_parameter<std::string>("logfilepath", "");

  std::string motionCaptureType = node->get_parameter("type").as_string();
  std::string motionCaptureHostname = node->get_parameter("hostname").as_string();
  std::string poses_qos = node->get_parameter("topics.poses.qos.mode").as_string();
  double poses_deadline = node->get_parameter("topics.poses.qos.deadline").as_double();
  std::string logFilePath = node->get_parameter("logfilepath").as_string();

  auto node_parameters_iface = node->get_node_parameters_interface();
  const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
      node_parameters_iface->get_parameter_overrides();

  librigidbodytracker::PointCloudLogger pointCloudLogger(logFilePath);
  const bool logClouds = !logFilePath.empty();
  std::cout << "logClouds=" << logClouds << std::endl; // 1

  // Make a new client
  std::map<std::string, std::string> cfg;
  cfg["hostname"] = motionCaptureHostname;

  // if the mock type is selected, add the defined rigid bodies
  if (motionCaptureType == "mock")
  {
    auto rigid_body_names = extract_names(parameter_overrides, "rigid_bodies");
    for (const auto &name : rigid_body_names)
    {
      const auto pos = get_vec(parameter_overrides.at("rigid_bodies." + name + ".initial_position"));
      cfg["rigid_bodies"] += name + "(" + std::to_string(pos[0]) + "," + std::to_string(pos[1]) + "," + std::to_string(pos[2]) + ",1,0,0,0);";
    }
  }

  libmotioncapture::MotionCapture *mocap = libmotioncapture::MotionCapture::connect(motionCaptureType, cfg);

  // prepare point cloud publisher
  auto pubPointCloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("pointCloud", 1);

  sensor_msgs::msg::PointCloud2 msgPointCloud;
  msgPointCloud.header.frame_id = "world";
  msgPointCloud.height = 1;

  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  msgPointCloud.fields.push_back(field);
  field.name = "y";
  field.offset = 4;
  msgPointCloud.fields.push_back(field);
  field.name = "z";
  field.offset = 8;
  msgPointCloud.fields.push_back(field);
  msgPointCloud.point_step = 12;
  msgPointCloud.is_bigendian = false;
  msgPointCloud.is_dense = true;

  // prepare pose array publisher
  rclcpp::Publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr pubPoses;
  if (poses_qos == "none")
  {
    pubPoses = node->create_publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>("poses", 1);
  }
  else if (poses_qos == "sensor")
  {
    rclcpp::SensorDataQoS sensor_data_qos;
    sensor_data_qos.keep_last(1);
    sensor_data_qos.deadline(rclcpp::Duration(0 /*s*/, (int)1e9 / poses_deadline /*ns*/));
    pubPoses = node->create_publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>("poses", sensor_data_qos);
  }
  else
  {
    throw std::runtime_error("Unknown QoS mode! " + poses_qos);
  }

  motion_capture_tracking_interfaces::msg::NamedPoseArray msgPoses;
  msgPoses.header.frame_id = "world";

  // prepare rigid body tracker

  auto dynamics_config_names = extract_names(parameter_overrides, "dynamics_configurations");
  std::vector<librigidbodytracker::DynamicsConfiguration> dynamicsConfigurations(dynamics_config_names.size());
  std::map<std::string, size_t> dynamics_name_to_index;
  size_t i = 0;
  for (const auto &name : dynamics_config_names)
  {
    const auto max_vel = get_vec(parameter_overrides.at("dynamics_configurations." + name + ".max_velocity"));
    dynamicsConfigurations[i].maxXVelocity = max_vel.at(0);
    dynamicsConfigurations[i].maxYVelocity = max_vel.at(1);
    dynamicsConfigurations[i].maxZVelocity = max_vel.at(2);
    const auto max_angular_velocity = get_vec(parameter_overrides.at("dynamics_configurations." + name + ".max_angular_velocity"));
    dynamicsConfigurations[i].maxRollRate = max_angular_velocity.at(0);
    dynamicsConfigurations[i].maxPitchRate = max_angular_velocity.at(1);
    dynamicsConfigurations[i].maxYawRate = max_angular_velocity.at(2);
    dynamicsConfigurations[i].maxRoll = parameter_overrides.at("dynamics_configurations." + name + ".max_roll").get<double>();
    dynamicsConfigurations[i].maxPitch = parameter_overrides.at("dynamics_configurations." + name + ".max_pitch").get<double>();
    dynamicsConfigurations[i].maxFitnessScore = parameter_overrides.at("dynamics_configurations." + name + ".max_fitness_score").get<double>();
    dynamics_name_to_index[name] = i;
    ++i;
  }

  auto marker_config_names = extract_names(parameter_overrides, "marker_configurations");
  std::vector<librigidbodytracker::MarkerConfiguration> markerConfigurations;
  std::map<std::string, size_t> marker_name_to_index;
  i = 0;
  for (const auto &name : marker_config_names)
  {
    markerConfigurations.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
    const auto offset = get_vec(parameter_overrides.at("marker_configurations." + name + ".offset"));
    for (const auto &param : parameter_overrides)
    {
      if (param.first.find("marker_configurations." + name + ".points") == 0)
      {
        const auto points = get_vec(param.second);
        markerConfigurations.back()->push_back(pcl::PointXYZ(points[0] + offset[0], points[1] + offset[1], points[2] + offset[2]));
      }
    }
    marker_name_to_index[name] = i;
    ++i;
  }

  std::vector<librigidbodytracker::RigidBody> rigidBodies;
  // only add the rigid bodies to the tracker if we are not using the "mock" mode
  if (motionCaptureType != "mock")
  {
    auto rigid_body_names = extract_names(parameter_overrides, "rigid_bodies");
    for (const auto &name : rigid_body_names)
    {
      const auto pos = get_vec(parameter_overrides.at("rigid_bodies." + name + ".initial_position"));
      Eigen::Affine3f m;
      m = Eigen::Translation3f(pos[0], pos[1], pos[2]);
      const auto marker = parameter_overrides.at("rigid_bodies." + name + ".marker").get<std::string>();
      const auto dynamics = parameter_overrides.at("rigid_bodies." + name + ".dynamics").get<std::string>();

      rigidBodies.push_back(librigidbodytracker::RigidBody(marker_name_to_index.at(marker), dynamics_name_to_index.at(dynamics), m, name));
    }
  }

  librigidbodytracker::RigidBodyTracker tracker(
      dynamicsConfigurations,
      markerConfigurations,
      rigidBodies);
  tracker.setLogWarningCallback(std::bind(logWarn, node->get_logger(), std::placeholders::_1));

  // prepare TF broadcaster
  tf2_ros::TransformBroadcaster tfbroadcaster(node);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);

  // Try to smooth out velocities
  std::unordered_map<std::string, std::deque<Eigen::Vector3f>> velocity_buffers;
  const int VELOCITY_WINDOW_SIZE = 8;

  std::unordered_map<std::string, std::deque<Eigen::Vector3f>> angular_velocity_buffers;
  // using the same size as the linear window for now, maybe configure a different one later if it is needed

  // Structures to save the data we're working with
  std::map<std::string, std::pair<Eigen::Vector3f, rclcpp::Time>> previous_positions;
  std::map<std::string, std::pair<Eigen::Quaternionf, rclcpp::Time>> previous_orientations;
  std::map<std::string, std::pair<Eigen::Vector3f, rclcpp::Time>> previous_linear_vel;
  std::map<std::string, std::pair<Eigen::Vector3f, rclcpp::Time>> previous_angular_vel;

  // Variables to control and select correctly when computing at a bigger frame difference
  std::map<std::string, int> lastFrameIDLinear;
  std::map<std::string, int> lastFrameIDAngular;

  // Unwrap the yaw angle when it makes a full circle turn
  double yaw_err = 0.0;
  long int currentFrameID = -1;

  // At how many frames I want to compute the velocity, 1 for a per-frame basis and more for a 5/10 frame difference
  int FRAME_DIFF = 1;

  // poses_deadline variable hold the QOS for the frame frquency

  for (size_t frameId = 0; rclcpp::ok(); ++frameId)
  {
    currentFrameID = currentFrameID + 1;

    // Get a frame
    mocap->waitForNextFrame();
    auto chrono_now = std::chrono::high_resolution_clock::now();
    auto time = node->now();
    auto pointcloud = mocap->pointCloud();

    // publish as pointcloud
    msgPointCloud.header.stamp = time;

    msgPointCloud.width = pointcloud.rows();
    msgPointCloud.data.resize(pointcloud.rows() * 3 * 4); // width * height * pointstep
    memcpy(msgPointCloud.data.data(), pointcloud.data(), msgPointCloud.data.size());
    msgPointCloud.row_step = msgPointCloud.data.size();

    pubPointCloud->publish(msgPointCloud);
    if (logClouds)
    {
      // pointCloudLogger.log(timestamp/1000, markers);  // point cloud log format: infinite repetitions of:  timestamp (milliseconds) : uint32
      // std::cout << "0000000000000before log" << std::endl;
      pointCloudLogger.log(markers);
    }

    // run tracker
    markers->clear();
    for (long int i = 0; i < pointcloud.rows(); ++i)
    {
      const auto &point = pointcloud.row(i);
      markers->push_back(pcl::PointXYZ(point(0), point(1), point(2)));
    }
    tracker.update(markers);

    transforms.clear();
    transforms.reserve(mocap->rigidBodies().size());
    for (const auto &iter : mocap->rigidBodies())
    {
      const auto &rigidBody = iter.second;

      // const auto& transform = rigidBody.transformation();
      // transforms.emplace_back(eigenToTransform(transform));
      transforms.resize(transforms.size() + 1);
      transforms.back().header.stamp = time;
      transforms.back().header.frame_id = "world";
      transforms.back().child_frame_id = rigidBody.name();
      transforms.back().transform.translation.x = rigidBody.position().x();
      transforms.back().transform.translation.y = rigidBody.position().y();
      transforms.back().transform.translation.z = rigidBody.position().z();
      transforms.back().transform.rotation.x = rigidBody.rotation().x();
      transforms.back().transform.rotation.y = rigidBody.rotation().y();
      transforms.back().transform.rotation.z = rigidBody.rotation().z();
      transforms.back().transform.rotation.w = rigidBody.rotation().w();
    }

    for (const auto &rigidBody : tracker.rigidBodies())
    {
      if (rigidBody.lastTransformationValid())
      {
        const Eigen::Affine3f &transform = rigidBody.transformation();
        Eigen::Quaternionf q(transform.rotation());
        const auto &translation = transform.translation();

        transforms.resize(transforms.size() + 1);
        transforms.back().header.stamp = time;
        transforms.back().header.frame_id = "world";
        transforms.back().child_frame_id = rigidBody.name();
        transforms.back().transform.translation.x = translation.x();
        transforms.back().transform.translation.y = translation.y();
        transforms.back().transform.translation.z = translation.z();
        if (rigidBody.orientationAvailable())
        {
          transforms.back().transform.rotation.x = q.x();
          transforms.back().transform.rotation.y = q.y();
          transforms.back().transform.rotation.z = q.z();
          transforms.back().transform.rotation.w = q.w();
        }
        else
        {
          transforms.back().transform.rotation.x = std::nan("");
          transforms.back().transform.rotation.y = std::nan("");
          transforms.back().transform.rotation.z = std::nan("");
          transforms.back().transform.rotation.w = std::nan("");
        }
      }
      else
      {
        std::chrono::duration<double> elapsedSeconds = chrono_now - rigidBody.lastValidTime();
        RCLCPP_WARN(node->get_logger(), "No updated pose for %s for %f s.", rigidBody.name().c_str(), elapsedSeconds.count());
      }
    }

    if (transforms.size() > 0)
    {
      // publish poses
      msgPoses.header.stamp = time;
      msgPoses.poses.resize(transforms.size());

      for (size_t i = 0; i < transforms.size(); ++i)
      {
        msgPoses.poses[i].name = transforms[i].child_frame_id;
        msgPoses.poses[i].pose.position.x = transforms[i].transform.translation.x;
        msgPoses.poses[i].pose.position.y = transforms[i].transform.translation.y;
        msgPoses.poses[i].pose.position.z = transforms[i].transform.translation.z;
        msgPoses.poses[i].pose.orientation = transforms[i].transform.rotation;

        Eigen::Vector3f linear_velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f angular_velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

        rclcpp::Time current_time(msgPoses.header.stamp.sec, msgPoses.header.stamp.nanosec);

        if (previous_positions.find(msgPoses.poses[i].name) == previous_positions.end())
        {
          // Add the first measurement of the position for the first frame iteration, also make the speed 0
          previous_positions[msgPoses.poses[i].name] = {Eigen::Vector3f(transforms[i].transform.translation.x, transforms[i].transform.translation.y, transforms[i].transform.translation.z), current_time};
          previous_linear_vel[msgPoses.poses[i].name] = {linear_velocity, current_time};
          // Initialize the velocity buffer for this drone
          velocity_buffers[msgPoses.poses[i].name] = std::deque<Eigen::Vector3f>();
        }
        else
        {
          if (previous_linear_vel.find(msgPoses.poses[i].name) == previous_linear_vel.end()) // this is impossible to happen, just a guard clause (because when we save the first positional data we also add the 0 measurement for speed)
          {
            previous_linear_vel[msgPoses.poses[i].name] = {linear_velocity, current_time};
          }
          else // This is the normal functioning when we compute the speeds at the frame difference that we want
          {
            const auto &[prev_lin_vel, prev_time] = previous_linear_vel[msgPoses.poses[i].name];

            if (currentFrameID - lastFrameIDLinear[msgPoses.poses[i].name] < FRAME_DIFF)
            {
              linear_velocity = prev_lin_vel;
            }
            else
            {
              // COMPUTE THE NEW VELOCITY AT EVERY FRAME_DIFF FRAMES
              const auto &[prev_position, prev_time_pos] = previous_positions[msgPoses.poses[i].name];
              double dt = std::round((current_time - prev_time).seconds() * 1000) / 1000; // rounding to the third decimal

              if (dt >= 0.009) // special case when we compute at a per-frame basis (FRAME_DIFF = 1) to make sure we divide by a positive time constant (encountered cases where this would be 0.000 because frames come in too fast)
              {
                linear_velocity = (Eigen::Vector3f(transforms[i].transform.translation.x,
                                                   transforms[i].transform.translation.y,
                                                   transforms[i].transform.translation.z) -
                                   prev_position) /
                                  dt;

                // Add the current velocity to the buffer for smoothing
                velocity_buffers[msgPoses.poses[i].name].push_back(linear_velocity);
                // If the buffer exceeds the desired size, remove the oldest velocity
                if (velocity_buffers[msgPoses.poses[i].name].size() > VELOCITY_WINDOW_SIZE)
                  velocity_buffers[msgPoses.poses[i].name].pop_front();

                // Apply the Simple Moving Average Filter
                Eigen::Vector3f smoothed_velocity = Eigen::Vector3f::Zero();
                for (const auto &v : velocity_buffers[msgPoses.poses[i].name])
                  smoothed_velocity += v;
                
                smoothed_velocity /= velocity_buffers[msgPoses.poses[i].name].size();
                linear_velocity = smoothed_velocity;

                previous_linear_vel[msgPoses.poses[i].name] = {linear_velocity, current_time};
                previous_positions[msgPoses.poses[i].name] = {Eigen::Vector3f(transforms[i].transform.translation.x,
                                                                              transforms[i].transform.translation.y, transforms[i].transform.translation.z),
                                                              current_time};
                lastFrameIDLinear[msgPoses.poses[i].name] = currentFrameID;
              }
              else // When dt is too small
              {
                linear_velocity = prev_lin_vel;
              }
            }
          }
        }

        if (previous_orientations.find(msgPoses.poses[i].name) == previous_orientations.end())
        {
          // Add the first measurement of the angles for the first frame iteration, also make the speed 0
          previous_orientations[msgPoses.poses[i].name] = {Eigen::Quaternionf(transforms[i].transform.rotation.w,
                                                                              transforms[i].transform.rotation.x, transforms[i].transform.rotation.y, transforms[i].transform.rotation.z),
                                                           current_time};
          previous_angular_vel[msgPoses.poses[i].name] = {angular_velocity, current_time};
          angular_velocity_buffers[msgPoses.poses[i].name] = std::deque<Eigen::Vector3f>(); // Initialize the velocity buffer for this drone
        }
        else
        {
          if (previous_angular_vel.find(msgPoses.poses[i].name) == previous_angular_vel.end()) // this is impossible to happen, just a guard clause (because when we save the first angular data we also add the 0 measurement for speed)
          {
            previous_angular_vel[msgPoses.poses[i].name] = {angular_velocity, current_time};
          }
          else // This is the normal functioning when we compute the speeds at the frame difference that we want
          {
            const auto &[prev_ang_vel, prev_time] = previous_angular_vel[msgPoses.poses[i].name];

            if (currentFrameID - lastFrameIDAngular[msgPoses.poses[i].name] < FRAME_DIFF)
            {
              angular_velocity = prev_ang_vel;
            }
            else
            {
              // COMPUTE THE NEW VELOCITY AT EVERY FRAME_DIFF FRAMES
              const auto &[prev_orientation, _] = previous_orientations[msgPoses.poses[i].name];
              double dt = std::round((current_time - prev_time).seconds() * 1000) / 1000;

              Eigen::Vector3f prev_euler_angles = quaternionToEuler(prev_orientation);
              Eigen::Vector3f euler_angles = quaternionToEuler(Eigen::Quaternionf(transforms[i].transform.rotation.w,
                                                                                  transforms[i].transform.rotation.x, transforms[i].transform.rotation.y, transforms[i].transform.rotation.z));

              // UNWRAP THE YAW ANGLE
              yaw_err = euler_angles.z() - prev_euler_angles.z();
              if (yaw_err > M_PI || yaw_err < -M_PI)
              {
                if (yaw_err > 0)
                  yaw_err = -yaw_err - 2 * M_PI;
                else
                  yaw_err = 2 * M_PI + yaw_err;
              }

              if (dt >= 0.009) // special case when we compute at a per-frame basis (FRAME_DIFF = 1) to make sure we divide by a positive time constant (encountered cases where this would be 0.000 because frames come in too fast)
              {
                angular_velocity = (euler_angles - prev_euler_angles) / dt;
                angular_velocity.z() = -yaw_err / dt;

                // Add the current velocity to the buffer for smoothing
                angular_velocity_buffers[msgPoses.poses[i].name].push_back(angular_velocity);
                // If the buffer exceeds the desired size, remove the oldest velocity
                if (angular_velocity_buffers[msgPoses.poses[i].name].size() > VELOCITY_WINDOW_SIZE)
                  angular_velocity_buffers[msgPoses.poses[i].name].pop_front();

                // Apply the Simple Moving Average Filter
                Eigen::Vector3f smoothed_ang_velocity = Eigen::Vector3f::Zero();
                for (const auto &v : angular_velocity_buffers[msgPoses.poses[i].name])
                  smoothed_ang_velocity += v;

                smoothed_ang_velocity /= angular_velocity_buffers[msgPoses.poses[i].name].size();
                angular_velocity = smoothed_ang_velocity;

                previous_angular_vel[msgPoses.poses[i].name] = {angular_velocity, current_time};
                previous_orientations[msgPoses.poses[i].name] = {Eigen::Quaternionf(transforms[i].transform.rotation.w,
                                                                                    transforms[i].transform.rotation.x, transforms[i].transform.rotation.y, transforms[i].transform.rotation.z),
                                                                 current_time};
                lastFrameIDAngular[msgPoses.poses[i].name] = currentFrameID;
              }
              else // When dt is too small
              {
                angular_velocity = prev_ang_vel;
              }
            }
          }
        }

        msgPoses.poses[i].velocity.linear.x = linear_velocity.x();
        msgPoses.poses[i].velocity.linear.y = linear_velocity.y();
        msgPoses.poses[i].velocity.linear.z = linear_velocity.z();
        msgPoses.poses[i].velocity.angular.x = angular_velocity.x();
        msgPoses.poses[i].velocity.angular.y = angular_velocity.y();
        msgPoses.poses[i].velocity.angular.z = angular_velocity.z();
      }
      pubPoses->publish(msgPoses);

      // send TF. Since RViz and others can't handle nan's, report a fake oriention if needed
      for (auto &tf : transforms)
      {
        if (std::isnan(tf.transform.rotation.x))
        {
          tf.transform.rotation.x = 0;
          tf.transform.rotation.y = 0;
          tf.transform.rotation.z = 0;
          tf.transform.rotation.w = 1;
        }
      }

      tfbroadcaster.sendTransform(transforms);
    }
    if (logClouds)
    {
      pointCloudLogger.flush();
    }
    rclcpp::spin_some(node);
  }

  if (logClouds)
  {
    pointCloudLogger.flush();
  }

  return 0;
}