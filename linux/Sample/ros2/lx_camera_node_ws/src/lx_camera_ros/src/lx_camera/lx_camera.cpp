#include "lx_camera/lx_camera.h"
#include "rclcpp/callback_group.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include "utils/json.hpp"
#include <cv_bridge/cv_bridge.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>

static DcLib *LX_DYNAMIC_LIB = nullptr;

geometry_msgs::msg::TransformStamped PoseToTf(const Eigen::Matrix4f &pose) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform.translation.x = pose(0, 3);
  transform_stamped.transform.translation.y = pose(1, 3);
  transform_stamped.transform.translation.z = pose(2, 3);
  Eigen::Matrix3f rotation_matrix = pose.topLeftCorner(3, 3);
  Eigen::Quaternionf quat(rotation_matrix);
  transform_stamped.transform.rotation.x = quat.x();
  transform_stamped.transform.rotation.y = quat.y();
  transform_stamped.transform.rotation.z = quat.z();
  transform_stamped.transform.rotation.w = quat.w();
  return transform_stamped;
}

LxCamera::LxCamera(DcLib *dynamic_lib, const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("lx_camera_node", options) {
  RCLCPP_INFO(this->get_logger(), "lx_camera_node start!");
  LX_DYNAMIC_LIB = dynamic_lib;
  qos_ = rmw_qos_profile_default;
}

LxCamera::~LxCamera() {
  StopWorker();
  if (is_start_) {
    Stop();
  }
  if (is_device_open_) {
    DcCloseDevice(handle_);
    is_device_open_ = false;
  }
}

void LxCamera::ActivatePublishers() {
  if (pub_rgb_) pub_rgb_->on_activate();
  if (pub_rgb_info_) pub_rgb_info_->on_activate();
  if (pub_amp_) pub_amp_->on_activate();
  if (pub_depth_) pub_depth_->on_activate();
  if (pub_tof_info_) pub_tof_info_->on_activate();
  if (pub_pallet_) pub_pallet_->on_activate();
  if (pub_temper_) pub_temper_->on_activate();
  if (pub_obstacle_) pub_obstacle_->on_activate();
  if (pub_cloud_) pub_cloud_->on_activate();
  if (pub_tf_) pub_tf_->on_activate();
  if (pub_location_) pub_location_->on_activate();
}

void LxCamera::DeactivatePublishers() {
  if (pub_rgb_) pub_rgb_->on_deactivate();
  if (pub_rgb_info_) pub_rgb_info_->on_deactivate();
  if (pub_amp_) pub_amp_->on_deactivate();
  if (pub_depth_) pub_depth_->on_deactivate();
  if (pub_tof_info_) pub_tof_info_->on_deactivate();
  if (pub_pallet_) pub_pallet_->on_deactivate();
  if (pub_temper_) pub_temper_->on_deactivate();
  if (pub_obstacle_) pub_obstacle_->on_deactivate();
  if (pub_cloud_) pub_cloud_->on_deactivate();
  if (pub_tf_) pub_tf_->on_deactivate();
  if (pub_location_) pub_location_->on_deactivate();
}

void LxCamera::StopWorker() {
  run_worker_.store(false);
  if (run_thread_.joinable()) {
    run_thread_.join();
  }
}

void LxCamera::UpdateDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &status) {
  std::lock_guard<std::mutex> lock(diagnostic_mutex_);
  const auto now = std::chrono::steady_clock::now();

  if (!is_device_open_) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                   "Device is not open");
  } else if (!is_start_) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   "Stream is not started");
  } else if (!has_frame_) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   "Waiting for first frame");
  } else {
    const auto age = now - last_frame_time_;
    if (age > frame_timeout_) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                     "No frames received recently");
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                     "Camera streaming normally");
    }
    const auto age_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(age).count();
    status.add("last_frame_age_ms", static_cast<int>(age_ms));
  }

  status.add("device_open", is_device_open_ ? 1 : 0);
  status.add("stream_started", is_start_ ? 1 : 0);
}

bool LxCamera::ConfigureDevice() {
  auto fail = [this]() {
    if (is_start_) {
      Stop();
    }
    if (is_device_open_) {
      DcCloseDevice(handle_);
      handle_ = 0;
      is_device_open_ = false;
    }
    return false;
  };
  // set sdk log
  RCLCPP_INFO(this->get_logger(), "Api version: %s", DcGetApiVersion());
  DcSetInfoOutput(1, true, log_path_.c_str());

  if (!SearchAndOpenDevice()) {
    return false;
  }
  is_device_open_ = true;

  if (raw_param_) {
    SetParam();
  }
  if (Check("LX_INT_ALGORITHM_MODE",
            DcSetIntValue(handle_, LX_INT_ALGORITHM_MODE, inside_app_)) !=
      LX_SUCCESS) {
    return fail();
  }
  if (Check("LX_BOOL_ENABLE_3D_DEPTH_STREAM",
            DcSetBoolValue(handle_, LX_BOOL_ENABLE_3D_DEPTH_STREAM,
                           is_xyz_ || is_depth_)) != LX_SUCCESS) {
    return fail();
  }
  if (Check("LX_BOOL_ENABLE_3D_AMP_STREAM",
            DcSetBoolValue(handle_, LX_BOOL_ENABLE_3D_AMP_STREAM, is_amp_)) !=
      LX_SUCCESS) {
    return fail();
  }
  if (Check("LX_BOOL_ENABLE_2D_STREAM",
            DcSetBoolValue(handle_, LX_BOOL_ENABLE_2D_STREAM, is_rgb_)) !=
      LX_SUCCESS) {
    return fail();
  }
  if (Check("LX_INT_WORK_MODE",
            DcSetIntValue(handle_, LX_INT_WORK_MODE, lx_work_mode_)) !=
      LX_SUCCESS) {
    return fail();
  }
  return true;
}

LxCamera::LifecycleCBReturn
LxCamera::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "on_configure()...");
  ReadParam();
  pub_rgb_ = this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Rgb", 1);
  pub_rgb_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "LxCamera_RgbInfo", 1);
  pub_amp_ = this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Amp", 1);
  pub_depth_ =
      this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Depth", 1);
  pub_tof_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "LxCamera_TofInfo", 1);
  pub_error_ =
      this->create_publisher<std_msgs::msg::String>("LxCamera_Error", 10);

  pub_pallet_ =
      this->create_publisher<lx_camera_ros::msg::Pallet>("LxCamera_Pallet", 1);
  pub_temper_ = this->create_publisher<lx_camera_ros::msg::FrameRate>(
      "LxCamera_FrameRate", 1);
  pub_obstacle_ = this->create_publisher<lx_camera_ros::msg::Obstacle>(
      "LxCamera_Obstacle", 1);
  pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "LxCamera_Cloud", 1);
  pub_tf_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
      "LxCamera_TF", 1);
  pub_location_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "LxCamera_Location", 1);

  srv_cmd_ = this->create_service<lx_camera_ros::srv::LxCmd>(
      "LxCamera_LxCmd", std::bind(&LxCamera::LxCmd, this, std::placeholders::_1,
                                  std::placeholders::_2));
  srv_int_ = this->create_service<lx_camera_ros::srv::LxInt>(
      "LxCamera_LxInt", std::bind(&LxCamera::LxInt, this, std::placeholders::_1,
                                  std::placeholders::_2));
  srv_bool_ = this->create_service<lx_camera_ros::srv::LxBool>(
      "LxCamera_LxBool",
      std::bind(&LxCamera::LxBool, this, std::placeholders::_1,
                std::placeholders::_2));
  srv_float_ = this->create_service<lx_camera_ros::srv::LxFloat>(
      "LxCamera_LxFloat",
      std::bind(&LxCamera::LxFloat, this, std::placeholders::_1,
                std::placeholders::_2));
  srv_string_ = this->create_service<lx_camera_ros::srv::LxString>(
      "LxCamera_LxString",
      std::bind(&LxCamera::LxString, this, std::placeholders::_1,
                std::placeholders::_2));
  if (!ConfigureDevice()) {
    return LifecycleCBReturn::FAILURE;
  }
  return LifecycleCBReturn::SUCCESS;
}

LxCamera::LifecycleCBReturn
LxCamera::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "on_activate()...");
  ActivatePublishers();
  if (!is_start_ && Start() != LX_SUCCESS) {
    return LifecycleCBReturn::FAILURE;
  }
  {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    has_frame_ = false;
    last_frame_time_ = std::chrono::steady_clock::now();
  }
  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostic_updater_->setHardwareID("lanxin_mrdvs");
  diagnostic_updater_->setPeriod(1.0);
  diagnostic_updater_->add("camera_stream_status", this,
                           &LxCamera::UpdateDiagnostics);
  run_worker_.store(true);
  run_thread_ = std::thread(&LxCamera::Run, this);
  return LifecycleCBReturn::SUCCESS;
}

LxCamera::LifecycleCBReturn
LxCamera::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "on_deactivate()...");
  StopWorker();
  if (is_start_) {
    Stop();
  }
  diagnostic_updater_.reset();
  DeactivatePublishers();
  return LifecycleCBReturn::SUCCESS;
}

LxCamera::LifecycleCBReturn
LxCamera::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(this->get_logger(), "on_cleanup()...");
  StopWorker();
  if (is_start_) {
    Stop();
  }
  DeactivatePublishers();
  pub_rgb_.reset();
  pub_rgb_info_.reset();
  pub_amp_.reset();
  pub_depth_.reset();
  pub_tof_info_.reset();
  pub_pallet_.reset();
  pub_temper_.reset();
  pub_obstacle_.reset();
  pub_cloud_.reset();
  pub_tf_.reset();
  pub_location_.reset();
  pub_error_.reset();
  diagnostic_updater_.reset();
  srv_cmd_.reset();
  srv_int_.reset();
  srv_bool_.reset();
  srv_float_.reset();
  srv_string_.reset();
  if (is_device_open_) {
    DcCloseDevice(handle_);
    is_device_open_ = false;
    handle_ = 0;
  }
  return LifecycleCBReturn::SUCCESS;
}

LxCamera::LifecycleCBReturn
LxCamera::on_shutdown(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(this->get_logger(), "on_shutdown()...");
  (void)on_cleanup(state);
  return LifecycleCBReturn::SUCCESS;
}

int LxCamera::Start() {
  LxIntValueInfo int_value;
  DcGetIntValue(handle_, LX_INT_3D_IMAGE_WIDTH, &int_value);
  tof_info_.width = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_3D_IMAGE_HEIGHT, &int_value);
  tof_info_.height = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_WIDTH, &int_value);
  rgb_info_.width = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_HEIGHT, &int_value);
  rgb_info_.height = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_DATA_TYPE, &int_value);
  rgb_type_ = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_CHANNEL, &int_value);
  rgb_channel_ = int_value.cur_value;

  DcGetBoolValue(handle_, LX_BOOL_ENABLE_3D_DEPTH_STREAM, (bool *)&is_depth_);
  DcGetBoolValue(handle_, LX_BOOL_ENABLE_3D_AMP_STREAM, (bool *)&is_amp_);
  DcGetBoolValue(handle_, LX_BOOL_ENABLE_2D_STREAM, (bool *)&is_rgb_);
  DcGetIntValue(handle_, LX_INT_ALGORITHM_MODE, &int_value);
  inside_app_ = int_value.cur_value;

  // get image params
  if (is_depth_ || is_amp_ || is_xyz_) {
    float *intr = nullptr, *ex_intr = nullptr;
    DcGetPtrValue(handle_, LX_PTR_3D_NEW_INTRIC_PARAM, (void **)&intr);
    DcGetPtrValue(handle_, LX_PTR_3D_EXTRIC_PARAM, (void **)&ex_intr);
    double d[14]{intr[4], intr[5], intr[6], intr[7], intr[8],intr[9],intr[10],intr[11],intr[12],intr[13],intr[14],intr[15],intr[16],intr[17]};
    double k[9]{intr[0], 0, intr[2], 0, intr[1], intr[3], 0, 0, 1};
    for (int i = 0; i < 9; i++) {
      tof_info_.k[i] = k[i];
    }
    for (int i = 0; i < 9; i++) {
      tof_info_.r[i] = ex_intr[i]; 
    }
    for (int i = 0; i < 14; i++)
    {
      tof_info_.d.push_back(d[i]);
    }
    
    auto q = ToQuaternion(install_yaw_, install_pitch_, install_roll_);
    tf_.transform.translation.x = install_x_;
    tf_.transform.translation.y = install_y_;
    tf_.transform.translation.z = install_z_;
    tf_.transform.rotation.x = q.x;
    tf_.transform.rotation.y = q.y;
    tf_.transform.rotation.z = q.z;
    tf_.transform.rotation.w = q.w;
    tof_info_.header.frame_id = "intrinsic_tof";
    tf_.header.frame_id = "intrinsic_tof";
    tf_.child_frame_id = "mrdvs";
  }

  if (is_rgb_) {
    float *intr = nullptr;
    DcGetPtrValue(handle_, LX_PTR_2D_NEW_INTRIC_PARAM, (void **)&intr);
    double d[14]{intr[4], intr[5], intr[6], intr[7], intr[8],intr[9],intr[10],intr[11],intr[12],intr[13],intr[14],intr[15],intr[16],intr[17]};
    double k[9]{intr[0], 0, intr[2], 0, intr[1], intr[3], 0, 0, 1};
    for (int i = 0; i < 9; i++)
      rgb_info_.k[i] = k[i];
    for (int i = 0;i < 14; i++)  
      rgb_info_.d.push_back(d[i]);

    rgb_info_.header.frame_id = "intrinsic_rgb";
  }
  auto ret = DcStartStream(handle_);
  is_start_ = (ret == LX_SUCCESS);
  return static_cast<int>(ret);
}

int LxCamera::Stop() {
  auto ret = DcStopStream(handle_);
  if (is_start_ && ret == LX_SUCCESS) {
    is_start_ = false;
  }
  return static_cast<int>(ret);
}

void LxCamera::Run() {
  Eigen::Matrix3f R = (Eigen::AngleAxisf(install_yaw_ / 180.f * M_PI,
                                         Eigen::Vector3f::UnitZ()) *
                       Eigen::AngleAxisf(install_pitch_ / 180.f * M_PI,
                                         Eigen::Vector3f::UnitY()) *
                       Eigen::AngleAxisf(install_roll_ / 180.f * M_PI,
                                         Eigen::Vector3f::UnitX()))
                          .toRotationMatrix();
  Eigen::Vector3f t{install_x_, install_y_, install_z_};
  Eigen::Matrix4f ext_base_tof = Eigen::Matrix4f::Identity();
  ext_base_tof.block(0, 0, 3, 3) = R;
  ext_base_tof.block(0, 3, 3, 1) = t;
  geometry_msgs::msg::TransformStamped tf_ext_base_tof = PoseToTf(ext_base_tof);

  Eigen::Matrix4f ext_tof_rgb = Eigen::Matrix4f::Identity();
  geometry_msgs::msg::TransformStamped tf_ext_tof_rgb;
  if ((is_xyz_ || is_depth_ || is_amp_) && is_rgb_) {
    Eigen::Matrix4f ext_rgb_tof = Eigen::Matrix4f::Identity();
    float *ext_param = nullptr;
    DcGetPtrValue(handle_, LX_PTR_3D_EXTRIC_PARAM, (void **)&ext_param);
    ext_rgb_tof << ext_param[0], ext_param[1], ext_param[2],
        ext_param[9] * 0.001, ext_param[3], ext_param[4], ext_param[5],
        ext_param[10] * 0.001, ext_param[6], ext_param[7], ext_param[8],
        ext_param[11] * 0.001, 0, 0, 0, 1;
    ext_tof_rgb = ext_rgb_tof.inverse();
    tf_ext_tof_rgb = PoseToTf(ext_tof_rgb);
    RCLCPP_INFO_STREAM(this->get_logger(), "ext_rgb_tof:" << ext_rgb_tof);
  }

  rclcpp::Rate rate(20);
  while (rclcpp::ok() && run_worker_.load()) {
    FrameInfo *one_frame = nullptr;
    auto sret = DcSetCmd(handle_, LX_CMD_GET_NEW_FRAME);
    if ((LX_SUCCESS != sret) && (LX_E_FRAME_ID_NOT_MATCH != sret) &&
        (LX_E_FRAME_MULTI_MACHINE != sret)) {
      continue;
    }
    if (Check("LX_PTR_FRAME_DATA",
              DcGetPtrValue(handle_, LX_PTR_FRAME_DATA, (void **)&one_frame))) {
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(diagnostic_mutex_);
      has_frame_ = true;
      last_frame_time_ = std::chrono::steady_clock::now();
    }
    if (diagnostic_updater_) {
      diagnostic_updater_->force_update();
    }
    rclcpp::Time now = this->get_clock()->now();
    float dep_fps = 0.0, amp_fps = 0.0, rgb_fps = 0.0, temp = 0.0;
    LxFloatValueInfo f_val;
    const bool has_cloud_sub = pub_cloud_ && pub_cloud_->get_subscription_count() > 0;
    const bool has_depth_sub = pub_depth_ && pub_depth_->get_subscription_count() > 0;
    const bool has_amp_sub = pub_amp_ && pub_amp_->get_subscription_count() > 0;
    const bool has_rgb_sub = pub_rgb_ && pub_rgb_->get_subscription_count() > 0;
    const bool has_tof_info_sub = pub_tof_info_ && pub_tof_info_->get_subscription_count() > 0;
    const bool has_rgb_info_sub = pub_rgb_info_ && pub_rgb_info_->get_subscription_count() > 0;
    const bool has_tf_sub = pub_tf_ && pub_tf_->get_subscription_count() > 0;
    const bool has_fr_sub = pub_temper_ && pub_temper_->get_subscription_count() > 0;
    const bool has_obstacle_sub = pub_obstacle_ && pub_obstacle_->get_subscription_count() > 0;
    const bool has_pallet_sub = pub_pallet_ && pub_pallet_->get_subscription_count() > 0;
    const bool has_location_sub = pub_location_ && pub_location_->get_subscription_count() > 0;

    if (is_xyz_ && has_cloud_sub) {
      float *xyz_data = nullptr;
      if (DcGetPtrValue(handle_, LX_PTR_XYZ_DATA, (void **)&xyz_data) ==
          LX_SUCCESS) {
        sensor_msgs::msg::PointCloud2 msg_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        auto buff_len = tof_info_.width * tof_info_.height;
        cloud->points.resize(buff_len);
        if (lx_tof_unit_) {
          for (long i = 0; i < buff_len; i++) {
            long index = 3 * i;
            cloud->points[i].x = xyz_data[index] / 1000.f;
            cloud->points[i].y = xyz_data[index + 1] / 1000.f;
            cloud->points[i].z = xyz_data[index + 2] / 1000.f;
          }
        } else {
          memcpy(cloud->points.data(), xyz_data, buff_len * 3 * 4);
        }
        cloud->height = tof_info_.height;
        cloud->width = tof_info_.width;
        pcl::toROSMsg(*cloud, msg_cloud);
        int64_t nanoseconds = 
            static_cast<int64_t>(one_frame->depth_data.sensor_timestamp * 1e3);
        msg_cloud.header.stamp.sec = nanoseconds / 1e9;
        msg_cloud.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
        msg_cloud.header.frame_id = "mrdvs_tof";
        pub_cloud_->publish(msg_cloud);
      } else
        RCLCPP_WARN(this->get_logger(), "%s",
                    std::string("Cloud point data is empty!").c_str());
    }

    if (is_depth_ && has_depth_sub) {
      void *dep_data = one_frame->depth_data.frame_data;
       
      if (dep_data) {
        cv_bridge::CvImage cv_img;
        sensor_msgs::msg::Image msg_depth;

        cv::Mat dep_img(one_frame->depth_data.frame_height,one_frame->depth_data.frame_width, 
                CV_MAKETYPE(one_frame->depth_data.frame_data_type, one_frame->depth_data.frame_channel), dep_data);
     
        cv::Mat dist_img;
        if (dep_img.type() == CV_32F) {
          cv::normalize(dep_img, dist_img, 0, 65535, cv::NORM_MINMAX);
          dist_img.convertTo(dist_img, CV_16UC1);
          cv_img.image = dist_img;
        }else {
          cv_img.image = dep_img;  
        }  
                   
        int64_t nanoseconds =
            static_cast<int64_t>(one_frame->depth_data.sensor_timestamp * 1e3);
        cv_img.header.stamp.sec = nanoseconds / 1e9;
        cv_img.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
      
        cv_img.header.frame_id = "mrdvs_tof";
        cv_img.encoding = "mono16";
        cv_img.toImageMsg(msg_depth);
        pub_depth_->publish(msg_depth);       
      } else
        RCLCPP_WARN(this->get_logger(), "%s",
                    std::string("Depth image is empty!").c_str());
      Check("LX_FLOAT_3D_DEPTH_FPS",
            DcGetFloatValue(handle_, LX_FLOAT_3D_DEPTH_FPS, &f_val));
      dep_fps = f_val.cur_value;      
    }   

    if (is_amp_ && has_amp_sub) {
      void *amp_data = one_frame->amp_data.frame_data;
      if (amp_data) {
        cv_bridge::CvImage cv_img;
        sensor_msgs::msg::Image msg_amp;

        cv::Mat amp_img(one_frame->amp_data.frame_height,one_frame->amp_data.frame_width, 
                CV_MAKETYPE(one_frame->amp_data.frame_data_type, one_frame->amp_data.frame_channel), amp_data);
        if (amp_img.type() == CV_8UC1) {
          cv_img.encoding = "mono8";
        }else {
          cv_img.encoding = "mono16";
        }
                        
        int64_t nanoseconds =
            static_cast<int64_t>(one_frame->amp_data.sensor_timestamp * 1e3);
        cv_img.header.stamp.sec = nanoseconds / 1e9;
        cv_img.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
        cv_img.header.frame_id = "mrdvs_tof";
        cv_img.image = amp_img;
        cv_img.toImageMsg(msg_amp);
        pub_amp_->publish(msg_amp);
      } else
        RCLCPP_WARN(this->get_logger(), "%s",
                    std::string("Amplitude image is empty!").c_str());
      Check("LX_FLOAT_3D_AMPLITUDE_FPS",
            DcGetFloatValue(handle_, LX_FLOAT_3D_AMPLITUDE_FPS, &f_val));
      amp_fps = f_val.cur_value;
    }
    

    if (is_rgb_ && has_rgb_sub) {
      void *rgb_data = one_frame->rgb_data.frame_data;
      if (rgb_data) {
        cv::Mat rgb_pub;
        cv_bridge::CvImage cv_img;
        sensor_msgs::msg::Image msg_rgb;
        auto type = CV_MAKETYPE(rgb_type_, rgb_channel_);
        cv::Mat rgb_img(one_frame->rgb_data.frame_height,
                        one_frame->rgb_data.frame_width, type, rgb_data);
        rgb_img.convertTo(rgb_pub, CV_8UC1, rgb_type_ == CV_16U ? 0.25 : 1);
        std::string rgb_type_ = rgb_channel_ == 3 ? "bgr8" : "mono8";
        int64_t nanoseconds =
            static_cast<int64_t>(one_frame->rgb_data.sensor_timestamp * 1e3);
        cv_img.header.stamp.sec = nanoseconds / 1e9;
        cv_img.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
        cv_img.header.frame_id = "mrdvs_rgb";
        cv_img.encoding = rgb_type_;
        cv_img.image = rgb_pub;
        cv_img.toImageMsg(msg_rgb);
        pub_rgb_->publish(msg_rgb);
      } else
        RCLCPP_WARN(this->get_logger(), "%s",
                    std::string("RGB image is empty!").c_str());
      Check("LX_FLOAT_2D_IMAGE_FPS",
            DcGetFloatValue(handle_, LX_FLOAT_2D_IMAGE_FPS, &f_val));
      rgb_fps = f_val.cur_value;
    }
     
    if ((is_amp_ || is_depth_ || is_xyz_) && has_tof_info_sub) {
      tof_info_.header.stamp = now;
      pub_tof_info_->publish(tof_info_);
    }
    if (is_rgb_ && has_rgb_info_sub) {
      rgb_info_.header.stamp = now;
      pub_rgb_info_->publish(rgb_info_);
    }
    if (is_xyz_ && has_tf_sub) {
      tf_.header.stamp = now;
      pub_tf_->publish(tf_);
    }

    if (has_fr_sub) {
      Check("LX_FLOAT_DEVICE_TEMPERATURE",
            DcGetFloatValue(handle_, LX_FLOAT_DEVICE_TEMPERATURE, &f_val));
      temp = f_val.cur_value;
      lx_camera_ros::msg::FrameRate fr;
      fr.header.frame_id = "mrdvs";
      fr.header.stamp = now;
      fr.amp = amp_fps;
      fr.rgb = rgb_fps;
      fr.depth = dep_fps;
      fr.temperature = temp;
      pub_temper_->publish(fr);
    }

    // pub TF
    tf_ext_base_tof.header.stamp = now;
    tf_ext_base_tof.header.frame_id = "base_link";
    tf_ext_base_tof.child_frame_id = "mrdvs_tof";
    PubTf(tf_ext_base_tof);
    if ((is_xyz_ || is_depth_ || is_amp_) && is_rgb_) {
      tf_ext_tof_rgb.header.stamp = now;
      tf_ext_tof_rgb.header.frame_id = "mrdvs_tof";
      tf_ext_tof_rgb.child_frame_id = "mrdvs_rgb";
      PubTf(tf_ext_tof_rgb);
    }  

    int ret = 0;
    void *app_ptr = one_frame->app_data.frame_data;
    switch (inside_app_) {
    case MODE_AVOID_OBSTACLE: {
      if (!has_obstacle_sub) {
        break;
      }
      lx_camera_ros::msg::Obstacle result;
      result.header.frame_id = "mrdvs";
      int64_t nanoseconds =
          static_cast<int64_t>(one_frame->app_data.sensor_timestamp * 1e3);
      result.header.stamp.sec = nanoseconds / 1e9;
      result.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
      Check("GetObstacleIO", DcSpecialControl(handle_, "GetObstacleIO",
                                              (void *)&result.io_output));
      if (ret || !app_ptr) {
        result.status = -1;
        pub_obstacle_->publish(result);
        break;
      }
      LxAvoidanceOutput *lao = (LxAvoidanceOutput *)app_ptr;
      result.status = lao->state;
      result.box_number = lao->number_box;
      for (int i = 0; i < lao->number_box; i++) {
        auto raw_box = lao->obstacleBoxs[i];
        lx_camera_ros::msg::ObstacleBox box;
        box.width = raw_box.width;
        box.depth = raw_box.depth;
        box.height = raw_box.height;
        for (int t = 0; t < 3; t++)
          box.center[t] = raw_box.center[t];
        for (int t = 0; t < 9; t++)
          box.rotation[t] = raw_box.pose.R[t];
        for (int t = 0; t < 3; t++)
          box.translation[t] = raw_box.pose.T[t];
        result.box.push_back(box);
      }
      pub_obstacle_->publish(result);
      break;
    }
    case MODE_PALLET_LOCATE: {
      if (!has_pallet_sub) {
        break;
      }
      if (ret || !app_ptr) {
        break;
      }
      lx_camera_ros::msg::Pallet result;
      result.header.frame_id = "mrdvs";
      int64_t nanoseconds =
          static_cast<int64_t>(one_frame->app_data.sensor_timestamp * 1e3);
      result.header.stamp.sec = nanoseconds / 1e9;
      result.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
      LxPalletPose *lao = (LxPalletPose *)app_ptr;
      result.status = lao->return_val;
      result.x = lao->x;
      result.y = lao->y;
      result.yaw = lao->yaw;
      pub_pallet_->publish(result);
      break;
    }
    case MODE_VISION_LOCATION: {
      if (!has_location_sub) {
        break;
      }
      if (ret || !app_ptr) {
        break;
      }
      LxLocation *val = (LxLocation *)app_ptr;
      if (!val->status) {
        geometry_msgs::msg::PoseStamped alg_val;
        int64_t nanoseconds =
            static_cast<int64_t>(one_frame->app_data.sensor_timestamp * 1e3);
        alg_val.header.stamp.sec = nanoseconds / 1e9;
        alg_val.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
        alg_val.header.frame_id = "mrdvs";
        auto qua_res = ToQuaternion(val->theta, 0, 0);
        alg_val.pose.position.x = val->x;
        alg_val.pose.position.y = val->y;
        alg_val.pose.position.z = 0;
        alg_val.pose.orientation.x = qua_res.x;
        alg_val.pose.orientation.y = qua_res.y;
        alg_val.pose.orientation.z = qua_res.z;
        alg_val.pose.orientation.w = qua_res.w;
        pub_location_->publish(alg_val);
      }
      break;
    }
    case MODE_AVOID_OBSTACLE2: {
      if (!has_obstacle_sub) {
        break;
      }
      lx_camera_ros::msg::Obstacle result;
      result.header.frame_id = "mrdvs";
      int64_t nanoseconds =
          static_cast<int64_t>(one_frame->app_data.sensor_timestamp * 1e3);
      result.header.stamp.sec = nanoseconds / 1e9;
      result.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
      Check("GetObstacleIO", DcSpecialControl(handle_, "GetObstacleIO",
                                              (void *)&result.io_output));
      if (ret || !app_ptr) {
        result.status = -1;
        pub_obstacle_->publish(result);
        break;
      }
      LxAvoidanceOutputN *lao = (LxAvoidanceOutputN *)app_ptr;
      result.status = lao->state;
      result.box_number = lao->number_box;
      for (int i = 0; i < lao->number_box; i++) {
        auto raw_box = lao->obstacleBoxs[i];
        lx_camera_ros::msg::ObstacleBox box;
        box.width = raw_box.width;
        box.depth = raw_box.depth;
        box.height = raw_box.height;
        for (int t = 0; t < 3; t++)
          box.center[t] = raw_box.center[t];
        for (int t = 0; t < 9; t++)
          box.rotation[t] = raw_box.pose.R[t];
        for (int t = 0; t < 3; t++)
          box.translation[t] = raw_box.pose.T[t];
        result.box.push_back(box);
      }
      pub_obstacle_->publish(result);
      break;
    }
    }
    rate.sleep();
  }
}

void LxCamera::ReadParam() {
  auto declare_int = [this](const std::string &name, int default_value) {
    if (!this->has_parameter(name)) {
      this->declare_parameter<int>(name, default_value);
    }
  };
  auto declare_float = [this](const std::string &name, float default_value) {
    if (!this->has_parameter(name)) {
      this->declare_parameter<float>(name, default_value);
    }
  };
  auto declare_string =
      [this](const std::string &name, const std::string &default_value) {
        if (!this->has_parameter(name)) {
          this->declare_parameter<std::string>(name, default_value);
        }
      };
  declare_string("ip", "0");
  declare_string("log_path", "./");
  declare_int("raw_param", 0);
  declare_int("is_xyz", 0);
  declare_int("is_rgb", 0);
  declare_int("is_amp", 0);
  declare_int("is_depth", 1);
  declare_int("lx_2d_binning", 0);
  declare_int("lx_2d_undistort", 0);
  declare_int("lx_2d_undistort_scale", 0);
  declare_int("lx_2d_auto_exposure", 0);
  declare_int("lx_2d_auto_exposure_value", 0);
  declare_int("lx_2d_exposure", 1000);
  declare_int("lx_2d_gain", 100);
  declare_int("lx_rgb_to_tof", 0);
  declare_int("lx_3d_binning", 0);
  declare_int("lx_mulit_mode", 0);
  declare_int("lx_3d_undistort", 0);
  declare_int("lx_3d_undistort_scale", 0);
  declare_int("lx_hdr", 0);
  declare_int("lx_3d_auto_exposure", 0);
  declare_int("lx_3d_auto_exposure_value", 0);
  declare_int("lx_3d_first_exposure", 0);
  declare_int("lx_3d_second_exposure", 0);
  declare_int("lx_3d_gain", 0);
  declare_int("lx_tof_unit", 0);
  declare_int("lx_min_depth", 0);
  declare_int("lx_max_depth", 6000);
  declare_int("lx_work_mode", 0);
  declare_int("lx_application", 0);
  declare_float("x", 0.0f);
  declare_float("y", 0.0f);
  declare_float("z", 0.0f);
  declare_float("yaw", 0.0f);
  declare_float("roll", 0.0f);
  declare_float("pitch", 0.0f);

  this->get_parameter<std::string>("ip", ip_);
  this->get_parameter<std::string>("log_path", log_path_);
  RCLCPP_INFO(this->get_logger(), "ip: %s", ip_.c_str());
  RCLCPP_INFO(this->get_logger(), "Log file path: %s", log_path_.c_str());

  this->get_parameter<int>("is_xyz", is_xyz_);
  this->get_parameter<int>("is_depth", is_depth_);
  this->get_parameter<int>("is_amp", is_amp_);
  this->get_parameter<int>("is_rgb", is_rgb_);
  this->get_parameter<int>("raw_param", raw_param_);
  RCLCPP_INFO(this->get_logger(), "publish xyz: %d", is_xyz_);
  RCLCPP_INFO(this->get_logger(), "publish depth: %d", is_depth_);
  RCLCPP_INFO(this->get_logger(), "publish amp: %d", is_amp_);
  RCLCPP_INFO(this->get_logger(), "publish rgb: %d", is_rgb_);
  RCLCPP_INFO(this->get_logger(), "raw_param: %d", raw_param_);

  this->get_parameter<int>("lx_2d_binning", lx_2d_binning_);
  this->get_parameter<int>("lx_2d_undistort", lx_2d_undistort_);
  this->get_parameter<int>("lx_2d_undistort_scale", lx_2d_undistort_scale_);
  this->get_parameter<int>("lx_2d_auto_exposure", lx_2d_auto_exposure_);
  this->get_parameter<int>("lx_2d_auto_exposure_value",
                           lx_2d_auto_exposure_value_);
  this->get_parameter<int>("lx_2d_exposure", lx_2d_exposure_);
  this->get_parameter<int>("lx_2d_gain", lx_2d_gain_);
  RCLCPP_INFO(this->get_logger(), "lx_2d_binning: %d", lx_2d_binning_);
  RCLCPP_INFO(this->get_logger(), "lx_2d_undistort: %d", lx_2d_undistort_);
  RCLCPP_INFO(this->get_logger(), "lx_2d_undistort_scale: %d",
              lx_2d_undistort_scale_);
  RCLCPP_INFO(this->get_logger(), "lx_2d_auto_exposure: %d",
              lx_2d_auto_exposure_);
  RCLCPP_INFO(this->get_logger(), "lx_2d_auto_exposure_value: %d",
              lx_2d_auto_exposure_value_);
  RCLCPP_INFO(this->get_logger(), "lx_2d_exposure: %d", lx_2d_exposure_);
  RCLCPP_INFO(this->get_logger(), "lx_2d_gain: %d", lx_2d_gain_);

  this->get_parameter<int>("lx_rgb_to_tof", lx_rgb_to_tof_);
  this->get_parameter<int>("lx_3d_binning", lx_3d_binning_);
  this->get_parameter<int>("lx_mulit_mode", lx_mulit_mode_);
  this->get_parameter<int>("lx_3d_undistort", lx_3d_undistort_);
  this->get_parameter<int>("lx_3d_undistort_scale", lx_3d_undistort_scale_);
  this->get_parameter<int>("lx_hdr", lx_hdr_);
  this->get_parameter<int>("lx_3d_auto_exposure", lx_3d_auto_exposure_);
  this->get_parameter<int>("lx_3d_auto_exposure_value",
                           lx_3d_auto_exposure_value_);
  this->get_parameter<int>("lx_3d_first_exposure", lx_3d_first_exposure_);
  this->get_parameter<int>("lx_3d_second_exposure", lx_3d_second_exposure_);
  this->get_parameter<int>("lx_3d_gain", lx_3d_gain_);
  RCLCPP_INFO(this->get_logger(), "lx_rgb_to_tof: %d", lx_rgb_to_tof_);
  RCLCPP_INFO(this->get_logger(), "lx_3d_binning: %d", lx_3d_binning_);
  RCLCPP_INFO(this->get_logger(), "lx_mulit_mode: %d", lx_mulit_mode_);
  RCLCPP_INFO(this->get_logger(), "lx_3d_undistort: %d", lx_3d_undistort_);
  RCLCPP_INFO(this->get_logger(), "lx_3d_undistort_scale: %d",
              lx_3d_undistort_scale_);
  RCLCPP_INFO(this->get_logger(), "lx_hdr: %d", lx_hdr_);
  RCLCPP_INFO(this->get_logger(), "lx_3d_auto_exposure: %d",
              lx_3d_auto_exposure_);
  RCLCPP_INFO(this->get_logger(), "lx_3d_auto_exposure_value: %d",
              lx_3d_auto_exposure_value_);
  RCLCPP_INFO(this->get_logger(), "lx_3d_first_exposure: %d",
              lx_3d_first_exposure_);
  RCLCPP_INFO(this->get_logger(), "lx_3d_second_exposure: %d",
              lx_3d_second_exposure_);
  RCLCPP_INFO(this->get_logger(), "lx_3d_gain: %d", lx_3d_gain_);

  this->get_parameter<int>("lx_tof_unit", lx_tof_unit_);
  this->get_parameter<int>("lx_min_depth", lx_min_depth_);
  this->get_parameter<int>("lx_max_depth", lx_max_depth_);
  this->get_parameter<int>("lx_work_mode", lx_work_mode_);
  this->get_parameter<int>("lx_application", inside_app_);
  RCLCPP_INFO(this->get_logger(), "lx_tof_unit: %d", lx_tof_unit_);
  RCLCPP_INFO(this->get_logger(), "lx_min_depth: %d", lx_min_depth_);
  RCLCPP_INFO(this->get_logger(), "lx_max_depth: %d", lx_max_depth_);
  RCLCPP_INFO(this->get_logger(), "lx_work_mode: %d", lx_work_mode_);
  RCLCPP_INFO(this->get_logger(), "lx_application mode: %d", inside_app_);

  this->get_parameter<float>("x", install_x_);
  this->get_parameter<float>("y", install_y_);
  this->get_parameter<float>("z", install_z_);
  this->get_parameter<float>("yaw", install_yaw_);
  this->get_parameter<float>("pitch", install_pitch_);
  this->get_parameter<float>("roll", install_roll_);
  RCLCPP_INFO(this->get_logger(), "x: %f", install_x_);
  RCLCPP_INFO(this->get_logger(), "y: %f", install_y_);
  RCLCPP_INFO(this->get_logger(), "z: %f", install_z_);
  RCLCPP_INFO(this->get_logger(), "yaw: %f", install_yaw_);
  RCLCPP_INFO(this->get_logger(), "pitch: %f", install_pitch_);
  RCLCPP_INFO(this->get_logger(), "roll: %f", install_roll_);
}

void LxCamera::SetParam() {
  RCLCPP_WARN(this->get_logger(), "SetParam...");
  Check("LX_INT_WORK_MODE", DcSetIntValue(handle_, LX_INT_WORK_MODE, 0));
  // Check("LX_BOOL_ENABLE_2D_TO_DEPTH",
  //       DcSetBoolValue(handle_, LX_BOOL_ENABLE_2D_TO_DEPTH, 0));
  Check("LX_INT_RGBD_ALIGN_MODE",
        DcSetBoolValue(handle_, LX_INT_RGBD_ALIGN_MODE, 0));

  Check("LX_INT_2D_BINNING_MODE",
        DcSetIntValue(handle_, LX_INT_2D_BINNING_MODE, lx_2d_binning_));
  Check("LX_BOOL_ENABLE_2D_UNDISTORT",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_2D_UNDISTORT, lx_2d_undistort_));
  Check("LX_INT_2D_UNDISTORT_SCALE",
        DcSetIntValue(handle_, LX_INT_2D_UNDISTORT_SCALE,
                      lx_2d_undistort_scale_));
  Check("LX_BOOL_ENABLE_2D_AUTO_EXPOSURE",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE,
                       lx_2d_auto_exposure_));
  if (!lx_2d_auto_exposure_) {
    Check("LX_INT_2D_MANUAL_EXPOSURE",
          DcSetIntValue(handle_, LX_INT_2D_MANUAL_EXPOSURE, lx_2d_exposure_));
    Check("LX_INT_2D_MANUAL_GAIN",
          DcSetIntValue(handle_, LX_INT_2D_MANUAL_GAIN, lx_2d_gain_));
  } else
    Check("LX_INT_2D_AUTO_EXPOSURE_LEVEL",
          DcSetIntValue(handle_, LX_INT_2D_AUTO_EXPOSURE_LEVEL,
                        lx_2d_auto_exposure_value_));

  // Check("LX_BOOL_ENABLE_2D_TO_DEPTH",
  //       DcSetBoolValue(handle_, LX_BOOL_ENABLE_2D_TO_DEPTH, lx_rgb_to_tof_));
  Check("LX_INT_RGBD_ALIGN_MODE",
        DcSetBoolValue(handle_, LX_INT_RGBD_ALIGN_MODE, lx_rgb_to_tof_));
  Check("LX_INT_3D_BINNING_MODE",
        DcSetIntValue(handle_, LX_INT_3D_BINNING_MODE, lx_3d_binning_));
  Check("LX_BOOL_ENABLE_MULTI_MACHINE",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_MULTI_MACHINE, lx_mulit_mode_));
  Check("LX_BOOL_ENABLE_3D_UNDISTORT",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_3D_UNDISTORT, lx_3d_undistort_));
  Check("LX_INT_3D_UNDISTORT_SCALE",
        DcSetIntValue(handle_, LX_INT_3D_UNDISTORT_SCALE,
                      lx_3d_undistort_scale_));
  if (!lx_3d_auto_exposure_) {
    Check("LX_BOOL_ENABLE_3D_AUTO_EXPOSURE",
          DcSetBoolValue(handle_, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE,
                         lx_3d_auto_exposure_));
    Check("LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR",
          DcSetBoolValue(handle_, LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR, lx_hdr_));
    Check("LX_INT_FIRST_EXPOSURE",
          DcSetIntValue(handle_, LX_INT_FIRST_EXPOSURE, lx_3d_first_exposure_));
    Check(
        "LX_INT_SECOND_EXPOSURE",
        DcSetIntValue(handle_, LX_INT_SECOND_EXPOSURE, lx_3d_second_exposure_));
    Check("LX_INT_GAIN", DcSetIntValue(handle_, LX_INT_GAIN, lx_3d_gain_));
  } else {
    Check("LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR",
          DcSetBoolValue(handle_, LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR, 0));
    Check("LX_BOOL_ENABLE_3D_AUTO_EXPOSURE",
          DcSetBoolValue(handle_, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE,
                         lx_3d_auto_exposure_));
    Check("LX_INT_3D_AUTO_EXPOSURE_LEVEL",
          DcSetIntValue(handle_, LX_INT_3D_AUTO_EXPOSURE_LEVEL,
                        lx_3d_auto_exposure_value_));
  }

  Check("LX_INT_MIN_DEPTH",
        DcSetIntValue(handle_, LX_INT_MIN_DEPTH, lx_min_depth_));
  Check("LX_INT_MAX_DEPTH",
        DcSetIntValue(handle_, LX_INT_MAX_DEPTH, lx_max_depth_));
  RCLCPP_WARN(this->get_logger(), "SetParam done!");
}

bool LxCamera::SearchAndOpenDevice() {
  // find device
  int search_num = 5;
  int devnum = 0;
  LxDeviceInfo *devlist = nullptr;
  Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
  while (!devnum) {
    if (!search_num) {
      break;
    }
    Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
    RCLCPP_ERROR(this->get_logger(), "Found device faild. retry...");
    std::this_thread::sleep_for(std::chrono::milliseconds(1 * 1000));
    search_num--;
  }

  // open device
  LxDeviceInfo info;
  if (ip_.empty()) {
    ip_ = "0";
  }
  auto mode =
      ip_.size() < 8 ? LX_OPEN_MODE::OPEN_BY_INDEX : LX_OPEN_MODE::OPEN_BY_IP;
  if (LX_SUCCESS != DcOpenDevice(mode, ip_.c_str(), &handle_, &info)) {
    RCLCPP_ERROR(this->get_logger(), "Open device failed!");
    return false;
  }
  RCLCPP_INFO(this->get_logger(),
              "Open device success:"
              "\ndevice handle:             %lld"
              "\ndevice name:               %s"
              "\ndevice id:                 %s"
              "\ndevice ip:                 %s"
              "\ndevice sn:                 %s"
              "\ndevice mac:                %s"
              "\ndevice firmware version:   %s"
              "\ndevice algorithm version:  %s",
              handle_, info.name, info.id, info.ip, info.sn, info.mac,
              info.firmware_ver, info.algor_ver);
  return true;
}

int LxCamera::Check(std::string command, int state) {
  LX_STATE lx_state = static_cast<LX_STATE>(state);
  if (LX_SUCCESS == lx_state) {
    return lx_state;
  }
  const char *m = DcGetErrorString(lx_state); // Get error message
  std_msgs::msg::String msg;
  setlocale(LC_ALL, "");
  msg.data = "#command: " + command +
             " #error code: " + std::to_string(lx_state) + " #report: " + m;
  pub_error_->publish(msg); // Publish error message
  RCLCPP_ERROR(this->get_logger(), "%s", msg.data.c_str());
  return state;
}

bool LxCamera::LxString(
    const lx_camera_ros::srv::LxString::Request::SharedPtr req,
    const lx_camera_ros::srv::LxString::Response::SharedPtr res) {
  if (req->is_set) {
    res->result.ret = DcSetStringValue(handle_, req->cmd, req->val.c_str());
  }
  char *buf = nullptr;
  if (!req->is_set) {
    res->result.ret = DcGetStringValue(handle_, req->cmd, &buf);
  }
  res->result.msg = DcGetErrorString((LX_STATE)res->result.ret);
  if (buf) {
    res->val = std::string(buf);
  }
  return true;
}

bool LxCamera::LxFloat(
    const lx_camera_ros::srv::LxFloat::Request::SharedPtr req,
    const lx_camera_ros::srv::LxFloat::Response::SharedPtr res) {
  if (req->is_set) {
    res->result.ret = DcSetFloatValue(handle_, req->cmd, req->val);
  }
  LxFloatValueInfo float_value{0, 0, 0, 0, 0};
  auto ret = DcGetFloatValue(handle_, req->cmd, &float_value);
  if (!req->is_set) {
    res->result.ret = ret;
  }
  res->cur_value = float_value.cur_value;
  res->max_value = float_value.max_value;
  res->min_value = float_value.min_value;
  res->available = float_value.set_available;
  res->result.msg = DcGetErrorString((LX_STATE)res->result.ret);
  return true;
}

bool LxCamera::LxBool(
    const lx_camera_ros::srv::LxBool::Request::SharedPtr req,
    const lx_camera_ros::srv::LxBool::Response::SharedPtr res) {
  if (req->is_set) {
    res->result.ret = DcSetBoolValue(handle_, req->cmd, req->val);
  }
  bool val;
  auto ret = DcGetBoolValue(handle_, req->cmd, &val);
  res->val = val;
  if (!req->is_set) {
    res->result.ret = ret;
  }
  res->result.msg = DcGetErrorString((LX_STATE)res->result.ret);
  return true;
}

bool LxCamera::LxCmd(const lx_camera_ros::srv::LxCmd::Request::SharedPtr req,
                     const lx_camera_ros::srv::LxCmd::Response::SharedPtr res) {
  auto Pub = [&](std::string msg, int ret) {
    lx_camera_ros::msg::Result result;
    result.ret = ret;
    result.msg = msg;
    res->result.push_back(result);
  };
  if (req->cmd == 1) {
    auto ret = static_cast<LX_STATE>(Start());
    Pub(DcGetErrorString(ret), ret);
  } else if (req->cmd == 2) {
    auto ret = static_cast<LX_STATE>(Stop());
    Pub(DcGetErrorString(ret), ret);
  } else if (req->cmd) {
    auto ret = static_cast<LX_STATE>(DcSetCmd(handle_, req->cmd));
    Pub(DcGetErrorString(ret), ret);
  } else {
    std::vector<std::pair<std::string, int>> cmd_vec;
    auto add = [&](std::string str, int cmd) {
      std::pair<std::string, int> val(str, cmd);
      cmd_vec.push_back(val);
    };
    add("INT      FIRST_EXPOSURE", 1001);
    add("INT      SECOND_EXPOSURE", 1002);
    add("INT      THIRD_EXPOSURE", 1003);
    add("INT      FOURTH_EXPOSURE", 1004);
    add("INT      GAIN", 1005);
    add("INT      MIN_DEPTH", 1011);
    add("INT      MAX_DEPTH", 1012);
    add("INT      MIN_AMPLITUDE", 1013);
    add("INT      MAX_AMPLITUDE", 1014);
    add("INT      CODE_MODE", 1016);
    add("INT      WORK_MODE", 1018);
    add("INT      LINK_SPEED", 1019);
    add("INT      3D_IMAGE_WIDTH", 1021);
    add("INT      3D_IMAGE_HEIGHT", 1022);
    add("INT      3D_IMAGE_OFFSET_X", 1023);
    add("INT      3D_IMAGE_OFFSET_Y", 1024);
    add("INT      3D_BINNING_MODE", 1025);
    add("INT      3D_DEPTH_DATA_TYPE", 1026);
    add("INT      3D_AMPLITUDE_CHANNEL", 1031);
    add("INT      3D_AMPLITUDE_GET_TYPE", 1032);
    add("INT      3D_AMPLITUDE_EXPOSURE", 1033);
    add("INT      3D_AMPLITUDE_INTENSITY", 1034);
    add("INT      3D_AMPLITUDE_DATA_TYPE", 1035);
    add("INT      3D_AUTO_EXPOSURE_LEVEL", 1036);
    add("INT      3D_AUTO_EXPOSURE_MAX", 1037);
    add("INT      3D_AUTO_EXPOSURE_MIN", 1038);
    add("INT      2D_IMAGE_WIDTH", 1041);
    add("INT      2D_IMAGE_HEIGHT", 1042);
    add("INT      2D_IMAGE_OFFSET_X", 1043);
    add("INT      2D_IMAGE_OFFSET_Y", 1044);
    add("INT      2D_BINNING_MODE", 1045);
    add("INT      2D_IMAGE_CHANNEL", 1046);
    add("INT      2D_IMAGE_DATA_TYPE", 1047);
    add("INT      2D_MANUAL_EXPOSURE", 1051);
    add("INT      2D_MANUAL_GAIN", 1052);
    add("INT      2D_ENCODE_TYPE", 1053);
    add("INT      2D_AUTO_EXPOSURE_LEVEL", 1054);
    add("INT      TOF_GLOBAL_OFFSET", 1061);
    add("INT      3D_UNDISTORT_SCALE", 1062);
    add("INT      ALGORITHM_MODE", 1065);
    add("INT      MODBUS_ADDR", 1066);
    add("INT      HEART_TIME", 1067);
    add("INT      GVSP_PACKET_SIZE", 1068);
    add("INT      TRIGGER_MODE", 1069);
    add("INT      CALCULATE_UP", 1070);
    add("INT      CAN_BAUD_RATE", 1072);
    add("INT      CUSTOM_PARAM_GROUP", 1075);
    add("FLOAT    FILTER_LEVEL", 2001);
    add("FLOAT    EST_OUT_EXPOSURE", 2002);
    add("FLOAT    LIGHT_INTENSITY", 2003);
    add("FLOAT    3D_DEPTH_FPS", 2004);
    add("FLOAT    3D_AMPLITUDE_FPS", 2005);
    add("FLOAT    2D_IMAGE_FPS", 2006);
    add("FLOAT    DEVICE_TEMPERATURE", 2007);
    add("BOOL     CONNECT_STATE", 3001);
    add("BOOL     ENABLE_3D_DEPTH_STREAM", 3002);
    add("BOOL     ENABLE_3D_AMP_STREAM", 3003);
    add("BOOL     ENABLE_3D_AUTO_EXPOSURE", 3006);
    add("BOOL     ENABLE_3D_UNDISTORT", 3007);
    add("BOOL     ENABLE_ANTI_FLICKER", 3008);
    add("BOOL     ENABLE_2D_STREAM", 3011);
    add("BOOL     ENABLE_2D_AUTO_EXPOSURE", 3012);
    add("BOOL     ENABLE_2D_UNDISTORT", 3015);
    add("BOOL     ENABLE_2D_TO_DEPTH", 3016);
    add("BOOL     ENABLE_BACKGROUND_AMP", 3017);
    add("BOOL     ENABLE_MULTI_MACHINE", 3018);
    add("BOOL     ENABLE_MULTI_EXPOSURE_HDR", 3019);
    add("BOOL     ENABLE_SYNC_FRAME", 3020);
    add("STRING   DEVICE_VERSION", 4001);
    add("STRING   DEVICE_LOG_NAME", 4002);
    add("STRING   FIRMWARE_NAME", 4003);
    add("STRING   FILTER_PARAMS", 4004);
    add("STRING   ALGORITHM_PARAMS", 4005);
    add("STRING   ALGORITHM_VERSION", 4006);
    add("STRING   DEVICE_OS_VERSION", 4007);
    add("CMD      GET_PARAM_LIST", 0);
    add("CMD      START_STREAM", 1);
    add("CMD      STOP_STREAM", 2);
    add("CMD      GET_NEW_FRAME", 5001);
    add("CMD      RETURN_VERSION", 5002);
    add("CMD      RESTART_DEVICE", 5003);
    add("CMD      WHITE_BALANCE", 5004);
    add("CMD      RESET_PARAM", 5007);
    add("CMD      CALIB_EXTRIC", 5008);
    for (auto &i : cmd_vec)
      while (i.first.length() < 40)
        i.first.push_back(' ');
    for (auto &i : cmd_vec)
      Pub(i.first, i.second);
  }
  return true;
}

bool LxCamera::LxInt(const lx_camera_ros::srv::LxInt::Request::SharedPtr req,
                     const lx_camera_ros::srv::LxInt::Response::SharedPtr res) {
  if (req->is_set) {
    res->result.ret = DcSetIntValue(handle_, req->cmd, req->val);
  }
  LxIntValueInfo int_value{0, 0, 0, 0, 0};
  auto ret = DcGetIntValue(handle_, req->cmd, &int_value);
  if (!req->is_set) {
    res->result.ret = ret;
  }
  res->cur_value = int_value.cur_value;
  res->max_value = int_value.max_value;
  res->min_value = int_value.min_value;
  res->available = int_value.set_available;
  res->result.msg = DcGetErrorString((LX_STATE)res->result.ret);
  return true;
}

void LxCamera::PubTf(
    const geometry_msgs::msg::TransformStamped &transform_stamped) {
  static std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_broadcaster->sendTransform(transform_stamped);
}
