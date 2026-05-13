#include <geometry_msgs/PoseStamped.h>
#include <mars_quadrotor_msgs/PositionCommand.h>
#include <mars_quadrotor_msgs/SO3Command.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <perfect_drone_sim/SetUavPose.h>

#include <Eigen/Dense>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "marsim_render/marsim_render.hpp"
#include "so3_controller/so3_controller.hpp"
#include "so3_quadrotor/quadrotor_dynamics.hpp"
#include "utils/yaml_loader.hpp"

typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 3, 3> Mat33;

typedef Eigen::Matrix<double, 3, 3> StatePVA;
typedef Eigen::Matrix<double, 3, 4> StatePVAJ;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DynamicMat;
typedef Eigen::MatrixX4d MatX4;
typedef std::pair<double, Vec3> TimePosPair;

namespace perfect_drone {
using std::cout;
using std::endl;
using std::string;
using std::vector;

inline double clampScalar(const double value, const double min_value,
                          const double max_value) {
  return value < min_value ? min_value
                           : (value > max_value ? max_value : value);
}

class SimulatorConfig {
 public:
  std::string mesh_resource;
  std::string position_cmd_topic;
  std::string body_rate_cmd_topic;
  Eigen::Vector3d init_pos;
  double init_yaw;
  double sensing_rate;
  double x_min;
  double x_max;
  double y_min;
  double y_max;
  double z_min;
  double z_max;

  SimulatorConfig() = default;

  SimulatorConfig(const std::string& cfg_path) {
    yaml_loader::YamlLoader loader(cfg_path);
    loader.LoadParam("mesh_resource", mesh_resource,
                     std::string("package://perfect_drone_sim/meshes/f250.dae"),
                     false);
    loader.LoadParam("position_cmd_topic", position_cmd_topic, std::string(""));
    loader.LoadParam("body_rate_cmd_topic", body_rate_cmd_topic,
                     std::string(""));
    loader.LoadParam("init_position/x", init_pos.x(), 0.0);
    loader.LoadParam("init_position/y", init_pos.y(), 0.0);
    loader.LoadParam("init_position/z", init_pos.z(), 1.5);
    loader.LoadParam("init_yaw", init_yaw, 0.0);
    loader.LoadParam("sensing_rate", sensing_rate, 10.0);
    loader.LoadParam("x_min", x_min, -1e9);
    loader.LoadParam("x_max", x_max, 1e9);
    loader.LoadParam("y_min", y_min, -1e9);
    loader.LoadParam("y_max", y_max, 1e9);
    loader.LoadParam("z_min", z_min, -1e9);
    loader.LoadParam("z_max", z_max, 1e9);
  }
};

class So3QuadrotorConfig {
 public:
  double simulation_rate;
  double odom_rate;
  double g;
  double mass;
  double Ixx;
  double Iyy;
  double Izz;
  double kf;
  double prop_radius;
  double arm_length;
  double motor_time_constant;
  double max_rpm;
  double min_rpm;

  double gains_rot_x;
  double gains_rot_y;
  double gains_rot_z;
  double gains_ang_x;
  double gains_ang_y;
  double gains_ang_z;
  double corrections_z;
  double corrections_r;
  double corrections_p;

  So3QuadrotorConfig() = default;

  So3QuadrotorConfig(const std::string& cfg_path) {
    yaml_loader::YamlLoader loader(cfg_path);
    loader.LoadParam("simulation_rate", simulation_rate, 1000.0);
    loader.LoadParam("odom_rate", odom_rate, 400.0);
    loader.LoadParam("g", g, 9.81);
    loader.LoadParam("mass", mass, 0.98);
    loader.LoadParam("Ixx", Ixx, 2.64e-3);
    loader.LoadParam("Iyy", Iyy, 2.64e-3);
    loader.LoadParam("Izz", Izz, 4.96e-3);
    loader.LoadParam("kf", kf, 8.98132e-9);
    loader.LoadParam("prop_radius", prop_radius, 0.062);
    loader.LoadParam("arm_length", arm_length, 0.26);
    loader.LoadParam("motor_time_constant", motor_time_constant, 0.03333);
    loader.LoadParam("max_rpm", max_rpm, 35000.0);
    loader.LoadParam("min_rpm", min_rpm, 1200.0);

    loader.LoadParam("gains_rot_x", gains_rot_x, 1.5);
    loader.LoadParam("gains_rot_y", gains_rot_y, 1.5);
    loader.LoadParam("gains_rot_z", gains_rot_z, 1.0);
    loader.LoadParam("gains_ang_x", gains_ang_x, 0.13);
    loader.LoadParam("gains_ang_y", gains_ang_y, 0.13);
    loader.LoadParam("gains_ang_z", gains_ang_z, 0.1);
    loader.LoadParam("corrections_z", corrections_z, 0.0);
    loader.LoadParam("corrections_r", corrections_r, 0.0);
    loader.LoadParam("corrections_p", corrections_p, 0.0);
  }
};

class PerfectDrone {
  marsim::MarsimRender::Ptr render_ptr_;
  std::shared_ptr<so3_quadrotor::Quadrotor> quadrotorPtr_;
  so3_quadrotor::Control control_;
  so3_quadrotor::Cmd cmd_;
  std::shared_ptr<so3_controller::SO3Controller> so3ControlPtr_;
  SimulatorConfig simu_cfg_;
  So3QuadrotorConfig quadrotor_cfg_;

 public:
  PerfectDrone(const ros::NodeHandle& n) : nh_(n) {
#define CONFIG_FILE_DIR(name) (string(string(ROOT_DIR) + "config/" + name))
    std::string dft_cfg_path = CONFIG_FILE_DIR("click.yaml");
    std::string cfg_path, cfg_name;
    if (nh_.param("config_path", cfg_path, dft_cfg_path)) {
      cout << " -- [Fsm-Test] Load config from: " << cfg_path << endl;
    } else if (nh_.param("config_name", cfg_name, dft_cfg_path)) {
      cfg_path = CONFIG_FILE_DIR(cfg_name);
      cout << " -- [Fsm-Test] Load config by file name: " << cfg_name << endl;
    }
    simu_cfg_ = SimulatorConfig(cfg_path);

    quadrotor_cfg_ = So3QuadrotorConfig(CONFIG_FILE_DIR("so3_quadrotor.yaml"));

    render_ptr_ = std::make_shared<marsim::MarsimRender>(cfg_path);
    if (!simu_cfg_.position_cmd_topic.empty() &&
        !simu_cfg_.body_rate_cmd_topic.empty()) {
      cout << " -- [PerfectDrone] Invalid command topic configuration."
           << " Both position_cmd_topic and body_rate_cmd_topic are set."
           << " position_cmd_topic: " << simu_cfg_.position_cmd_topic
           << ", body_rate_cmd_topic: " << simu_cfg_.body_rate_cmd_topic
           << endl;
      std::exit(EXIT_FAILURE);
    } else if (simu_cfg_.position_cmd_topic.empty() &&
               simu_cfg_.body_rate_cmd_topic.empty()) {
      cout << " -- [PerfectDrone] Invalid command topic configuration."
           << " Both position_cmd_topic and body_rate_cmd_topic are not set."
           << " position_cmd_topic: " << simu_cfg_.position_cmd_topic
           << ", body_rate_cmd_topic: " << simu_cfg_.body_rate_cmd_topic
           << endl;
      std::exit(EXIT_FAILURE);
    }
    if (!simu_cfg_.position_cmd_topic.empty()) {
      cmd_sub_ = nh_.subscribe(simu_cfg_.position_cmd_topic, 100,
                               &PerfectDrone::cmdCallback, this,
                               ros::TransportHints().tcpNoDelay());
    }
    if (!simu_cfg_.body_rate_cmd_topic.empty()) {
      body_rate_cmd_sub_ =
          nh_.subscribe(simu_cfg_.body_rate_cmd_topic, 100,
                        &PerfectDrone::bodyRateCmdCallback, this,
                        ros::TransportHints().tcpNoDelay());
    }
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/lidar_slam/odom", 100);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);
    pose_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/lidar_slam/pose", 100);
    robot_pub_ = nh_.advertise<visualization_msgs::Marker>("robot", 100);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 100);
    local_pc_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
    sensor_pc_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/cloud_sensor", 100);
    global_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_pc", 100);
    vel_pub_ = nh_.advertise<visualization_msgs::Marker>("vel_text", 100);
    set_uav_pose_srv_ =
        nh_.advertiseService("SetUavPose", &PerfectDrone::setUavPoseCallback,
                             this);

    // config of quadrotor
    so3_quadrotor::Config config;
    config.g = quadrotor_cfg_.g;
    config.mass = quadrotor_cfg_.mass;
    config.J = Eigen::Vector3d(quadrotor_cfg_.Ixx, quadrotor_cfg_.Iyy,
                               quadrotor_cfg_.Izz)
                   .asDiagonal();
    config.kf = quadrotor_cfg_.kf;
    config.km = 0.07 * (3 * quadrotor_cfg_.prop_radius) * config.kf;
    config.arm_length = quadrotor_cfg_.arm_length;
    config.motor_time_constant = quadrotor_cfg_.motor_time_constant;
    config.max_rpm = quadrotor_cfg_.max_rpm;
    config.min_rpm = quadrotor_cfg_.min_rpm;

    // config of so3 controller
    so3ControlPtr_ = std::make_shared<so3_controller::SO3Controller>(
        quadrotor_cfg_.mass, quadrotor_cfg_.g);
    so3cmd_.header.frame_id = "world";
    so3cmd_.kR[0] = quadrotor_cfg_.gains_rot_x;
    so3cmd_.kR[1] = quadrotor_cfg_.gains_rot_y;
    so3cmd_.kR[2] = quadrotor_cfg_.gains_rot_z;
    so3cmd_.kOm[0] = quadrotor_cfg_.gains_ang_x;
    so3cmd_.kOm[1] = quadrotor_cfg_.gains_ang_y;
    so3cmd_.kOm[2] = quadrotor_cfg_.gains_ang_z;
    so3cmd_.aux.kf_correction = quadrotor_cfg_.corrections_z;
    so3cmd_.aux.angle_corrections[0] = quadrotor_cfg_.corrections_r;
    so3cmd_.aux.angle_corrections[1] = quadrotor_cfg_.corrections_p;

    // quadrotor init
    quadrotorPtr_ = std::make_shared<so3_quadrotor::Quadrotor>(config);
    quadrotorPtr_->setPos(clampPosition(simu_cfg_.init_pos));
    quadrotorPtr_->setYpr(Eigen::Vector3d(simu_cfg_.init_yaw, 0, 0));
    double rpm = sqrt(config.mass * config.g / 4 / config.kf);
    quadrotorPtr_->setRpm(Eigen::Vector4d(rpm, rpm, rpm, rpm));
    Eigen::Quaterniond quat =
        uav_utils::ypr_to_quaternion(Eigen::Vector3d(simu_cfg_.init_yaw, 0, 0));
    cmd_.force[2] = config.mass * config.g;
    cmd_.qw = quat.w();
    cmd_.qx = quat.x();
    cmd_.qy = quat.y();
    cmd_.qz = quat.z();
    body_rate_cmd_.thrust = config.mass * config.g /
                            (config.kf * config.max_rpm * config.max_rpm * 4);
    body_rate_cmd_.kOm[0] = quadrotor_cfg_.gains_ang_x;
    body_rate_cmd_.kOm[1] = quadrotor_cfg_.gains_ang_y;
    body_rate_cmd_.kOm[2] = quadrotor_cfg_.gains_ang_z;
    body_rate_cmd_.corrections[0] = quadrotor_cfg_.corrections_z;
    body_rate_cmd_.corrections[1] = quadrotor_cfg_.corrections_r;
    body_rate_cmd_.corrections[2] = quadrotor_cfg_.corrections_p;

    // controller init
    // init target position and yaw when no command received
    des_pos_ = clampPosition(simu_cfg_.init_pos);
    des_yaw_ = simu_cfg_.init_yaw;
    so3cmd_.aux.enable_motors = true;
    so3cmd_.aux.use_external_yaw = false;

    position_ = clampPosition(simu_cfg_.init_pos);

    mesh_resource_ = simu_cfg_.mesh_resource;
    q_ = quat;
    odom_.header.frame_id = "world";
    imu_.header.frame_id = "world";
    simulation_timer =
        nh_.createTimer(ros::Duration(1.0 / quadrotor_cfg_.simulation_rate),
                        &PerfectDrone::quadrotor_timer_callback, this);
    state_timer_ = nh_.createTimer(
        ros::Duration(0.1), &PerfectDrone::controller_timer_callback, this);
    odom_pub_timer_ =
        nh_.createTimer(ros::Duration(0.01), &PerfectDrone::publishOdom, this);

    global_pc_pub_timer_ = nh_.createTimer(
        ros::Duration(0.001), &PerfectDrone::publishGlobalPC, this);
    path_.poses.clear();
    path_.header.frame_id = "world";
    path_.header.stamp = ros::Time::now();
  }

  double getSensingRate() { return simu_cfg_.sensing_rate; }

  void visualizeText(const ros::Publisher& pub, const std::string& ns,
                     const std::string& text, const Vec3& position,
                     const double& size, const int& id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.ns = ns.c_str();
    if (id >= 0) {
      marker.id = id;
    } else {
      static int id = 0;
      marker.id = id++;
    }
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = size;
    marker.color.a = 1.0;
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
    marker.text = text;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z() + 5.0;
    marker.pose.orientation.w = 1.0;
    pub.publish(marker);
  }

  void publishPC() {
    pcl::PointCloud<marsim::PointType>::Ptr local_map(
        new pcl::PointCloud<marsim::PointType>);
    pcl::PointCloud<marsim::PointType>::Ptr sensor_pc(
        new pcl::PointCloud<marsim::PointType>);
    render_ptr_->renderOnceInWorld(position_.cast<float>(), q_.cast<float>(),
                                   ros::Time::now().toSec(), local_map);

    Eigen::Matrix3f rot(q_.cast<float>());
    Eigen::Matrix4f sensor2world;
    sensor2world << rot(0, 0), rot(0, 1), rot(0, 2), position_.x(),  //
        rot(1, 0), rot(1, 1), rot(1, 2), position_.y(),              //
        rot(2, 0), rot(2, 1), rot(2, 2), position_.z(),              //
        0, 0, 0, 1;
    Eigen::Matrix4f world2sensor;
    world2sensor = sensor2world.inverse();
    sensor_pc->points.clear();
    pcl::transformPointCloud(*local_map, *sensor_pc, world2sensor);

    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*local_map, pc_msg);
    pc_msg.header.frame_id = "world";
    pc_msg.header.stamp = ros::Time::now();
    std::cout << "Publish local map size: " << local_map->size() << std::endl;
    local_pc_pub_.publish(pc_msg);

    sensor_msgs::PointCloud2 sensor_pc_msg;
    pcl::toROSMsg(*sensor_pc, sensor_pc_msg);
    sensor_pc_msg.header.frame_id = "perfect_drone";
    sensor_pc_msg.header.stamp = pc_msg.header.stamp;
    sensor_pc_pub_.publish(sensor_pc_msg);
  }

  ~PerfectDrone() {}

 private:
  nav_msgs::Path path_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber body_rate_cmd_sub_;
  ros::ServiceServer set_uav_pose_srv_;
  ros::Publisher odom_pub_, imu_pub_, robot_pub_, pose_pub_, path_pub_,
      global_pc_pub_, local_pc_pub_, vel_pub_;
  ros::Publisher sensor_pc_pub_;
  ros::Timer odom_pub_timer_;
  ros::Timer pc_pub_timer_;
  ros::Timer global_pc_pub_timer_;
  ros::Timer simulation_timer;
  ros::Timer state_timer_;
  ros::NodeHandle nh_;
  Vec3 position_;
  Eigen::Quaterniond q_;
  nav_msgs::Odometry odom_;
  sensor_msgs::Imu imu_;
  std::string mesh_resource_;
  mars_quadrotor_msgs::SO3Command so3cmd_;
  so3_quadrotor::BodyRateCmd body_rate_cmd_;
  bool position_cmd_received_flag_ = false;
  Eigen::Vector3d des_pos_;
  double des_yaw_;

  Eigen::Vector3d clampPosition(const Eigen::Vector3d& pos) const {
    Eigen::Vector3d clamped = pos;
    clamped.x() = clampScalar(pos.x(), simu_cfg_.x_min, simu_cfg_.x_max);
    clamped.y() = clampScalar(pos.y(), simu_cfg_.y_min, simu_cfg_.y_max);
    clamped.z() = clampScalar(pos.z(), simu_cfg_.z_min, simu_cfg_.z_max);
    return clamped;
  }

  void so3cmd_callback(const mars_quadrotor_msgs::SO3Command& cmd_msg) {
    cmd_.force[0] = cmd_msg.force.x;
    cmd_.force[1] = cmd_msg.force.y;
    cmd_.force[2] = cmd_msg.force.z;
    cmd_.qx = cmd_msg.orientation.x;
    cmd_.qy = cmd_msg.orientation.y;
    cmd_.qz = cmd_msg.orientation.z;
    cmd_.qw = cmd_msg.orientation.w;
    cmd_.kR[0] = cmd_msg.kR[0];
    cmd_.kR[1] = cmd_msg.kR[1];
    cmd_.kR[2] = cmd_msg.kR[2];
    cmd_.kOm[0] = cmd_msg.kOm[0];
    cmd_.kOm[1] = cmd_msg.kOm[1];
    cmd_.kOm[2] = cmd_msg.kOm[2];
    cmd_.corrections[0] = cmd_msg.aux.kf_correction;
    cmd_.corrections[1] = cmd_msg.aux.angle_corrections[0];
    cmd_.corrections[2] = cmd_msg.aux.angle_corrections[1];
    cmd_.current_yaw = cmd_msg.aux.current_yaw;
    cmd_.use_external_yaw = cmd_msg.aux.use_external_yaw;
  }

  void bodyRateCmdCallback(const mavros_msgs::AttitudeTargetConstPtr& msg) {
    body_rate_cmd_.body_rate[0] = msg->body_rate.x;
    body_rate_cmd_.body_rate[1] = msg->body_rate.y;
    body_rate_cmd_.body_rate[2] = msg->body_rate.z;
    body_rate_cmd_.thrust = msg->thrust;
  }

  void controller_timer_callback(const ros::TimerEvent& event) {
    if (position_cmd_received_flag_) {
      position_cmd_received_flag_ = false;
    } else {
      Eigen::Vector3d des_vel(0, 0, 0);
      Eigen::Vector3d des_acc(0, 0, 0);
      Eigen::Vector3d kx(5.7, 5.7, 6.2);
      Eigen::Vector3d kv(3.4, 3.4, 4.0);
      so3ControlPtr_->calculateControl(des_pos_, des_vel, des_acc, des_yaw_, 0,
                                       kx, kv);
      const Eigen::Vector3d& f = so3ControlPtr_->getF();
      const Eigen::Quaterniond& q = so3ControlPtr_->getQ();
      so3cmd_.force.x = f(0);
      so3cmd_.force.y = f(1);
      so3cmd_.force.z = f(2);
      so3cmd_.orientation.x = q.x();
      so3cmd_.orientation.y = q.y();
      so3cmd_.orientation.z = q.z();
      so3cmd_.orientation.w = q.w();
      so3cmd_callback(so3cmd_);
    }
  }

  void cmdCallback(const mars_quadrotor_msgs::PositionCommandConstPtr& msg) {
    position_cmd_received_flag_ = true;
    Eigen::Vector3d des_pos(msg->position.x, msg->position.y, msg->position.z);
    des_pos = clampPosition(des_pos);
    Eigen::Vector3d des_vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    Eigen::Vector3d des_acc(msg->acceleration.x, msg->acceleration.y,
                            msg->acceleration.z);
    Eigen::Vector3d kx(msg->kx[0], msg->kx[1], msg->kx[2]);
    Eigen::Vector3d kv(msg->kv[0], msg->kv[1], msg->kv[2]);
    if (msg->kx[0] == 0) {
      kx(0) = 5.7;
      kx(1) = 5.7;
      kx(2) = 6.2;
      kv(0) = 3.4;
      kv(1) = 3.4;
      kv(2) = 4.0;
    }
    double des_yaw = msg->yaw;
    double des_yaw_dot = msg->yaw_dot;
    so3ControlPtr_->calculateControl(des_pos, des_vel, des_acc, des_yaw,
                                     des_yaw_dot, kx, kv);
    const Eigen::Vector3d& f = so3ControlPtr_->getF();
    const Eigen::Quaterniond& q = so3ControlPtr_->getQ();
    so3cmd_.force.x = f(0);
    so3cmd_.force.y = f(1);
    so3cmd_.force.z = f(2);
    so3cmd_.orientation.x = q.x();
    so3cmd_.orientation.y = q.y();
    so3cmd_.orientation.z = q.z();
    so3cmd_.orientation.w = q.w();
    so3cmd_callback(so3cmd_);
    // store last des_pos and des_yaw
    des_pos_ = des_pos;
    des_yaw_ = des_yaw;
  }

  bool setUavPoseCallback(perfect_drone_sim::SetUavPose::Request& req,
                          perfect_drone_sim::SetUavPose::Response& res) {
    const Eigen::Vector3d request_pos(req.pos_x, req.pos_y, req.pos_z);
    const Eigen::Vector3d clamped_pos = clampPosition(request_pos);

    quadrotorPtr_->setPos(clamped_pos);
    // setYpr expects [yaw, pitch, roll]
    quadrotorPtr_->setYpr(Eigen::Vector3d(req.yaw, req.pitch, req.roll));

    position_ = clamped_pos;
    q_ = uav_utils::ypr_to_quaternion(Eigen::Vector3d(req.yaw, req.pitch, req.roll));
    des_pos_ = clamped_pos;
    des_yaw_ = req.yaw;

    res.success = true;
    return true;
  }

  void quadrotor_timer_callback(const ros::TimerEvent& event) {
    auto last_control = control_;
    if (!simu_cfg_.body_rate_cmd_topic.empty()) {
      control_ = quadrotorPtr_->getBodyRateControl(body_rate_cmd_);
    } else {
      control_ = quadrotorPtr_->getControl(cmd_);
    }
    for (size_t i = 0; i < 4; ++i) {
      if (std::isnan(control_.rpm[i])) control_.rpm[i] = last_control.rpm[i];
    }
    quadrotorPtr_->setInput(control_.rpm[0], control_.rpm[1], control_.rpm[2],
                            control_.rpm[3]);
    quadrotorPtr_->step(1.0 / quadrotor_cfg_.simulation_rate);
    const Eigen::Vector3d raw_pos = quadrotorPtr_->getPos();
    position_ = clampPosition(raw_pos);
    if ((position_ - raw_pos).squaredNorm() > 1e-12) {
      quadrotorPtr_->setPos(position_);
    }
    q_ = quadrotorPtr_->getQuat();
  }

  void publishGlobalPC(const ros::TimerEvent& e) {
    static int last_sub_num = 0;
    // update sub num
    int sub_num = global_pc_pub_.getNumSubscribers();
    if (sub_num > 0 && last_sub_num != sub_num) {
      ros::Duration(0.5).sleep();
      pcl::PointCloud<marsim::PointType>::Ptr global_map(
          new pcl::PointCloud<marsim::PointType>);
      render_ptr_->getGlobalMap(global_map);
      sensor_msgs::PointCloud2 pc_msg;
      pcl::toROSMsg(*global_map, pc_msg);
      pc_msg.header.frame_id = "world";
      pc_msg.header.stamp = ros::Time::now();
      global_pc_pub_.publish(pc_msg);
      std::cout << "Publish global map" << std::endl;
    }
    last_sub_num = sub_num;
  }

  void publishOdom(const ros::TimerEvent& e) {
    static tf::TransformBroadcaster tf_br;

    const Eigen::Vector3d& pos = quadrotorPtr_->getPos();
    const Eigen::Vector3d& vel = quadrotorPtr_->getVel();
    const Eigen::Vector3d& acc = quadrotorPtr_->getAcc();
    const Eigen::Quaterniond& quat = quadrotorPtr_->getQuat();
    const Eigen::Vector3d& omega = quadrotorPtr_->getOmega();

    ros::Time tnow = ros::Time::now();
    // tf
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pos.x(), pos.y(), pos.z()));
    transform.setRotation(
        tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
    tf_br.sendTransform(tf::StampedTransform(transform, tnow, "world", "body"));
    // odom
    odom_.header.stamp = tnow;
    odom_.pose.pose.position.x = pos(0);
    odom_.pose.pose.position.y = pos(1);
    odom_.pose.pose.position.z = pos(2);
    odom_.pose.pose.orientation.x = quat.x();
    odom_.pose.pose.orientation.y = quat.y();
    odom_.pose.pose.orientation.z = quat.z();
    odom_.pose.pose.orientation.w = quat.w();

    odom_.twist.twist.linear.x = vel(0);
    odom_.twist.twist.linear.y = vel(1);
    odom_.twist.twist.linear.z = vel(2);

    odom_.twist.twist.angular.x = omega(0);
    odom_.twist.twist.angular.y = omega(1);
    odom_.twist.twist.angular.z = omega(2);

    so3ControlPtr_->setPos(pos);
    so3ControlPtr_->setVel(vel);
    so3cmd_.aux.current_yaw = tf::getYaw(odom_.pose.pose.orientation);

    odom_pub_.publish(odom_);

    // imu
    imu_.header.stamp = tnow;
    imu_.orientation.x = quat.x();
    imu_.orientation.y = quat.y();
    imu_.orientation.z = quat.z();
    imu_.orientation.w = quat.w();

    imu_.angular_velocity.x = omega(0);
    imu_.angular_velocity.y = omega(1);
    imu_.angular_velocity.z = omega(2);

    const tf::Vector3 gravity_world(0.0, 0.0, quadrotor_cfg_.g);
    const tf::Vector3 gravity_body =
        transform.getBasis().inverse() * gravity_world;
    const Eigen::Vector3d imu_acc =
        acc +
        Eigen::Vector3d(gravity_body.x(), gravity_body.y(), gravity_body.z());

    imu_.linear_acceleration.x = imu_acc.x();
    imu_.linear_acceleration.y = imu_acc.y();
    imu_.linear_acceleration.z = imu_acc.z();

    so3ControlPtr_->setAcc(acc);

    imu_pub_.publish(imu_);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << vel.norm();  // 设置两位小数
    visualizeText(vel_pub_, "vel", "Speed: " + oss.str() + " m/s", position_,
                  3.0, 0);

    geometry_msgs::PoseStamped pose;
    pose.pose = odom_.pose.pose;
    pose.header = odom_.header;
    pose_pub_.publish(pose);

    static tf2_ros::TransformBroadcaster br_map_ego;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = odom_.header.stamp;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "perfect_drone";
    transformStamped.transform.translation.x = odom_.pose.pose.position.x;
    transformStamped.transform.translation.y = odom_.pose.pose.position.y;
    transformStamped.transform.translation.z = odom_.pose.pose.position.z;
    transformStamped.transform.rotation.x = odom_.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom_.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom_.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom_.pose.pose.orientation.w;
    br_map_ego.sendTransform(transformStamped);

    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = "world";
    meshROS.header.stamp = odom_.header.stamp;
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position = odom_.pose.pose.position;
    meshROS.pose.orientation = odom_.pose.pose.orientation;
    meshROS.scale.x = 1;
    meshROS.scale.y = 1;
    meshROS.scale.z = 1;
    meshROS.mesh_resource = mesh_resource_;
    meshROS.mesh_use_embedded_materials = true;
    meshROS.color.a = 1.0;
    meshROS.color.r = 1.0;
    meshROS.color.g = 1.0;
    meshROS.color.b = 1.0;
    robot_pub_.publish(meshROS);
    static int slow_down = 0;
    if (slow_down++ % 10 == 0) {
      if ((position_.head(2) - Vec3(0, -50, 1.5).head(2)).norm() < 1) {
        path_.poses.clear();
        path_.poses.reserve(10000);
      }
      path_.poses.push_back(pose);
      path_.header = odom_.header;
      path_pub_.publish(path_);
    }
  }
};
}  // namespace perfect_drone

int main(int argc, char** argv) {
  ros::init(argc, argv, "perfect_tracking");
  ros::NodeHandle n("~");
  perfect_drone::PerfectDrone dp(n);
  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::Rate lprt(dp.getSensingRate());
  while (ros::ok()) {
    dp.publishPC();
    lprt.sleep();
  }

  ros::waitForShutdown();
  return 0;
}
