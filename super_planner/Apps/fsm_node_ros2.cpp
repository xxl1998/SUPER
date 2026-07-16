/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/



#include <ros_interface/ros2/fsm_ros2.hpp>
#include <metric_monitor.hpp>

/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1

#include "utils/header/backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}


using namespace fsm;
using namespace std;
FsmRos2::Ptr fsm_ptr;

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 设置PCL日志级别（保持不变）
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // 创建节点（替换ROS1的NodeHandle）
    auto node = std::make_shared<rclcpp::Node>("fsm_node");

    // 检查是否使用仿真时间
    while (rclcpp::ok()) {
        bool use_sim_time;

        // 获取参数
        if (node->get_parameter("use_sim_time", use_sim_time)) {
            if (!use_sim_time) {
                // 如果 use_sim_time 为 false，打印信息并退出循环
                std::cout << " -- [Bench] Use sim time is false, begin replay." << std::endl;
                break;
            } else {
                // 如果 use_sim_time 为 true，设置为 false
                node->set_parameter(rclcpp::Parameter("use_sim_time", false));
            }
        } else {
            // 如果参数不存在，设置为 false
            node->set_parameter(rclcpp::Parameter("use_sim_time", false));
        }

        // 休眠 1 秒
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 初始化FSM

#define CONFIG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/"+(name)))
    auto fsm_ptr = std::make_shared<FsmRos2>();
    std::string cfg_path = "click.yaml";
    node->declare_parameter("config_name", cfg_path);
    node->get_parameter("config_name", cfg_path);
    cfg_path = CONFIG_FILE_DIR(cfg_path);

    metric_monitor::MetricMonitorConfig metric_config;
    metric_config.module_name = "super_planner";
    metric_config.log_directory = std::string(ROOT_DIR) + "log/metric_monitor";
    metric_config.snapshot_directory = "/tmp";
    metric_config.process_interval_sec = 0.02;
    metric_config.metrics_csv_write_interval_sec = 0.5;
    metric_config.tmp_snapshot_write_interval_sec = 0.5;
    metric_config.statistics_window_sec = 2.0;
    metric_config.no_data_timeout_sec = 5.0;
    metric_config.data_lost_timeout_sec = 0.2;
    auto metric_monitor =
            std::make_shared<metric_monitor::MetricMonitor>(metric_config);
    metric_monitor->registerHeaderMetric("odometry", 0.1, 0.15, 9.0, 8.0);
    metric_monitor->registerHeaderMetric("point_cloud", 0.15, 0.2, 9.0, 8.0);
    metric_monitor->registerCostMetric("super_time_cost_sec", 0.009, 0.010);
    metric_monitor->registerCostMetric("super_replan_time_cost_sec", 0.060, 0.066);
    metric_monitor->start();

    fsm_ptr->init(node, cfg_path, metric_monitor);

    // 打印启动信息（ROS2风格）
    RCLCPP_INFO(node->get_logger(), "\033[32m -- [Fsm-Test] Begin.\033[0m");

    // 创建执行器（替换AsyncSpinner）
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    // Deterministic shutdown order
    executor.remove_node(node);
    fsm_ptr.reset();
    metric_monitor->stop();
    metric_monitor.reset();
    node.reset();
    rclcpp::shutdown();
    std::cout << "main exit..." << std::endl;
    fflush(stdout);
    return 0;
}
