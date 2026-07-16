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

#include "ros_interface/ros1/fsm_ros1.hpp"
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
FsmRos1::Ptr fsm_ptr;

#include <ros/console.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    cout << GREEN << " -- [Fsm-Test] Begin." << RESET << endl;

#define CONFIG_FILE_DIR(name) (string(string(ROOT_DIR) + "config/"+name))
    std::string dft_cfg_path = CONFIG_FILE_DIR("click.yaml");
    std::string cfg_path, cfg_name;
    if (nh.param("config_path", cfg_path, dft_cfg_path)) {
        cout << " -- [Fsm-Test] Load config from: " << cfg_path << endl;
    } else if(nh.param("config_name", cfg_name, dft_cfg_path)){
        cfg_path = CONFIG_FILE_DIR(cfg_name);
        cout << " -- [Fsm-Test] Load config by file name: " << cfg_name << endl;
    }

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

    fsm_ptr = make_shared<FsmRos1>();
    fsm_ptr->init(nh, cfg_path, metric_monitor);

    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    spinner.stop();
    fsm_ptr.reset();
    metric_monitor->stop();
    metric_monitor.reset();
    ros::shutdown();
    std::cout << "main exit..." << std::endl;
    fflush(stdout);
    return 0;
}
