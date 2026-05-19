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


#ifndef SUPER_FSM_CONFIG_HPP
#define SUPER_FSM_CONFIG_HPP


#include <super_core/config.hpp>
#include <vector>
#include <cstring>
#include <utils/header/yaml_loader.hpp>

namespace fsm {
    using namespace traj_opt;
    using namespace super_planner;
    static constexpr int MPC_PVAJ_MODE = 1;
    static constexpr int MPC_POLYTRAJ_MODE = 2;

    class Config {
    public:
        bool timer_en{true};

        // Fsm Params
        bool click_goal_en{},visualization_en{};
        double replan_rate{}, resolution{};
        double click_height{};
        bool modify_goal_height{};

        bool click_yaw_en{};
        string cmd_topic, mpc_cmd_topic, click_goal_topic;
        string poly_cmd_topic;
        double yaw_dot_max{};

        // MPC parameters
        int mpc_horizon{};
        double mpc_dt{};
        int mpc_mode{}; // 1 for PVAJ mode, 2 for PolynomialTrajectory mode

        // Auto takeoff parameters
        bool auto_takeoff_enable{};
        double auto_takeoff_height{};
        double auto_takeoff_threshold{};
        double auto_takeoff_duration{};  // 起飞时间设置（秒）
        double auto_takeoff_max_velocity{};  // 起飞最大速度设置（m/s）

        // Goal reaching threshold
        double goal_reach_threshold{};

        bool save_full_replan_log{};

        Config() = default;

        Config(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            vector<double> tem_gain;
            loader.LoadParam("fsm/timer_en", timer_en, false);
            loader.LoadParam("fsm/click_goal_en", click_goal_en, false);
            loader.LoadParam("fsm/click_yaw_en", click_yaw_en, false);
            loader.LoadParam("fsm/replan_rate", replan_rate, 10.0);
            loader.LoadParam("fsm/modify_goal_height", modify_goal_height, false);
            loader.LoadParam("fsm/click_height", click_height, 1.5);
            loader.LoadParam("fsm/cmd_topic", cmd_topic, string("/planning/pos_cmd"));
            loader.LoadParam("fsm/poly_cmd_topic", poly_cmd_topic, string("/planning_cmd/poly_traj"));
            loader.LoadParam("fsm/mpc_cmd_topic", mpc_cmd_topic, string("/planning_cmd/mpc"));
            loader.LoadParam("fsm/click_goal_topic", click_goal_topic, string("/planning/click_goal_topic"));

            // Load MPC parameters
            loader.LoadParam("fsm/mpc_horizon", mpc_horizon, 10);
            loader.LoadParam("fsm/mpc_dt", mpc_dt, 0.01);
            loader.LoadParam("fsm/mpc_mode", mpc_mode, MPC_PVAJ_MODE);

            // Load auto takeoff parameters
            loader.LoadParam("fsm/auto_takeoff_enable", auto_takeoff_enable, true);
            loader.LoadParam("fsm/auto_takeoff_height", auto_takeoff_height, 1.5);
            loader.LoadParam("fsm/auto_takeoff_threshold", auto_takeoff_threshold, 0.2);
            loader.LoadParam("fsm/auto_takeoff_duration", auto_takeoff_duration, 3.0);  // 默认3秒起飞时间
            loader.LoadParam("fsm/auto_takeoff_max_velocity", auto_takeoff_max_velocity, 1.0);  // 默认最大速度1m/s

            // Load goal reaching threshold
            loader.LoadParam("fsm/goal_reach_threshold", goal_reach_threshold, 0.3);

            loader.LoadParam("super_planner/yaw_dot_max", yaw_dot_max, 1.0, true);
            loader.LoadParam("super_planner/visualization_en", visualization_en, false, true);
            loader.LoadParam("rog_map/resolution", resolution, 0.1, true);

            loader.LoadParam("fsm/save_full_replan_log", save_full_replan_log, false);
        }
    };
}

#endif //SUPER_FSM_CONFIG_H
