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

#include <fsm/fsm.h>
#include <memory>
#include <malloc.h>

using namespace super_utils;

namespace fsm {
    Fsm::~Fsm() {
        write_time_.close();
    }

    void Fsm::WriteTimeToLog() {
        write_time_ << (ros_ptr_->getSimTime() - system_start_time_) << ", ";
        for (long unsigned int i = 0; i < log_module_time.size(); i++) {
            write_time_ << log_module_time[i];
            if (i != log_module_time.size() - 1) {
                write_time_ << ", ";
            }
        }
        write_time_ << endl;
    }

    void Fsm::callReplanOnce() {
        if (stop) {
            return;
        }

        if (machine_state_ != FOLLOW_TRAJ) {
            return;
        }

        if (finish_plan) {
            return;
        }

        if (plan_from_rest_) {
            plan_from_rest_ = false;
            return;
        }

        planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, gi_.goal_p, gi_.goal_p, 3.0);

        TimeConsuming replan_once_time("replan_once_time", false);

        RET_CODE ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
        if (ret_code == FAILED) {
//            cout << YELLOW << " -- [Fsm] ReplanOnce failed." << RESET << endl;
        } else { cout << GREEN << " -- [Fsm] ReplanOnce succeed." << RESET << endl; }

        if (ret_code == EMER) {
            ChangeState("ReplanTimerCallback", EMER_STOP);
        } else if (ret_code == NEW_TRAJ) {
            ChangeState("ReplanTimerCallback", GENERATE_TRAJ);
        } else if (ret_code == SUCCESS || ret_code == FINISH) {
            gi_.new_goal = false;
            publishPolyTraj();
        }

        planner_ptr_->getModuleTimeConsuming(log_module_time);
        log_module_time[log_module_time.size() - 2] = replan_once_time.stop();
        // save on log
        if( !cfg_.save_full_replan_log) {
            replan_logs_.clear();
        }
        replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
        WriteTimeToLog();
    }

    void Fsm::callMainFsmOnce() {
        if (stop) {
            return;
        }
        // enable for memory optimize, but this will hurt effiency
        // static double last_trim_t = -1.0;
        // const double now_trim_t = ros_ptr_->getSimTime();
        // if (last_trim_t < 0 || (now_trim_t - last_trim_t) > 1.0) {
        //     last_trim_t = now_trim_t;
        //     malloc_trim(0);
        // }
        static double fsm_start_time = ros_ptr_->getSimTime();
        double cur_t = (ros_ptr_->getSimTime() - fsm_start_time);
        static double last_print_t = 0.0;
        planner_ptr_->getRobotState(robot_state_);


        if (cur_t - last_print_t > 1.0) {
            last_print_t = cur_t;
            if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                return;
            }
            if (!started_) {
                cout << YELLOW << " -- [Fsm] Wait for goal." << RESET << endl;
            }
            cout << std::fixed << std::setprecision(3);
            cout << GREEN << " -- [Fsm " << cur_t << "] Current state: " << MACHINE_STATE_STR[machine_state_]
                 << RESET << endl;
        }

        switch (machine_state_) {
            case INIT: {
                if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                    cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                    return;
                }

                // Check if auto takeoff is needed (regardless of started_ status)
                if (isRobotBelowVirtualGround())
                {
                    fmt::print(fg(fmt::color::yellow), " -- [Fsm] Robot below virtual ground (z={:.2f}), triggering auto takeoff to height {:.2f}m.\n",
                            robot_state_.p.z(), cfg_.auto_takeoff_height);
                    auto_takeoff_triggered_ = true;
                    auto_takeoff_start_time_ = ros_ptr_->getSimTime();
                    takeoff_status_ = TAKEOFF_IN_PROGRESS;
                    ChangeState("MainFsmCallback", AUTO_TAKEOFF);
                }
                else if (started_)
                {
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                }
                break;
            }
            case AUTO_TAKEOFF:
            {
                // Check if robot has reached desired takeoff height
                double height_diff = std::abs(robot_state_.p.z() - cfg_.auto_takeoff_height);
                double takeoff_duration = ros_ptr_->getSimTime() - auto_takeoff_start_time_;

                // Additional checks for takeoff success
                bool height_reached = height_diff < 0.1;
                bool position_stable = robot_state_.p.z() > (cfg_.auto_takeoff_height - 0.2); // Within reasonable range
                // TODO: 60 seconds after MPC node online
                bool timeout_reached = takeoff_duration > 300.0;

                // Safety checks
                bool position_reasonable = robot_state_.p.z() > -1.0 && robot_state_.p.z() < 10.0; // Reasonable height range
                bool velocity_reasonable = robot_state_.v.norm() < 5.0; // Reasonable velocity limit
                
                if (!position_reasonable || !velocity_reasonable)
                {
                    // Emergency stop if position or velocity is unreasonable
                    takeoff_status_ = TAKEOFF_FAILED_UNSTABLE;
                    auto_takeoff_triggered_ = false;
                    fmt::print(fg(fmt::color::red), " -- [Fsm] Auto takeoff EMERGENCY STOP: unreasonable state! Height: {:.2f}m, velocity: {:.2f}m/s.\n", 
                            robot_state_.p.z(), robot_state_.v.norm());
                    ChangeState("MainFsmCallback", EMER_STOP);
                    break;
                }

                if ((height_reached && position_stable) || timeout_reached)
                {
                    auto_takeoff_triggered_ = false;
                    
                    if (timeout_reached && !height_reached)
                    {
                        // Takeoff failed due to timeout
                        takeoff_status_ = TAKEOFF_FAILED_TIMEOUT;
                        fmt::print(fg(fmt::color::red), " -- [Fsm] Auto takeoff FAILED: timeout after {:.1f}s. Current height: {:.2f}m, target: {:.2f}m.\n", 
                                takeoff_duration, robot_state_.p.z(), cfg_.auto_takeoff_height);
                        fmt::print(fg(fmt::color::yellow), " -- [Fsm] Switching to manual mode. Please check drone status.\n");
                    }
                    else
                    {
                        // Successful takeoff
                        takeoff_status_ = TAKEOFF_SUCCESS;
                        fmt::print(fg(fmt::color::green), " -- [Fsm] Auto takeoff SUCCESS: height {:.2f}m reached in {:.1f}s, velocity: {:.2f}m/s.\n", 
                                robot_state_.p.z(), takeoff_duration, robot_state_.v.norm());
                    }
                    
                    started_ = true; // Mark as started after takeoff attempt
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                }
                else
                {
                    // Continue publishing takeoff command with status feedback
                    static double last_status_print = 0.0;
                    if (takeoff_duration - last_status_print > 1.0) // Print status every second
                    {
                        last_status_print = takeoff_duration;
                        fmt::print(fg(fmt::color::cyan), " -- [Fsm] Takeoff progress: height {:.2f}m/{:.2f}m, velocity {:.2f}m/s, time {:.1f}s\n", 
                                robot_state_.p.z(), cfg_.auto_takeoff_height, robot_state_.v.norm(), takeoff_duration);
                    }
                    publishAutoTakeoffCommand();
                }
                break;
            }
            case WAIT_GOAL: {
                if (!gi_.new_goal) {
                    return;
                } else {
                    ChangeState("MainFsmCallback", GENERATE_TRAJ);
                }
                resetVisualizedPath();
                break;
            }
            case GENERATE_TRAJ: {
                if (closeToGoal(cfg_.goal_reach_threshold)) {
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    gi_.new_goal = false;
                    finish_plan = true;
                    return;
                }
                int retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
                if (!planner_ptr_->goalValid()) {
                    cout << YELLOW << " -- [Fsm] Goal is invalid, skip this goal." << RESET << endl;
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    return;
                }
                if (retcode == SUCCESS || retcode == FINISH) {
                    gi_.new_goal = false;
                    plan_from_rest_ = true;
                    finish_plan = false;
                    if (retcode == FINISH) {
                        finish_plan = true;
                    }

                    publishPolyTraj();

                    ChangeState("MainFsmCallback", FOLLOW_TRAJ);
                } else {
                    cout << YELLOW << " -- [Fsm] PlanFromRest failed, try replan." << RESET << endl;
                    // ros::Duration(0.1).sleep();
                }
                if( !cfg_.save_full_replan_log) {
                    replan_logs_.clear();
                }
                replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
                break;
            }
            case FOLLOW_TRAJ: {
                publishCurPoseToPath();
                break;
            }
            case EMER_STOP: {
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            default:
                break;
        }
    }

    bool Fsm::closeToGoal(const double &thresh_dis) {
        /// The close to goal should consider the the local shift
        /// All goal should be in the known free on inf map.
        /// The intermedia points should be in free space.
        double dis = (robot_state_.p - gi_.goal_p).norm();
        return dis < thresh_dis;
    }

    void Fsm::setGoalPosiAndYaw(const Vec3f &p, const Quatf &q) {

        auto click_point = p;
        if (cfg_.click_height > -5 && cfg_.modify_goal_height) {
            click_point.z() = cfg_.click_height;
        }

        if((click_point - gi_.goal_p).norm() < cfg_.goal_reach_threshold) {
            // cout << GREEN << " -- [Fsm] Ignore target goal at: " << click_point.transpose()
            //     << " current goal: " << gi_.goal_p.transpose() << RESET << endl;
            return;
        }

        if (planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, click_point, gi_.goal_p, 3.0)) {
            cout << GREEN << " -- [Fsm] Get goal at " << RESET << gi_.goal_p.transpose() << endl;
        } else {
            fmt::print(fg(fmt::color::indian_red), "Goal is deeply occupied, skip this goal.\n");
            return;
        }

        if ((robot_state_.p - gi_.goal_p).norm() <
            0.1) {
            //                print(fg(color::gray), " -- [Rviz] Too close to goal, skip this target.\n");
            return;
        }

        if (cfg_.click_yaw_en) {
            if (isnan(q.w()) || isnan(q.x()) || isnan(q.y()) || isnan(q.z())) {
                gi_.goal_yaw = NAN;
                ros_ptr_->info(" -- [Fsm] Receive click goal at: [{}, {}, {}]; goal yaw disabled",
                               gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z());
            } else {
                gi_.goal_yaw = geometry_utils::get_yaw_from_quaternion(q);
                cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw: "
                     << gi_.goal_yaw * 57.3 << " deg" << RESET << endl;
            }

        } else {
            gi_.goal_yaw = NAN;
            cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw disabled"
                 << RESET << endl;
        }

        started_ = true;
        gi_.new_goal = true;
    }

    void Fsm::ChangeState(const string &call_func, const MACHINE_STATE &new_state) {
        fmt::print(fg(fmt::color::green), " -- [Fsm]: [{}] change state from [{}] to [{}].\n", call_func,
                   MACHINE_STATE_STR[int(machine_state_)], MACHINE_STATE_STR[int(new_state)]);
        machine_state_ = new_state;
    }

    bool Fsm::isRobotBelowVirtualGround()
    {
        if (!cfg_.auto_takeoff_enable)
        {
            return false;
        }

        // Get virtual ground height from ROG map configuration
        if (!planner_ptr_ || !planner_ptr_->getMap())
        {
            return false;
        }

        const auto &rog_map_cfg = planner_ptr_->getMap()->getMapConfig();
        double virtual_ground_height = rog_map_cfg.virtual_ground_height;

        // Check if robot is below virtual ground threshold
        double height_diff = robot_state_.p.z() - virtual_ground_height;
        return height_diff < cfg_.auto_takeoff_threshold;
    }
}
