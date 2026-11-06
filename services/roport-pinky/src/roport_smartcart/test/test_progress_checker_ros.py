"""Installed Nav2 plugin with synthetic clock/poses; no controller or motor topic."""
import os
from pathlib import Path
import subprocess

from roport_smartcart.profiles import build_params


HARNESS = r'''
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "nav2_core/progress_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("smartcart_progress_test");
  node->declare_parameter("progress_checker.required_movement_radius", std::stod(argv[2]));
  node->declare_parameter("progress_checker.required_movement_angle", std::stod(argv[3]));
  node->declare_parameter("progress_checker.movement_time_allowance", std::stod(argv[4]));
  pluginlib::ClassLoader<nav2_core::ProgressChecker> loader("nav2_core", "nav2_core::ProgressChecker");
  auto checker = loader.createSharedInstance(argv[1]);
  checker->initialize(node, "progress_checker");
  auto clock = node->get_clock()->get_clock_handle();
  if (rcl_enable_ros_time_override(clock) != RCL_RET_OK) throw std::runtime_error("clock override");
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  auto check = [&](double seconds, double angle) {
    if (rcl_set_ros_time_override(clock, static_cast<int64_t>(seconds * 1e9)) != RCL_RET_OK)
      throw std::runtime_error("clock update");
    pose.pose.orientation.z = std::sin(angle / 2);
    pose.pose.orientation.w = std::cos(angle / 2);
    return checker->check(pose);
  };
  double base = 1.;
  for (double duration : {16., std::acos(-1.) / .15}) {
    checker->reset();
    for (double t = 0.; t < duration; t += .05)
      if (!check(base + t, .15 * t)) throw std::runtime_error("slow rotation rejected");
    if (!check(base + duration, .15 * duration)) throw std::runtime_error("final rotation rejected");
    std::cout << "rotation_pass " << duration << std::endl;
    base += duration + 1.;
  }
  checker->reset();
  if (!check(base, 0.) || !check(base + 14.9, 0.)) throw std::runtime_error("early stall failure");
  if (check(base + 15.1, 0.)) throw std::runtime_error("stationary stall accepted");
  std::cout << "stall_rejected 15.1" << std::endl;
  checker.reset();
  node.reset();
  rclcpp::shutdown();
}
'''


def test_installed_progress_plugin_accepts_slow_rotation_but_rejects_stall(tmp_path):
    package = Path(__file__).parents[1]
    params = build_params(package.parent / 'pinky_pro/pinky_navigation/params/nav2_params.yaml',
                          package / 'config/nav2_overrides.yaml', 'real',
                          footprint=[[.055, .065], [.055, -.065], [-.085, -.065], [-.085, .065]],
                          max_linear=.03, max_angular=.15)
    controller = params['controller_server']['ros__parameters']
    assert controller['progress_checker_plugins'] == ['progress_checker']
    checker = controller['progress_checker']
    (tmp_path / 'progress.cpp').write_text(HARNESS)
    (tmp_path / 'CMakeLists.txt').write_text('''
cmake_minimum_required(VERSION 3.16)
project(smartcart_progress_test)
find_package(ament_cmake REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
add_executable(progress_test progress.cpp)
ament_target_dependencies(progress_test nav2_controller pluginlib nav2_core rclcpp_lifecycle)
''')
    env = {**os.environ, 'ROS_DOMAIN_ID': '191', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    build = tmp_path / 'build'
    for command in (['cmake', '-S', str(tmp_path), '-B', str(build)],
                    ['cmake', '--build', str(build), '--parallel', '2']):
        completed = subprocess.run(command, env=env, capture_output=True, text=True, timeout=90)
        assert completed.returncode == 0, completed.stdout + completed.stderr
    completed = subprocess.run([str(build / 'progress_test'), checker['plugin'],
        str(checker['required_movement_radius']), str(checker['required_movement_angle']),
        str(checker['movement_time_allowance'])], env=env, capture_output=True, text=True, timeout=15)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert 'rotation_pass 16' in completed.stdout
    assert 'rotation_pass 20.944' in completed.stdout
    assert 'stall_rejected 15.1' in completed.stdout
