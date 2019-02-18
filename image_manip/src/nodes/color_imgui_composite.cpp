/*
 * Copyright (c) 2018 Lucas Walter
 * October 2018
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>
#include <imgui_ros/imgui_ros.h>
#include <image_manip/color.hpp>
#include <internal_pub_sub/internal_pub_sub.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <thread>

void run_node(std::shared_ptr<rclcpp::Node> node)
{
  // rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto core = std::make_shared<internal_pub_sub::Core>();

  std::vector<std::thread> threads;
  // std::map<std::string, std::thread>

  // imgui_ros has to be single threaded for now to avoid context switches with opengl
  auto imgui_ros = std::make_shared<imgui_ros::ImguiRos>(core);
  threads.push_back(std::thread(std::bind(run_node, imgui_ros)));

  for (size_t i = 0; i < 2; ++i) {
    auto color = std::make_shared<image_manip::Color>();
    // TODO(lucasw) couldn't these three be combined?
    color->init("color_" + std::to_string(i));
    color->postInit(core);
    threads.push_back(std::thread(std::bind(run_node, color)));
  }

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  for (size_t i = 0; i < threads.size(); ++i) {
    threads[i].join();
  }

  rclcpp::shutdown();
  return 0;
}
