/*
 * Copyright (c) 2017 Lucas Walter
 * November 2017
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

#include <filesystem>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_folder_publisher");
  ros::NodeHandle nh;

  ros::Rate rate(5.0);

  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("image", 5);

  std::string path = ".";
  nh.getParam("path", path);

  while (ros::ok()) {
    for (const auto& entry: std::filesystem::directory_iterator(path)) {
      try {
        const std::string filename = entry.path();
        cv_bridge::CvImage cv_image;
        cv_image.encoding = "bgr8";
        cv_image.image = cv::imread(filename, cv::IMREAD_COLOR);
        if (cv_image.image.size() == cv::Size(0, 0)) {
          continue;
        }
        ROS_INFO_STREAM(filename);
        auto image_msg = cv_image.toImageMsg();
        // ROS_INFO_STREAM(cv_image.image.size() << " " << image_msg->data.size());
        image_pub.publish(image_msg);
      } catch (const std::exception& ex) {
        ROS_WARN_STREAM(ex.what());
      }
      rate.sleep();
    }
  }

  return 0;
}
