/*
 * Copyright (c) 2017 Lucas Walter
 * July 2017
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

#include <cv_bridge/cv_bridge.h>
#include <deque>
// #include <image_manip/image_deque.h>
#include <image_manip/utility.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16.hpp>
using std::placeholders::_1;


namespace image_manip
{
class ImageDeque : public rclcpp::Node
{
public:
  ImageDeque();
  ~ImageDeque();

  void init();
private:
  // send a bool to indicate that an image was saved
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr captured_trigger_pub_;
  // publish the most recent captured image
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr captured_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr anim_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // this is for appending onto the animation output
  sensor_msgs::msg::Image::SharedPtr live_frame_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> images_;

  // TODO(lucasw) some of these ought to be parameters
  float frame_rate_ = 5.0;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr frame_rate_sub_;
  void frameRateCallback(const std_msgs::msg::Float32::SharedPtr msg);

  bool capture_single_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr capture_single_sub_;
  void captureSingleCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      RCLCPP_INFO(get_logger(), "capture");
      capture_single_ = msg->data;
      dirty_ = true;
    }
  }

  unsigned int index_ = 0;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr index_sub_;
  void indexCallback(const std_msgs::msg::UInt16::SharedPtr msg);

  // TODO(lucasw) index_fraction

  unsigned int start_index_ = 0;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr start_index_sub_;
  void startIndexCallback(const std_msgs::msg::UInt16::SharedPtr msg)
  {
    if (start_index_ != msg->data) {
      RCLCPP_INFO(get_logger(), "new start index %d", start_index_);
    }
    start_index_ = msg->data;
    dirty_ = true;
  }

  bool use_live_frame_ = true;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr use_live_frame_sub_;
  void useLiveFrameCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    use_live_frame_ = msg->data;
    dirty_ = true;
  }

  bool capture_continuous_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr capture_continuous_sub_;
  void captureContinuousCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    capture_continuous_ = msg->data;
    dirty_ = true;
  }

  unsigned int max_size_ = 1000;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr max_size_sub_;
  void maxSizeCallback(const std_msgs::msg::UInt16::SharedPtr msg)
  {
    max_size_ = msg->data;
    dirty_ = true;
  }
  //////

  bool dirty_ = false;
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::TimerBase::SharedPtr timer_;
  void update();
};  // ImageDeque

ImageDeque::ImageDeque() : Node("image_deque")
{
  init();
}

ImageDeque::~ImageDeque()
{
}

void ImageDeque::frameRateCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  const float frame_rate = msg->data;
  if (frame_rate == frame_rate_)
    return;
  frame_rate_ = frame_rate;

  if (frame_rate <= 0)
  {
    timer_->cancel();
  }
  else
  {
    const int period_ms = 1000.0 / frame_rate;
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
        std::bind(&ImageDeque::update, this));
  }
}

void ImageDeque::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!use_live_frame_ && !(capture_single_ || capture_continuous_))
    return;

  live_frame_ = msg;

  if (!(capture_single_ || capture_continuous_))
    return;

  RCLCPP_INFO(get_logger(), "capturing image %d + 1, %d", images_.size(), max_size_);
  images_.push_back(msg);

  // TODO(lucasw) make this optional
  // save the image with unique timestamp from init time + counter
  // Or have node that just saves any image it receives?

  if (capture_single_)
  {
    // should capture single capture the one already received, rather
    // than here it captures the next one received?
    // RCLCPP("capturing single");
    capture_single_ = false;
  }

  if (max_size_ > 0)
  {
    while (images_.size() > max_size_)
      // TODO(lucasw) also could have mode where it fills and then doesn't accept any more
      images_.pop_front();
  }

  // TODO(lucasw) could put this in separate thread.
  // publish the exact same message received
  captured_pub_->publish(msg);
  std_msgs::msg::Bool trigger;
  trigger.data = true;
  captured_trigger_pub_->publish(trigger);

  // TODO(lucasw) was this disabled earlier to work with the stop motion launch?
  dirty_ = true;
}

void ImageDeque::indexCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
  // if index is incrementing slower than the capture rate,
  // then odd backwards motion may result.
  if (msg->data != index_)
    dirty_ = true;
  index_ = msg->data;

  if (frame_rate_ == 0)
  {
    update();
  }
}

void ImageDeque::update()
{
  if (!dirty_ && !use_live_frame_)
    return;

  // ROS_DEBUG_STREAM(index_ << " " << images_.size());
  // TEMP code to show output of frames
  // TODO(lucasw) have index be controlled by topic
  if (index_ <= images_.size())
  {
    if (index_ < images_.size())
    {
      anim_pub_->publish(images_[index_]);
    }
    else if (use_live_frame_ && live_frame_)
    {
      // preview the live frame at the end of the saved animation
      anim_pub_->publish(live_frame_);
    }
    // TODO(lwalter) else publish something else?
    index_++;
  }

  // RCLCPP_INFO(get_logger(), "%d %d\n", images_.size(), index_);

  // TODO(lucasw) maybe if start_index changes
  // it should force index_ to it, instead of waiting to cycle.
  if ((use_live_frame_ && (index_ > images_.size())) ||
      (!use_live_frame_ && (index_ >= images_.size())))
  {
    index_ = start_index_;
  }

  dirty_ = false;
}

void ImageDeque::init()
{
  captured_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("captured_image_trigger");
  captured_pub_ = create_publisher<sensor_msgs::msg::Image>("captured_image");
  anim_pub_ = create_publisher<sensor_msgs::msg::Image>("anim");

  image_sub_ = create_subscription<sensor_msgs::msg::Image>("image",
      std::bind(&ImageDeque::imageCallback, this, _1));

  frame_rate_sub_ = this->create_subscription<std_msgs::msg::Float32>("frame_rate",
      std::bind(&ImageDeque::frameRateCallback, this, _1));
  capture_single_sub_ = create_subscription<std_msgs::msg::Bool>("capture_single",
      std::bind(&ImageDeque::captureSingleCallback, this, _1));
  index_sub_ = create_subscription<std_msgs::msg::UInt16>("index",
      std::bind(&ImageDeque::indexCallback, this, _1));
  start_index_sub_ = create_subscription<std_msgs::msg::UInt16>("start_index",
      std::bind(&ImageDeque::startIndexCallback, this, _1));
  use_live_frame_sub_ = create_subscription<std_msgs::msg::Bool>("use_live_frame",
      std::bind(&ImageDeque::useLiveFrameCallback, this, _1));
  capture_continuous_sub_ = create_subscription<std_msgs::msg::Bool>("capture_continuous",
      std::bind(&ImageDeque::captureContinuousCallback, this, _1));
  max_size_sub_ = create_subscription<std_msgs::msg::UInt16>("max_size",
      std::bind(&ImageDeque::maxSizeCallback, this, _1));

  const int period_ms = 1000.0 / frame_rate_;
  timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
      std::bind(&ImageDeque::update, this));
}

}  // namespace image_manip

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::spin(std::make_shared<image_manip::ImageDeque>());
  rclcpp::shutdown();
  return 0;
}
