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

#include <opencv2/opencv.hpp>

namespace image_manip
{

// TODO(lucasw) mirror initUndistortRectifyMap as much as makes sense
// Add m1type parameter
// And R rectification matrix?
// Need to investigate non-floating point version.
void initDistortMap(const cv::Mat& cameraMatrix, const cv::Mat distCoeffs,
    const cv::Size size,
    cv::Mat& map1, cv::Mat& map2)
{
  cv::Mat pixel_locations_src = cv::Mat(size.width * size.height, 1, CV_32FC2);

  int ind = 0;
  for (int i = 0; i < size.height; i++) {
    for (int j = 0; j < size.width; j++) {
      // TODO(lucasw) maybe would want x and y offsets here to make the output
      // image bigger or smaller than the input?
      pixel_locations_src.at<cv::Point2f>(ind, 0) = cv::Point2f(j, i);
      ++ind;
    }
  }

  cv::Mat fractional_locations_dst = cv::Mat(pixel_locations_src.size(), CV_32FC2);
  cv::undistortPoints(pixel_locations_src, fractional_locations_dst, cameraMatrix, distCoeffs);

  const float fx = cameraMatrix.at<double>(0, 0);
  const float fy = cameraMatrix.at<double>(1, 1);
  const float cx = cameraMatrix.at<double>(0, 2);
  const float cy = cameraMatrix.at<double>(1, 2);

  // TODO(lucasw) is there a faster way to do this?
  // A matrix operation?
  // If the x and y were separate matrices it would be possible,
  // and remap does take separate x and y maps.
  cv::Mat pixel_locations_dst = cv::Mat(size, CV_32FC2);
  ind = 0;
  for (int i = 0; i < size.height; i++) {
    for (int j = 0; j < size.width; j++) {
      const float x = fractional_locations_dst.at<cv::Point2f>(ind, 0).x * fx + cx;
      const float y = fractional_locations_dst.at<cv::Point2f>(ind, 0).y * fy + cy;
      pixel_locations_dst.at<cv::Point2f>(i, j) = cv::Point2f(x, y);
      // if ((i == 0) && (j == 0))
      //  ROS_INFO_STREAM(ind << ": " << i << " " << j << ", " << y << " " << x);
      ++ind;
    }
  }

  map1 = pixel_locations_dst;
  map2 = cv::Mat();
}

// adapted from http://code.opencv.org/issues/1387 (which had errors)
// TODO(lucasw) I think this can make use of convertMaps to make it even more efficient
void distort(const cv::Mat& src, cv::Mat& image_dst,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
  cv::Mat map1;
  cv::Mat map2;

  // TODO(lucasw) will passing in a different size work as expected?
  // or will it require adjustment of cx/cy?
  initDistortMap(cameraMatrix, distCoeffs, src.size(), map1, map2);
  cv::remap(src, image_dst, map1, map2, cv::INTER_LINEAR);
}

}  // namespace image_manip
