/*
 * Copyright (c) 2017 Lucas Walter
 * June 2017
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


#include <image_manip/utility.h>
#include <opencv2/imgproc.hpp>

namespace image_manip
{

/// resize the source tmp0 mat to fit inside tmp1 with borders
/// tmp0 and tmp1 have to be initialized already
/// TBD add another mode which chops off the edges so there
/// are no borders?
bool resizeFixAspect(const cv::Mat& tmp0, cv::Mat& tmp1,
    const cv::Size sz, const int mode)
{
  tmp1 = cv::Mat(sz, tmp0.type(), cv::Scalar::all(0));

  const float aspect_0 = static_cast<float>(tmp0.cols) / static_cast<float>(tmp0.rows);
  const float aspect_1 = static_cast<float>(tmp1.cols) / static_cast<float>(tmp1.rows);

  // this is the subimage that has to fit within tmp1
  // it will be shrunk down as necessary and border offset
  // values adjusted
  cv::Size tmp_sz = tmp1.size();
  int off_x = 0;
  int off_y = 0;

  // TBD could have epsilon defined by 1 pixel width
  if (aspect_0 > aspect_1)
  {
    // have to have a border on top
    tmp_sz.height = tmp_sz.width / aspect_0;
    off_y = (sz.height - tmp_sz.height) / 2;
  }
  else if (aspect_0 < aspect_1)
  {
    // have a border on the sides
    tmp_sz.width = tmp_sz.height * aspect_0;
    off_x = (sz.width - tmp_sz.width) / 2;
  }

  #if 0
  ROS_DEBUG_STREAM_COND(log_level > 2, "fix aspect " << aspect_0 << " " << aspect_1 << ", "
                        << off_x << " " << off_y << ", "
                        << tmp_sz.width << " " << tmp_sz.height << ", "
                        << sz.width << " " << sz.height);
  #endif

  // the source image with the right aspect ratio and size
  // to fit within the dest image
  cv::Mat tmp_aspect;
  cv::resize(tmp0, tmp_aspect, tmp_sz, 0, 0, mode);

  // TBD put offset so image is centered
  cv::Mat tmp1_roi = tmp1(cv::Rect(off_x, off_y, tmp_sz.width, tmp_sz.height));
  tmp_aspect.copyTo(tmp1_roi);

  return true;
}

/// another mode which chops off the edges so there
/// are no borders
bool resizeFixAspectFill(const cv::Mat& tmp0, cv::Mat& tmp1,
    const cv::Size sz, const int mode)
{
  tmp1 = cv::Mat(sz, tmp0.type(), cv::Scalar::all(0));
  // width/height
  const float aspect_0 = static_cast<float>(tmp0.cols) / static_cast<float>(tmp0.rows);
  const float aspect_1 = static_cast<float>(tmp1.cols) / static_cast<float>(tmp1.rows);

  // const cv::Size src_sz = tmp0.size();

  // this is the subimage that has to fit within tmp1
  // it will be shrunk down as necessary and border offset
  // values adjusted
  cv::Size tmp_sz = sz;
  int off_x = 0;
  int off_y = 0;

  // TBD could have epsilon defined by 1 pixel width
  if (aspect_0 > aspect_1)
  {
    // lose the edges off the sides
    tmp_sz.width = sz.height * aspect_0;
    off_x = (tmp_sz.width - sz.width) / 2;
  }
  else if (aspect_0 < aspect_1)
  {
    // lose the edges of the top and bottom
    tmp_sz.height = sz.width / aspect_0;
    off_y = (tmp_sz.height - sz.height) / 2;
  }

  #if 0
  ROS_DEBUG_STREAM_COND(log_level > 2, "fix aspect fill "
                        << "src " << src_sz.width << " " << src_sz.height << ", "
                        << "tmp " << off_x << " " << off_y << " "
                        << tmp_sz.width << " " << tmp_sz.height << ", "
                        << "dst " << sz.width << " " << sz.height);
  #endif

  // resize the source image so that it is equal to the dest
  // image in at least one dimension,
  cv::Mat tmp_aspect;
  cv::resize(tmp0, tmp_aspect, tmp_sz, 0, 0, mode);

  // now take just the subimage so the aspect is preserved
  // while the resolution is dst
  cv::Mat tmp1_roi = tmp_aspect(cv::Rect(off_x, off_y, sz.width, sz.height));
  tmp1_roi.copyTo(tmp1);

  return true;
}

void getPerspectiveTransform(const float& wd, const float& ht,
    const float& phi, const float& theta, const float& psi,
    float& off_x, float& off_y, const float& z, const float& z_scale,
    cv::Point3f& center, cv::Mat& transform)
{
  float scale = 1.0;
  const bool nrm_px = true;
  // TODO(lucasw) this has no effect
  center.z = 0.1;  // config_.center_z;
  float off_z = 0.0;  // config_.off_z;

  if (nrm_px)
  {
    center.x = center.x * wd;  // + wd * 3 / 4;
    center.y = center.y * ht;  // + ht * 3 / 4;
    // center.z *= ht;

    off_x = off_x * wd + wd / 2;
    off_y = off_y * ht + ht / 2;
    // off_z *= ht;
  }

  cv::Mat in_p = (cv::Mat_<float>(3, 4) <<
                  0, wd, wd, 0,
                  0, 0,  ht, ht,
                  0, 0, 0, 0);

  cv::Mat in_roi = in_p.t()(cv::Rect(0, 0, 2, 4));  // ).clone();
  in_roi = in_roi.clone();

  cv::Mat out_roi;
  {
    // This implements a standard rotozoom
    // move the image prior to rotation
    cv::Mat offset = (cv::Mat_<float>(3, 4) <<
                      off_x, off_x, off_x, off_x,
                      off_y, off_y, off_y, off_y,
                      off_z, off_z, off_z, off_z);

    // shift the image after rotation
    cv::Mat center_m = (cv::Mat_<float>(3, 4) <<
                        center.x, center.x, center.x, center.x,
                        center.y, center.y, center.y, center.y,
                        center.z, center.z, center.z, center.z);

    // Rotation matrices
    cv::Mat rotz = (cv::Mat_<float>(3, 3) <<
                    cos(phi), -sin(phi), 0,
                    sin(phi),  cos(phi), 0,
                    0, 0, 1);

    cv::Mat roty = (cv::Mat_<float>(3, 3) <<
                    cos(theta),  0, sin(theta),
                    0, 1, 0,
                    -sin(theta), 0, cos(theta));

    cv::Mat rotx = (cv::Mat_<float>(3, 3) <<
                    1,  0,        0,
                    0,  cos(psi), sin(psi),
                    0, -sin(psi), cos(psi));

    // TBD reformat the matrices so all the transposes aren't necessary

    // Transform into ideal coords
    // float fx = getSignal("fx");
    cv::Mat out_p = (in_p - offset).t() * rotx.t() * roty.t() * rotz.t() * scale;  // + center_m.t();
    out_roi = out_p(cv::Rect(0, 0, 2, 4)).clone();

    // this moves the image away from the 0 plane
    // TODO(lucasw) this has no effect
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        // this makes the projection non-orthographic
        const float out_p_z = out_p.at<float>(i, 2);
        out_roi.at<float>(i, j) = out_p.at<float>(i, j) / (out_p_z * z_scale + z) +
            center_m.at<float>(j, i) + offset.at<float>(j, i);
      }
    }
  }
  transform = cv::getPerspectiveTransform(in_roi, out_roi);
}

}  // namespace image_manip
