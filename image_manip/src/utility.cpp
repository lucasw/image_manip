#include <image_manip/utility.h>

namespace image_manip
{

// TODO(lucasw) move to utility library
void updateTimer(ros::Timer& timer, const float frame_rate,
    const float old_frame_rate)
{
  if (frame_rate == old_frame_rate)
    return;

  if (frame_rate <= 0)
  {
    timer.stop();
    return;
  }
  else if ((frame_rate > 0) && (old_frame_rate <= 0))
  {
    timer.start();
  }

  const float period = 1.0 / frame_rate;
  const bool reset = true;
  timer.setPeriod(ros::Duration(period), reset);
}

}
