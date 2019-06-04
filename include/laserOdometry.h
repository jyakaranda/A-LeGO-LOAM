#include "utility.h"

namespace loam
{
class LaserOdometry : public nodelet::Nodelet
{
public:
  virtual void onInit();
};
} // namespace loam

PLUGINLIB_EXPORT_CLASS(loam::LaserOdometry, nodelet::Nodelet)