#include "utility.h"

namespace loam
{
class LaserMapping : public nodelet::Nodelet
{
public:
  virtual void onInit();
};
} // namespace loam

PLUGINLIB_EXPORT_CLASS(loam::LaserMapping, nodelet::Nodelet)