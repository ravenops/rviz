#ifndef RVIZ_DUMP_IMAGES_CONFIG_H
#define RVIZ_DUMP_IMAGES_CONFIG_H

#include <stddef.h>
#include <string>
#include <vector>

namespace rviz
{

struct DumpImagesConfig {
  bool enabled;
  std::string folder;
  float fps;
  float timeout;

  uint delayFrames;
  double frameWidth;
  double bagDuration;
  double nextTime;
  double lastEventTime;
};

} // namespace rviz

#endif
