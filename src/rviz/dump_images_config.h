#ifndef RVIZ_DUMP_IMAGES_CONFIG_H
#define RVIZ_DUMP_IMAGES_CONFIG_H

#include <stddef.h>
#include <string>
#include <vector>

namespace rviz
{

struct DumpImagesConfig {
  bool enabled;
  std::string captured_path;
  std::string keyed_path;
  std::string thumb_path;
  std::string poster_path;
  int thumbWidth;
  int maxWidth;
  int fpsNum;
  int fpsDen;
  double timeout;

  uint delayFrames;
  double frameWidth;
  double bagDuration;
  double nextTime;
  double lastEventTime;
};

} // namespace rviz

#endif
