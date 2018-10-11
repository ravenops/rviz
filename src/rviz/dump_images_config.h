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
  int scale;
};

} // namespace rviz

#endif
