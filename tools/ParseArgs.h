#pragma once

#include <memory>

#include "libvideoio/DataSource.h"
#include "util/Undistorter.h"

namespace lsd_slam {

struct ParseArgs {

  ParseArgs( int argc, char **argv );

  std::shared_ptr<libvideoio::DataSource> dataSource;
  std::shared_ptr<Undistorter> undistorter;

};


}
