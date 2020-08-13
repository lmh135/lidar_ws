#include <iostream>
#include "xsens_mt.h"

namespace xsens = octopus::device::xsens;

int main(int argc, char **argv) {

  xsens::XsensDriver driver;
  if(!driver.Init()) {
    std::cout << "xsens init failed!" << std::endl;
  }
  driver.Run();
  middleware::Spin();
  driver.Stop();
  return 0;
}
