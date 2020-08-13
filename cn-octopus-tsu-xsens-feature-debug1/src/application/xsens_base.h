#pragma once

namespace octopus {
namespace device {
namespace xsens {

class Xsens {
 public:
  Xsens() {}
  ~Xsens() {}

  virtual void GetParam() = 0;
  virtual bool Init() = 0;
  virtual void Run() = 0;
  virtual void Stop() = 0;
};

} // namespace xsens
} // namespace device
} // namespace octopus