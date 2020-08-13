#pragma once

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <thread>
#include <mutex>
#include <atomic>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <xsens_msgs/msg/gnsspvtdata.hpp>
#include <xsens_msgs/msg/imudata.hpp>
#include <xsens_msgs/msg/rawimu.hpp>

#include "xsens.h"
#include "xsens_base.h"
#include "middleware/middleware.h"

//#include "transformation_util2/kinematic_util/math_util.hpp"
//#include "transformation_util2/kinematic_util/camera_projection.hpp"
#include "transformation_util2/kinematic_util/rotation_transformation.hpp"

namespace octopus {
namespace device {
namespace xsens {

struct Quaternion {
  double w;
  double x;
  double y;
  double z;
};

struct AppParamXsens {
  ParamXsens param;
  std::string ns;
  int no_rotation_duration;
  std::string frame_id_;
};

class XsensDriver : public Xsens {
  public:
    XsensDriver() : run_status_(true), connect_status_(false) { }
    ~XsensDriver() {}

    void GetParam();
    bool Init();
    void Run();
    void Stop();
    std::atomic_bool run_status_;
    std::atomic_bool connect_status_;

  private:
    /*
    * convert ros message
    */
    int ResetMessage();
    int FillMessage();
    int FillHeader();
    int FillGPS();
    int FillGNSS();
    int FillPosition();
    int FillVelocity();
    int FillMagnetic();
    int FillAcceleration();
    int FillOrientationData();
    int FillAngularVelocity();

    /*
    * xsenx control
    */
    void CleanUp();
    bool ConnectDevice();
    /*
    * monitor xsens message status
    */
    void Monitor(void);

    /*
    * Direct data analysis
    */
    void TaskXsens();
    int XsensDataProcAndPost();
    void ConstructPublishers();

    /*
    * tools
    */
    inline double ToDegree(double radians);
    inline double ToRadian(double degree);
    void ConvertFrameLocal(std::string sformat);
    Quaternion QMilt(Quaternion Q0, Quaternion Q1);
    Quaternion ConvertQuat(Quaternion q, CoordFormat source, CoordFormat dest);
    void ConvertCoords(double &x, double &y, double &z, CoordFormat source, CoordFormat dest = CoordFormat::Local);

    void StreamMonitorOption(middleware::DataStreamMonitorOption &opt, int topic_hz, bool monitor);

  private:
    AppParamXsens param_;
    std::shared_ptr<XsensDevice> xsens_;
    std::thread task_xsens_;
    bool bimu_pub_ = {false};
    bool bmagnetic_field_pub_ = {false};
    bool btwist_stamped_pub_ = {false};
    bool bxsens_imu_data_pub_ = {false};
    bool bimu_data_str_pub_ = {false};
    bool bgnss_pvt_data_pub_ = {false};
    std_msgs::msg::Header header_;
    sensor_msgs::msg::Imu imu_;
    sensor_msgs::msg::MagneticField magnetic_field_;
    geometry_msgs::msg::TwistStamped twist_stamped_;
    xsens_msgs::msg::IMUDATA xsens_imu_data_;
    std_msgs::msg::String imu_data_str_;
    xsens_msgs::msg::GNSSPVTDATA gnss_pvt_data_;
    std::shared_ptr<middleware::CustomMonitor> xsensCustomMonitorPtr_;
    std::shared_ptr<middleware::Publisher<sensor_msgs::msg::Imu>> pub_imu_;
    std::shared_ptr<middleware::Publisher<std_msgs::msg::String>> pub_imu_data_str_;
    std::shared_ptr<middleware::Publisher<xsens_msgs::msg::IMUDATA>> pub_xsens_imu_data_;
    std::shared_ptr<middleware::Publisher<geometry_msgs::msg::TwistStamped>> pub_twist_stamped_;
    std::shared_ptr<middleware::Publisher<sensor_msgs::msg::MagneticField>> pub_magnetic_field_;
    std::shared_ptr<middleware::Publisher<xsens_msgs::msg::GNSSPVTDATA>> pub_gnss_pvt_data_;

};

}  // namespace xsens
}  // namespace device
}  // namespace octopus
