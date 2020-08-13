#include "xsens_mt.h"
#include "sensor_common_api.h"
#include "sensor_data_defs.h"
#include "middle_sensor_interface.h"

namespace octopus {
namespace device {
namespace xsens {

using namespace kinematic_util;
int XsensDriver::FillHeader() {
  uint64_t gps_timestamp = 0, gps_timestamp_sec = 0;
  uint64_t sys_timestamp = 0;
  sys_timestamp = middleware::Time::Now().ToNsec();
  UTime *ptime = &(xsens_->msg_.time);
  if (ptime->use && ptime->utc_time_use) {
    struct tm gps_time;
    // struct tm utc time
    // year must subtract 1900, Record the number of years since 1900
    // month must subtract 1, the month begin zoro
    gps_time.tm_year = ptime->utc_time.year - 1900;
    gps_time.tm_mon = ptime->utc_time.month - 1;
    gps_time.tm_mday = ptime->utc_time.day;
    gps_time.tm_hour = ptime->utc_time.hour;
    gps_time.tm_min = ptime->utc_time.minute;
    gps_time.tm_sec = ptime->utc_time.second;
    gps_time.tm_isdst = 0;
    gps_timestamp_sec = (uint64_t)timegm(&gps_time);
    gps_timestamp = gps_timestamp_sec * 1e9 + (uint64_t)ptime->utc_time.ns;
  } else {
    MW_ERROR("gps msg none\n");
  }
  if (((double)(abs(gps_timestamp - sys_timestamp)) * 1e-6) > 30.0) {
    header_.stamp = middleware::Time(sys_timestamp);
    header_.frame_id = param_.frame_id_ + "_sys";
    MW_INFO("gps time is not ok, please check it gps:{:<19} -> sys:{:<19}", gps_timestamp, sys_timestamp);
  } else {
    header_.stamp.sec = gps_timestamp_sec;
    header_.stamp.nanosec = ptime->utc_time.ns;
    header_.frame_id = param_.frame_id_ + "_gps";
  }
  return 0;
}

int XsensDriver::FillOrientationData() {

  OrientationData *pori = &(xsens_->msg_.orientation_data);
  if (!pori->use)
    return -1;
  double w, x, y, z;
  if(pori->q_use) {
    x = pori->q1;
    y = pori->q2;
    z = pori->q3;
    w = pori->q0;
  } else if (pori->ai_use) {
    Eigen::Matrix3d matrix;
    matrix << pori->a, pori->b, pori->c,
              pori->d, pori->e, pori->f,
              pori->g, pori->h, pori->i;
    Eigen::Vector4d qtemp = rotationToQuaternion(matrix);
    x = qtemp[0];
    y = qtemp[1];
    z = qtemp[2];
    w = qtemp[3];
  } else if (pori->coord_use) {
    Eigen::Vector3d angle_xyz;
    angle_xyz[0] = ToRadian(pori->roll);
    angle_xyz[1] = ToRadian(pori->pitch);
    angle_xyz[2] = ToRadian(pori->yaw);
    Eigen::Vector4d qtemp = eulerToQuateration(angle_xyz);
    x = qtemp[0];
    y = qtemp[1];
    z = qtemp[2];
    w = qtemp[3];
  } else {
    return -2;
  }
  Quaternion Qtemp;
  Quaternion Qtemp1;
  Qtemp.w = w;
  Qtemp.x = x;
  Qtemp.y = y;
  Qtemp.z = z;
  Qtemp1 = ConvertQuat(Qtemp, pori->coord_format, param_.param.frame_local);
  w = Qtemp1.w;
  x = Qtemp1.x;
  y = Qtemp1.y;
  z = Qtemp1.z;
  imu_.orientation.x = x;
  imu_.orientation.y = y;
  imu_.orientation.z = z;
  imu_.orientation.w = w;
  imu_.orientation_covariance = {
    ToRadian(1.0), 0.0,           0.0,
    0.0,           ToRadian(1.0), 0.0,
    0.0,           0.0,           ToRadian(9.0)
  };
  if (pori->coord_use) {
    xsens_imu_data_.yaw = pori->yaw;
    xsens_imu_data_.pitch = pori->pitch;
    xsens_imu_data_.roll = pori->roll;
    bxsens_imu_data_pub_ = true;
  }
  return 0;
}

int XsensDriver::FillAcceleration() {
  Acceleration *pacc = &(xsens_->msg_.acceleration);
  if (!pacc->use)
    return -1;

  double accx, accy, accz;
  if(pacc->delta.use) {
    accx = pacc->delta.X;
    accy = pacc->delta.Y;
    accz = pacc->delta.Z;
  } else if (pacc->free_acc.use) {
    accx = pacc->free_acc.X;
    accy = pacc->free_acc.Y;
    accz = pacc->free_acc.Z;      
  } else if (pacc->acc.use) {
    accx = pacc->acc.X;
    accy = pacc->acc.Y;
    accz = pacc->acc.Z;
  } else {
    return -3;
  }
  ConvertCoords(accx, accy, accz, pacc->coord_format, param_.param.frame_local);
  imu_.linear_acceleration.x = accx;
  imu_.linear_acceleration.y = accy;
  imu_.linear_acceleration.z = accz;   
  imu_.linear_acceleration_covariance = {
    0.0004, 0.0, 0.0,
    0.0, 0.0004, 0.0,
    0.0, 0.0, 0.0004
  };

  if (pacc->acc.use) {
    xsens_imu_data_.accx = pacc->acc.X;
    xsens_imu_data_.accy = pacc->acc.Y;
    xsens_imu_data_.accz = pacc->acc.Z;
    bxsens_imu_data_pub_ = true;
  }
  return 0;
}

int XsensDriver::FillAngularVelocity() {
  AngularVelocity *pang = &(xsens_->msg_.angular_velocity);
  double velx, vely, velz;
  if (!pang->use)
    return -1;

  if(pang->delta.use) {
    Quaternion dq, Demp;
    Demp.w = pang->delta_add;
    Demp.x = pang->delta.X;
    Demp.y = pang->delta.Y;
    Demp.z = pang->delta.Z;
    dq = ConvertQuat(Demp, pang->coord_format, param_.param.frame_local);
    uint64_t now = middleware::Time::Now().ToNsec();
    static uint64_t last_delta_q_time = 0;
    if(last_delta_q_time == 0) {
      last_delta_q_time = now;
    } else {
      uint64_t delta_t = (now - last_delta_q_time);
      static double delta_q_rate = 0;
      if(delta_q_rate == 0) {
        delta_q_rate = 1.0 / (delta_t / 1e-9);
      }
      double delta_t_filtered = 0.95 / delta_q_rate + 0.05 * (delta_t / 1e-9);
      delta_q_rate = round(1.0 / delta_t_filtered);
      last_delta_q_time = now;
      double ca_2 = dq.w;
      double sa_2 = sqrt(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z);
      double ca = ca_2 * ca_2 - sa_2 * sa_2;
      double sa = 2 * ca_2 * sa_2;
      double rotation_angle = atan2(sa, ca);
      double rotation_speed = rotation_angle * delta_q_rate;
      double f = rotation_speed / sa_2;
      velx = f * dq.x;
      vely = f * dq.y;
      velz = f * dq.z;
    }
  } else if(pang->gyr.use) {
    velx = pang->gyr.X;
    vely = pang->gyr.Y;
    velz = pang->gyr.Z;
    ConvertCoords(velx, vely, velz, pang->coord_format, param_.param.frame_local);        
  } else {
    return -4;
  }
  bimu_pub_ = true;
  imu_.angular_velocity.x = velx;
  imu_.angular_velocity.y = vely;
  imu_.angular_velocity.z = velz;
  imu_.angular_velocity_covariance = {   
    ToRadian(0.025), 0.0, 0.0,
    0.0, ToRadian(0.025), 0.0,
    0.0, 0.0, ToRadian(0.025)
  };

  btwist_stamped_pub_ = true;
  twist_stamped_.twist.angular.x = velx;
  twist_stamped_.twist.angular.y = vely;
  twist_stamped_.twist.angular.z = velz;

  if (pang->gyr.use) {
    xsens_imu_data_.gyrx = pang->gyr.X;
    xsens_imu_data_.gyry = pang->gyr.Y;
    xsens_imu_data_.gyrz = pang->gyr.Z;
  }
  return 0;
}

int XsensDriver::FillVelocity() {
  Velocity *pvel = &(xsens_->msg_.velocity);
  if (!pvel->use)
    return -1;
  
  if (pvel->vel.use) {
    double x, y, z;
    x = pvel->vel.X;
    y = pvel->vel.Y;
    z = pvel->vel.Z;
    ConvertCoords(x, y, z, pvel->coord_format, param_.param.frame_local);
    btwist_stamped_pub_ = true;
    twist_stamped_.twist.linear.x = x;
    twist_stamped_.twist.linear.y = y;
    twist_stamped_.twist.linear.z = z;

    xsens_imu_data_.velx = x;
    xsens_imu_data_.vely = y;
    xsens_imu_data_.velz = z;
  }
  return 0;
}

int XsensDriver::FillMagnetic() {
  Magnetic *pmag = &(xsens_->msg_.magnetic);
  if (pmag->use && pmag->mag.use) {
    double x, y, z;
    x = pmag->mag.X;
    y = pmag->mag.Y;
    z = pmag->mag.Z;
    ConvertCoords(x, y, z, pmag->coord_format, param_.param.frame_local);        
    magnetic_field_.magnetic_field.x = x;
    magnetic_field_.magnetic_field.y = y;
    magnetic_field_.magnetic_field.z = z;
    int count = sizeof(magnetic_field_.magnetic_field_covariance) / sizeof(magnetic_field_.magnetic_field_covariance[0]);
    for (int i = 0; i < count; i++)
      magnetic_field_.magnetic_field_covariance[i] = 0x00;
    bmagnetic_field_pub_ = true;
  }
  return 0;
}

int XsensDriver::FillGNSS() {
  Gnss *pgnss = &(xsens_->msg_.gnss);
  if (!pgnss->use)
    return -1;
  
  if (pgnss->pvt_use) {
    bgnss_pvt_data_pub_ = true;
    gnss_pvt_data_.itow = pgnss->itow;
    gnss_pvt_data_.year = pgnss->year;
    gnss_pvt_data_.month = pgnss->month;
    gnss_pvt_data_.day = pgnss->day;
    gnss_pvt_data_.hour = pgnss->hour;
    gnss_pvt_data_.min = pgnss->min;
    gnss_pvt_data_.sec = pgnss->sec;
    gnss_pvt_data_.valid = pgnss->valid;
    gnss_pvt_data_.tacc = pgnss->tAcc;
    gnss_pvt_data_.fixtype = pgnss->fixtype;
    gnss_pvt_data_.flags = pgnss->flags;
    gnss_pvt_data_.numsv = pgnss->numSV;
    gnss_pvt_data_.latitude = pgnss->lat;
    gnss_pvt_data_.longitude = pgnss->lon;
    gnss_pvt_data_.height = pgnss->height;
    gnss_pvt_data_.hmsl = pgnss->hMSL;
    gnss_pvt_data_.hacc = pgnss->hAcc;
    gnss_pvt_data_.vacc = pgnss->vAcc;
    gnss_pvt_data_.veln = pgnss->velN;
    gnss_pvt_data_.vele = pgnss->velE;
    gnss_pvt_data_.veld = pgnss->velD;
    gnss_pvt_data_.gspeed = pgnss->gSpeed;
    gnss_pvt_data_.headmot = pgnss->headMot;
    gnss_pvt_data_.sacc = pgnss->sAcc;
    gnss_pvt_data_.headacc = pgnss->headAcc;
    gnss_pvt_data_.headveh = pgnss->headVeh;
    gnss_pvt_data_.gdop = pgnss->gdop;
    gnss_pvt_data_.pdop = pgnss->pdop;
    gnss_pvt_data_.tdop = pgnss->tdop;
    gnss_pvt_data_.vdop = pgnss->vdop;
    gnss_pvt_data_.ndop = pgnss->ndop;
    gnss_pvt_data_.edop = pgnss->edop;
  }
  return 0;
}

int XsensDriver::FillGPS() {
  // TODO
  return 0;
}

int XsensDriver::FillPosition() {
  Position *ppos = &(xsens_->msg_.position);
  if (!ppos->use)
    return -1;
  double alt = 0;
  if (ppos->alt_msl_use)
    alt = ppos->alt_msl;
  if (ppos->alt_ellipsoid_use)
    alt = ppos->alt_ellipsoid;

  if (ppos->lat_lon_use) {
    xsens_imu_data_.latitude = ppos->lat;
    xsens_imu_data_.longitude = ppos->lon;
    xsens_imu_data_.altitude = alt;
  }
  if (ppos->ecef.use) {
    // don't support
  }
  return 0;
}

int XsensDriver::ResetMessage() {
  bimu_pub_ = false;
  bimu_data_str_pub_ = false;
  bmagnetic_field_pub_ = false;
  btwist_stamped_pub_ = false;
  bxsens_imu_data_pub_ = false;
  bgnss_pvt_data_pub_ = false;
  return 0;
}

int XsensDriver::FillMessage() {
  FillHeader();
  FillOrientationData();
  FillAcceleration();
  FillAngularVelocity();
  FillVelocity();
  FillMagnetic();
  //FillGNSS();
  FillGPS();
  FillPosition();
  return 0;
}

/*****************************************************************************/
double XsensDriver::ToDegree(double radians) {
  return radians * 180.0 / M_PI;
}
double XsensDriver::ToRadian(double degree) {
  return degree / 180.0 * M_PI;
}

Quaternion XsensDriver::QMilt(Quaternion Q0, Quaternion Q1) {
  Quaternion Q;
  Q.w = Q0.w * Q1.w - Q0.x * Q1.x - Q0.y * Q1.y - Q0.z * Q1.z;
  Q.x = Q0.w * Q1.x + Q0.x * Q1.w + Q0.y * Q1.z - Q0.z * Q1.y;
  Q.y = Q0.w * Q1.y - Q0.x * Q1.z + Q0.y * Q1.w + Q0.z * Q1.x;
  Q.z = Q0.w * Q1.z + Q0.x * Q1.y - Q0.y * Q1.x + Q0.z * Q1.w;
  return Q;
}

Quaternion XsensDriver::ConvertQuat(Quaternion q, CoordFormat source, CoordFormat dest) {
  Quaternion q_enu_ned = {0, 1.0/sqrt(2), 1.0/sqrt(2), 0};
  Quaternion q_enu_nwu = {1.0/sqrt(2), 0, 0, -1.0/sqrt(2)};
  Quaternion q_ned_nwu = {0, -1, 0, 0};
  Quaternion q_ned_enu = {0, -1.0/sqrt(2), -1.0/sqrt(2), 0}; 
  Quaternion q_nwu_enu = {1.0/sqrt(2), 0, 0, 1.0/sqrt(2)};
  Quaternion q_nwu_ned = {0, 1, 0, 0};
  if(CoordFormat::ENU == source) {
    if(CoordFormat::ENU == dest)
      return q;
    else if(CoordFormat::NED == dest)
      return QMilt(q_enu_ned, q);
    else if(CoordFormat::NWU == dest)
      return QMilt(q_enu_nwu, q);
  } else if(CoordFormat::NED == source) {
    if(CoordFormat::ENU == dest)
      return QMilt(q_ned_enu,q);
    else if(CoordFormat::NED == dest)
      return q;
    else if(CoordFormat::NWU == dest)
      return QMilt(q_ned_nwu, q);
  } else if(CoordFormat::NWU == source) {
    if(CoordFormat::ENU == dest)
      return QMilt(q_nwu_enu,q);
    else if(CoordFormat::NED == dest)
      return QMilt(q_nwu_ned, q);
    else if(CoordFormat::NWU == dest)
      return q;    
  }
}

/*
 * Convert the coordinates between ENU, NED, and NWU.
*/
void XsensDriver::ConvertCoords(double &x, double &y, double &z, CoordFormat source, CoordFormat dest) {
  double x1 = x, y1 = y, z1 = z;
  if (CoordFormat::NED == source) {
    x = y1; y = x1; z = -z1;
  } else if (CoordFormat::NWU == source) {
    x = -y1; y = x; z = z1;
  }
  if (CoordFormat::NED == dest) {
    x = y1; y = x1; z = -z1;
  } else if (CoordFormat::NWU == dest) {
    x = y1; y = -x1; z = z1;
  }
}

void XsensDriver::ConvertFrameLocal(std::string sformat) {
  if ("ENU" == sformat)
    param_.param.frame_local = CoordFormat::ENU;
  else if ("NED" == sformat)
    param_.param.frame_local = CoordFormat::NED;
  else if ("NWU" == sformat)
    param_.param.frame_local = CoordFormat::NWU;
  else
    MW_ERROR("{} error", sformat.c_str());
}

void XsensDriver::GetParam() {
  param_.param.stream_type = middleware::GetParam("~stream_type", "serial");
  param_.param.dev_name = middleware::GetParam("~dev_name", "/dev/oc_xsens");
  param_.param.baudrate = std::atoi(middleware::GetParam("~baudrate", "921600").data());
  param_.param.timeout = std::atof(middleware::GetParam("~timeout", "0.002").data());
  param_.param.serial = "xsens";
  ConvertFrameLocal(middleware::GetParam("~frame_local", "ENU"));
  param_.no_rotation_duration = std::atoi(middleware::GetParam("~no_rotation_duration", "0").data());
  param_.frame_id_ = middleware::GetParam("~frame_id", "/base_imu");
  param_.ns = middleware::GetParam("~ns", "xsens");
}

void XsensDriver::StreamMonitorOption(middleware::DataStreamMonitorOption &opt, int topic_hz, bool monitor) {
  if (monitor) {
    opt.enable_monitoring = true; 
    opt.warning_lower_bound = topic_hz * 0.9;
    opt.error_lower_bound = topic_hz * 0.8;
    opt.warning_upper_bound = topic_hz * 1.1;
    opt.error_upper_bound = topic_hz * 1.2;
  } else {
    opt.enable_monitoring = true;
    opt.allow_timeout = true;
  }
}

void XsensDriver::ConstructPublishers() {
  int topic_hz = 200; // test
  middleware::DataStreamMonitorOption opt_monitor;
  middleware::DataStreamMonitorOption opt_no_monitor;
  //StreamMonitorOption(opt_no_monitor, 0, false);
  StreamMonitorOption(opt_monitor, 100, true);
  std::string msg_name = "/" + param_.ns + "/imu/mag";
  pub_magnetic_field_ = std::make_shared<middleware::Publisher<sensor_msgs::msg::MagneticField>>(msg_name, opt_no_monitor);
  StreamMonitorOption(opt_monitor, 200, true);
  msg_name = "/" + param_.ns + "/imupos";
  pub_xsens_imu_data_ = std::make_shared<middleware::Publisher<xsens_msgs::msg::IMUDATA>>(msg_name, opt_no_monitor);
  msg_name = "/" + param_.ns + "/imu/data";
  pub_imu_ = std::make_shared<middleware::Publisher<sensor_msgs::msg::Imu>>(msg_name, opt_no_monitor);
  //msg_name = "/" + param_.ns + "/imu_data_str";
  //pub_imu_data_str_ = std::make_shared<middleware::Publisher<std_msgs::msg::String>>(msg_name, opt_no_monitor);
  msg_name = "/" + param_.ns + "/velocity";
  pub_twist_stamped_ = std::make_shared<middleware::Publisher<geometry_msgs::msg::TwistStamped>>(msg_name, opt_no_monitor);
  //msg_name = "/" + param_.ns + "/rawgnss";
  //pub_gnss_pvt_data_ = std::make_shared<middleware::Publisher<xsens_msgs::msg::GNSSPVTDATA>>(msg_name, opt_no_monitor);
}

bool XsensDriver::ConnectDevice() {
  connect_status_ = false;
  while (!xsens_->Connect()) {
    if(!run_status_) 
      return false;
  }
  connect_status_ = true;
  return true;
}

bool XsensDriver::Init() {
  GetParam();
  MW_INFO("Xsens Init...");
  xsens_.reset();
  xsens_ = std::make_shared<XsensDevice>(param_.param);
  if(!ConnectDevice())
    return false;
  ConstructPublishers();
  if (param_.no_rotation_duration)
    xsens_->SetNoRotation(param_.no_rotation_duration);
  task_xsens_ = std::thread(std::bind(&XsensDriver::TaskXsens, this));
  // add a Monitor
  std::vector<std::function<void(middleware::DiagnosticStatus&)>> tasks;
  xsensCustomMonitorPtr_ = std::make_shared<middleware::CustomMonitor>(std::string("xsens_custom_monitor"), tasks);
  return true;
}

void XsensDriver::Monitor(void) {
  if(!run_status_) {
    return;
  }

}

int XsensDriver::XsensDataProcAndPost() {
  
  ResetMessage();
  FillMessage();
  if (bimu_pub_) {
    imu_.header = header_;
    pub_imu_->Publish(imu_);
  }
  //if (bimu_data_str_pub_) {
  //  pub_imu_data_str_->Publish(imu_data_str_);
  //}
  if (bmagnetic_field_pub_) {
    magnetic_field_.header = header_;
    pub_magnetic_field_->Publish(magnetic_field_);
  }
  if (btwist_stamped_pub_) {
    twist_stamped_.header = header_;
    pub_twist_stamped_->Publish(twist_stamped_);
  }
  if (bxsens_imu_data_pub_) {
    xsens_imu_data_.header = header_;
    pub_xsens_imu_data_->Publish(xsens_imu_data_);
  }
  //if (bgnss_pvt_data_pub_) {
  //  gnss_pvt_data_.header = header_;
  //  pub_gnss_pvt_data_->Publish(gnss_pvt_data_);
  //}

  return 0;
}


void XsensDriver::TaskXsens() {
  int ret = -1;
  MW_INFO("Thread {} is running...", __func__);

  while(run_status_) {
    ret = xsens_->GetMsg();
    if (ret < 0) {
      MW_INFO("Get Xsens message error {}", ret);
      continue;
    }
    xsens_->HandleMsg();
    Monitor();
    XsensDataProcAndPost();
  } // while

  MW_INFO("{} Exit!", __func__);
}

void XsensDriver::Run() {
  
}

void XsensDriver::CleanUp() {
  if(connect_status_) {
    
  }
}

void XsensDriver::Stop() {
  run_status_ = false;
  if (task_xsens_.joinable()) {
    task_xsens_.join();
  }
  CleanUp();
  xsens_->StopStream();
  xsens_->Disconnect();
  xsens_.reset();
}

} // namespace xsens
} // namespace device
} // namespace octopus
