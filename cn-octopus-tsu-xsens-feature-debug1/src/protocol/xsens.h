#pragma once

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <mutex>
#include <memory>
#include <iostream>

#include "stream.h"
#include "middleware/middleware.h"

namespace octopus {
namespace device {
namespace xsens {

#define htonll(x)    (((long long int)(x) & 0x00000000000000ff) << 7*8) | \
      							 (((long long int)(x) & 0x000000000000ff00) << 5*8) | \
      							 (((long long int)(x) & 0x0000000000ff0000) << 3*8) | \
      							 (((long long int)(x) & 0x00000000ff000000) << 1*8) | \
      							 (((long long int)(x) & 0x000000ff00000000) >> 1*8) | \
      							 (((long long int)(x) & 0x0000ff0000000000) >> 3*8) | \
      							 (((long long int)(x) & 0x00ff000000000000) >> 5*8) | \
      							 (((long long int)(x) & 0xff00000000000000) >> 7*8)

#define ntohll(x)    (((long long int)(x) & 0x00000000000000ff) << 7*8) | \
      							 (((long long int)(x) & 0x000000000000ff00) << 5*8) | \
      							 (((long long int)(x) & 0x0000000000ff0000) << 3*8) | \
      							 (((long long int)(x) & 0x00000000ff000000) << 1*8) | \
      							 (((long long int)(x) & 0x000000ff00000000) >> 1*8) | \
      							 (((long long int)(x) & 0x0000ff0000000000) >> 3*8) | \
      							 (((long long int)(x) & 0x00ff000000000000) >> 5*8) | \
      							 (((long long int)(x) & 0xff00000000000000) >> 7*8)

#define DATAID_GROUP_MASK       0xF800
#define DATAID_TYPE_MASK        0xF0
#define DATAID_COODR_MASK       0xC0
#define DATAID_FORMAT_MASK      0x03
#define SATEMAX                 32

const uint32_t XBUS_RETRY_TIMES = 512;
const uint32_t DATAID_OFFSET = 3;

enum ByteOrder {
   BigEndian = 0x00
  ,LittleEndian
};

enum IDFormat {
   IDFloat32 = 0x00
  ,IDFp1220
  ,IDFp1632
  ,IDFloat64
};

enum CoordFormat {
  // East-North-Up coordinate system
   ENU = 0x00
  // North-East-Down coordinate system
  ,NED = 0x04
  // North-West-Up
  ,NWU = 0x08
  // only for application, not protocol
  ,Local = 0x0B
  ,UNKOWN = 0x0F
};

enum MID {
    //"""Values for the message id (MID)"""
    // Error message, 1 data byte
     Error = 0x42

    // State MID
    // Wake up procedure
    ,WakeUp = 0x3E
    // Wake up ack to put device in config mode
    ,WakeUpAck = 0x3F
    // Switch to config state
    ,GoToConfig = 0x30
    // Switch to measurement state
    ,GoToMeasurement = 0x10
    // Reset device
    ,Reset = 0x40

    // Informational messages
    // Request device id
    ,ReqDID = 0x00
    // DeviceID, 4 bytes: HH HL LH LL
    ,DeviceID = 0x01
    // Request product code in plain text
    ,ReqProductCode = 0x1C
    // Product code (max 20 bytes data)
    ,ProductCode = 0x1D
    // Request firmware revision
    ,ReqFWRev = 0x12
    // Firmware revision, 3 bytes: major minor rev
    ,FirmwareRev = 0x13

    // Device specific messages
    // Restore factory defaults
    ,RestoreFactoryDef = 0x0E
    // Baudrate, 1 byte
    ,SetBaudrate = 0x18
    // Run the built-in self test (MTi-1/10/100 series)
    ,RunSelftest = 0x24
    // Self test results, 2 bytes
    ,SelftestAck = 0x25
    // Error mode, 2 bytes, 0000, 0001, 0002, 0003 (default 0001)
    ,SetErrorMode = 0xDA
    // Transmit delay (RS485), 2 bytes, number of clock ticks (1/29.4912 MHz)
    ,SetTransmitDelay = 0xDC
    // Set state of OptionFlags (MTi-1/2/3), 4 + 4 bytes
    ,SetOptionFlags = 0x48
    // Location ID, 2 bytes, arbitrary, default is 0
    ,SetLocationID = 0x84

    // Synchronization messages
    // Synchronization settings (MTi-1/10/100 series only), N*12 bytes
    ,SetSyncSettings = 0x2C

    // Configuration messages
    // Request configuration
    ,ReqConfiguration = 0x0C
    // Configuration, 118 bytes
    ,Configuration = 0x0D
    // Sampling period (MTi/MTi-G only), 2 bytes
    ,SetPeriod = 0x04
    // Extended output mode (MTi-10/100), 2 bytes, bit 4 for extended UART
    ,SetExtOutputMode = 0x86
    // Output configuration (MTi-1/10/100 series only), N*4 bytes
    ,SetOutputConfiguration = 0xC0
    // Configure NMEA data output (MTi-10/100), 2 bytes
    ,SetStringOutputType = 0x8E
    // Set sensor of local alignment quaternion
    ,SetAlignmentRotation = 0xEC
    // Output mode (MTi/MTi-G only), 2 bytes
    ,SetOutputMode = 0xD0
    // Output settings (MTi/MTi-G only), 4 bytes
    ,SetOutputSettings = 0xD2

    // Data messages
    // Request MTData message (for 65535 skip factor)
    ,ReqData = 0x34
    // Legacy data packet
    ,MTData = 0x32
    // Newer data packet (MTi-10/100 series only)
    ,MTData2 = 0x36

    // Filter messages
    // Reset orientation, 2 bytes
    ,ResetOrientation = 0xA4
    // Request or set UTC time from sensor (MTI-G and MTi-10/100 series)
    ,SetUTCTime = 0x60
    // Set correction ticks to UTC time
    ,AdjustUTCTime = 0xA8
    // UTC Time (MTI-G and MTi-10/100 series), 12 bytes
    ,UTCTime = 0x61
    // Request the available XKF scenarios on the device
    ,ReqAvailableScenarios = 0x62
    // Available Scenarios
    ,AvailableScenarios = 0x63
    // Current XKF scenario, 2 bytes
    ,SetCurrentScenario = 0x64
    // Magnitude of the gravity used for the sensor fusion mechanism, 4 bytes
    ,SetGravityMagnitude = 0x66
    // Latitude, Longitude and Altitude for local declination and gravity
    // (MTi-10/100 series only), 24 bytes
    ,SetLatLonAlt = 0x6E
    // Initiate No Rotation procedure (not on MTi-G), 2 bytes
    ,SetNoRotations = 0x22
};

enum XDIGroup {
    //"""Values for the XDI groups."""
     Temperature = 0x0800
    ,Timestamp = 0x1000
    ,OrientationDatas = 0x2000
    ,Pressure = 0x3000
    ,Accelerations = 0x4000
    ,Positions = 0x5000
    ,GNSSs = 0x7000
    ,AngularVelocitys = 0x8000
    ,GPS = 0x8800
    ,SensorComponentReadout = 0xA000
    ,AnalogIn = 0xB000  //# deprecated
    ,Magnetics = 0xC000
    ,Velocitys = 0xD000
    ,Status = 0xE000
};

#pragma pack(1)

struct XbusHeader {
  uint8_t preamble;
  uint8_t bid;
  uint8_t mid;
  uint16_t lenght;
};

struct XDIDataHeader {
  uint16_t id;
  uint8_t lenght;
};

struct XDIemperature {


};

struct XDIUTCTime {
  uint32_t ns;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t flags;
};

/******************* used for driver *******************/

struct SateInfo {
  uint8_t gnssId;
  uint8_t svId;
  uint8_t con;
  uint8_t flags;
};

struct GenCoord {
  bool use;
  double X;
  double Y;
  double Z;
};

struct Magnetic {
  bool use;
  CoordFormat coord_format;
  GenCoord mag;
};

struct Acceleration {
  bool use;
  CoordFormat coord_format;
  GenCoord acc;
  GenCoord free_acc;
  GenCoord delta;
};

struct AngularVelocity {
  bool use;
  CoordFormat coord_format;
  GenCoord gyr;
  GenCoord delta;
  double delta_add;
};

struct Velocity {
  bool use;
  CoordFormat coord_format;
  GenCoord vel;
};

struct OrientationData {
  bool use;
  CoordFormat coord_format;
  bool q_use;
  double q0;
  double q1;
  double q2;
  double q3;
  bool ai_use;
  double a;
  double b;
  double c;
  double d;
  double e;
  double f;
  double g;
  double h;
  double i;
  bool coord_use;
  double roll;
  double pitch;
  double yaw;
};

struct Gnss {
  bool use;
  bool pvt_use;
  uint32_t itow;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;

  uint32_t tAcc;
  int32_t nano;

  uint8_t fixtype;
  uint8_t flags;
  uint8_t numSV;
  uint8_t Reserved1;

  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hMSL;
  uint32_t hAcc;
  uint32_t vAcc;

  uint32_t velN;
  int32_t velE;
  int32_t velD;

  int32_t gSpeed;
  int32_t headMot;

  uint32_t sAcc;
  uint32_t headAcc;
  int32_t headVeh;

  uint16_t gdop;
  uint16_t pdop;
  uint16_t tdop;
  uint16_t vdop;
  uint16_t hdop;
  uint16_t ndop;
  uint16_t edop;

  bool sta_use;
  uint32_t iTow;
  uint8_t numSvs;
  SateInfo sate[SATEMAX];
};

struct Position {
  bool use;
  CoordFormat coord_format;
  GenCoord ecef;
  bool alt_msl_use;
  double alt_msl;
  bool alt_ellipsoid_use;
  double alt_ellipsoid;
  bool lat_lon_use;
  double lat;
  double lon;
};

struct UTime {
  bool use;
  bool utc_time_use;
  XDIUTCTime utc_time;
  bool packet_counter_use;
  uint16_t packet_counter;
  bool time_of_week_use;
  uint32_t time_of_week;
  bool gps_age_use;
  uint8_t gps_age;
  bool pressure_age_use;
  uint8_t pressure_age;
  bool sample_time_fine_use;
  uint32_t sample_time_fine;
  bool sample_time_coarse_use;
  uint32_t sample_time_coarse;
  bool frame_use;
  uint16_t start_frame;
  uint16_t end_frame;
};

#pragma pack()

struct ParamXsens {
  std::string stream_type;
  std::string dev_name;
  int baudrate;
  CoordFormat frame_local;
  std::string serial;
  float timeout;
};

struct XsensMsg {
  bool use;
  XbusHeader header;
  Gnss gnss;
  UTime time;
  Velocity velocity;
  Position position;
  Magnetic magnetic;
  Acceleration acceleration;
  OrientationData orientation_data;
  AngularVelocity angular_velocity;

  // data
  uint8_t buffer[1024];
};

class XsensDevice {
  public:
    XsensDevice(ParamXsens &param);
    ~XsensDevice() {};

    /*
     * device control
    */
    int Connect();
    int Disconnect();
    void StopStream();

    /*
     * message parse
    */
    void MsgPrint(XsensMsg *pmsg);
    int GetMsg(int lenght = 0);
    void HandleMsg();
    /*
     * xsens control
    */
    int SetNoRotation(int duration);

    private:
    /*
      * parse data message
    */
    void ParseTemperature(uint16_t data_id, uint8_t *content, IDFormat format);
    void ParseTimestamp(uint16_t data_id, uint8_t *content, IDFormat format);
    void ParseOrientationData(uint16_t data_id, uint8_t *content, IDFormat format);
    void ParsePressure(uint16_t data_id, uint8_t *content, IDFormat format);
    void ParseAcceleration(uint16_t data_id, uint8_t *content, IDFormat format);
    void ParsePosition(uint16_t data_id, uint8_t *content, IDFormat format);
    void ParseGNSS(uint16_t data_id, uint8_t *content, IDFormat format);
    void ParseAngularVelocity(uint16_t data_id, uint8_t *content, IDFormat format);
    void ParseGPS(uint16_t data_id, uint8_t *content, IDFormat format);
    void ParseAnalogIn(uint16_t data_id, uint8_t *content, IDFormat format);
    void ParseMagnetic(uint16_t data_id, uint8_t *content, IDFormat format);
    void ParseVelocity(uint16_t data_id, uint8_t *content, IDFormat format);
    void ParseStatus(uint16_t data_id, uint8_t *content, IDFormat format);
    /*
      * tools
    */
    void MemcpyUint64(uint64_t src, double &dst);
    void MemcpyUint32(uint32_t src, double &dst);
    void MemcpyUint32(uint32_t src, float &dst);
    uint8_t ConvertUint8(uint8_t *psrc, ByteOrder endian = ByteOrder::BigEndian);
    uint16_t ConvertUint16(uint8_t *psrc, ByteOrder endian = ByteOrder::BigEndian);
    uint32_t ConvertUint32(uint8_t *psrc, ByteOrder endian = ByteOrder::BigEndian);
    uint64_t ConvertUint64(uint8_t *psrc, ByteOrder endian = ByteOrder::BigEndian);
    float ConvertFloat(uint8_t *psrc, ByteOrder endian = ByteOrder::BigEndian);
    double ConvertDouble(uint8_t *psrc, ByteOrder endian = ByteOrder::BigEndian);
    double ConvertAuto(uint8_t *psrc, IDFormat format, ByteOrder endian = ByteOrder::BigEndian);

  private:
    ParamXsens param_;
    Stream* stream_;
  
  public:
    /*
     * parse message for user
    */
    XsensMsg msg_;
    bool debug_ = true;
};

}  // namespace xsens
}  // namespace device
}  // namespace octopus
