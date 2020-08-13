#include "xsens.h"
#include "sensor_common_api.h"
#include "sensor_data_defs.h"
#include "middle_sensor_interface.h"

namespace octopus {
namespace device {
namespace xsens {

XsensDevice::XsensDevice(ParamXsens &param):param_(param) {
  memset(&msg_, 0, sizeof(msg_));
  stream_ = nullptr;
}

int XsensDevice::Connect() {
  if (stream_ != nullptr) {
    stream_->Disconnect();
    delete stream_;
    stream_ = nullptr;
  }
  if (param_.stream_type == "serial") {
    MW_INFO("{} {}@{}", param_.stream_type.c_str(), param_.dev_name.c_str(), param_.baudrate);
    stream_ = Stream::Serial(param_.dev_name, param_.baudrate);
  } else {
    MW_INFO("Connect failed! stream_type [{}] is not supported!", param_.stream_type);
    exit(1);
  }

  return stream_->Connect();
}

int XsensDevice::Disconnect() {
    if (stream_ != nullptr) {
      stream_->Disconnect();
      delete stream_;
      stream_ = nullptr;
    }
  return 0;
}

void XsensDevice::StopStream() {
  stream_->exit_flag_ = true;
}

/********************** Xsens Control ****************************/

int XsensDevice::SetNoRotation(int duration) {
  // TODO
  return 0;
}


/**********************Debug ****************************/
void XsensDevice::MsgPrint(XsensMsg *pmsg) {
  printf("%s ..\n", __func__);
  printf(" %02x %02x %02x %x(%d)", pmsg->header.preamble, pmsg->header.bid, pmsg->header.mid, pmsg->header.lenght, pmsg->header.lenght);
  for (int i = 0; i < pmsg->header.lenght + 1; i++) {
      if (0 == i%16)
        printf("\n");
      printf(" %02x", pmsg->buffer[i]);
    }
    printf("\n");
}

/********************** Get message ****************************/

int XsensDevice::GetMsg(int lenght) {
  uint32_t cnt = 0;
  uint8_t temp = 0;
  msg_.use = 0;
  memset(&msg_, 0, sizeof(msg_));
  // After chenge to timeout TODO
  while (cnt++ < XBUS_RETRY_TIMES) {
    // preamble
    if(1 != stream_->Read(&(msg_.header.preamble), 1, 1))
      return -2;
    if (0xFA != msg_.header.preamble) continue;
    // bid
    if(1 != stream_->Read(&(msg_.header.bid), 1, 1))
      return -3;
    if (0xFF != msg_.header.bid) continue;
    // mid
    if(1 != stream_->Read(&msg_.header.mid, 1, 1))
      return -4;
    // lenght
    if(1 != stream_->Read(&temp, 1, 1))
      return -5;
    if (0xFF == temp) {
      if(2 != stream_->Read((uint8_t *)&msg_.header.lenght, 2, 2))
        return -6;
        msg_.header.lenght = ntohs(msg_.header.lenght);
    } else {
      msg_.header.lenght = temp;
    }
    if (0 == msg_.header.lenght)
      MW_WARN("msg_.header.lenght: {}\n", msg_.header.lenght);
    // Get data and checksum
    if ((msg_.header.lenght + 1) != stream_->Read(msg_.buffer, msg_.header.lenght + 1, sizeof(msg_.buffer)))
      return -7;
    if (debug_)
      MsgPrint(&msg_);
    // Calc checksum
    uint32_t checksum = 0xFF + msg_.header.mid + (msg_.header.lenght > 0xFF ? (msg_.header.lenght + 0xFF) : msg_.header.lenght);
    for (int i = 0; i < msg_.header.lenght + 1; i++)
      checksum += msg_.buffer[i];
    if (checksum & 0xFF) {
      MsgPrint(&msg_);
      return -8;
    }
    return msg_.header.lenght;
  }
  return -1;
}

/********************** Parse data ****************************/
/* use msg
 * ParseTimestamp 10
 * ParseOrientationData 30
 * ParseAcceleration 20
 * ParseAngularVelocity 20
 * ParseMagnetic 20
 * ParsePosition 40 20
 * ParseVelocity 10
 * ParseGNSS 20
*/
void XsensDevice::HandleMsg() {
  if (msg_.header.lenght <= 0)
    return;
  if (MID::MTData2 != msg_.header.mid)
    return;

  uint16_t data_id;
  IDFormat format;
  XDIDataHeader *pheader;
  uint8_t *p = msg_.buffer;
  do {
    pheader = reinterpret_cast<XDIDataHeader *>(p);
    data_id = ntohs(pheader->id); // for big end convert
    if (IDFormat::IDFloat32 == (DATAID_FORMAT_MASK & data_id)) {
      format = IDFormat::IDFloat32;
    } else if (IDFormat::IDFloat64 == (DATAID_FORMAT_MASK & data_id)) {
      format = IDFormat::IDFloat64;
    } else {
      MW_INFO("fixed point precision not supported.");
      return;
    }
    if (debug_) {
      printf("header:%02x->%02x len:%d[%x], {%x} format:%02x\n", pheader->id, data_id, pheader->lenght, pheader->lenght, data_id & 0xF800, format);
      for (int i = 0; i < pheader->lenght; i++) {
        if (0 == i%16)
          printf("\n");
        printf(" %02x", p[i + DATAID_OFFSET]);
      }
      printf("\n");
    }
    switch(data_id & DATAID_GROUP_MASK) {
      case XDIGroup::Temperature:
        ParseTemperature(data_id, p + DATAID_OFFSET, format);
        break;
      case XDIGroup::Timestamp:
        ParseTimestamp(data_id, p + DATAID_OFFSET, format);
        break;
      case XDIGroup::OrientationDatas:
        ParseOrientationData(data_id, p + DATAID_OFFSET, format);
        break;
       case XDIGroup::Pressure:
        ParsePressure(data_id, p + DATAID_OFFSET, format);
        break;
      case XDIGroup::Accelerations:
        ParseAcceleration(data_id, p + DATAID_OFFSET, format);
        break;
      case XDIGroup::Positions:
        ParsePosition(data_id, p + DATAID_OFFSET, format);
        break;
       case XDIGroup::GNSSs:
        ParseGNSS(data_id, p + DATAID_OFFSET, format);
        break;
      case XDIGroup::AngularVelocitys:
        ParseAngularVelocity(data_id, p + DATAID_OFFSET, format);
        break;
      case XDIGroup::GPS:
        ParseGPS(data_id, p + DATAID_OFFSET, format);
        break;
       case XDIGroup::AnalogIn:
        ParseAnalogIn(data_id, p + DATAID_OFFSET, format);
        break;
      case XDIGroup::Magnetics:
        ParseMagnetic(data_id, p + DATAID_OFFSET, format);
        break;
      case XDIGroup::Velocitys:
        ParseVelocity(data_id, p + DATAID_OFFSET, format);
        break;
      case XDIGroup::Status:
        ParseStatus(data_id, p + DATAID_OFFSET, format);
        break;
      default:
        MW_INFO("unknown XDI group: 0x{:x}", pheader->id & DATAID_GROUP_MASK);
        break;
    }
    p += (pheader->lenght + DATAID_OFFSET);
  } while (p < (msg_.buffer + msg_.header.lenght));
  msg_.use = true;
}

/********************** Tools ****************************/

void XsensDevice::MemcpyUint64(uint64_t src, double &dst) {
  memcpy(&dst, &src, sizeof(uint64_t));
}

void XsensDevice::MemcpyUint32(uint32_t src, double &dst) {
  float ftemp;
  memcpy(&ftemp, &src, sizeof(uint32_t));
  dst = ftemp;
}

void XsensDevice::MemcpyUint32(uint32_t src, float &dst) {
  memcpy(&dst, &src, sizeof(uint32_t));
}

uint8_t XsensDevice::ConvertUint8(uint8_t *psrc, ByteOrder endian) {
  return psrc[0];
}

uint16_t XsensDevice::ConvertUint16(uint8_t *psrc, ByteOrder endian) {
  uint16_t temp;
  memcpy(&temp, psrc, sizeof(uint16_t));
  if (ByteOrder::BigEndian == endian)
    temp = ntohs(temp);
  return temp;
}

uint32_t XsensDevice::ConvertUint32(uint8_t *psrc, ByteOrder endian) {
  uint32_t temp;
  memcpy(&temp, psrc, sizeof(uint32_t));
  if (ByteOrder::BigEndian == endian)
    temp = ntohl(temp);
  return temp;
}

uint64_t XsensDevice::ConvertUint64(uint8_t *psrc, ByteOrder endian) {
  uint64_t temp;
  memcpy(&temp, psrc, sizeof(uint64_t));
  if (ByteOrder::BigEndian == endian)
    temp = ntohll(temp);
  return temp;
}

float XsensDevice::ConvertFloat(uint8_t *psrc, ByteOrder endian) {
  uint32_t temp;
  float ftemp;
  memcpy(&temp, psrc, sizeof(uint32_t));
  if (ByteOrder::BigEndian == endian)
    temp = ntohl(temp);
  memcpy(&ftemp, &temp, sizeof(uint32_t));
  return ftemp;
}

double XsensDevice::ConvertDouble(uint8_t *psrc, ByteOrder endian) {
  uint64_t temp;
  double dtemp;
  memcpy(&temp, psrc, sizeof(uint64_t));
  if (ByteOrder::BigEndian == endian)
    temp = ntohll(temp);
  memcpy(&dtemp, &temp, sizeof(uint64_t));
  return dtemp;
}

double XsensDevice::ConvertAuto(uint8_t *psrc, IDFormat format, ByteOrder endian) {
  if (IDFormat::IDFloat32 == format)
    return ConvertFloat(psrc, endian);
  else if (IDFormat::IDFloat64 == format)
    return ConvertDouble(psrc, endian);
  else
    MW_INFO("don't support {:x}", format);
}

/********************** Parse Data Packet ****************************/

void XsensDevice::ParseTemperature(uint16_t data_id, uint8_t *content, IDFormat format) {
  if (debug_)
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

void XsensDevice::ParseTimestamp(uint16_t data_id, uint8_t *content, IDFormat format) {
  uint8_t *p = content;
  UTime *ptime = &(msg_.time);
  ptime->use = true;
  switch (data_id & DATAID_TYPE_MASK) {
    case 0x10: // UTC Time, now use
      msg_.time.utc_time_use = true;
      ptime->utc_time.ns = ConvertUint32(p); p += sizeof(uint32_t);
      ptime->utc_time.year = ConvertUint16(p); p += sizeof(uint16_t);
      ptime->utc_time.month = ConvertUint8(p); p += sizeof(uint8_t);
      ptime->utc_time.day = ConvertUint8(p); p += sizeof(uint8_t);
      ptime->utc_time.hour = ConvertUint8(p); p += sizeof(uint8_t);
      ptime->utc_time.minute = ConvertUint8(p); p += sizeof(uint8_t);
      ptime->utc_time.second = ConvertUint8(p); p += sizeof(uint8_t);
      ptime->utc_time.flags = ConvertUint8(p); p += sizeof(uint8_t);
      break;
    case 0x20: // Packet Counter
      ptime->packet_counter_use = true;
      ptime->packet_counter = ConvertUint16(p); p += sizeof(uint16_t);
      break;
    case 0x30: // Integer Time of Week
      ptime->time_of_week_use = true;
      ptime->time_of_week = ConvertUint32(p); p += sizeof(uint32_t);
      break;
    case 0x40: // GPS Age  # deprecated
      ptime->gps_age_use = true;
      ptime->gps_age = ConvertUint8(p); p += sizeof(uint8_t);
      break;
    case 0x50: // Pressure Age  # deprecated
      ptime->pressure_age_use = true;
      ptime->pressure_age = ConvertUint8(p); p += sizeof(uint8_t);
      break;
    case 0x60: // Sample Time Fine
      ptime->sample_time_fine_use = true;
      ptime->sample_time_fine = ConvertUint32(p); p += sizeof(uint32_t);
      break;
    case 0x70: // Sample Time Coarse
      ptime->sample_time_coarse = true;
      ptime->sample_time_coarse = ConvertUint32(p); p += sizeof(uint32_t);
      break;
    case 0x80: // Frame Range
      ptime->frame_use = true;
      ptime->start_frame = ConvertUint16(p); p += sizeof(uint16_t);
      ptime->end_frame = ConvertUint16(p); p += sizeof(uint16_t);
      break;
    default:
      MW_INFO("unknown packet: {:x}.", data_id);
      break;
  }
  if (debug_)
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

void XsensDevice::ParseOrientationData(uint16_t data_id, uint8_t *content, IDFormat format) {
  uint8_t *p = content;
  OrientationData *pori = &(msg_.orientation_data);
  int offset = (IDFormat::IDFloat32 == format ? sizeof(float) : sizeof(double));
  if (CoordFormat::ENU == (data_id & DATAID_COODR_MASK))
    pori->coord_format = CoordFormat::ENU;
  else if (CoordFormat::NED == (data_id & DATAID_COODR_MASK))
    pori->coord_format = CoordFormat::NED;
  else if (CoordFormat::NWU == (data_id & DATAID_COODR_MASK))
    pori->coord_format = CoordFormat::NWU;
  
  pori->use = true;
  switch (data_id & DATAID_TYPE_MASK) {
    case 0x10: // Quaternion
      pori->q_use = true;
      pori->q0 = ConvertAuto(p, format); p += offset;
      pori->q1 = ConvertAuto(p, format); p += offset;
      pori->q2 = ConvertAuto(p, format); p += offset;
      pori->q3 = ConvertAuto(p, format); p += offset;
      break;
    case 0x20: // Rotation Matrix
      pori->ai_use = true;
      pori->a = ConvertAuto(p, format); p += offset;
      pori->b = ConvertAuto(p, format); p += offset;
      pori->c = ConvertAuto(p, format); p += offset;
      pori->d = ConvertAuto(p, format); p += offset;
      pori->e = ConvertAuto(p, format); p += offset;
      pori->f = ConvertAuto(p, format); p += offset;
      pori->g = ConvertAuto(p, format); p += offset;
      pori->h = ConvertAuto(p, format); p += offset;
      pori->i = ConvertAuto(p, format); p += offset;
      break;
    case 0x30: // Euler Angles, now use
      pori->coord_use = true;
      pori->roll = ConvertAuto(p, format); p += offset;
      pori->pitch = ConvertAuto(p, format); p += offset;
      pori->yaw = ConvertAuto(p, format); p += offset;
      break;
    default:
      MW_INFO("unknown packet: {:x}.", data_id);
      break;
  }
  if (debug_)
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

void XsensDevice::ParsePressure(uint16_t data_id, uint8_t *content, IDFormat format) {
  if (debug_)
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

void XsensDevice::ParseAcceleration(uint16_t data_id, uint8_t *content, IDFormat format) {
  GenCoord *pcoord = NULL;
  uint8_t *p = content;
  Acceleration *pacc = &(msg_.acceleration);
  int offset = (IDFormat::IDFloat32 == format ? sizeof(float) : sizeof(double));
  if (CoordFormat::ENU == (data_id & DATAID_COODR_MASK))
    pacc->coord_format = CoordFormat::ENU;
  else if (CoordFormat::NED == (data_id & DATAID_COODR_MASK))
    pacc->coord_format = CoordFormat::NED;
  else if (CoordFormat::NWU == (data_id & DATAID_COODR_MASK))
    pacc->coord_format = CoordFormat::NWU;
  
  pacc->use = true;
  switch (data_id & DATAID_TYPE_MASK) {
    case 0x10: // Delta V
      pacc->delta.use = true;
      pcoord = &(pacc->delta);
      break;
    case 0x20: // Acceleration, now use
    case 0x40: // AccelerationHR
      pacc->acc.use = true;
      pcoord = &(pacc->acc);
      break;
    case 0x30: // Free Acceleration
      pacc->free_acc.use = true;
      pcoord = &(pacc->free_acc);
      break;
    default:
      MW_INFO("unknown packet: {:x}.", data_id);
      break;
  }
  if (pcoord) {
    pcoord->X = ConvertAuto(p, format); p += offset;
    pcoord->Y = ConvertAuto(p, format); p += offset;
    pcoord->Z = ConvertAuto(p, format); p += offset;
  }
  if (debug_)
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

void XsensDevice::ParsePosition(uint16_t data_id, uint8_t *content, IDFormat format) {
  uint8_t *p = content;
  Position *ppos = &(msg_.position);
  GenCoord *pcoord = &(ppos->ecef);
  int offset = (IDFormat::IDFloat32 == format ? sizeof(float) : sizeof(double));
  if (CoordFormat::ENU == (data_id & DATAID_COODR_MASK))
    ppos->coord_format = CoordFormat::ENU;
  else if (CoordFormat::NED == (data_id & DATAID_COODR_MASK))
    ppos->coord_format = CoordFormat::NED;
  else if (CoordFormat::NWU == (data_id & DATAID_COODR_MASK))
    ppos->coord_format = CoordFormat::NWU;
  
  ppos->use = true;
  switch (data_id & DATAID_TYPE_MASK) {
    case 0x10: // Altitude MSL  # deprecated
      ppos->alt_msl_use = true;
      ppos->alt_msl = ConvertAuto(p, format); p += offset;
      break;
    case 0x20: // Altitude Ellipsoid mow use
      ppos->alt_ellipsoid_use = true;
      ppos->alt_ellipsoid = ConvertAuto(p, format); p += offset;
      break;
    case 0x30: // Position ECEF
      pcoord->use = true;
      pcoord->X = ConvertAuto(p, format); p += offset;
      pcoord->Y = ConvertAuto(p, format); p += offset;
      pcoord->Z = ConvertAuto(p, format); p += offset;
      break;
    case 0x40: // LatLon now use
      ppos->lat_lon_use = true;
      ppos->lat = ConvertAuto(p, format); p += offset;
      ppos->lon = ConvertAuto(p, format); p += offset;
      break;
    default:
      MW_INFO("unknown packet: {:x}.", data_id);
      break;
  }
  if (debug_)
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

void XsensDevice::ParseGNSS(uint16_t data_id, uint8_t *content, IDFormat format) {
  uint8_t *p = content;
  Gnss *pgnss = &(msg_.gnss);

  pgnss->use = true;
  switch (data_id & DATAID_TYPE_MASK) {
    case 0x10: //  GNSS PVT data, now use
      pgnss->pvt_use = true;
      pgnss->itow = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->year = ConvertUint16(p); p += sizeof(uint16_t);
      pgnss->month = ConvertUint8(p); p += sizeof(uint8_t);
      pgnss->day = ConvertUint8(p); p += sizeof(uint8_t);
      pgnss->hour = ConvertUint8(p); p += sizeof(uint8_t);
      pgnss->min = ConvertUint8(p); p += sizeof(uint8_t);
      pgnss->sec = ConvertUint8(p); p += sizeof(uint8_t);
      pgnss->valid = ConvertUint8(p); p += sizeof(uint8_t);
      pgnss->tAcc = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->nano = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->fixtype = ConvertUint8(p); p += sizeof(uint8_t);
      pgnss->flags = ConvertUint8(p); p += sizeof(uint8_t);
      pgnss->numSV = ConvertUint8(p); p += (sizeof(uint8_t) * 2);
      pgnss->lon = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->lat = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->height = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->hMSL = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->hAcc = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->vAcc = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->velN = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->velE = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->velD = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->gSpeed = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->headMot = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->sAcc = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->headAcc = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->headVeh = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->gdop = ConvertUint16(p); p += sizeof(uint16_t);
      pgnss->pdop = ConvertUint16(p); p += sizeof(uint16_t);
      pgnss->tdop = ConvertUint16(p); p += sizeof(uint16_t);
      pgnss->vdop = ConvertUint16(p); p += sizeof(uint16_t);
      pgnss->hdop = ConvertUint16(p); p += sizeof(uint16_t);
      pgnss->ndop = ConvertUint16(p); p += sizeof(uint16_t);
      pgnss->edop = ConvertUint16(p); p += sizeof(uint16_t);
      // scaling correction
      pgnss->lon *= 1e-7;
      pgnss->lat *= 1e-7;
      pgnss->headAcc *= 1e-5;
      pgnss->headVeh *= 1e-5;
      pgnss->gdop *= 0.01;
      pgnss->pdop *= 0.01;
      pgnss->tdop *= 0.01;
      pgnss->vdop *= 0.01;
      pgnss->hdop *= 0.01;
      pgnss->ndop *= 0.01;
      pgnss->edop *= 0.01;
      break;
    case 0x20: // GNSS satellites info, now use
      pgnss->sta_use = true;
      pgnss->iTow = ConvertUint32(p); p += sizeof(uint32_t);
      pgnss->numSvs = ConvertUint8(p); p += sizeof(uint8_t);
      pgnss->numSvs = pgnss->numSvs > SATEMAX ? SATEMAX : pgnss->numSvs; // TODO
      for (int i = 0; i < pgnss->numSvs; i++) {
        pgnss->sate[i].gnssId = ConvertUint8(p); p += sizeof(uint8_t);
        pgnss->sate[i].svId = ConvertUint8(p); p += sizeof(uint8_t);
        pgnss->sate[i].con = ConvertUint8(p); p += sizeof(uint8_t);
        pgnss->sate[i].flags = ConvertUint8(p); p += sizeof(uint8_t);
      }
      //printf("<%s>:%x %x\n", __func__, pgnss->iTow, pgnss->numSvs);
      break;
    default:
      MW_INFO("unknown packet: {:x}.", data_id);
      break;
  }
  if (debug_)
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

void XsensDevice::ParseAngularVelocity(uint16_t data_id, uint8_t *content, IDFormat format) {
  GenCoord *pcoord = NULL;
  uint8_t *p = content;
  AngularVelocity *pang = &(msg_.angular_velocity);
  int offset = (IDFormat::IDFloat32 == format ? sizeof(float) : sizeof(double));
  if (CoordFormat::ENU == (data_id & DATAID_COODR_MASK))
    pang->coord_format = CoordFormat::ENU;
  else if (CoordFormat::NED == (data_id & DATAID_COODR_MASK))
    pang->coord_format = CoordFormat::NED;
  else if (CoordFormat::NWU == (data_id & DATAID_COODR_MASK))
    pang->coord_format = CoordFormat::NWU;
  
  pang->use = true;
  switch (data_id & DATAID_TYPE_MASK) {
    case 0x20: // Rate of Turn, now use
    case 0x40: // RateOfTurnHR
      pang->gyr.use = true;
      pcoord = &(pang->gyr);
      break;
    case 0x30: // Delta Q
      pang->delta.use = true;
      pcoord = &(pang->delta);
      if (IDFormat::IDFloat32 == format)
        pang->delta_add = ConvertUint32(p + offset * 3);
      else
        pang->delta_add = ConvertUint64(p + offset * 3);
      break;
    default:
      MW_INFO("unknown packet: {:x}.", data_id);
      break;
  }
  if (pcoord) {
    pcoord->X = ConvertAuto(p, format); p += offset;
    pcoord->Y = ConvertAuto(p, format); p += offset;
    pcoord->Z = ConvertAuto(p, format); p += offset;
  }
  if (debug_)
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

void XsensDevice::ParseGPS(uint16_t data_id, uint8_t *content, IDFormat format) {
  if (debug_) 
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

void XsensDevice::ParseAnalogIn(uint16_t data_id, uint8_t *content, IDFormat format) {
  if (debug_)
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

void XsensDevice::ParseMagnetic(uint16_t data_id, uint8_t *content, IDFormat format) {
  GenCoord *pcoord = NULL;
  uint8_t *p = content;
  Magnetic *pmag = &(msg_.magnetic);
  int offset = (IDFormat::IDFloat32 == format ? sizeof(float) : sizeof(double));
  if (CoordFormat::ENU == (data_id & DATAID_COODR_MASK))
    pmag->coord_format = CoordFormat::ENU;
  else if (CoordFormat::NED == (data_id & DATAID_COODR_MASK))
    pmag->coord_format = CoordFormat::NED;
  else if (CoordFormat::NWU == (data_id & DATAID_COODR_MASK))
    pmag->coord_format = CoordFormat::NWU;

  pmag->use = true;
  if (0x20 == (data_id & DATAID_TYPE_MASK)) { // now use
    pmag->mag.use = true;
    pcoord = &(pmag->mag);
  } else {
    MW_INFO("{} unknown packet: 0x{:X}", __func__, data_id);
  }
  if (pcoord) {
    pcoord->X = ConvertAuto(p, format); p += offset;
    pcoord->Y = ConvertAuto(p, format); p += offset;
    pcoord->Z = ConvertAuto(p, format); p += offset;
  }
  if (debug_)
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

void XsensDevice::ParseVelocity(uint16_t data_id, uint8_t *content, IDFormat format) {
  GenCoord *pcoord = NULL;
  uint8_t *p = content;
  Velocity *pvel = &(msg_.velocity);
  int offset = (IDFormat::IDFloat32 == format ? sizeof(float) : sizeof(double));
  if (CoordFormat::ENU == (data_id & DATAID_COODR_MASK))
    pvel->coord_format = CoordFormat::ENU;
  else if (CoordFormat::NED == (data_id & DATAID_COODR_MASK))
    pvel->coord_format = CoordFormat::NED;
  else if (CoordFormat::NWU == (data_id & DATAID_COODR_MASK))
    pvel->coord_format = CoordFormat::NWU;

  pvel->use = true;
  if (0x10 == (data_id & DATAID_TYPE_MASK)) { // Velocity XYZ
    pvel->vel.use = true;
    pcoord = &(pvel->vel);
  } else {
    MW_INFO("{} unknown packet: 0x{:X}", __func__, data_id);
  }
  if (pcoord) {
    pcoord->X = ConvertAuto(p, format); p += offset;
    pcoord->Y = ConvertAuto(p, format); p += offset;
    pcoord->Z = ConvertAuto(p, format); p += offset;
  }
  if (debug_)
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

void XsensDevice::ParseStatus(uint16_t data_id, uint8_t *content, IDFormat format) {
  if (debug_)
    printf("<%s> data_id:%0x\n", __func__, data_id & DATAID_TYPE_MASK);
}

} // namespace xsens
} // namespace device
} // namespace octopus
