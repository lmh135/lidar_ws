#ifndef _DATA_H
#define _DATA_H

#define LASER_NUM 1248
#pragma pack(1)

#define start_num 200
#define heading_bais 2.93 //初始gps航向安置偏差

#define T_DELAY_SYS_IMU	60//imu系统时延 us
#define T_DELAY_SYS_LASER	50//laser系统时延 us

typedef struct _Laser_Data
{
	float Azimuth[12];
	float distance[12][16];
	//double reflection[12][16];
	long long timestamp;
	long long laser_timestamp;
	//long long id_num;

}Laser_Data;

typedef struct _GPS_Data
{
	double ned_xyz[3];
	double ned_uvw[3];
	double heading;
	unsigned int gps_sats;
	unsigned int gps_mod;
	long long timestamp;
	//long long id_num;
}GPS_Data;


typedef struct _IMU_Data
{
	float imu_gyro[3];
	float imu_accel[3];
	float imu_magn[3];
	float imu_euler[3];
	long long timestamp;
	//long long id_num;

}IMU_Data;

typedef struct _Packet
{
	IMU_Data imu_data;
	GPS_Data gps_data;
	Laser_Data laser_data;
	//long long timestamp;
	long long id_num;

}Packet;




#pragma pack()


#endif
