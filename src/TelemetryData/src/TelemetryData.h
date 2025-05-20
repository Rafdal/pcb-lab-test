#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <stdint.h>

#define MAX_COMMAND_ECHO_LENGTH 16

typedef struct {
    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
} Time_Data_t;

typedef struct { 
    int16_t x_pitch;
    int16_t y_roll;
    int16_t z_yaw;
} Axis_Data_t;

typedef struct TelemetryData {
    Time_Data_t mission_time_rtc;
    uint16_t packet_count;
    uint8_t mode;
    uint8_t state;
    uint16_t altitude;
    uint16_t temperature;
    uint8_t pressure;
    uint8_t voltage;
    Axis_Data_t gyro;
    Axis_Data_t accel;
    Axis_Data_t mag;
    uint16_t ag1_rot_rate;
    uint16_t ag2_rot_rate;  // LAST
    Time_Data_t gps_time;
    uint16_t gps_altitude;
    float gps_latitude;
    float gps_longitude;
    uint8_t gps_sats;
    char cmd_echo[MAX_COMMAND_ECHO_LENGTH]; // Buffer for command echo
} TelemetryData_t;

class TelemetryPacket {
public:
    TelemetryPacket();

    // Initialize the telemetry data
    void init();

    // Update the telemetry data
    void update();

    // Get the current telemetry data
    const char* getData() const;
};

/*
TEAM_ID,                    3165,       //  [const]     Team ID
MISSION_TIME,               01:22:10,   //  (3 bytes)   RTC time, UTC [hh:mm:ss]
PACKET_COUNT,               50,         //  (2 bytes)   packet count
MODE,                       F,          //  (1 byte)    'F' or 'S' flight/sim mode
STATE,                      ASCENT,     //  (1 byte)    FSM state
ALTITUDE,                   500.3,      //  (2 bytes)   Altitude %.1f [m]
TEMPERATURE,                25.7,       //  (2 bytes)   Temp [°C]
PRESSURE,                   101.2,      //  (1 byte)    Air pressure [kPa] from 83.0 to 108.0
VOLTAGE,                    8.3,        //  (1 byte)    Battery Voltage [V]
GYRO_R,                     18,         //  (2 bytes)   Gyro Roll   [°/s]
GYRO_P,                     21,         //  (2 bytes)   Gyro Pitch  [°/s]
GYRO_Y,                     20,         //  (2 bytes)   Gyro Yaw    [°/s]
ACCEL_R,                    30,         //  (2 bytes)   Accel Roll  [°/s]
ACCEL_P,                    35,         //  (2 bytes)   Accel Pitch [°/s]
ACCEL_Y,                    33,         //  (2 bytes)   Accel Yaw   [°/s]
MAG_R,                      0.22,       //  (2 bytes)   Mag Roll    [gauss]
MAG_P,                      0.03,       //  (2 bytes)   Mag Pitch   [gauss]
MAG_Y,                      0.09,       //  (2 bytes)   Mag Yaw     [gauss]
AUTO_GYRO_ROTATION_RATE,    2165,       //  (2 byte)    Rotation rate [°/s] for auto-rotation
GPS_TIME,                   13:14:02,   //  (3 bytes)   GPS time, UTC [hh:mm:ss]
GPS_ALTITUDE,               200.8,      //  (2 bytes)   GPS Altitude %.1f [m]
GPS_LATITUDE,               3.8793,     //  (4 bytes)   GPS Latitude in degrees
GPS_LONGITUDE,              18.3672,    //  (4 bytes)   GPS Longitude in degrees
GPS_SATS,                   5,          //  (1 byte)    Num of Sats tracked by GPS
CMD_ECHO,                   CXON,       //  (N bytes)   Text of the last command received and processed
*/

#endif // TelemetryData.h