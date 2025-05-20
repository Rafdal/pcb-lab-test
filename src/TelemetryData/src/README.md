
```c++
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


sprintf(buffer, "3165,01:22:10,%d,F,ASCENT,%0.1f,%0.1f,%0.1f,8.3,18,21,20,30,35,33,0.22,0.03,0.09,2165,13:14:02,200.8,3.8793,18.3672,5,%s,", packet_count++, alt, t, p, CommandParser.last_command());

```