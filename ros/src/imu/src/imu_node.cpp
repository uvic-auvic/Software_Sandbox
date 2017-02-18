// not going to bother fighting with include directories right now
//: TODO: do better
//#include "/home/odroid/catkin_ws/devel/include/imu/IMU_Data.h"
//#include "../../../devel/include/imu/IMU_Data.h"
#include "IMU_Data.h"

#include "ros/ros.h"
#include "serial/serial.h"

#include "ros/time.h"
#include "std_msgs/String.h"

#include <string>
#include <iostream>
#include <vector>
#include <ios>
#include <stdint.h>


/*
 *
 * IMU config values (as of July 20, 2016):
 * ========================================
 *
 * RAM:      Accel:     0x0064
 *           Magneto:   0x0064
 *           Gyro bias: 0x000A
 *
 * EEPROM:   Accel:     0x2134
 *           Magneto:   0x07D0
 *           Gyro gain: 0x2710
 *
 * Units are:
 *           Accel:   G's (1 G = 9.81m/s/s)
 *           Magnet:  Gauss
 *           AngRate: rad/s
 *           Timer:   s
 *
 * For info on quaternions, see: http://www.j3d.org/matrix_faq/matrfaq_latest.html#Q47
 * Converting to Euler angles: http://bediyap.com/programming/convert-quaternion-to-euler-rotations/
 *
 * Global angles are given with X pointing North, Y pointing East, and Z pointing down
 *
 */

const uint32_t IMU_SCALE_CONSTANT = 32768000;
const uint16_t IMU_QUAT_SCALE_CONSTANT = 8192;

// vectors
typedef struct vec16
{
    int16_t x, y, z;
} vec16;

typedef struct vecf
{
    float x, y, z;
} vecf;

// quaternions, w is scalar, x, y, z are imaginary
typedef struct quat16
{
    int16_t w, x, y, z;
} quat16;

typedef struct quatf
{
    float w, x, y, z;
} quatf;

serial::Serial *ser;

// gain scale constants (specified in command protocol)
float MagGainScale;
float AccelGainScale;
float GyroGainScale;
double TimerScale;

// persistent time; will get reported instead of IMU time
// (avoids overflow every 7 minutes from the IMU)
// should be good for ~1 year of continuous usage
// updated after vector/quat poll
uint32_t node_ticks = 0;
uint16_t last_time = 0;

// byte utils
uint16_t concat_bytes(uint8_t msb, uint8_t lsb)
{
    return (((uint16_t) msb) << 8) | ((uint16_t) lsb);
}

// doesn't perform conversion; reinterprets bits
int16_t to_signed(uint16_t s)
{
    return *reinterpret_cast<int16_t*>(&s);
}

uint8_t msb(uint16_t s)
{
    return static_cast<uint8_t>(s >> 8);
}

uint8_t lsb(uint16_t s)
{
    return static_cast<uint8_t>(s & 0xFF);
}

bool imu_checksum(const std::vector<uint8_t> &data)
{
    auto len = data.size();
    std::vector<uint16_t> pairs;

    if(len < 3)
        return false;

    uint16_t command = (uint16_t) data[0];
    pairs.push_back(command);

    uint16_t curr_pair;

    for(int i = 1; i < len - 2; i += 2)
    {
        curr_pair = concat_bytes(data[i], data[i+1]);
        pairs.push_back(curr_pair);
    }

    uint16_t sum = 0;
    for(uint16_t p : pairs)
        sum += p;

    uint16_t checksum = concat_bytes(data[len-2], data[len-1]);

    return sum == checksum;
}

// for single byte commands
std::vector<uint8_t> imu_command(uint8_t com, std::size_t responseSize)
{
    ser->write(&com, 1);
    std::vector<uint8_t> response;
    ser->read(response, responseSize);
    // throw out data if incomplete
    // should get caught by checksum if used reasonably
    if(response.size() != responseSize)
        response = std::vector<uint8_t>();
    // no checksum calculation because EEPROM reads don't have checksums
    return response;
}

// for multi-byte commands
std::vector<uint8_t> imu_command(const std::vector<uint8_t> &com, std::size_t responseSize)
{
    ser->write(com);
    std::vector<uint8_t> response;
    ser->read(response, responseSize);
    // throw out data if incomplete
    // should get caught by checksum if used reasonably
    if(response.size() != responseSize)
        response = std::vector<uint8_t>();
    // no checksum calculation because EEPROM reads don't have checksums
    return response;
}

bool read_eeprom(uint16_t address, uint16_t &value)
{
    // command, address_msb, address_lsb
    std::vector<uint8_t> command = {0x28, msb(address), lsb(address)};
    std::vector<uint8_t> data = imu_command(command, 7);

    if(!imu_checksum(data))
        return false;

    value = concat_bytes(data[1], data[2]);

    return true;
}

bool init_gain_scales()
{
    bool success = true;

    uint16_t mag_scale_factor;
    success &= read_eeprom(0x00E8, mag_scale_factor);

    uint16_t accel_scale_factor;
    success &= read_eeprom(0x00E6, accel_scale_factor);

    uint16_t gyro_scale_factor;
    success &= read_eeprom(0x0082, gyro_scale_factor);

    uint16_t timer_scale_1, timer_scale_2, timer_scale_3, timer_scale_4;
    success &= read_eeprom(0x00EE, timer_scale_1);
    success &= read_eeprom(0x00F0, timer_scale_2);
    success &= read_eeprom(0x00F2, timer_scale_3);
    success &= read_eeprom(0x00F6, timer_scale_4);

    std::cout << "Read scale factors of:" << std::endl;
    std::cout << "  Magnet:   " << mag_scale_factor << std::endl;
    std::cout << "  Accel:    " << accel_scale_factor << std::endl;
    std::cout << "  Gyro:     " << gyro_scale_factor << std::endl;
    std::cout << "  Timer:    " << timer_scale_1 << ", " << timer_scale_2
                 << ", " << timer_scale_3 << ", " << timer_scale_4 << std::endl;

    MagGainScale = ((float) IMU_SCALE_CONSTANT) / mag_scale_factor;
    AccelGainScale = ((float) IMU_SCALE_CONSTANT) / accel_scale_factor;
    GyroGainScale = ((float) IMU_SCALE_CONSTANT) / gyro_scale_factor;
    TimerScale = (((double) timer_scale_1 * timer_scale_2) * timer_scale_3) * timer_scale_4;
    TimerScale *= 1e-7;

    std::cout << "Gain scales:" << std::endl;
    std::cout << "  Magnet:   " << MagGainScale << std::endl;
    std::cout << "  Accel:    " << AccelGainScale << std::endl;
    std::cout << "  Gyro:     " << GyroGainScale << std::endl;
    std::cout << "  Tick len: " << TimerScale << "s" << std::endl;

    return success;
}

// reads stabilized vectors and orientation quaternion
bool read_stab_vecs_quat(vec16 *accel, vec16 *magnet, vec16 *angRate, quat16 *quat, uint16_t *time)
{
    std::vector<uint8_t> data = imu_command(0x0C, 31);
    if(!imu_checksum(data))
        return false;

    if(accel)
    {
        accel->x = to_signed(concat_bytes(data[15], data[16]));
        accel->y = to_signed(concat_bytes(data[17], data[18]));
        accel->z = to_signed(concat_bytes(data[19], data[20]));
    }

    if(magnet)
    {
        magnet->x = to_signed(concat_bytes(data[9], data[10]));
        magnet->y = to_signed(concat_bytes(data[11], data[12]));
        magnet->z = to_signed(concat_bytes(data[13], data[14]));
    }

    if(angRate)
    {
        angRate->x = to_signed(concat_bytes(data[21], data[22]));
        angRate->y = to_signed(concat_bytes(data[23], data[24]));
        angRate->z = to_signed(concat_bytes(data[25], data[26]));
    }

    if(quat)
    {
        quat->w = to_signed(concat_bytes(data[1], data[2]));
        quat->x = to_signed(concat_bytes(data[3], data[4]));
        quat->y = to_signed(concat_bytes(data[5], data[6]));
        quat->z = to_signed(concat_bytes(data[7], data[8]));
    }

    uint16_t read_time = concat_bytes(data[27], data[28]);
    if(time)
        *time = read_time;

    return true;
}

// converts to physical units
bool read_stab_vecs_quat_phys(vecf *accel, vecf *magnet, vecf *angRate, quatf *quat, double *time)
{
    vec16 accel_raw, magnet_raw, angRate_raw;
    quat16 quat_raw;
    uint16_t time_raw;

    bool success = read_stab_vecs_quat(&accel_raw, &magnet_raw, &angRate_raw, &quat_raw, &time_raw);

    if(!success)
        return false;

    if(accel)
    {
        accel->x = accel_raw.x / AccelGainScale;
        accel->y = accel_raw.y / AccelGainScale;
        accel->z = accel_raw.z / AccelGainScale;
    }

    if(magnet)
    {
        magnet->x = magnet_raw.x / MagGainScale;
        magnet->y = magnet_raw.y / MagGainScale;
        magnet->z = magnet_raw.z / MagGainScale;
    }

    if(angRate)
    {
        angRate->x = angRate_raw.x / GyroGainScale;
        angRate->y = angRate_raw.y / GyroGainScale;
        angRate->z = angRate_raw.z / GyroGainScale;
    }

    if(quat)
    {
        quat->w = quat_raw.w / (float) IMU_QUAT_SCALE_CONSTANT;
        quat->x = quat_raw.x / (float) IMU_QUAT_SCALE_CONSTANT;
        quat->y = quat_raw.y / (float) IMU_QUAT_SCALE_CONSTANT;
        quat->z = quat_raw.z / (float) IMU_QUAT_SCALE_CONSTANT;
    }


    if(last_time > time_raw)
    {
        node_ticks += (65536 - last_time) + time_raw;
    }
    else
    {
        node_ticks += (time_raw - last_time);
    }
    last_time = time_raw;

    if(time)
        *time = node_ticks * TimerScale;

    return true;
}

imu::IMU_Data generate_message()
{
    imu::IMU_Data data;

    vecf accel, magnet, angRate;
    quatf quat;
    double time;

    bool success = read_stab_vecs_quat_phys(&accel, &magnet, &angRate, &quat, &time);
    if(!success)
    {
        ROS_ERROR("Couldn't read IMU data - checksum failed");
        data.time = -1;
        return data;
    }

    data.accel[0] = accel.x;
    data.accel[1] = accel.y;
    data.accel[2] = accel.z;

    data.mag[0] = magnet.x;
    data.mag[1] = magnet.y;
    data.mag[2] = magnet.z;

    data.angrate[0] = angRate.x;
    data.angrate[1] = angRate.y;
    data.angrate[2] = angRate.z;

    data.quat[0] = quat.w;
    data.quat[1] = quat.x;
    data.quat[2] = quat.y;
    data.quat[3] = quat.z;

    data.time = time;

    return data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<imu::IMU_Data>("/input/imu", 1000);

    ROS_INFO("running...");

    std::string port = "/dev/ttySAC0";
    if(argc > 1)
        port = argv[1];
    ser = new serial::Serial(port, 38400, serial::Timeout::simpleTimeout(1000));
    std::cout << "Opened on port " << ser->getPort() << std::endl;

    if(!ser->isOpen())
    {
        ROS_ERROR("Serial didn't open");
    }

    // setup gain scale constants
    bool gain_scale_success = init_gain_scales();
    if(!gain_scale_success)
    {
        ROS_ERROR("Failed to init gain scales");
    }

    imu::IMU_Data data = generate_message();
    pub.publish(data);

    ros::Time time = ros::Time::now();
    ros::Duration second = ros::Duration(1,0);

    int loop_count = 0;

    while(ros::ok())
    {
        ++loop_count;
        ros::Time now = ros::Time::now();
        if(now > time + second)
        {
            ROS_INFO("Running at %d Hz", loop_count);
            loop_count = 0;
            time = now;
        }

        imu::IMU_Data msg = generate_message();
        pub.publish(msg);

        ros::spinOnce();
    }

    delete ser;

    return 0;
}
