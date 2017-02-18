#include "IMU_Data.h"
#include "IMU_Int_Data.h"
#include "ImuIntCommand.h"

#include "ros/ros.h"

#include <cmath>
#include <iostream>

// change if too expensive with -DATA_TYPE=float
#ifdef ATA_TYPE
typedef ATA_TYPE data_t;
#else
typedef double data_t;
#endif

//----------- MATH FUNCTIONS -------------

typedef struct vec3
{
    data_t x, y, z;

    vec3& operator+=(const vec3& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;

        return *this;
    }

    vec3& operator-=(const vec3& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;

        return *this;
    }

    vec3& operator*=(data_t rhs)
    {
        x *= rhs;
        y *= rhs;
        z *= rhs;

        return *this;
    }

    vec3& operator/=(data_t rhs)
    {
        x /= rhs;
        y /= rhs;
        z /= rhs;

        return *this;
    }
} vec3;

vec3 operator+(vec3 lhs, const vec3& rhs)
{
    return (lhs += rhs);
}

vec3 operator-(vec3 lhs, const vec3& rhs)
{
    return (lhs -= rhs);
}

vec3 operator*(vec3 lhs, data_t rhs)
{
    return (lhs *= rhs);
}

vec3 operator*(data_t lhs, vec3 rhs)
{
    return (rhs *= lhs);
}

vec3 operator/(vec3 lhs, data_t rhs)
{
    return (lhs /= rhs);
}

vec3 operator-(const vec3& lhs)
{
    return lhs * -1;
}

data_t dot(vec3 a, vec3 b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

vec3 cross(vec3 a, vec3 b)
{
    vec3 result;
    result.x = a.y*b.z - a.z*b.y;
    result.y = a.z*b.x - a.x*b.z;
    result.z = a.x*b.y - a.y*b.x;
    return result;
}

typedef struct quat
{
    // w is scalar
    data_t w, x, y, z;

    data_t scalar() const
    {
        return w;
    }

    vec3 vec() const
    {
        return vec3{x,y,z};
    }

    quat& operator*=(data_t rhs)
    {
        x *= rhs;
        y *= rhs;
        z *= rhs;
        w *= rhs;

        return *this;
    }

    quat& operator/=(data_t rhs)
    {
        x /= rhs;
        y /= rhs;
        z /= rhs;
        w /= rhs;

        return *this;
    }

    quat& operator*=(const quat& rhs)
    {
        this->w = this->scalar() * rhs.scalar() - dot(this->vec(), rhs.vec());
        vec3 tmp = this->scalar() * rhs.vec();
        tmp += rhs.scalar() * this->vec();
        tmp += cross(this->vec(), rhs.vec());

        this->x = tmp.x;
        this->y = tmp.y;
        this->z = tmp.z;
        
        return *this;
    }
} quat;

quat operator*(quat lhs, data_t rhs)
{
    return (lhs *= rhs);
}

quat operator*(data_t lhs, quat rhs)
{
    return (rhs *= lhs);
}

quat operator/(quat lhs, data_t rhs)
{
    return (lhs /= rhs);
}

quat operator*(quat lhs, const quat& rhs)
{
    return (lhs *= rhs);
}

data_t norm(const vec3& a)
{
    return std::sqrt(dot(a,a));
}

quat conj(const quat& q)
{
    return quat{q.w, -q.x, -q.y, -q.z};
}

data_t norm_sq(const quat& q)
{
    return q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
}

data_t norm(const quat& q)
{
    return std::sqrt(norm_sq(q));
}

quat inverse(const quat& q)
{
    return conj(q) / norm_sq(q);
}

quat normalize(const quat& q)
{
    return q / norm(q);
}

//--------------- END OF MATH FUNCTIONS -----------

ros::Publisher pub;

vec3 last_accel {0, 0, 0};
// note velocity and position are zeroed when node starts; KEEP AUV STATIONARY when starting
// position is really just a displacement from where the AUV was at startup time
vec3 last_vel {0, 0, 0};
vec3 position {0, 0, 0};
double last_time = 0;

vec3 transform_accel_to_world(vec3 accel, quat transform_q)
{
    quat accel_q = {0, accel.x, accel.y, accel.z};
    
    // V_world = Q^-1 * V_imu * Q (from IMU specification)
    quat accel_world = inverse(transform_q) * accel_q * transform_q;

    return vec3{accel_world.x, accel_world.y, accel_world.z};
}

void integrate(vec3 accel_data, quat orientation_data, double timestamp)
{
    vec3 new_accel = transform_accel_to_world(accel_data, orientation_data);
    // IMU provides accel in units of G (9.81 m/s/s), want it in SI
    new_accel *= 9.81;

    if(last_time == 0)
    {
        // first piece of data; timestamp is likely nonzero since the IMU has been running for a short time
        // initialize state variables, do no processing
        last_accel = new_accel;
        last_time = timestamp;
        return;
    }
    
    // probably not the first piece of data then; can do integration

    double del_time = timestamp - last_time;
    last_time = timestamp;

    // based on trapezoid approximation
    // relies on data being in matching units (accel in m/s/s, time in s)
    vec3 average_accel = (new_accel + last_accel) / 2;
    vec3 del_vel = average_accel * del_time;
    
    vec3 new_vel = last_vel + del_vel;
    // velocity is integrated now; move on to position
    
    // probably a better way of doing this second integration (something tells me overlapping trapezoid integrations are not the best)
    // TODO: do better with more math to back this up
    vec3 average_vel = (new_vel + last_vel) / 2;
    vec3 del_position = average_vel * del_time;
    
    position += del_position;

    ROS_INFO("At position (%.3f, %.3f, %.3f)", position.x, position.y, position.z);
    ROS_INFO("With velocity (%.3f, %.3f, %.3f)", new_vel.x, new_vel.y, new_vel.z);
}

void publish_data()
{
    imu_int::IMU_Int_Data data;
    
    data.accel[0] = last_accel.x;
    data.accel[1] = last_accel.y;
    data.accel[2] = last_accel.z;

    data.vel[0] = last_vel.x;
    data.vel[1] = last_vel.y;
    data.vel[2] = last_vel.z;
    
    data.pos[0] = position.x;
    data.pos[1] = position.y;
    data.pos[2] = position.z;
    
    data.time = last_time;

    pub.publish(data);
}

void imu_callback(const imu::IMU_Data::ConstPtr& msg)
{
    vec3 accel_data{msg->accel[0], msg->accel[1], msg->accel[2]};
    quat orientation_data{msg->quat[0], msg->quat[1], msg->quat[2], msg->quat[3]};
    double time_data = msg->time;

    integrate(accel_data, orientation_data, time_data);

    publish_data();
}

bool issue_command(imu_int::ImuIntCommand::Request &req, imu_int::ImuIntCommand::Response &res)
{
    if(req.reset)
    {
        last_vel = vec3{0, 0, 0};
        position = vec3{0, 0, 0};
        ROS_INFO("Zeroed position and velocity");
    }
    else
    {
        ROS_INFO("Received service command; did not reset (reset was false)");
    }
    
    res.echo = req.reset;
    
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_int_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<imu::IMU_Data>("/input/imu", 100, imu_callback);

    pub = n.advertise<imu_int::IMU_Int_Data>("/input/imu_int", 1000);
    
    ros::ServiceServer serv = n.advertiseService("imu_int_service", issue_command);

    ROS_INFO("Started up.");

    ros::spin();

    return 0;
}
