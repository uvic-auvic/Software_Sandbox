#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <ignition/math/Rand.hh>
#include "std_msgs/String.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

namespace gazebo
{
    class Shark : public ModelPlugin
    {

        private: int delay, scaleFactor, delayFactor;

        private: bool left;

        private: math::Vector3 forceCurrent_, torqueCurrent_, torqueTail_;

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            delay = 0;
            scaleFactor = 100;
            left = true;
            delayFactor = ignition::math::Rand::DblUniform(1000,4000);

          // Store the pointer to the model
          this->model = _parent;

          this->updateConnection = event::Events::ConnectWorldUpdateBegin(
              boost::bind(&Shark::OnUpdate, this, _1));
        }

        // Called by the world update start event
        public: void OnUpdate(const common::UpdateInfo & /*_info*/)
        {
            if(((++delay)%10) == 0)
            {
                physics::LinkPtr Shark;
                Shark = this->model->GetLink("body");
                math::Pose worldPose = this->model->GetWorldPose();
                math::Quaternion angle = worldPose.rot;
                forceCurrent_ = angle.RotateVector(math::Vector3(0,ignition::math::Rand::DblUniform(-1,0),0));

                Shark->AddForce(forceCurrent_*scaleFactor*10);
                if((delay % 200) == 0)
                {
                    if(left == true)
                    {
                        torqueTail_ = angle.RotateVector(math::Vector3(0,0,1));
                        left = false;
                    }
                    else
                    {
                        torqueTail_ = angle.RotateVector(math::Vector3(0,0,-1));
                        left = true;
                    }
					Shark->AddTorque(torqueCurrent_*scaleFactor*1000);
					

                }
				Shark->AddTorque(torqueTail_*scaleFactor);
                if(delay == delayFactor)
                {
                    torqueCurrent_ = angle.RotateVector(math::Vector3(0,ignition::math::Rand::DblUniform(-1,1),ignition::math::Rand::DblUniform(-1,1)));
                    Shark->AddTorque(torqueCurrent_*scaleFactor);
					delay = 0;
                    delayFactor = ignition::math::Rand::DblUniform(1000,4000);

                }
				
            }
        }


        // Pointer to the mode
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(Shark);
}
