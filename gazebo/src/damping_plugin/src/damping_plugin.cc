#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

namespace gazebo
{
    class Damping : public ModelPlugin
    {

        private: int delay;

        private: math::Vector3 forceOld_, forceCurrent_, torqueOld_, torqueCurrent_;

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            delay = 0;
            forceOld_ = math::Vector3(0,0,0);
            torqueOld_ = math::Vector3(0,0,0);

          // Store the pointer to the model
          this->model = _parent;

          this->updateConnection = event::Events::ConnectWorldUpdateBegin(
              boost::bind(&Damping::OnUpdate, this, _1));
        }

        // Called by the world update start event
        public: void OnUpdate(const common::UpdateInfo & /*_info*/)
        {
            if(((++delay)%10) == 0)
            {
                physics::LinkPtr buoy;
                buoy = this->model->GetLink("body");
                forceOld_ = buoy->GetRelativeLinearVel();
                torqueOld_ = buoy->GetRelativeAngularVel();
                buoy->AddRelativeForce(-forceOld_*200);
                buoy->AddRelativeTorque(-torqueOld_*200);
                if(delay ==1500)
                {
                    delay = 0;
                }
            }
        }


        // Pointer to the mode
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(Damping);
}
