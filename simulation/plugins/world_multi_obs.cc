#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/WorldState.h"

namespace gazebo
{
  
  class ROSControl : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;
      this->model_name = _parent->GetName();

      if (!ros::isInitialized())
      {
          int argc = 0;
          char** argv = NULL;
          ros::init(argc, argv, "obstacle_control", ros::init_options::NoSigintHandler);  
      }
      
      this->rosNode.reset(new ros::NodeHandle()); 

      this->rosSub = this->rosNode->subscribe("/obstacles_velocity", 1000, &ROSControl::obstacleCallback, this);

      // Store the pointer to the model 
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ROSControl::OnUpdate, this));
      // ros::spin();
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(ignition::math::Vector3d(this->linear_x, this->linear_y, 0));
      this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
    }

    void obstacleCallback(const gazebo_msgs::WorldState::ConstPtr& message)
    {
      auto name = this->model_name;
      int num = message->name.size();
      
      auto iter_name = message->name.begin();
      auto iter_twist = message->twist.begin();

      for (int i = 0; i < num; i++)
      {
          if (name == *iter_name)
          {
              this->linear_x = (*iter_twist).linear.x;
              this->linear_y = (*iter_twist).linear.y;
          }
          else
          {
              iter_name++;
              iter_twist++;
          }
      }
      
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: std::unique_ptr<ros::NodeHandle> rosNode;

    private: float linear_x{};
    private: float linear_y{};
    private: ros::Subscriber rosSub;
    private: std::string model_name;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSControl)
}