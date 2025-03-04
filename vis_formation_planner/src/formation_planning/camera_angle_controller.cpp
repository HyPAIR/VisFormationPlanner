#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
  class CameraAngleController : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
        this->model = _model;

        this->joint = _model->GetJoint("camera_joint1");
        if (!this->joint)
        {
            gzerr << "Joint [camera_joint1] not found!\n";
            return;
        }

        std::string robot_namespace;
        if (_sdf->HasElement("robotNamespace"))
        {
            robot_namespace = _sdf->Get<std::string>("robotNamespace");
        }
        else
        {
            gzerr << "No <robotNamespace> specified in SDF. Using default namespace.\n";
            robot_namespace = "/";
        }

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "gazebo_camera_angle_controller",
                    ros::init_options::NoSigintHandler);
        }
        this->nh = std::make_shared<ros::NodeHandle>(robot_namespace);

        std::string topic_name = robot_namespace + "/camera_angle";
        this->angleSub = this->nh->subscribe<std_msgs::Float64>(
            topic_name, 1,
            [this](const std_msgs::Float64::ConstPtr &msg)
            {
                this->SetCameraAngle(msg->data);
            });

        gzmsg << "Subscribed to topic: " << topic_name << std::endl;
    }

    void SetCameraAngle(double angle)
    {
      if (this->joint)
      {
        this->joint->SetPosition(0, angle); 
      }
    }

  private:
    physics::ModelPtr model;
    physics::JointPtr joint;
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Subscriber angleSub;
  };

  GZ_REGISTER_MODEL_PLUGIN(CameraAngleController)
}