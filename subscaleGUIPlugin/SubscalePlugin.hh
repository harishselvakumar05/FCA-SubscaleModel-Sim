#ifndef GAZEBO_PLUGINS_SubscalePLUGIN_HH_
#define GAZEBO_PLUGINS_SubscalePLUGIN_HH_

#include <array>
#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include <boost/shared_ptr.hpp>
#include <ignition/transport/Node.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif
#include <gazebo/transport/TransportTypes.hh>

#include "msgs/build/subscale.pb.h"

namespace gazebo
{

   typedef const boost::shared_ptr<const SubscaleGUIPlugin_msgs::msgs::Subscale> SubscalePtr;
  class GZ_PLUGIN_VISIBLE SubscalePlugin : public ModelPlugin
  {
    public: SubscalePlugin();

    public: ~SubscalePlugin();
    private: void update();
    //private: void write(std::string tag, float val);
   public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private: bool FindJoint(const std::string &_sdfParam,
        sdf::ElementPtr _sdf, physics::JointPtr &_joint);

    private: void Update(const common::UpdateInfo &_info);
    
  //  public: 

    private: void OnControl(const gazebo::SubscalePtr &msg);
    private: void PublishState();

    private: static const unsigned int kLeftAileron  = 0;
    private: static const unsigned int kRightAileron = 1;
    private: static const unsigned int kLeftElevator    = 2;
    private: static const unsigned int kRightElevator    = 2;
    private: static const unsigned int kRudder       = 3;


    private: event::ConnectionPtr updateConnection;


 


    private: gazebo::transport::SubscriberPtr controlSub;


    private: gazebo::transport::PublisherPtr statePub;

    private: physics::ModelPtr model;


    private: std::array<physics::JointPtr, 7> joints;


    private: std::array<float, 4> cmds;



    private: gazebo::common::Time lastControllerUpdateTime;

    private: std::mutex mutex;

    private: ignition::transport::Node nodeIgn;


    private: ignition::transport::Node::Publisher statePubIgn;
    private: const float tRad = 3.14/180;
  };
}
#endif
