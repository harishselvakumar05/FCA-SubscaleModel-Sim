#include <functional>
#include <string>
#include <sdf/sdf.hh>
#include <ignition/common/Profiler.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <boost/shared_ptr.hpp>
#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif
#include "SubscalePlugin.hh"
  #include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip> 
#include <gazebo/transport/TransportTypes.hh>
#include "msgs/build/subscale.pb.h"

using namespace gazebo;
 std::array<float, 4> ejoints;
 std::array<std::string, 4> stringdata;
 const char *path = "/home/harish/SubscaleGUIPlugin/state.txt";
  double dtt = 0;
void SubscalePlugin::update(){
    std::fstream state(path);
    std::string line;
        int i = 0;
        while (getline(state,line))
        {
          if(line.find(".")){
           ejoints[i] = std::stof(line.substr(line.find(":") + 1)); 
           stringdata[i] = line;
          i++;
          }
        }
}

GZ_REGISTER_MODEL_PLUGIN(SubscalePlugin)

////////////////////////////////////////////////////////////////////////////////
SubscalePlugin::SubscalePlugin()
{
  this->cmds.fill(0.0f);
  update();
  for(int x = 0; x < stringdata.size(); x++){
    gzerr << stringdata[x] << std::endl;
  }

}

/////////////////////////////////////////////////
SubscalePlugin::~SubscalePlugin()
{
  this->updateConnection.reset();
  gzerr << "HEY";
}


/////////////////////////////////////////////////
bool SubscalePlugin::FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf,
    physics::JointPtr &_joint)
{
  // Read the required plugin parameters.
  if (!_sdf->HasElement(_sdfParam))
  {
    gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
    return false;
  }

  std::string jointName = _sdf->Get<std::string>(_sdfParam);
  _joint = this->model->GetJoint(jointName);
  
  if (!_joint)
  {
    gzerr << "Failed to find joint [" << jointName
          << "] aborting plugin load." << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void SubscalePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "SubscalePlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "SubscalePlugin _sdf pointer is NULL");
  this->model = _model;
  


  // Read the required joint name parameters.
  std::vector<std::string> requiredParams = {"left_aileron",
    "right_aileron", "left_elevator", "right_elevator",  "rudder"};

  for (size_t i = 0; i < requiredParams.size(); ++i)
  {
    if (!this->FindJoint(requiredParams[i], _sdf, this->joints[i]))
      return;
  }


  // Controller time control.
  this->lastControllerUpdateTime = this->model->GetWorld()->SimTime();

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&SubscalePlugin::Update, this, std::placeholders::_1));

  // Initialize transport.
 transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  std::string prefix = "~/" + this->model->GetName() + "/";
  
   this->statePub = node->Advertise<SubscaleGUIPlugin_msgs::msgs::Subscale>(prefix + "state");
  this->controlSub = node->Subscribe(prefix + "control", &SubscalePlugin::OnControl, this);

}
void SubscalePlugin::OnControl(const gazebo::SubscalePtr &msg )
{
	gzerr << "WORKED";
  std::lock_guard<std::mutex> lock(this->mutex);


}

/////////////////////////////////////////////////
void SubscalePlugin::Update(const common::UpdateInfo &/*_info*/)
{

  std::lock_guard<std::mutex> lock(this->mutex);
  gazebo::common::Time curTime = this->model->GetWorld()->SimTime();

  if (curTime > this->lastControllerUpdateTime)
  {
    // Update the control surfaces and publish the new state.
    update();
   // this->model->GetLink("subscale_model::motor")->SetLinearVel({40,0,5});
    double dt = (curTime-lastControllerUpdateTime).Double();
    double  rAV =( (ejoints[0]*tRad) - (joints[0]->Position(0))) / dt;
    double  lAV =( (ejoints[1]*tRad) - (joints[1]->Position(0))) / dt;
    double  rEV =( (ejoints[2]*tRad) - (joints[2]->Position(0))) / dt;
    double  RV =(  (ejoints[3]*tRad) - (joints[4]->Position(0))) / dt;
    //gzerr << vel << " " << (ejoints[kLeftAileron]*tRad) << " " << (joints[kLeftAileron]->Position(0)) << " " << dt << " " << std::endl;
    
    joints[0]->SetVelocity(0, rAV );
    joints[1]->SetVelocity(0, lAV );
    joints[2]->SetVelocity(0, rEV );
    joints[3]->SetVelocity(0, rEV );
    joints[4]->SetVelocity(0, RV );


    if(joints[0]->WorldPose().Rot().X() < 0.35 && -0.35 < joints[0]->WorldPose().Rot().X()){
      if(dtt == 0){
        dtt = curTime.Double();
      }
     // gzdbg << "Time: " << curTime.Double() - dtt << "    Position: " << joints[0]->WorldPose().Rot().X() << "    Input: " << ejoints[0] << std::endl;
    
    } else {
      dtt = 0;
    }



    ///oints[0]->SetPosition(0, ejoints[0] * this->tRad);
    /*
    joints[1]->SetPosition(0, ejoints[1] * this->tRad);
    joints[2]->SetPosition(0, ejoints[2] * this->tRad);
    joints[3]->SetPosition(0, ejoints[2] * this->tRad);
    joints[4]->SetPosition(0, ejoints[3] * this->tRad);
    */
    this->PublishState();
    this->lastControllerUpdateTime = curTime;
  }
}

/////////////////////////////////////////////////


/////////////////////////////////////////////////
void SubscalePlugin::PublishState()
{
  // Read the current state.

  float leftAileron = this->joints[kLeftAileron]->Position(0);
  float rightAileron = this->joints[kRightAileron]->Position(0);
  float Lelevator = this->joints[kLeftElevator]->Position(0);
  float Relevator = this->joints[kRightElevator]->Position(0);
  float rudder = this->joints[kRudder]->Position(0);
  
  /*
SubscaleGUIPlugin_msgs::msgs::Subscale msg;

  msg.set_left_aileron(leftAileron);
  msg.set_right_aileron(rightAileron);
  msg.set_elevators(elevators);
  msg.set_rudder(rudder);

  // Set the target state.
  msg.set_cmd_left_aileron(this->cmds[kLeftAileron]);
  msg.set_cmd_right_aileron(this->cmds[kRightAileron]);
  msg.set_cmd_elevators(this->cmds[kElevators]);
  msg.set_cmd_rudder(this->cmds[kRudder]);

  this->statePub->Publish(msg);
  
*/
}
