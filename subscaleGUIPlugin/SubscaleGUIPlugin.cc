#include <algorithm>

#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/Actions.hh>
#include "SubscaleGUIPlugin.hh"
#include "msgs/build/subscale.pb.h"

#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif
#include <iostream>
#include <fstream>
#include <array>
#include <string>
#include <iomanip>
#include <sstream>

using namespace gazebo;
 std::array<float, 4> joints;
 std::array<std::string, 4> stringdata;
 const char *path = "/home/harish/SubscaleGUIPlugin/state.txt";
GZ_REGISTER_GUI_PLUGIN(SubscaleGUIPlugin)

void SubscaleGUIPlugin::update(){
    std::fstream state(path);
    std::string line;
        int i = 0;
        while (getline(state,line))
        {
          if(line.find(".")){
           joints[i] = std::stof(line.substr(line.find(":") + 1)); 
           stringdata[i] = line;
          i++;
          }
        }
        state.close();
}

void SubscaleGUIPlugin::write(std::string tag, float val){
    std::ostringstream out;
    out.precision(3);
    std::string d = ":";
    std::ofstream state(path);
    std::string line;
    if(state.is_open()){
      for(int x = 0; x < 4; x++)
      {
        if((stringdata[x].find(tag))){
            out << std::fixed << stringdata[x] + "\n";
        } else {
            out << std::fixed << tag + ":" << val << "\n";
        }
      }
    state << out.str();
    state.close();  
    update(); 
  }}

SubscaleGUIPlugin::SubscaleGUIPlugin()
  : GUIPlugin()
{
  this->move(-1, -1);
  this->resize(1, 1);
  this->angleStep.Degree(1.0);
  update();

  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->stateSub = this->gzNode->Subscribe<SubscaleGUIPlugin_msgs::msgs::Subscale>("~/subscale/state", &SubscaleGUIPlugin::OnState, this);

    this->controlPub = this->gzNode->Advertise<SubscaleGUIPlugin_msgs::msgs::Subscale>("~/subscale/control");
    




  QShortcut *increaseThrust = new QShortcut(QKeySequence("w"), this);
  QObject::connect(increaseThrust, SIGNAL(activated()), this,
      SLOT(OnIncreaseThrust()));

  QShortcut *decreaseThrust = new QShortcut(QKeySequence("s"), this);
  QObject::connect(decreaseThrust, SIGNAL(activated()), this,
      SLOT(OnDecreaseThrust()));


  QShortcut *increaseRoll = new QShortcut(QKeySequence(Qt::Key_Left), this);
  QObject::connect(increaseRoll, SIGNAL(activated()), this,
      SLOT(OnIncreaseRoll()));

  QShortcut *decreaseRoll = new QShortcut(QKeySequence(Qt::Key_Right), this);
  QObject::connect(decreaseRoll, SIGNAL(activated()), this,
      SLOT(OnDecreaseRoll()));

  QShortcut *increaseElevators =
    new QShortcut(QKeySequence(Qt::Key_Down), this);
  QObject::connect(increaseElevators, SIGNAL(activated()), this,
      SLOT(OnIncreaseElevators()));

  QShortcut *decreaseElevators = new QShortcut(QKeySequence(Qt::Key_Up), this);
  QObject::connect(decreaseElevators, SIGNAL(activated()), this,
      SLOT(OnDecreaseElevators()));

  QShortcut *increaseRudder = new QShortcut(QKeySequence("d"), this);
  QObject::connect(increaseRudder, SIGNAL(activated()), this,
      SLOT(OnIncreaseRudder()));

  QShortcut *decreaseRudder = new QShortcut(QKeySequence("a"), this);
  QObject::connect(decreaseRudder, SIGNAL(activated()), this,
      SLOT(OnDecreaseRudder()));

  QShortcut *presetTakeOff = new QShortcut(QKeySequence('1'), this);
  QObject::connect(presetTakeOff, SIGNAL(activated()), this,
      SLOT(OnPresetTakeOff()));

  QShortcut *presetCruise = new QShortcut(QKeySequence('2'), this);
  QObject::connect(presetCruise, SIGNAL(activated()), this,
      SLOT(OnPresetCruise()));

  QShortcut *presetLanding = new QShortcut(QKeySequence('3'), this);
  QObject::connect(presetLanding, SIGNAL(activated()), this,
      SLOT(OnPresetLanding()));
}


SubscaleGUIPlugin::~SubscaleGUIPlugin()
{
  write("left_aileron", 0);
  write("right_aileron", 0);
  write("elevators", 0);
  write("rudder", 0);
}

void SubscaleGUIPlugin::OnState(SubscalePtr &msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->state = *msg;
}


void SubscaleGUIPlugin::OnIncreaseThrust()
{
  float thrust;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    thrust = 10;
  }


 SubscaleGUIPlugin_msgs::msgs::Subscale msg;
  thrust = std::min(thrust + 0.1f, 1.0f);
  msg.set_cmd_propeller_speed(thrust);
  this->controlPub->Publish(msg);
}

void SubscaleGUIPlugin::OnDecreaseThrust()
{
  float thrust;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    thrust = this->state.cmd_propeller_speed();
  }

SubscaleGUIPlugin_msgs::msgs::Subscale msg;
  thrust = std::max(thrust - 0.1f, 0.0f);
  msg.set_cmd_propeller_speed(thrust);
  this->controlPub->Publish(msg);
}

void SubscaleGUIPlugin::OnIncreaseRoll()
{
  ignition::math::Angle laileron;
  ignition::math::Angle raileron;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    laileron.Degree(joints[0]);
    raileron.Degree(joints[1]);
  }

  if (laileron.Degree() + this->angleStep.Degree() <= 20)
  {
    laileron += this->angleStep;
    write("left_aileron", laileron.Degree());
  } 
  if(raileron.Degree() - this->angleStep.Degree() >= -15){
    raileron -= this->angleStep;
    write("right_aileron", raileron.Degree());
  }  
    update();
}

void SubscaleGUIPlugin::OnDecreaseRoll()
{
  ignition::math::Angle laileron;
  ignition::math::Angle raileron;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    laileron.Degree(joints[0]);
    raileron.Degree(joints[1]);
  }

  if (laileron.Degree() - this->angleStep.Degree() >= -15)
  {
    laileron -= this->angleStep;
    write("left_aileron", laileron.Degree());
  } 
  if(raileron.Degree() + this->angleStep.Degree() <= 20){
    raileron += this->angleStep;
    write("right_aileron", raileron.Degree());
  }  
    update();
}

void SubscaleGUIPlugin::OnIncreaseElevators()
{
 ignition::math::Angle elevators;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    elevators.Degree(joints[2]);
  }

  if (elevators.Degree() + this->angleStep.Degree() <= 20)
  {
    elevators += this->angleStep;
    write("elevators", elevators.Degree());
    update();
  }
}

void SubscaleGUIPlugin::OnDecreaseElevators()
{
 ignition::math::Angle elevators;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    elevators.Degree(joints[2]);
  }
  if (elevators.Degree() - this->angleStep.Degree() >= -15)
  {
    elevators -= this->angleStep;
    write("elevators", elevators.Degree());
    update();
  }
}

void SubscaleGUIPlugin::OnIncreaseRudder()
{
 ignition::math::Angle rudder;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    rudder.Degree(joints[3]);
  }

  if (rudder.Degree() + this->angleStep.Degree() <= 20)
  {
    rudder += this->angleStep;
    write("rudder", rudder.Degree());
    update();
  }
}


void SubscaleGUIPlugin::OnDecreaseRudder()
{
 ignition::math::Angle rudder;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    rudder.Degree(joints[3]);
  }

  if (rudder.Degree() - this->angleStep.Degree()  >= -20)
  {
    rudder -= this->angleStep;
    write("rudder", rudder.Degree());
    update();
  }
}

void SubscaleGUIPlugin::OnPresetTakeOff()
{
SubscaleGUIPlugin_msgs::msgs::Subscale msg;
  msg.set_cmd_propeller_speed(0.8);
  msg.set_cmd_left_aileron(-0.017);
  msg.set_cmd_right_aileron(0.017);
  msg.set_cmd_elevators(0.033);
  msg.set_cmd_rudder(-0.035);
  this->controlPub->Publish(msg);
}


void SubscaleGUIPlugin::OnPresetCruise()
{
SubscaleGUIPlugin_msgs::msgs::Subscale msg;
  msg.set_cmd_propeller_speed(0.6);
  msg.set_cmd_left_aileron(0);
  msg.set_cmd_right_aileron(0);
  msg.set_cmd_elevators(0.12);
  msg.set_cmd_rudder(-0.035);
  this->controlPub->Publish(msg);
}


void SubscaleGUIPlugin::OnPresetLanding()
{
SubscaleGUIPlugin_msgs::msgs::Subscale msg;
  msg.set_cmd_propeller_speed(0.3);
  msg.set_cmd_left_aileron(0);
  msg.set_cmd_right_aileron(0);
  msg.set_cmd_elevators(0.16);
  msg.set_cmd_rudder(-0.035);
  this->controlPub->Publish(msg);
}

