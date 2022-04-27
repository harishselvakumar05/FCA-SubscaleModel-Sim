#ifndef GAZEBO_PLUGINS_SUBSCALEGUIPLUGIN_HH_
#define GAZEBO_PLUGINS_SUBSCALEGUIPLUGIN_HH_

#include <mutex>

#include <ignition/math/Angle.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
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

namespace gazebo
{
typedef const boost::shared_ptr<const SubscaleGUIPlugin_msgs::msgs::Subscale> SubscalePtr;
  class GZ_PLUGIN_VISIBLE SubscaleGUIPlugin : public GUIPlugin
  {
    Q_OBJECT

    public: SubscaleGUIPlugin();

    public: virtual ~SubscaleGUIPlugin();
    
    private: void update();
    private: void write(std::string tag, float val);

    private: void OnState(const SubscalePtr &msg);

    private slots: void OnIncreaseThrust();

    private slots: void OnDecreaseThrust();

    private slots: void OnIncreaseFlaps();

    private slots: void OnDecreaseFlaps();

    private slots: void OnIncreaseRoll();

    private slots: void OnDecreaseRoll();

    private slots: void OnIncreaseElevators();

    private slots: void OnDecreaseElevators();

    private slots: void OnIncreaseRudder();

    private slots: void OnDecreaseRudder();

    private slots: void OnPresetTakeOff();

    private slots: void OnPresetCruise();

    private slots: void OnPresetLanding();

    private: sdf::ElementPtr sdf;

    private: transport::NodePtr gzNode;

    private: transport::PublisherPtr controlPub;

    private: transport::SubscriberPtr stateSub;

    private: ignition::math::Angle angleStep;

    private: SubscaleGUIPlugin_msgs::msgs::Subscale state;

    private: std::mutex mutex;


  };
}
#endif
