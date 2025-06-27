#include <gz/sim/System.hh>
#include <gz/msgs/boolean.pb.h>
//#include <gz/sensor/Sensor.hh>
#include <sdf/sdf.hh>
#include <gz/transport/Node.hh>

namespace gz{
namespace sim{
namespace systems{
//forward decleration
class VacuumGripperPluginPrivate;

class GZ_SIM_VISIBLE VacuumGripperPlugin: 
      public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
    //\brief simple constructor
    public: VacuumGripperPlugin();
    // brief simple constructor 
    public: ~VacuumGripperPlugin();
    // \brief method implementsed to solve ISystemPreUpdate
    public: virtual void Configure(const Entity  &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       EntityComponentManager &_ecm,
                       EventManager &_eventMgr) final;
    // \brief method implementsed to solve ISystemPreUpdate
    public: virtual void PreUpdate(const UpdateInfo &_info,
                                   EntityComponentManager &_ecm) final;
    // \brief callback for a publisher to decide when and when not gripping
    private: void onEnableCallback(const msgs::Boolean &_msg);

    private: std::unique_ptr<VacuumGripperPluginPrivate> dataPtr;
    };
  }//namespace systems
}//namespace sim
}// namespace gz
