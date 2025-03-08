#include <gz/msgs/boolean.pb.h>

#include <gz/register/plugin.hh>
#include <gz/common/Register.hh>
//#include <gz/sensors/Noise.hh> #TODO: add a flag to implement noise
//#include <gz/sensors/SensorFactory.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

//#include "vacuum_plugin.hh"
// pulling actuator params from sdf
// max vaccum force acruation distance
//    -- topic to activate
//    -- actuation distance 
//
//
//
//
//Taken from for publishibng https://github.com/ArduPilot/ardupilot_gazebo/blob/main/src/ArduPilotPlugin.cc literally got no fucking clue 
/*
template <typename M>
class OnMessageWrapper
{
  public: typedef std::function<void(const M &)> callback_t;

  public: callback_t callback;

  public: OnMessageWrapper(const callback_t &_callback)
    : callback(_callback)
  {
  }

  public: void OnMessage(const M &_msg)
  {
    if (callback)
    {
      callback(_msg);
    }
  }
};
*/
typename std::shared_ptr<OnMessageWrapper<gz::msgs::Boolean>> GraspingOnMessageWrapperPtr;
// Private data class
class VacuumGripperPluginPrivate{

  public gz::sim::systems::Model model{gz::sim::kNullEntity}; // model
  public gz::sim::systems::Entity gripperLink{gz::sim::kNullEntity};// link
  //
  public std::string modelName;
  public std::string linkName;
  //  variables for topic names
  public std::string namespace, enable_topic, gripping_topic;
  // 
  public bool enabled_succ;
  public float max_force;
  public float scale;

  //publisher for publishing whether or not the gripper is currently gripping something
  public gz::transport::Node::Publisher pub;
  // one is a msg for dat like grasping
  //public gz::msgs::Boolean grasping_msg;
  //whether or not the cvacuum is on
  public bool enable_vacuum;
  public std::mutex _mutex;

}

GZ_ADD_PLUGIN(gz::sim::systems::VacuumGripperPlugin,
              gz::sim::System,
              gz::sim::system::VacuumGripper::ISystemConfigure,
              gz::sim::system::VacuumGripper::ISystemPreUpdate);


GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::VacuumGripperPlugin, "VacuumGripperPlugin");


void VacuumGripperPlugin::Configure(const gz::sim::systemsL::Entity   &_entity,
                       const std::shared_ptr<const gz::sim::sdf::Element> &_sdf,
                       gz::sim::systems::EntityComponentManager &_ecm,
                       gz::sim::systems::EventManager &_eventMgr){
  //TODO:
    //check if model is valid ?
  this->dataPtr->model = Model(_entity);
  if(!this -> model.Valid(_ecm)){
    gzerr << "vaccum plugin should be attached to a model, entity failed to initialize " << std::endl;
    return;
  }
  //TODO initialize 

  //TODO: parse sdf and look for tags 
  //namespace done
  //<topic>
  //<max_force>
  auto sdfClone = _sdf->Clone();
  if(sdfClone->hasElement("enable_topic")){
    this->topic_partial = sdfClone->Get<std::string>("enable_topic");
  }
  else{
    gzerr << "a topic was not given failed to configure plugin" << std::endl;
    return; 
  }

  if(sdfClone->hasElement("namespace")){
    this->dataPtr->namespace = sdfClone->Get<std::string>("namespace");
  }
  else{
    this->namespace = ""; //INFO: empty by default reccomended to fill this out for the robots namespace
  }
  if(sdfClone->hasElement("max_depth")){
    this->max_depth = sdfClone->Get<float>("namespace");
  }
  else{
    this->max_depth = 0.05; //INFO: empty by default reccomended to fill this out for the robots namespace
    gzmsg << "min depth is not assigned in the sdf defaults to 0.05" << std::endl;
  }//
  if(sdfClone->hasElement("min_depth")){
    this->min_depth = sdfClone->Get<float>("min_depth");

  }
  else{
    this->min_depth = 0.01; 
    gzmsg << "min depth is not assigned in the sdf defaults to 0.01" <<std::endl;
  }
  if(this->namespace == ""){
    std::string topic{this->namespace + "/" + this -> topic_partial}
  }else{
    std::string topic{this->topic_partial}
  },,
  this-> dataPtr-> graspingPub = this->node.Advertise<msgs::boolean>();
  this->dataPtr->this->dataPtr->node.Subscribe<>(topic, &VaccumGripper::onGripperUpdate, this);// subscripe to enable topic of transport with in gazebo
};


void VacuumGripper::onGrippingCallback(const gz::msgs::boolean &_msg){
    // lock o update
  std::lock<std::mutex> lock(this->dataPtr->enable_vacuum);
  this->dataPtr->enable_vacuum = _msg.data;//returns the value of the msg
}


void VacuumGripper::PreUpdate(const UpdateInfo &_info,
                                const EntityComponentManager &_ecm){

  //INFO:  get the position of the plane joint and get the normal force
  // check current state of the gripper if it is true
  //  get gripper position and check if there is a gripper at adistance from the position of the thing
  gz::msgs::Boolean grasping_msg;
  if(!this->enabled_vacuum){
    grasping_msg.set_data(false);
    this->dataPtr->pub.Publish(grasping_msg);
    return;
  }
  else
  {
      gz::math::Pose3d parent_pose  = WorldPose(_ecm, _ecm.entityByName(this->dataPtr->parentName));
      _ecm.Each<components::Link,components::WorldPose>(
              [&](const Entity &_entity,
                  const components::Link *) -> bool{
        if (_entity.GetName() != this->dataPtr->modelName ||
            _entity.linkName() != this->dataPtr->parentName)
            gz::sim::Link link = Link(_entity);
            link_pos = worldPose(_entity, _ecm);
            gz::math::Vector3d diff  = parent_pose.Pos() - link_pos.Pos() ;
            double norm  = diff.length;
            if (norm < this->max_distance){
                // TODO apply velocity
                math::Vector3d parentLinearVelocity = parent_entity.WorldLinearVelocity();
                math::Vector3d parentAngularVelocity = parent_entity.WorldAngularVelcoity();
                norm_force = 1/norm;
          }
      });
  }
  //TODO: implement the normal force matching
};
void VacuumGripper::onGripperUpdate(const msgs::boolean &_msg){
  

}



GZ_ADD_PLUGIN(VaccumGripper::VaccumGripper,
              gz::sim::System,
              VaccumGripper::ISystemPreUpdate);
