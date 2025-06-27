#include "VacuumPlugin.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
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
//typename std::shared_ptr<OnMessageWrapper<gz::msgs::Boolean>> GraspingOnMessageWrapperPtr;
// Private data class
//
GZ_ADD_PLUGIN(gz::sim::systems::VacuumGripperPlugin,
              gz::sim::System,
              gz::sim::systems::VacuumGripperPlugin::ISystemConfigure,
              gz::sim::systems::VacuumGripperPlugin::ISystemPreUpdate);


GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::VacuumGripperPlugin, "VacuumGripperPlugin");


class gz::sim::systems::VacuumGripperPluginPrivate{

public: gz::sim::Model model{gz::sim::kNullEntity}; //model
public: gz::sim::Entity parentLink{gz::sim::kNullEntity};//link entity referenc e
public: std::string modelName;
public: std::string parentName;// vacuum_link name
public: std::string robot_namespace;
public: std::string enable_topic;
public: std::string grasping_topic;

public: float max_force;
public: float scale;
public: float min_depth;
public: float max_depth;

  //publisher for publishing whether or not the gripper is currently gripping something
public: gz::transport::Node::Publisher graspingPub;
public: gz::transport::Node node;
  // one is a msg for dat like grasping
  //public gz::msgs::Boolean grasping_msg;
  //whether or not the cvacuum is on
public: bool enable_vacuum;
public: std::mutex enableMsgMutex;

public: void enableCb(const gz::msgs::Boolean &_msg){
  std::lock_guard<std::mutex> lock(this->enableMsgMutex);
  this->enable_vacuum = Convert(_msg);//returns the value of the msg
  }
};

gz::sim::systems::VacuumGripperPlugin::VacuumGripperPlugin():
  dataPtr(new VacuumGripperPluginPrivate){
}

gz::sim::systems::VacuumGripperPlugin::~VacuumGripperPlugin(){
}


void gz::sim::systems::VacuumGripperPlugin::Configure(const gz::sim::Entity   &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventMgr){
  //TODO:
    //check if model is valid ?
  this->dataPtr->model = Model(_entity);
  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);
  if(!this ->dataPtr-> model.Valid(_ecm)){
    gzerr << "vaccum plugin should be attached to a model, entity failed to initialize " << std::endl;
    return;
  }
  //TODO initialize 

  //TODO: parse sdf and look for tags 
  //namespace done
  //<topic>
  //<max_force>
  auto sdfClone = _sdf->Clone();
  if(sdfClone->HasElement("robotNamespace")){
    this->dataPtr->robot_namespace = sdfClone->Get<std::string>("robotNamespace");
  }
  else{
    this->dataPtr->robot_namespace = ""; //INFO: empty by default reccomended to fill this out for the robots namespace
  }
  if(sdfClone->HasElement("enableTopic")){
    this->dataPtr->enable_topic = {this->dataPtr->robot_namespace + "/" + sdfClone->Get<std::string>("enableTopic")};
  }
  else{
    gzerr << "a topic was not given failed to configure plugin" << std::endl;
    return; 
  }
  if(sdfClone->HasElement("graspingTopic")){
    this->dataPtr->grasping_topic = {this->dataPtr->robot_namespace +"/" + sdfClone->Get<std::string>("graspingTopic")};
  }
  else{
    gzerr << "a grasping topic was not given failed to configure plugin" << std::endl;
    return; 
  }
  if(sdfClone->HasElement("gripperLink" )){
    this->dataPtr->parentName = sdfClone->Get<std::string>("gripperLink");
    this->dataPtr->parentLink = this->dataPtr->model.LinkByName(_ecm, this->dataPtr->parentName);
  }else{
    gzerr << "a gripper link was not given or malformed gripper_link name was given and the plugin failed to configure " << std::endl;
    return; 
  }
  if(sdfClone->HasElement("maxDepth")){
    this->dataPtr->max_depth = sdfClone->Get<float>("maxDepth");
  }
  else{
    this->dataPtr->max_depth = 0.05; //INFO: empty by default reccomended to fill this out for the robots namespace
    gzmsg << "min depth is not assigned in the sdf defaults to 0.05" << std::endl;
  }//
  if(sdfClone->HasElement("minDepth")){
    this->dataPtr->min_depth = sdfClone->Get<float>("minDepth");
  }
  else{
    this->dataPtr->min_depth = 0.01; 
    gzmsg << "min depth is not assigned in the sdf defaults to 0.01" <<std::endl;
  }
  this->dataPtr->model = Model(_entity);
 
  // ensure vacuum_link has world angular velocity
  if (!_ecm.EntityHasComponentType(this->dataPtr->parentLink, gz::sim::components::WorldAngularVelocity::typeId)){
    _ecm.CreateComponent(this->dataPtr->parentLink, gz::sim::components::WorldAngularVelocity());
  }
  if (!_ecm.EntityHasComponentType(this->dataPtr->parentLink, gz::sim::components::WorldLinearVelocity::typeId)){
    _ecm.CreateComponent(this->dataPtr->parentLink, gz::sim::components::WorldLinearVelocity());
  }
  if (!_ecm.EntityHasComponentType(this->dataPtr->parentLink, gz::sim::components::WorldPose::typeId)){
    _ecm.CreateComponent(this->dataPtr->parentLink, gz::sim::components::WorldPose());
  }
  this->dataPtr->graspingPub = this->dataPtr->node.Advertise<gz::msgs::Boolean>(this->dataPtr->grasping_topic);
  this->dataPtr->node.Subscribe(this->dataPtr->enable_topic,
                                &gz::sim::systems::VacuumGripperPluginPrivate::enableCb, 
                                this->dataPtr.get());// subscripe to enable topic of transport with in gazebo
};


void gz::sim::systems::VacuumGripperPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                                                      gz::sim::EntityComponentManager &_ecm){
  //info:  get the position of the plane joint and get the normal force
  // check current state of the gripper if it is true
  //  get gripper position and check if there is a gripper at adistance from the position of the thing
  gz::msgs::Boolean grasping_msg;
  if(!this->dataPtr->enable_vacuum){
    grasping_msg.set_data(false);
  }
  else{
    gz::sim::Link parentLink(this->dataPtr->parentLink);
    // optionals not checked because they are enabled in Configure
    gz::math::Pose3d parentPose = Link(this->dataPtr->parentLink).WorldPose(_ecm).value();
    gz::math::Vector3d parentLinearVelocity = parentLink.WorldLinearVelocity(_ecm).value();
    gz::math::Vector3d parentAngularVelocity = parentLink.WorldAngularVelocity(_ecm).value();
    std::vector<gz::sim::Entity> models = _ecm.EntitiesByComponents(gz::sim::components::Model());
    for(gz::sim::Entity entity: models){
      gz::sim::Model model{entity};
      if (model.Name(_ecm) != this->dataPtr->modelName){
        std::vector<gz::sim::Entity> links = model.Links(_ecm);
        for(gz::sim::Entity entity_sub: links)
        {
          gz::sim::Link link = Link(entity_sub);
          std::optional<gz::math::Pose3d> linkPosOptional =  link.WorldPose(_ecm);
          //returns an optional and needs to be handled
          if(!linkPosOptional.has_value()){
            continue;
          }
          gz::math::Pose3d diff = parentPose - linkPosOptional.value();
          double norm = diff.Pos().Length();
          //TODO fix this to fit under 
          if(norm< 0.05){
            link.SetLinearVelocity(_ecm, parentLinearVelocity);
            link.SetAngularVelocity(_ecm, parentLinearVelocity);
            double norm_force = 1/norm;
            if(norm_force>50){
              norm_force = 50;
            }
            gz::math::Vector3d appliedForce = diff.Pos().Normalize() * norm_force;
            link.AddWorldForce(_ecm, appliedForce);
            grasping_msg.set_data(true);
          }
        }
      }
    }
  }
  this->dataPtr->graspingPub.Publish(grasping_msg);
}
