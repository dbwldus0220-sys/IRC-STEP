#include <memory>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace step::sim
{
class ConstantPoseInitializer final :
    public gz::sim::System,
    public gz::sim::ISystemConfigure
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    const gz::sim::Model model(_entity);
    if (!model.Valid(_ecm))
    {
      gzerr << "ConstantPoseInitializer must be attached to a model.\n";
      return;
    }

    if (!_sdf->HasElement("joint"))
    {
      gzerr << "ConstantPoseInitializer requires at least one <joint>.\n";
      return;
    }

    sdf::ElementConstPtr jointElement = _sdf->FindElement("joint");
    while (jointElement)
    {
      const std::string jointName = jointElement->Get<std::string>("name");
      const double position = jointElement->Get<double>();
      const gz::sim::Entity jointEntity = model.JointByName(_ecm, jointName);
      if (jointEntity == gz::sim::kNullEntity)
      {
        gzerr << "ConstantPoseInitializer could not find joint ["
              << jointName << "].\n";
      }
      else
      {
        gz::sim::Joint joint(jointEntity);
        joint.ResetPosition(_ecm, {position});
        joint.ResetVelocity(_ecm, {0.0});
      }
      jointElement = jointElement->GetNextElement("joint");
    }
  }
};
}

GZ_ADD_PLUGIN(
    step::sim::ConstantPoseInitializer,
    gz::sim::System,
    gz::sim::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(
    step::sim::ConstantPoseInitializer,
    "step::sim::ConstantPoseInitializer")
