#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointVelocity.hh>

namespace step::sim
{
class HipYawTorqueInstrumentation final :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
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
      gzerr << "HipYawTorqueInstrumentation must be attached to a model.\n";
      return;
    }

    this->rightJoint = model.JointByName(_ecm, "right_hip_yaw_joint");
    this->leftJoint = model.JointByName(_ecm, "left_hip_yaw_joint");
    if (this->rightJoint == gz::sim::kNullEntity ||
        this->leftJoint == gz::sim::kNullEntity)
    {
      gzerr << "HipYawTorqueInstrumentation could not find both hip-yaw "
            << "joints.\n";
      return;
    }

    // These calls only request physics-owned measurement components. They do
    // not set position, velocity, force, or JointForceCmd values.
    for (const auto entity : {this->rightJoint, this->leftJoint})
    {
      const gz::sim::Joint joint(entity);
      joint.EnablePositionCheck(_ecm);
      joint.EnableVelocityCheck(_ecm);
      joint.EnableTransmittedWrenchCheck(_ecm);
    }

    const std::string defaultOutput =
        "Simulation/Gazebo/logs/hip_yaw_torque_measurement.csv";
    const std::string output =
        _sdf->Get<std::string>("output", defaultOutput).first;
    const std::filesystem::path outputPath(output);
    if (outputPath.has_parent_path())
      std::filesystem::create_directories(outputPath.parent_path());
    this->stream.open(outputPath, std::ios::out | std::ios::trunc);
    if (!this->stream)
    {
      gzerr << "HipYawTorqueInstrumentation could not open [" << output
            << "].\n";
      return;
    }
    this->stream << std::setprecision(17);
    this->stream
        << "simulation_time,right_joint_force_cmd,left_joint_force_cmd,"
        << "right_joint_force,left_joint_force,"
        << "right_transmitted_tx,right_transmitted_ty,"
        << "right_transmitted_tz,left_transmitted_tx,"
        << "left_transmitted_ty,left_transmitted_tz,"
        << "right_position,left_position,right_velocity,left_velocity\n";
    this->configured = true;
    gzmsg << "HipYawTorqueInstrumentation writing read-only measurements to ["
          << output << "].\n";
  }

  public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
      return;

    const auto *rightCmd = _ecm.Component<gz::sim::components::JointForceCmd>(
        this->rightJoint);
    const auto *leftCmd = _ecm.Component<gz::sim::components::JointForceCmd>(
        this->leftJoint);
    const auto *rightForce = _ecm.Component<gz::sim::components::JointForce>(
        this->rightJoint);
    const auto *leftForce = _ecm.Component<gz::sim::components::JointForce>(
        this->leftJoint);
    const auto *rightWrench =
        _ecm.Component<gz::sim::components::JointTransmittedWrench>(
            this->rightJoint);
    const auto *leftWrench =
        _ecm.Component<gz::sim::components::JointTransmittedWrench>(
            this->leftJoint);
    const auto *rightPosition =
        _ecm.Component<gz::sim::components::JointPosition>(this->rightJoint);
    const auto *leftPosition =
        _ecm.Component<gz::sim::components::JointPosition>(this->leftJoint);
    const auto *rightVelocity =
        _ecm.Component<gz::sim::components::JointVelocity>(this->rightJoint);
    const auto *leftVelocity =
        _ecm.Component<gz::sim::components::JointVelocity>(this->leftJoint);

    const double simulationTime =
        std::chrono::duration<double>(_info.simTime).count();
    this->stream
        << simulationTime << ','
        << VectorFirst(rightCmd) << ',' << VectorFirst(leftCmd) << ','
        << VectorFirst(rightForce) << ',' << VectorFirst(leftForce) << ','
        << Torque(rightWrench, 0) << ',' << Torque(rightWrench, 1) << ','
        << Torque(rightWrench, 2) << ',' << Torque(leftWrench, 0) << ','
        << Torque(leftWrench, 1) << ',' << Torque(leftWrench, 2) << ','
        << VectorFirst(rightPosition) << ',' << VectorFirst(leftPosition) << ','
        << VectorFirst(rightVelocity) << ',' << VectorFirst(leftVelocity)
        << '\n';
    if (++this->rowsSinceFlush >= 100)
    {
      this->stream.flush();
      this->rowsSinceFlush = 0;
    }
  }

  private: template<typename ComponentType>
  static double VectorFirst(const ComponentType *_component)
  {
    if (_component == nullptr || _component->Data().empty())
      return std::numeric_limits<double>::quiet_NaN();
    return _component->Data().front();
  }

  private: static double Torque(
      const gz::sim::components::JointTransmittedWrench *_component,
      const int _axis)
  {
    if (_component == nullptr)
      return std::numeric_limits<double>::quiet_NaN();
    const auto &torque = _component->Data().torque();
    if (_axis == 0)
      return torque.x();
    if (_axis == 1)
      return torque.y();
    return torque.z();
  }

  private: gz::sim::Entity rightJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity leftJoint{gz::sim::kNullEntity};
  private: std::ofstream stream;
  private: std::size_t rowsSinceFlush{0};
  private: bool configured{false};
};
}

GZ_ADD_PLUGIN(
    step::sim::HipYawTorqueInstrumentation,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    step::sim::HipYawTorqueInstrumentation,
    "step::sim::HipYawTorqueInstrumentation")
