#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <thread>

namespace gazebo {

enum Wheel : int {
  LF = 0,
  RF = 1,
  LB = 2,
  RB = 3,
};

using namespace std::chrono_literals;
/// \brief A plugin to control a Velodyne sensor.
class VelodynePlugin : public ModelPlugin {
  /// \brief Constructor
public:
  VelodynePlugin() {}

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

    this->model = _model;
    this->world = _model->GetWorld();

    this->wheels = _model->GetJoints();

    // wheel[Wheel::LF]->SetForce(1, 1000);

    // std::cout << wheel.size() << std::endl;
    // std::cout << wheel[Wheel::LF]->GetForceTorque() << std::endl;
    // std::cout << wheel[Wheel::LF]->GetScopedName() << std::endl;
    // std::cout << wheel[Wheel::RF]->GetScopedName() << std::endl;
    // std::cout << wheel[Wheel::LB]->GetScopedName() << std::endl;
    // std::cout << wheel[Wheel::RB]->GetScopedName() << std::endl;

    // std::cout << wheel->GetType() << std::endl;
    // std::cout << wheel->GetScopedName() << std::endl;

    updateBeginConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        [this](const gazebo::common::UpdateInfo &info) {
          // std::cout << info.simTime.nsec << std::endl;
          this->wheels[Wheel::LF]->SetForce(1, -2);
          this->wheels[Wheel::RF]->SetForce(1, -2);
          this->wheels[Wheel::LB]->SetForce(1, -2);
          this->wheels[Wheel::RB]->SetForce(1, -2);
        });

    updateEndConnection = gazebo::event::Events::ConnectWorldUpdateEnd(
        [this] {
        std::cout << this->wheels[Wheel::LF]->Position(1) << std::endl;

        });

    // std::thread t1([=]() {
    //   auto pid = common::PID(0.1, 0, 0);
    //   for (int i = 0; i < 100000; i++) {
    //     std::this_thread::sleep_for(4ms);
    //     // this->world.Step(1);
    //   }
    // });
    // t1.detach();

    //
    // auto pid = common::PID(0.1, 0, 0);
    //
    // this->model->GetJointController()->SetVelocityPID(
    //     wheel[Wheel::LF]->GetScopedName(), pid);
    // this->model->GetJointController()->SetVelocityPID(
    //     wheel[Wheel::RF]->GetScopedName(), pid);
    //
    // this->model->GetJointController()->SetVelocityTarget(
    //     wheel[Wheel::LF]->GetScopedName(), 10.0);
    // this->model->GetJointController()->SetVelocityTarget(
    //     wheel[Wheel::RF]->GetScopedName(), 10.0);
  }

private:
  physics::ModelPtr model;
  physics::WorldPtr world;
  std::vector<physics::JointPtr> wheels;

  gazebo::event::ConnectionPtr updateBeginConnection;
  gazebo::event::ConnectionPtr updateEndConnection;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
} // namespace gazebo
#endif
