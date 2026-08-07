// Copyright 2026 Larry
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \brief gz-sim system plugin implementing the active aerodynamic
/// stabilizer (paper Fig. 6A / SM Eq. S33).
///
/// The plugin subscribes to a boolean topic (cable tightened = active) and,
/// at the physics rate, applies the flat-plate aligning torque to the body
/// link.  The torque is computed from the link's own world velocity (no
/// position/velocity feedback is used by the stabilizer), which is exactly
/// the sensor-free strategy of the paper.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <utility>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/MessageInfo.hh>

#include "quadcopter_gz_plugins/aero_stabilizer.hh"

namespace quadcopter_gz_plugins
{

AeroWrench AeroStabilizerWrench(
    const gz::math::Vector3d &zBody,
    const gz::math::Vector3d &velocity,
    double rho,
    double surface,
    double kEff,
    double momentArm,
    double speedEps)
{
  AeroWrench out;
  const double speed = velocity.Length();
  const double zNorm = zBody.Length();
  if (speed < speedEps || zNorm < 1e-12)
  {
    out.speed = speed;
    return out;
  }

  const gz::math::Vector3d z = zBody / zNorm;
  const gz::math::Vector3d a = -velocity / speed;  // -v_hat
  const double sinTheta = std::min(1.0, std::max(0.0, z.Cross(a).Length()));
  const double theta = std::atan2(sinTheta, z.Dot(a));
  out.thetaLd = theta;
  out.speed = speed;
  if (sinTheta <= 1e-9)
  {
    return out;
  }

  // Torque axis z x a: with the rotational dynamics z_dot ~ tau x z the
  // body axis moves toward a, i.e. aligns with the opposite of the
  // velocity (zb -> -v_hat).
  const gz::math::Vector3d n = z.Cross(a) / sinTheta;
  const double force = kEff * rho * surface * sinTheta * speed * speed;
  out.forceMag = force;
  out.torque = momentArm * force * n;
  return out;
}

class AerodynamicStabilizerSystem final : public gz::sim::System,
                                          public gz::sim::ISystemConfigure,
                                          public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override;

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  private: struct Data
  {
    gz::sim::Entity linkEntity{gz::sim::kNullEntity};
    double rho = 1.2;
    double surface = 2.0 * 39.0e-4;
    double kEff = 1.0;
    double momentArm = 0.06;
    double speedEps = 0.05;
    double maxSpeed = 30.0;
    bool active = false;
    std::string topic = "/model/quadcopter/stabilizer_active";
    std::string diagTopic = "/model/quadcopter/stabilizer_state";
    gz::transport::Node node;
    gz::transport::Node::Publisher diagPub;
    std::deque<std::pair<double, gz::math::Vector3d>> poseHistory;
    gz::math::Vector3d smoothVel = gz::math::Vector3d::Zero;
  };

  private: std::unique_ptr<Data> data_{std::make_unique<Data>()};
};

void AerodynamicStabilizerSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
{
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "AerodynamicStabilizerSystem requires a model entity.\n";
    return;
  }

  std::string linkName = "base_link";
  if (_sdf->HasElement("link_name"))
    linkName = _sdf->Get<std::string>("link_name");
  if (_sdf->HasElement("rho"))
    data_->rho = _sdf->Get<double>("rho");
  if (_sdf->HasElement("surface"))
    data_->surface = _sdf->Get<double>("surface");
  if (_sdf->HasElement("k_eff"))
    data_->kEff = _sdf->Get<double>("k_eff");
  if (_sdf->HasElement("moment_arm"))
    data_->momentArm = _sdf->Get<double>("moment_arm");
  if (_sdf->HasElement("speed_eps"))
    data_->speedEps = _sdf->Get<double>("speed_eps");
  if (_sdf->HasElement("max_speed"))
    data_->maxSpeed = _sdf->Get<double>("max_speed");
  if (_sdf->HasElement("topic"))
    data_->topic = _sdf->Get<std::string>("topic");
  if (_sdf->HasElement("diag_topic"))
    data_->diagTopic = _sdf->Get<std::string>("diag_topic");

  data_->linkEntity = model.LinkByName(_ecm, linkName);
  if (data_->linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "AerodynamicStabilizerSystem: link '" << linkName
          << "' not found.\n";
    return;
  }

  std::function<void(const gz::msgs::Boolean &,
                     const gz::transport::MessageInfo &)> cb =
      [this](const gz::msgs::Boolean &_msg,
             const gz::transport::MessageInfo &)
      {
        data_->active = _msg.data();
      };
  data_->node.Subscribe(data_->topic, cb);
  data_->diagPub = data_->node.Advertise<gz::msgs::Float_V>(data_->diagTopic);
  gzmsg << "AerodynamicStabilizerSystem configured: rho=" << data_->rho
        << " kg/m^3, S=" << data_->surface << " m^2, k_eff=" << data_->kEff
        << ", arm=" << data_->momentArm << " m, topic=" << data_->topic
        << std::endl;
}

void AerodynamicStabilizerSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;
  if (data_->linkEntity == gz::sim::kNullEntity)
    return;

  gz::sim::Link link(data_->linkEntity);
  const auto worldPose = link.WorldPose(_ecm);
  if (!worldPose)
    return;

  const double simTime =
      std::chrono::duration<double>(_info.simTime).count();
  data_->poseHistory.push_back({simTime, worldPose->Pos()});
  while (data_->poseHistory.size() > 1 &&
         simTime - data_->poseHistory.front().first > 0.05)
  {
    data_->poseHistory.pop_front();
  }

  gz::math::Vector3d worldVel = gz::math::Vector3d::Zero;
  // Velocity from a ~20 ms pose window: smooths the 1 kHz position noise
  // while staying fast enough for the ~400 ms descent phase.
  for (auto it = data_->poseHistory.rbegin();
       it != data_->poseHistory.rend(); ++it)
  {
    const double dt = simTime - it->first;
    if (dt >= 0.015)
    {
      worldVel = (worldPose->Pos() - it->second) / dt;
      break;
    }
  }
  // Reject pose-teleport spikes (e.g. the first frames after spawn).
  if (worldVel.Length() <= data_->maxSpeed)
  {
    data_->smoothVel = worldVel;
  }
  worldVel = data_->smoothVel;

  const gz::math::Vector3d zBody = worldPose->Rot() *
      gz::math::Vector3d::UnitZ;
  const AeroWrench wrench = AeroStabilizerWrench(
      zBody, worldVel, data_->rho, data_->surface, data_->kEff,
      data_->momentArm, data_->speedEps);

  if (data_->active && wrench.torque.Length() > 0.0)
  {
    link.AddWorldWrench(_ecm, wrench.force, wrench.torque);
  }

  gz::msgs::Float_V msg;
  msg.mutable_data()->Resize(7, 0.0);
  msg.set_data(0, static_cast<float>(simTime));
  msg.set_data(1, static_cast<float>(data_->active ? 1.0 : 0.0));
  msg.set_data(2, static_cast<float>(wrench.thetaLd * 180.0 / M_PI));
  msg.set_data(3, static_cast<float>(wrench.speed));
  msg.set_data(4, static_cast<float>(wrench.torque.X()));
  msg.set_data(5, static_cast<float>(wrench.torque.Y()));
  msg.set_data(6, static_cast<float>(wrench.torque.Z()));
  data_->diagPub.Publish(msg);
}

}  // namespace quadcopter_gz_plugins

GZ_ADD_PLUGIN(
    quadcopter_gz_plugins::AerodynamicStabilizerSystem,
    gz::sim::System,
    quadcopter_gz_plugins::AerodynamicStabilizerSystem::ISystemConfigure,
    quadcopter_gz_plugins::AerodynamicStabilizerSystem::ISystemPreUpdate)
