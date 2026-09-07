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

/// \brief gz-sim system plugin implementing a passive spring-damper leg.
///
/// Applied at the physics rate (1 ms in this project): reads the prismatic
/// leg joint state and sets the generalized force from paper Eq. 7.  The
/// plugin also publishes [sim_time, q, q_dot, F, foot_z] as gz.msgs.Float_V
/// on `/model/quadcopter/leg_state` for the ROS phase/logging node.

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/float_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>

#include "quadcopter_gz_plugins/spring_force.hh"

namespace quadcopter_gz_plugins
{
class SpringDamperLegSystem final : public gz::sim::System,
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
    gz::sim::Joint joint{gz::sim::kNullEntity};
    gz::sim::Entity footLink{gz::sim::kNullEntity};
    double k = 174.0;
    double f_c = 0.442;
    double l_p = 0.0197;
    double velEps = 0.02;
    std::string topic = "/model/quadcopter/leg_state";
    gz::transport::Node node;
    gz::transport::Node::Publisher pub;
  };

  private: std::unique_ptr<Data> data_{std::make_unique<Data>()};
};

void SpringDamperLegSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
{
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "SpringDamperLegSystem requires a model entity.\n";
    return;
  }

  if (_sdf->HasElement("leg_joint"))
  {
    data_->joint = gz::sim::Joint(
        model.JointByName(_ecm, _sdf->Get<std::string>("leg_joint")));
  }
  if (_sdf->HasElement("foot_link"))
  {
    data_->footLink =
        model.LinkByName(_ecm, _sdf->Get<std::string>("foot_link"));
  }
  if (_sdf->HasElement("k"))
    data_->k = _sdf->Get<double>("k");
  if (_sdf->HasElement("f_c"))
    data_->f_c = _sdf->Get<double>("f_c");
  if (_sdf->HasElement("l_p"))
    data_->l_p = _sdf->Get<double>("l_p");
  if (_sdf->HasElement("vel_eps"))
    data_->velEps = _sdf->Get<double>("vel_eps");
  if (_sdf->HasElement("topic"))
    data_->topic = _sdf->Get<std::string>("topic");

  if (!data_->joint.Valid(_ecm))
  {
    gzerr << "SpringDamperLegSystem: leg joint not found.\n";
    return;
  }
  if (data_->footLink == gz::sim::kNullEntity)
  {
    gzerr << "SpringDamperLegSystem: foot link not found.\n";
    return;
  }

  data_->pub = data_->node.Advertise<gz::msgs::Float_V>(data_->topic);
  gzmsg << "SpringDamperLegSystem configured: k=" << data_->k
        << " N/m, f_c=" << data_->f_c << " N, l_p=" << data_->l_p
        << " m, topic=" << data_->topic << std::endl;
}

void SpringDamperLegSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  const auto *position =
      _ecm.Component<gz::sim::components::JointPosition>(data_->joint.Entity());
  const auto *velocity =
      _ecm.Component<gz::sim::components::JointVelocity>(data_->joint.Entity());
  if (position == nullptr || velocity == nullptr ||
      position->Data().empty() || velocity->Data().empty())
  {
    return;
  }

  const double q = position->Data()[0];
  const double qDot = velocity->Data()[0];
  const double force =
      SpringDamperForce(q, qDot, data_->k, data_->f_c, data_->l_p,
                        data_->velEps);
  data_->joint.SetForce(_ecm, {force});

  gz::sim::Link foot(data_->footLink);
  const auto worldPose = foot.WorldPose(_ecm);
  if (!worldPose)
    return;

  const double simTime =
      std::chrono::duration<double>(_info.simTime).count();
  gz::msgs::Float_V msg;
  msg.mutable_data()->Resize(5, 0.0);
  msg.set_data(0, static_cast<float>(simTime));
  msg.set_data(1, static_cast<float>(q));
  msg.set_data(2, static_cast<float>(qDot));
  msg.set_data(3, static_cast<float>(force));
  msg.set_data(4, static_cast<float>(worldPose->Z()));
  data_->pub.Publish(msg);
}
}  // namespace quadcopter_gz_plugins

GZ_ADD_PLUGIN(
    quadcopter_gz_plugins::SpringDamperLegSystem,
    gz::sim::System,
    quadcopter_gz_plugins::SpringDamperLegSystem::ISystemConfigure,
    quadcopter_gz_plugins::SpringDamperLegSystem::ISystemPreUpdate)
