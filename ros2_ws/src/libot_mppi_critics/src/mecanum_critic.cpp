// Copyright (c) 2026 Duatic AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "libot_mppi_critics/mecanum_critic.hpp"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace
{

float safeInverseAbs(const float value)
{
  constexpr float min_abs = 1e-6f;
  return 1.0f / std::max(std::abs(value), min_abs);
}

}  // namespace

namespace mppi::critics
{

void MecanumCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0f);
  RCLCPP_INFO(
    logger_, "MecanumCritic instantiated with %d power and %f weight.",
    power_, weight_);

  getParam(vx_max_, "vx_max", 0.5f);
  getParam(vy_max_, "vy_max", 0.5f);
  getParam(wz_max_, "wz_max", 1.7f);
}

void MecanumCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_ || !data.motion_model->isHolonomic()) {
    return;
  }

  auto & vx = data.state.vx;
  auto & vy = data.state.vy;
  auto & wz = data.state.wz;
  auto violation = xt::maximum(
    xt::fabs(vx) * safeInverseAbs(vx_max_) +
    xt::fabs(vy) * safeInverseAbs(vy_max_) +
    xt::fabs(wz) * safeInverseAbs(wz_max_) - 1.0f,
    0.0f);
  auto cost = xt::eval(
    xt::sum(violation, {1}, immediate) * data.model_dt * weight_ *
    std::max(vx_max_, vy_max_));

  if (power_ > 1u) {
    data.costs += xt::pow(cost, power_);
  } else {
    data.costs += cost;
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::MecanumCritic,
  mppi::critics::CriticFunction)
