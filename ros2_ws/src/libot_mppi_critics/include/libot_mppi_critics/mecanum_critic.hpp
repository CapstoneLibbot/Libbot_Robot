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

#ifndef LIBOT_MPPI_CRITICS__MECANUM_CRITIC_HPP_
#define LIBOT_MPPI_CRITICS__MECANUM_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"

namespace mppi::critics
{

class MecanumCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float vx_max_{0};
  float vy_max_{0};
  float wz_max_{0};
};

}  // namespace mppi::critics

#endif  // LIBOT_MPPI_CRITICS__MECANUM_CRITIC_HPP_
