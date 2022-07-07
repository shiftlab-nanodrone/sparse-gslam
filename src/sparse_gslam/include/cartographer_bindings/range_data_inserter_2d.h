/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <utility>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer_bindings/range_data_2d.h"

namespace cartographer {
namespace mapping {

class MultirangeDataInserter {
 public:
  explicit MultirangeDataInserter(double hit_prob, double miss_prob);

  MultirangeDataInserter(
      const MultirangeDataInserter&) = delete;
  MultirangeDataInserter& operator=(
      const MultirangeDataInserter&) = delete;

  // Inserts 'range_data' into 'probability_grid'.
  void Insert(const sensor::RangeData2D& range_data, GridInterface* grid) const;

 private:
  const std::vector<uint16> hit_table_;
  const std::vector<uint16> miss_table_;
};

}  // namespace mapping
}  // namespace cartographer
