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

#include "cartographer_bindings/range_data_inserter_2d.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer_bindings/ray_to_pixel_mask.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

// Factor for subpixel accuracy of start and end point for ray casts.
constexpr int kSubpixelScale = 1000;

void GrowAsNeeded(const sensor::RangeData2D& range_data,
                  ProbabilityGrid* const probability_grid) {
    Eigen::AlignedBox2f bounding_box;
    // Padding around bounding box to avoid numerical issues at cell boundaries.
    constexpr float kPadding = 1e-6f;
    for (const auto& point : range_data.points_) {
        bounding_box.extend(point);
    }
    probability_grid->GrowLimits(bounding_box.min() -
                                 kPadding * Eigen::Vector2f::Ones());
    probability_grid->GrowLimits(bounding_box.max() +
                                 kPadding * Eigen::Vector2f::Ones());
}

}  // namespace

MultirangeDataInserter::MultirangeDataInserter(double hit_prob, double miss_prob)
    : hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(hit_prob))),
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(miss_prob))) {}

void MultirangeDataInserter::Insert(
    const sensor::RangeData2D& range_data, GridInterface* const grid) const {
    ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);
    CHECK(probability_grid != nullptr);

    GrowAsNeeded(range_data, probability_grid);

    const MapLimits& limits = probability_grid->limits();
    const double superscaled_resolution = limits.resolution() / kSubpixelScale;
    const MapLimits superscaled_limits(
        superscaled_resolution, limits.max(),
        CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                   limits.cell_limits().num_y_cells * kSubpixelScale));

    int i = 0;
    for (const auto& meta_data : range_data.meta_data_) {
        const Eigen::Array2i begin = superscaled_limits.GetCellIndex(meta_data.origin);

        for (; i < meta_data.return_end; i++) {
            auto end = superscaled_limits.GetCellIndex(range_data.points_[i]);
            probability_grid->ApplyLookupTable(end / kSubpixelScale, hit_table_);

            // trace the miss space
            std::vector<Eigen::Array2i> ray = RayToPixelMask(begin, end, kSubpixelScale);
            for (const Eigen::Array2i& cell_index : ray) {
                probability_grid->ApplyLookupTable(cell_index, miss_table_);
            }
        }
        for (; i < meta_data.end; i++) {
            auto end = superscaled_limits.GetCellIndex(range_data.points_[i]);
            std::vector<Eigen::Array2i> ray = RayToPixelMask(begin, end, kSubpixelScale);
            for (const Eigen::Array2i& cell_index : ray) {
                probability_grid->ApplyLookupTable(cell_index, miss_table_);
            }
        }
        probability_grid->FinishUpdate();
    }
    // By not finishing the update after hits are inserted, we give hits priority
    // (i.e. no hits will be ignored because of a miss in the same cell).
}

}  // namespace mapping
}  // namespace cartographer
