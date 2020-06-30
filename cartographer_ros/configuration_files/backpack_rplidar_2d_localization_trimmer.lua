-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "backpack_rplidar_2d.lua"

TRAJECTORY_BUILDER.pure_localization = false
TRAJECTORY_BUILDER.overlapping_submaps_trimmer_2d = {
  -- same cell, trim after the latest 'fresh_submaps_count'
  fresh_submaps_count = 2,
  -- submap convers area < min_covered_area might be trimmed
  min_covered_area = 4,
  -- submap not be trimmed > min_added_submaps_count to continue trimming
  min_added_submaps_count = 1,
}
-- POSE_GRAPH.optimize_every_n_nodes = 20

return options
