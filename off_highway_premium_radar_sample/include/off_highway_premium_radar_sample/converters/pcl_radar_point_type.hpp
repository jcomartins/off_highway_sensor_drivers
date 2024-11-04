// Copyright 2023 Robert Bosch GmbH and its subsidiaries
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

#pragma once

#include "off_highway_premium_radar_sample/pdu_definitions.hpp"

#include "pcl/pcl_macros.h"
#include "pcl/point_types.h"
#include "pcl/register_point_struct.h"

namespace off_highway_premium_radar_sample
{
/**
 * \brief Location as PCL point
 */
struct EIGEN_ALIGN16 PclPointLocation
{
  float x{.0};
  float y{.0};
  float z{.0};
  uint8_t intensity{0};
  uint8_t return_type{0};
  uint16_t channel{0};

  /**
   * \brief Construct a new PclPointLocation object from a location data packet
   *
   * \param l Location data packet input
   */
  explicit PclPointLocation(const LocData_Packet_i_j & l)
  {
    const float & phi = l.LocData_EleAng_i_j;
    const float & theta = l.LocData_AziAng_i_j;
    float cos_phi = cos(phi);
    float sin_theta = sin(theta);

    x = l.LocData_RadDist_i_j * sqrt(cos_phi * cos_phi - sin_theta * sin_theta);
    y = l.LocData_RadDist_i_j * sin_theta;
    z = l.LocData_RadDist_i_j * sin(phi);
    intensity = l.LocData_Snr_i_j;
    // return_type = 0 
    channel = int(phi*28.662420382+8);
  }
  PclPointLocation() {}
};  // SSE padding

}  // namespace off_highway_premium_radar_sample

/* *INDENT-OFF* */
POINT_CLOUD_REGISTER_POINT_STRUCT(off_highway_premium_radar_sample::PclPointLocation,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, intensity,intensity)
                                  (uint8_t, return_type,return_type)
                                  (uint16_t, channel, channel))
/* *INDENT-ON* */
