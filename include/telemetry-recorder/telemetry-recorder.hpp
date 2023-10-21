// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <libhal/functional.hpp>
#include <libhal/serial.hpp>

#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

#include <libhal-icm/icm20948.hpp>
#include <libhal-microsd/microsd.hpp>
#include <libhal-mpl/mpl3115a2.hpp>
#include <libhal-neo/neo-m9n.hpp>
#include <libhal-xbee/xbee.hpp>

namespace hal::telemetry_recorder {
class telemetry_recorder
{

public:
  struct telemetry_data
  {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float imu_temp;
    float mag_x;
    float mag_y;
    float mag_z;
    bool gps_locked;
    float gps_time;
    float gps_lat;
    float gps_long;
    int gps_sats;
    float gps_alt;
    float baro_temp;
    float baro_pressure;
    float baro_altitude;
  };

  [[nodiscard]] static result<telemetry_recorder> create(
    hal::icm::icm20948& p_imu,
    hal::neo::neo_m9n& p_gps,
    hal::mpl::mpl3115a2& p_baro,
    hal::microsd::microsd_card& p_microsd,
    hal::xbee::xbee_radio& p_xbee);

  hal::result<telemetry_data> record();
  hal::result<float> gps_baro_altitude_offset();
  hal::result<std::span<hal::byte>> recieve();
  hal::status transmit(std::string_view message);
  hal::status store(std::string_view message);

private:
  hal::icm::icm20948* m_icm;
  hal::neo::neo_m9n* m_gps;
  hal::mpl::mpl3115a2* m_mpl;
  hal::microsd::microsd_card* m_microsd;
  hal::xbee::xbee_radio* m_xbee;

  explicit telemetry_recorder(hal::icm::icm20948& p_imu,
                              hal::neo::neo_m9n& p_gps,
                              hal::mpl::mpl3115a2& p_baro,
                              hal::microsd::microsd_card& p_microsd,
                              hal::xbee::xbee_radio& p_xbee)
    : m_icm(&p_imu)
    , m_gps(&p_gps)
    , m_mpl(&p_baro)
    , m_microsd(&p_microsd)
    , m_xbee(&p_xbee)
  {
  }

  telemetry_data m_data;
};
}  // namespace hal::telemetry_recorder
