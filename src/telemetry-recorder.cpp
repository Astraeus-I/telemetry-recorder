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

#include "telemetry-recorder/telemetry-recorder.hpp"

namespace hal::telemetry_recorder {

result<telemetry_recorder> telemetry_recorder::create(hal::icm::icm20948& p_imu, hal::neo::neo_GPS& p_gps, hal::mpl::mpl3115a2& p_baro, hal::microsd::microsd_card& p_microsd, hal::xbee::xbee_radio& p_xbee)
{
  return telemetry_recorder(p_imu, p_gps, p_baro, p_microsd, p_xbee);
}

hal::result<telemetry_data> telemetry_recorder::record()
{
    auto accel = m_icm.read_acceleration();
    auto gyro = m_icm.read_gyroscope();
    auto imu_temp = m_icm.read_temperature();

    auto gps = m_gps.read();

    auto baro_temp = m_baro.read_temperature().temperature;
    auto pressure = m_baro.read_pressure().pressure;
    auto altitude = m_baro.read_altitude().altitude;


    m_data.accel_x = accel.x;
    m_data.accel_y = accel.y;
    m_data.accel_z = accel.z;
    m_data.gyro_x = gyro.x;
    m_data.gyro_y = gyro.y;
    m_data.gyro_z = gyro.z;
    m_data.imu_temp = imu_temp;

    m_data.gps_time = gps.time;
    m_data.gps_lat = gps.latitude;
    m_data.gps_long = gps.longitude;
    m_data.gps_sats = gps.satellites_used;
    m_data.gps_alt = gps.altitude;

    m_data.baro_temp = baro_temp;
    m_data.baro_pressure = pressure;
    m_data.baro_altitude = altitude;

    
    hal::result<telemetry_data>{m_data};
}


hal::result<std::span<hal::byte>> telemetry_recorder::recieve()
{
    auto data = HAL_CHECK(m_xbee.read());
    return data;
}

hal::status<void> telemetry_recorder::transmit(std::string_view message)
{
    m_xbee.write(hal::as_bytes(message));
    return hal::success();
}

hal::status<void> telemetry_recorder::store(std::string_view message)
{
    m_microsd.write(hal::as_bytes(message));
    return hal::success();
}


}  // namespace hal::elemetry_recorder
