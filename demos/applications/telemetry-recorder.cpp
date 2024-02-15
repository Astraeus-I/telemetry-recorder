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

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <libhal-lpc40/output_pin.hpp>
#include <libhal-lpc40/spi.hpp>
#include <libhal-util/spi.hpp>

#include "../hardware_map.hpp"
#include <libhal-xbee/xbee.hpp>
#include <telemetry-recorder/telemetry-recorder.hpp>

#define M_PI 3.14159265358979323846

float computeHeading(float x, float y, float offset = 0.0)
{
  float angle = 360 - (atan2(y, x) * (180.0 / M_PI));
  angle += offset;  // Apply offset
  if (angle < 0) {
    angle += 360;
  } else if (angle >= 360) {
    angle -= 360;
  }
  return angle;
}

hal::status application(hardware_map& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& xbee = *p_map.xbee;
  auto& gps = *p_map.gps;
  auto& i2c = *p_map.i2c;

  hal::print(console,
             "\n\nTelemetry Recorder Starting. May take a few seconds...\n\n");

  (void)hal::delay(clock, 100ms);
  auto neoGPS = HAL_CHECK(hal::neo::neo_m9n::create(gps));
  (void)hal::delay(clock, 100ms);
  auto xbee_module = HAL_CHECK(hal::xbee::xbee_radio::create(xbee, clock));
  (void)hal::delay(clock, 100ms);
  auto mpl_device = HAL_CHECK(hal::mpl::mpl3115a2::create(
    i2c,
    hal::mpl::mpl3115a2::mpl_os_rate::os64));  // change barometer sampling rate
  (void)hal::delay(clock, 100ms);
  auto icm_device = HAL_CHECK(hal::icm::icm20948::create(i2c));
  (void)hal::delay(clock, 100ms);

  auto telemetry_recorder =
    HAL_CHECK(hal::telemetry_recorder::telemetry_recorder::create(
      icm_device, neoGPS, mpl_device));

  icm_device.init_mag();
  (void)hal::delay(clock, 100ms);
  icm_device.auto_offsets();

  auto alt_offset = 80;
  mpl_device.set_altitude_offset(alt_offset);  // set initial altitude offset
  float slp = 101325;                          // Default is 101325 Pa
  mpl_device.set_sea_pressure(slp);

  // ======= Uncomment if you have an XBEE radio connected =======

  // xbee_module.configure_xbee("C", "2015");  // Setting radio to channel C,
  // and PANID 2015

  // hal::print(console, "\nTelemetry Configuration Complete...\n\n");

  while (true) {
    hal::print(console, "\n=================== Data ===================\n");
    auto telemetry_recorder_data = HAL_CHECK(telemetry_recorder.record());

    // ======= Uncomment below to check for full GPS lock =======

    // if (telemetry_recorder_data.gps_locked == false) {
    //   hal::print(console, "!!!GPS not fully locked!!!\n");
    // } else {
    //   hal::print(console, "GPS locked\n");
    //   auto gps_offset =
    //     HAL_CHECK(telemetry_recorder.gps_baro_altitude_offset());
    //   mpl_device.set_altitude_offset(gps_offset);
    // }

    char telem_data[512];
    snprintf(telem_data,
             sizeof(telem_data),
             "G-Accel Values: x = %fg, y = %fg, z = %fg\n"
             "Gyro Values: x = %f, y = %f, z = %f\n"
             "Mag Values: x = %f, y = %f, z = %f\n"
             "IMU Temperature: %f°C\n"
             "Barometer Temperature: % f°C\n"
             "Measured Pressure: %fPa\n"
             "Barometer Measured Altitude: %fm\n"
             "\n\nLatitude: %f\n"
             "Longitude: %f\n"
             "Number of satellites seen: %d\n"
             "Altitude: %f meters\n"
             "Time: %f\n",
             telemetry_recorder_data.accel_x,
             telemetry_recorder_data.accel_y,
             telemetry_recorder_data.accel_z,
             telemetry_recorder_data.gyro_x,
             telemetry_recorder_data.gyro_y,
             telemetry_recorder_data.gyro_z,
             telemetry_recorder_data.mag_x,
             telemetry_recorder_data.mag_y,
             telemetry_recorder_data.mag_z,
             telemetry_recorder_data.imu_temp,
             telemetry_recorder_data.baro_temp,
             telemetry_recorder_data.baro_pressure,
             telemetry_recorder_data.baro_altitude,
             telemetry_recorder_data.gps_lat,
             telemetry_recorder_data.gps_long,
             telemetry_recorder_data.gps_sats,
             telemetry_recorder_data.gps_alt,
             telemetry_recorder_data.gps_time);

    hal::print<512>(console, telem_data);
    float heading = computeHeading(
      telemetry_recorder_data.mag_x, telemetry_recorder_data.mag_y, 0.0);
    hal::print<128>(console, "\n\nHeading: %f°", heading);

    hal::print(console, "\n\n============================================\n\n");

    // ======= Uncomment below if you have an XBEE radio connected =======

    // hal::delay(clock, 1000ms);
    // auto recieved_data = HAL_CHECK(xbee_module.read());
    // hal::print(console,
    //            "\n=================== RECIEVED DATA ===================\n");
    // hal::print(console, recieved_data);

    // std::string_view message = "Hello from the other side";
    // xbee_module.write(hal::as_bytes(message));
    // hal::print(console,
    //            "\n=================== TRANSMITTED DATA ===================\n");
    // hal::print(console, message);

  }

  return hal::success();
}
