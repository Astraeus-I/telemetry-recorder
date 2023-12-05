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

  auto spi2 = HAL_CHECK(hal::lpc40::spi::get(2));
  auto chip_select = HAL_CHECK(hal::lpc40::output_pin::get(1, 8));

  hal::print(console,
             "\n\nTelemetry Recorder Starting. May take a few seconds...\n\n");

  // Device initialization
  auto micro_sd =
    HAL_CHECK(hal::microsd::microsd_card::create(spi2, chip_select));
  (void)hal::delay(clock, 100ms);
  auto neoGPS = HAL_CHECK(hal::neo::neo_m9n::create(gps));
  (void)hal::delay(clock, 100ms);
  auto xbee_module = HAL_CHECK(hal::xbee::xbee_radio::create(xbee, clock));
  (void)hal::delay(clock, 100ms);
  auto mpl_device = HAL_CHECK(hal::mpl::mpl3115a2::create(
    i2c,
    hal::mpl::mpl3115a2::mpl_os_rate::os32));  // change barometer sampling rate
  (void)hal::delay(clock, 100ms);
  auto icm_device = HAL_CHECK(hal::icm::icm20948::create(i2c));
  (void)hal::delay(clock, 100ms);

  auto telemetry_recorder =
    HAL_CHECK(hal::telemetry_recorder::telemetry_recorder::create(
      icm_device, neoGPS, mpl_device, micro_sd, xbee_module));

  icm_device.init_mag();
  (void)hal::delay(clock, 100ms);
  icm_device.auto_offsets();

  auto alt_offset = 80;
  mpl_device.set_altitude_offset(alt_offset);  // set initial altitude offset
  float slp = 101325;                          // Default is 101325 Pa
  mpl_device.set_sea_pressure(slp);

  xbee_module.configure_xbee(
    "C", "2015");  // Setting radio to channel C, and PANID 2015

  hal::print(console, "\nTelemetry Configuration Complete...\n\n");

  while (true) {
    hal::print(console, "\n=================== Data ===================\n");
    auto telemetry_recorder_data = HAL_CHECK(telemetry_recorder.record());

    // auto gps_offset = HAL_CHECK(telemetry_recorder.gps_baro_altitude_offset());
    // mpl_device.set_altitude_offset(gps_offset);

    char telem_data[1024];
    snprintf(telem_data,
             sizeof(telem_data),
             "{"
             "\"accel_x\": %f, "
             "\"accel_y\": %f, "
             "\"accel_z\": %f, "
             "\"gyro_x\": %f, "
             "\"gyro_y\": %f, "
             "\"gyro_z\": %f, "
             "\"magnetometer_x\": %f, "
             "\"magnetometer_y\": %f, "
             "\"magnetometer_z\": %f, "
             "\"heading\": %f, "
             "\"imu_temperature\": %f, "
             "\"barometer_temperature\": %f, "
             "\"barometer_pressure\": %f, "
             "\"barometer_altitude\": %f, "
             "\"gps_latitude\": %f, "
             "\"gps_longitude\": %f, "
             "\"gps_satellite_count\": %d, "
             "\"gps_altitude\": %f, "
             "\"gps_time\": %f"
             "}",
             telemetry_recorder_data.accel_x,
             telemetry_recorder_data.accel_y,
             telemetry_recorder_data.accel_z,
             telemetry_recorder_data.gyro_x,
             telemetry_recorder_data.gyro_y,
             telemetry_recorder_data.gyro_z,
             telemetry_recorder_data.mag_x,
             telemetry_recorder_data.mag_y,
             telemetry_recorder_data.mag_z,
             computeHeading(telemetry_recorder_data.mag_x,
                            telemetry_recorder_data.mag_y,
                            0.0),
             telemetry_recorder_data.imu_temp,
             telemetry_recorder_data.baro_temp,
             telemetry_recorder_data.baro_pressure,
             42.0,
             telemetry_recorder_data.gps_lat,
             telemetry_recorder_data.gps_long,
             telemetry_recorder_data.gps_sats,
             telemetry_recorder_data.gps_alt,
             telemetry_recorder_data.gps_time);

    hal::print<512>(console, telem_data);
    hal::print(console, "\n\n============================================\n\n");
    hal::print(console, "Transmitting Data to Ground Station...\n\n");

    telemetry_recorder.transmit(telem_data);

    hal::print(console, "Recieveing Data from Ground Station...\n\n");
    auto recieved_data1 = HAL_CHECK(telemetry_recorder.recieve());
    hal::print(console,
               "\n=================== RECIEVED DATA ===================\n");
    hal::print(console, recieved_data1);
    hal::print(console,
               "\n======================================================\n\n");
  }


  return hal::success();
}