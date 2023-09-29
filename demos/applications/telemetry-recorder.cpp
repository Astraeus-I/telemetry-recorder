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
#include <libhal/libhal-icm.hpp>
#include <libhal/libhal-microsd.hpp>
#include <libhal/libhal-mpl.hpp>
#include <libhal/libhal-neo.hpp>
#include <libhal/libhal-xbee.hpp>

#include "../hardware_map.hpp"

hal::status application(hardware_map& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& gps = *p_map.gps;
  auto& xbee = *p_map.xbee;
  auto& i2c = *p_map.i2c;

  auto spi2 = HAL_CHECK(hal::lpc40::spi::get(2));
  auto chip_select = HAL_CHECK(hal::lpc40::output_pin::get(1, 8));

  auto micro_sd =
    HAL_CHECK(hal::microsd::microsd_card::create(spi2, chip_select));
  auto neoGPS = HAL_CHECK(hal::neo::neo_GPS::create(gps));
  auto xbee_module = HAL_CHECK(hal::xbee::xbee_radio::create(xbee));
  auto mpl_device = HAL_CHECK(hal::mpl::mpl3115a2::create(i2c));
  auto icm_device = HAL_CHECK(hal::icm::icm20948::create(i2c, 0x69));
  hal::delay(clock, 500ms);

  xbee_module.configure_xbee("C", "2015"); // Channel C, PANID 2015

  auto telemetry_recorder =
    HAL_CHECK(hal::telemetry_recorder::telemetry_recorder::create(
      icm_device, neoGPS, mpl_device, micro_sd, xbee_module));
  hal::delay(clock, 500ms);

  hal::print(console, "Demo Telemetry Recorder Starting...\n\n");

  while (true) {
    hal::delay(clock, 500ms);
    hal::print(console, "\n=================== Data ===================\n");

    auto data = hal::print<512>(console,
                    "G-Accel Values: x = %fg, y = %fg, z = %fg\n
                    Gyro Values: x = %f, y = %f, z = %f\n 
                    IMU Temperature: %f°C\n
                    Barometer Temperature: % f°C\n
                    Measured Pressure: %fPa\n
                    Barometer Measured Altitude: %fm\n
                    Latitude: %f\n
                    Longitude: %f\n
                    Number of satellites seen: %d\n
                    Altitude: %f meters\n
                    Time: %f\n",
                      telemetry_recorder.accel_x,
                      telemetry_recorder.accel_y,
                      telemetry_recorder.accel_z,
                      telemetry_recorder.gyro_x,
                      telemetry_recorder.gyro_y,
                      telemetry_recorder.gyro_z,
                      telemetry_recorder.imu_temp,
                      telemetry_recorder.baro_temp,
                      telemetry_recorder.baro_pressure,
                      telemetry_recorder.baro_altitude,
                      telemetry_recorder.gps_lat,
                      telemetry_recorder.gps_long,
                      telemetry_recorder.gps_sats,
                      telemetry_recorder.gps_alt,
                      telemetry_recorder.gps_time);

    hal::print(console, "============================================\n\n");
  }

  hal::print(console, "Transmitting Data to Ground Station...\n\n");
  telemetry_recorder.transmit("Here is some data!\n");
  telemetry_recorder.transmit(data);

  hal::print(console, "Storing Data to SD Card...\n\n");
  telemetry_recorder.store(data);
  hal::delay(clock, 500ms);

  hal::print(console, "Recieveing Data from Ground Station...\n\n");
  auto recieved_data = HAL_CHECK(telemetry_recorder.recieve());
  hal::print(console, "\n=================== RECIEVED DATA ===================\n");
  hal::print(console, recieved_data);
  hal::print(console, "======================================================\n\n");

  return hal::success();
}
