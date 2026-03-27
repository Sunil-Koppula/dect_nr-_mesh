#
# Copyright (c) 2024 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_dect)

target_sources(app PRIVATE
  src/main.c
  src/radio.c
  src/mesh.c
  src/queue.c
  src/nvs_store.c
  src/identity.c
  src/large_data.c
  src/flash_store.c
  src/ota_store.c
  src/ota.c
  src/at_cmd.c
  src/display.c
  src/gateway/gateway.c
  src/anchor/anchor.c
  src/sensor/sensor.c
  src/psram.c
)
