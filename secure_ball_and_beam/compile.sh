#!/bin/bash

(cd build; cmake -GNinja -DBOARD=nrf5340dk_nrf5340_cpuapp_ns ..)
(cd build; ninja)
