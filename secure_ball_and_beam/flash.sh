#!/bin/bash
(cd build; nrfjprog -f NRF53 --program tfm/bin/bl2.hex --sectorerase)
(cd build; ninja flash)
