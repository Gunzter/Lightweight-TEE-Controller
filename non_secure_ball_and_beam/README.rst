This is a test application to test the inverted pendulum without the TEE.
If this application does not build on your system make sure that your
ZEPHYR_BASE enviorment variable is set. It should be set to
zephyrproject/zephyr

Connect MAX3232
RX <-> P1.01
Tx <-> P1.02
VCC <-> VCC
GND <-> GND

Build with:
west build -p auto -b nrf5340dk_nrf5340_cpuapp
Flash with:
west flash

Connect to process with:
minicom -D /dev/ttyACM2
