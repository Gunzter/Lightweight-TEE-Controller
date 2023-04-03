cmake_minimum_required(VERSION 3.20.0)
#set(BOARD qemu_x86) change to nRF3540.....

find_package(Zephyr)
project(uart_test)

target_sources(app PRIVATE 
  src/main.c
  src/serial2002.c
)


