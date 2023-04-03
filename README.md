# Lightweight-TEE-Controller
This repository contains the code for the paper "Trusted Execution of Periodic Tasks for Embedded Systems".
The paper was accepted for publication at IFAC world congress 2023, Yokohama.

## Repository structure
The folder `secure_ball_and_beam` contains the controller implemented in a TEE.
In the folder `non_secure_ball_and_beam` the controller implemented in the REE can be found.
These folders are external Zephyr-OS applications.

This repository also contains the `JitterTime` analysis as well as the experimental data in the julia folder.

## Authors
* Martin Gunnarsson: `[firstname.per.lastname]@gmail.com`
* Nils Vreman: `[firstname.lastname]@control.lth.se`
* Martina Maggio: `[lastname]@cs.uni-saarland.de`
