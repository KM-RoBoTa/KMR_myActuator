# Introduction {#mainpage}
[TOC]

This library provides an easy way to use Dynamixel motors in a project, in **protocol 2**. \n 
Its main strength is the abstraction of adresses: the user does not need to concern themselves with manually getting or inputting
addresses from the motors' control tables, this is done automatically by the library. Similarly, in case of a handler (reader or writer) needing
to work through indirect addresses, the library automatically takes care of it. 


## Working concept
Instead of having to manually create Dynamixel's GroupSyncWrite and GroupSyncRead objects and inputting the required addresses and byte lengths of their control fields, everything is done through this library's KMR::dxl::Reader and KMR::dxl::Writer classes.

The KMR::dxl::BaseRobot class provides a basis to be inherited by a Robot class specific to the project. BaseRobot contains general-use functions such as enabling/disabling motors and resetting them in multiturn control mode. \n
The user needs to create custom Reader and Writer objects in their child Robot class, handling the fields they need, as well as their reading/writing functions. Those functions are very straightforward to implement.

This library is written in C++. \n 
It uses SI units, the only exception being temperature expressed in °C instead of Kelvins.


## Links

- Repository: https://github.com/KM-RoBoTa/KMR_dxl
- How to [setup](#setup)
- How to [use](#how-to-use)

## About

KM-RoBoTa sarl's KMR_dxl library to facilitate Dynamixel control.

### Authors
Library written by Katarina Lichardova: katarina.lichardova@km-robota.com

based on code from:
- Laura Paez: laura.paez@km-robota.com
- Kamilo Melo: kamilo.melo@km-robota.com

### Copyright
Copyright 2021-2023, Laura Paez Coy and Kamilo Melo. \n
This code is under MIT licence: https://opensource.org/licenses/MIT


## Supported models

- MX-28
- MX-64
- MX-106
- XH540-W150
- XH540-W270
- XM540-W150
- XM540-W270
- XM430-W350
- XW430-T200
- XW540-T260
- XW540-T140
