# eModbus

This is a WIP library to facilitate communication of devices via Modbus RTU and TCP protocols.

## Why?

I started it, because I had multiple modbus devices and ended up reimplementing much of the code. Tried multiple libraries, but most are C only. I really liked Mazurel/ModbusC++ library, but then I needed to create a fast embedded RTU to TCP converter and I needed something that is zerocopy.

## What I want to achieve:

* zero cost abstractions
* low resources footprint
* modular
* easy to use

## Contents:

* **ModbusFrame.hpp** - a header only parser and builder for modbus frames. It consists of eModbus::FrameView and eModbus::Frame, where View is nonowning, and Frame is owning. Allows for fast and on the spot (zerocopy) edit of all the fields of modbus frame. Allows to build custom modbus drivers.
* **IStreamDevice.hpp** - Interface that needs to be implemented to use more advanced modbus drivers.
* **ModbusMasterBase.hpp** - the simplest modbus master driver. Allows to send and receive modbus frames via IStreamDevice
* **ModbusRegisterBuffer.hpp** - utility that simplify access to data coded in the registers. Allows to convert the registers to custom data such as (u)int8/16/32, ascii, byte buffers or user defined.
* **ModbusMasterTag.hpp** - modbus master driver that's tag based. Define a repository of tags with register types and numbers, and read them efficiently without a thought about modbus internals.

## Use Cases:
TODO
