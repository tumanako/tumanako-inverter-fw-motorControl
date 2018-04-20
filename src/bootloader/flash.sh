#!/bin/sh

openocd -f interface/ftdi/olimex-arm-usb-ocd-h.cfg -f board/olimex_stm32_h103.cfg -f flash.openocd
