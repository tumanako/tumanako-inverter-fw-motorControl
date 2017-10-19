#!/bin/sh

openocd -f /usr/local/share/openocd/scripts/interface/olimex-arm-usb-ocd-h.cfg -f /usr/local/share/openocd/scripts/board/olimex_stm32_h103.cfg -f flash.openocd
