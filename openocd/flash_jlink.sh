#!/bin/bash

openocd -f interface/jlink.cfg -f stm32f4x.cfg -f flash.cfg -c "flash_stm32f4 $1"
