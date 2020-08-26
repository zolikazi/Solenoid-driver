#!/bin/bash

openocd -f stm32f4discovery.cfg -f flash.cfg -c "flash_stm32f4 $1"
