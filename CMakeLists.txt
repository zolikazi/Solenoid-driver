project(solenoid_driver C ASM)
cmake_minimum_required(VERSION 2.8)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_PROCESSOR arm)

SET(CMAKE_C_COMPILER   arm-none-eabi-gcc)
SET(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

add_definitions(-DSERIAL_DEBUG)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -mcpu=cortex-m4 -mlittle-endian -mthumb -Os -DSTM32F407xx --specs=nosys.specs")
SET(CMAKE_EXE_LINKER_FLAGS "-T../platform/STM32F407_FLASH.ld")

##set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -mcpu=cortex-m4 -mlittle-endian -mthumb -Os -DSTM32F407xx --specs=rdimon.specs")
##SET(CMAKE_EXE_LINKER_FLAGS "-T../platform/STM32F407_FLASH.ld -lc -lrdimon")

SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)

#set(STMCUBE_PATH "/home/eistmol/Documents/STM32/STM32Cube_FW_F4_V1.14.0")
#********************* Library *****************************
add_executable(${PROJECT_NAME}

src/main.c
#src/spi_pov.c
src/serial_debug.c
src/tinyprintf.c
src/system_stm32f4xx.c
src/stm32f4xx_it.c
src/netconf.c
src/stm32f4x7_eth_bsp.c
src/packetizer.c
src/ptpsync.c

	#***          <GroupName>STM32F4 Discovery</GroupName>
#	../../../../Utilities/STM32F4-Discovery/stm32f4_discovery_lcd.c
libraries/STM32F4-Discovery/stm32f4_discovery.c

libraries/STM32F4xx_StdPeriph_Driver/src/misc.c
libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c
libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c
#	../../../../Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.c
libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c
libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c
#libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c
libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c
libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c
libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c

#***          <GroupName>STM32F4x7_ETH_Driver</GroupName>
libraries/STM32F4x7_ETH_Driver/src/stm32f4x7_eth.c
#***          <GroupName>LWIP</GroupName>
libraries/lwip-1.4.1/port/STM32F4x7/arch/ethernetif.c
#libraries/lwip-1.4.1/port/STM32F4x7/arch/sys_arch.c
libraries/lwip-1.4.1/src/api/api_lib.c
libraries/lwip-1.4.1/src/api/api_msg.c
libraries/lwip-1.4.1/src/api/err.c
libraries/lwip-1.4.1/src/api/netbuf.c
libraries/lwip-1.4.1/src/api/netdb.c
libraries/lwip-1.4.1/src/api/netifapi.c
libraries/lwip-1.4.1/src/api/sockets.c
libraries/lwip-1.4.1/src/api/tcpip.c
libraries/lwip-1.4.1/src/core/dhcp.c
libraries/lwip-1.4.1/src/core/def.c
libraries/lwip-1.4.1/src/core/dns.c
libraries/lwip-1.4.1/src/core/init.c
libraries/lwip-1.4.1/src/core/mem.c
libraries/lwip-1.4.1/src/core/memp.c
libraries/lwip-1.4.1/src/core/netif.c
libraries/lwip-1.4.1/src/core/packet.c
libraries/lwip-1.4.1/src/core/pbuf.c
libraries/lwip-1.4.1/src/core/raw.c
libraries/lwip-1.4.1/src/core/stats.c
libraries/lwip-1.4.1/src/core/sys.c
libraries/lwip-1.4.1/src/core/tcp.c
libraries/lwip-1.4.1/src/core/tcp_in.c
libraries/lwip-1.4.1/src/core/tcp_out.c
libraries/lwip-1.4.1/src/core/timers.c
libraries/lwip-1.4.1/src/core/udp.c
libraries/lwip-1.4.1/src/core/ipv4/autoip.c
libraries/lwip-1.4.1/src/core/ipv4/icmp.c
libraries/lwip-1.4.1/src/core/ipv4/igmp.c
libraries/lwip-1.4.1/src/core/ipv4/inet.c
libraries/lwip-1.4.1/src/core/ipv4/inet_chksum.c
libraries/lwip-1.4.1/src/core/ipv4/ip.c
libraries/lwip-1.4.1/src/core/ipv4/ip_addr.c
libraries/lwip-1.4.1/src/core/ipv4/ip_frag.c
libraries/lwip-1.4.1/src/netif/etharp.c
#libraries/lwip-1.4.1/src/netif/loopif.c
libraries/lwip-1.4.1/src/netif/slipif.c

##	platform/system_stm32f4xx.c
	platform/startup_stm32f407xx.s
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR})
#target_include_directories(${PROJECT_NAME} PUBLIC ../../)
target_include_directories(${PROJECT_NAME} PUBLIC libraries/CMSIS/Include)
target_include_directories(${PROJECT_NAME} PUBLIC libraries/RTX-v4.73/INC)
target_include_directories(${PROJECT_NAME} PUBLIC libraries/STM32F4x7_ETH_Driver/inc)
target_include_directories(${PROJECT_NAME} PUBLIC libraries/STM32F4xx_StdPeriph_Driver/inc)
target_include_directories(${PROJECT_NAME} PUBLIC libraries/STM32F4-Discovery)

target_include_directories(${PROJECT_NAME} PUBLIC libraries/lwip-1.4.1/port/STM32F4x7)
target_include_directories(${PROJECT_NAME} PUBLIC libraries/lwip-1.4.1/port/STM32F4x7/arch)
target_include_directories(${PROJECT_NAME} PUBLIC libraries/lwip-1.4.1/src/include)
target_include_directories(${PROJECT_NAME} PUBLIC libraries/lwip-1.4.1/src/include/ipv4)
target_include_directories(${PROJECT_NAME} PUBLIC libraries/lwip-1.4.1/src/include/lwip)
target_include_directories(${PROJECT_NAME} PUBLIC libraries/lwip-1.4.1/src/include/netif)
target_include_directories(${PROJECT_NAME} PUBLIC inc)
target_include_directories(${PROJECT_NAME} PUBLIC src)
target_include_directories(${PROJECT_NAME} PUBLIC platform)
