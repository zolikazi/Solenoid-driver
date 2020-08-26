/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; Portions COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/**
  ******************************************************************************
  * <h2><center>&copy; Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.</center></h2>
  * @file    main.c
  * @author  CMP Team
  * @version V1.0.0
  * @date    28-December-2012
  * @brief   Main program body
  *          Modified to support the STM32F4DISCOVERY, STM32F4DIS-BB and
  *          STM32F4DIS-LCD modules.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, Embest SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
  * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
  * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
  * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "serial_debug.h"
//#include "tinyprintf.h"
#include "main.h"
//#include "spi_pov.h"
#include "lwip/tcp.h"
#include "packetizer.h"
#include "ptpsync.h"
#include "ethernetif.h"

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  10

/*--------------- LCD Messages ---------------*/
#define MESSAGE1   "     STM32F4x7      "
#define MESSAGE2   "  STM32F-4 Series   "
#define MESSAGE3   " UDP echoclient Demo"
#define MESSAGE4   "                    "

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;
uint32_t Button_TimerBack;
__IO uint32_t Button_Flag;

int RPM, RPS;
__IO uint32_t NextSec = 0; /* this variable is used to measure seconds */


/* Private function prototypes -----------------------------------------------*/
void LCD_LED_BUTTON_Init(void);
uint8_t Button_State(void);

/* Private functions ---------------------------------------------------------*/

#define delayUS_ASM(us) do {\
	asm volatile (	"MOV R0,%[loops]\n\t"\
			"1: \n\t"\
			"SUB R0, #1\n\t"\
			"CMP R0, #0\n\t"\
			"BNE 1b \n\t" : : [loops] "r" (16*us) : "memory"\
		      );\
} while(0)


int delay_loop(int end)
{
    volatile int i, j;
    for (i=0; i<end; i++) {
        j += i;
    }
    return j;
}


#ifdef SERIAL_DEBUG
#define serial_printf tfp_printf
#else
#define serial_printf 0
#endif

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured to
      144 MHz, this is done through SystemInit() function which is called from
      startup file (startup_stm32f4xx.s) before to branch to application main.
      To reconfigure the default setting of SystemInit() function, refer to
      system_stm32f4xx.c file
      */


    struct pbuf *frame;
    uint8_t data[1500];


    int recvd = 0; // received trigger

    uint32_t last_blink = 0; // last time we blinked
    uint32_t button_pressed = 0; // last time the button got pressed
    uint8_t last_button = 0; // last state of the blue button
    uint8_t current_button = 0; // current state of the blue button

#ifdef SERIAL_DEBUG
    DebugComPort_Init();
#endif

    /*Initialize LCD and Leds */
    LCD_LED_BUTTON_Init();

    /* Configure ethernet (GPIOs, clocks, MAC, DMA) */
    ETH_BSP_Config();

    /* Initilaize the LwIP stack */
    LwIP_Init();

    //Pov_SPI_Init();

    udp_packetizer_init();
    ptp_sync_init();

    serial_printf("\r\n STM32 based solenoid driver. \r\n");

    /* Configure the pin for have the MOSFET operate on PA3 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_ResetBits(GPIOA, GPIO_Pin_8);    // set it

    /* Infinite loop */
    while (1)
    {
        recvd = 0;
        
        /* trigger on received eth packet */
        if (ETH_CheckFrameReceived()) {
            STM_EVAL_LEDOn(LED3);
            
            //LwIP_Pkt_Handle();
            
            /* process received ethernet packet */
            frame = ETH_Interrupt_Input(netif_default);
            /* no packet could be read, silently ignore this */
            if (frame != NULL) {
                //recvd = 1;
                // get the ETH header
                pbuf_copy_partial(frame, &data, 14, 0);   // frame, buffer, len, offset
                //serial_printf("src %X:%X:%X:%X:%X:%X\r\n", data[6], data[7], data[8], data[9], data[10], data[11]);
                if((data[0]==MAC_ADDR0) && (data[5]==MAC_ADDR5)){
                    // get just the payload
                    pbuf_copy_partial(frame, &data, 4, 42);   // frame, buffer, len, offset
                    recvd = 2;
                }
                pbuf_free(frame);
                frame=NULL;
            }
        }
        
        STM_EVAL_LEDOff(LED3);


        if (recvd) {
            //Give inpulse on GPIOA Pin 8 (PA8)
            //Delay(250);
            STM_EVAL_LEDToggle(LED6);
            GPIO_ToggleBits(GPIOA, GPIO_Pin_8);
            //serial_printf("Hit the ball :)\r\n");
            Delay(50);
            STM_EVAL_LEDToggle(LED6);
            GPIO_ToggleBits(GPIOA, GPIO_Pin_8);
        }

        current_button = GPIO_ReadInputDataBit(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN);
        if (current_button != last_button) {
            serial_printf("button event\r\n");
            last_button = current_button;
        }

        if (LocalTime - last_blink > 500) {
            STM_EVAL_LEDToggle(LED4);
            last_blink = LocalTime;
        }
        //Delay(100);
    }
}

/**
  * @brief  Button state.
  * @param  None.
  * @retval 1: button is pressed.
  *         0: button is unpressed.
  */
uint8_t Button_State(void)
{
  uint8_t state = GPIO_ReadInputDataBit(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN);

  switch(Button_Flag) {
    case 0:
      if (state) {
        Button_Flag = 1;
        Button_TimerBack = LocalTime;
      }
      state = 0;
      break;
    case 1:
      if ((state) && ((LocalTime - Button_TimerBack) >= 40)) {
        Button_Flag = 2;
      } else {
        state = 0;
      }
      break;
    default:
      if (state == 0) {
        Button_Flag = 0;
      }
      state = 0;
      break;
  }
  return state;
}
/**
  * @brief  Inserts a delay time.
  * @param  nCount: number of 10ms periods to wait for.
  * @retval None
  */
void Delay(uint32_t nCount)
{
  /* Capture the current local time */
  timingdelay = LocalTime + nCount;

  /* wait until the desired delay finish */
  while (timingdelay > LocalTime);
}

/**
  * @brief  Updates the system local time
  * @param  None
  * @retval None
  */
void Time_Update(void)
{
  LocalTime += SYSTEMTICK_PERIOD_MS;

	if(LocalTime >= NextSec){
			RPM = RPS ;//* 60;
			RPS = 0;
			NextSec = LocalTime + 6000;
	}
}

/**
  * @brief  Initializes the STM324xG-EVAL's LCD and LEDs resources.
  * @param  None
  * @retval None
  */
void LCD_LED_BUTTON_Init(void)
{
  //Enable the GPIOD Clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

  // GPIOD Configuration
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);
    STM_EVAL_LEDInit(LED5);
    STM_EVAL_LEDInit(LED6);



#ifdef USE_LCD
  /* Initialize the STM324xG-EVAL's LCD */
  STM32f4_Discovery_LCD_Init();
#endif

#ifdef USE_LCD
  /* Clear the LCD */
  LCD_Clear(Black);

  /* Set the LCD Back Color */
  LCD_SetBackColor(Black);

  /* Set the LCD Text Color */
  LCD_SetTextColor(White);

  /* Display message on the LCD*/
  LCD_DisplayStringLine(Line0, (uint8_t*)MESSAGE1);
  LCD_DisplayStringLine(Line1, (uint8_t*)MESSAGE2);
  LCD_DisplayStringLine(Line2, (uint8_t*)MESSAGE3);
  LCD_DisplayStringLine(Line3, (uint8_t*)MESSAGE4);
#endif
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif


/*********** Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.*****END OF FILE****/
