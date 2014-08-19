/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "main.h"
#include "tcpip.h"
#include "serial_debug.h"
#include "misc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*--------------- LCD Messages ---------------*/
#if defined (STM32F40XX)
#define MESSAGE1   "    STM32F40/41x     "
#elif defined (STM32F427X)
#define MESSAGE1   "     STM32F427x      "
#endif
#define MESSAGE2   "  STM32F-4 Series   "
#define MESSAGE3   " UDP/TCP EchoServer "
#define MESSAGE4   "                    "
// declare the pin numbers
#define PORTA_ETH_REFCLK 1
#define PORTA_ETH_MDIO   2
#define PORTA_ETH_CRSDV  7
#define PORTB_ETH_TXEN   11
#define PORTB_ETH_TXD0   12
#define PORTB_ETH_TXD1   13
#define PORTB_PHY_RESET  14
#define PORTC_ETH_MDC    1
#define PORTC_ETH_RXD0   4
#define PORTC_ETH_RXD1   5
#define PORTE_LED0 3
#define PORTE_LED1 4
  // address is hard-wired on board using internal chip pullups.
#define ENET_PHY_ADDR 0x01

/*--------------- Tasks Priority -------------*/
#define MAIN_TASK_PRIO   ( tskIDLE_PRIORITY + 1 )
#define DHCP_TASK_PRIO   ( tskIDLE_PRIORITY + 4 )
#define LED_TASK_PRIO    ( tskIDLE_PRIORITY + 2 )
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern struct netif xnetif;
__IO uint32_t test;
 
/* Private function prototypes -----------------------------------------------*/
void LCD_LED_Init(void);
void Main_task(void * pvParameters);
void led_flash_task(void *pvParameters);
void ToggleLed4(void * pvParameters);
void configureEthernet(void);
extern void tcpecho_init(void);
extern void udpecho_init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured to 
       168 MHz, this is done through SystemInit() function which is called from
       startup file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

 /* Configures the priority grouping: 4 bits pre-emption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  // Configure the Ethernet
  configureEthernet();

  /* main task */
  xTaskCreate(Main_task, 
              (int8_t *)"Main",
              configMINIMAL_STACK_SIZE * 2, 
              NULL,
              MAIN_TASK_PRIO, 
              NULL);

  /* Create a task to flash the LED. */
  xTaskCreate(led_flash_task,
              (signed portCHAR *) "LED Flash",
              512 /* stack size */, 
              NULL,
              tskIDLE_PRIORITY + 5, 
              NULL);
  
  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  for( ;; );
}

uint16_t enet_read_phy_reg(const uint8_t reg_idx)
{
  while (ETH->MACMIIAR & ETH_MACMIIAR_MB) { } // ensure MII is idle
  ETH->MACMIIAR = (ENET_PHY_ADDR << 11) | 
                  ((reg_idx & 0x1f) << 6) |
                  ETH_MACMIIAR_CR_Div102  | // clock divider
                  ETH_MACMIIAR_MB; 
  while (ETH->MACMIIAR & ETH_MACMIIAR_MB) { } // spin waiting for MII to finish
  return ETH->MACMIIDR & 0xffff;
}

void enet_write_phy_reg(const uint8_t reg_idx, const uint16_t reg_val)
{
  while (ETH->MACMIIAR & ETH_MACMIIAR_MB) { } // ensure MII is idle
  ETH->MACMIIDR = reg_val; // set the outgoing data word
  ETH->MACMIIAR = (ENET_PHY_ADDR << 11)   |
                  ((reg_idx & 0x1f) << 6) |
                  ETH_MACMIIAR_CR_Div102  | // MDC clock divider
                  ETH_MACMIIAR_MW         | // set the write bit
                  ETH_MACMIIAR_MB; // start it up
  while (ETH->MACMIIAR & ETH_MACMIIAR_MB) { } // spin waiting for MII to finish
  uint16_t readback_val = enet_read_phy_reg(reg_idx);
  if (readback_val != reg_val)
  {
    printf("woah there. tried to write 0x%04x to reg %02d but it read back %04x\r\n",
           reg_val, reg_idx, readback_val);
  }
}

void configureEthernet(void)
{
 printf("enet_init()\r\n");
  // set up the pins on port a
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN     |  // and all the i/o ports we 
                  RCC_AHB1ENR_GPIOBEN     |  // will be using to talk to
                  RCC_AHB1ENR_GPIOCEN;       // the ethernet PHY

  GPIOA->MODER |= (2 << (PORTA_ETH_REFCLK * 2)) |
                  (2 << (PORTA_ETH_MDIO   * 2)) |
                  (2 << (PORTA_ETH_CRSDV  * 2)); // set these guys as AF pins
  GPIOA->AFR[0] |= (11 << (PORTA_ETH_REFCLK * 4)) |
                   (11 << (PORTA_ETH_MDIO   * 4)) |
                   (11 << (PORTA_ETH_CRSDV  * 4));

  // set up the ethernet pins on port b
  GPIOB->MODER |= (2 << (PORTB_ETH_TXEN * 2)) |
                  (2 << (PORTB_ETH_TXD0 * 2)) |
                  (2 << (PORTB_ETH_TXD1 * 2));
  GPIOB->AFR[1] |= (11 << ((PORTB_ETH_TXEN - 8) * 4)) |
                   (11 << ((PORTB_ETH_TXD0 - 8) * 4)) |
                   (11 << ((PORTB_ETH_TXD1 - 8) * 4));
  GPIOB->OSPEEDR |= (3 << (PORTB_ETH_TXEN * 2)) |
                    (3 << (PORTB_ETH_TXD0 * 2)) |
                    (3 << (PORTB_ETH_TXD1 * 2)); // make the tx pins go fast
  GPIOB->MODER |= (1 << (PORTB_PHY_RESET * 2)); // reset = gpio output pin

  // set up the ethernet pins on port c
  GPIOC->MODER  |= ( 2 << (PORTC_ETH_MDC  * 2)) |
                   ( 2 << (PORTC_ETH_RXD0 * 2)) |
                   ( 2 << (PORTC_ETH_RXD1 * 2));
  GPIOC->AFR[0] |= (11 << (PORTC_ETH_MDC  * 4)) |
                   (11 << (PORTC_ETH_RXD0 * 4)) |
                   (11 << (PORTC_ETH_RXD1 * 4));

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // enable the sysconfig block
  RCC->AHB1RSTR |= RCC_AHB1RSTR_ETHMACRST;
  for (volatile int i = 0; i < 1000; i++) { } // wait for sysconfig to come up
  // hold the MAC in reset while we set it to RMII mode
  for (volatile int i = 0; i < 1000; i++) { } // wait for sysconfig to come up
  SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL; // set the MAC in RMII mode  
  for (volatile int i = 0; i < 100000; i++) { } // wait for sysconfig to come up
  RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACRXEN  |
                  RCC_AHB1ENR_ETHMACTXEN  |
                  RCC_AHB1ENR_ETHMACEN    ;  // turn on ur ethernet plz
  for (volatile int i = 0; i < 100000; i++) { } // wait 
  RCC->AHB1RSTR &= ~RCC_AHB1RSTR_ETHMACRST; // release MAC reset
  for (volatile int i = 0; i < 100000; i++) { } // wait 
  RCC->AHB1RSTR |= RCC_AHB1RSTR_ETHMACRST;
  for (volatile int i = 0; i < 100000; i++) { } // wait 
  RCC->AHB1RSTR &= ~RCC_AHB1RSTR_ETHMACRST; // release MAC reset
  for (volatile int i = 0; i < 100000; i++) { } // wait for sysconfig ... (?)

  ETH->DMABMR |= ETH_DMABMR_SR;
  for (volatile uint32_t i = 0; i < 100000; i++) { } 
  while (ETH->DMABMR & ETH_DMABMR_SR) { } // wait for it to reset
  for (volatile uint32_t i = 0; i < 100000; i++) { } 
  ETH->DMAOMR |= ETH_DMAOMR_FTF; // flush DMA
  while (ETH->DMAOMR & ETH_DMAOMR_FTF) { } // wait for it to flush

  // now, configure the ethernet peripheral
  ETH->MACCR |= 0x02000000 | // CSTF = strip FCS. why isn't there a symbol ?
                ETH_MACCR_FES  | // enable 100 mbit mode
                ETH_MACCR_DM   | // full duplex
                ETH_MACCR_IPCO | // ipv4 checksum auto-generation for RX
                ETH_MACCR_APCS;  // automatically remove pad+CRC from frames
  ETH->MACFFR |= ETH_MACFFR_RA; // for now, don't try to filter in hardware

///////////////////////////////////////
// generate a decent reset pulse now
  GPIOB->BSRRL = 1 << PORTB_PHY_RESET;
  for (volatile uint32_t i = 0; i < 100000; i++) { } 
  GPIOB->BSRRH = 1 << PORTB_PHY_RESET; // assert reset (pull it low)
  for (volatile uint32_t i = 0; i < 100000; i++) { } // let some time pass
  GPIOB->BSRRL = 1 << PORTB_PHY_RESET; // de-assert reset (pull it high)
  // todo: only need to wait until registers read back something other
  // than 0xffff . then we don't have to wait as long.
  for (volatile uint32_t i = 0; i < 1000000; i++) { } // let it initialize
  printf("waiting for PHY to wake up...\r\n");
  while (enet_read_phy_reg(0) == 0xffff) { }
  for (volatile uint32_t i = 0; i < 1000000; i++) { } // let it initialize
  printf("done with PHY reset.\r\n");
  printf("setting software strap registers...\r\n");
  enet_write_phy_reg(0x09, 0x7821); // enable auto MDIX,
                                    // set INT/PWDN to be interrupt output
                                    // enable auto-negotiation
  enet_write_phy_reg(0x09, 0xf821); // exit software-strap mode
  enet_write_phy_reg(0x04, 0x0101); // only advertise 100-FD mode
///////////////////////////////////////
}

void leds_init()
{
  RCC->AHB1ENR   |= RCC_AHB1ENR_GPIOEEN;
  GPIOE->MODER   |= (1 << (PORTE_LED0 * 2)) |
                    (1 << (PORTE_LED1 * 2));
}

void led_flash_task(void *pvParameters)
{
  leds_init();

  while (1) {
    /* Toggle the LED. */
    GPIOE->ODR ^= 0x00000018;

    /* Wait one second. */
    vTaskDelay(100);
  }
}

/**
  * @brief  Main task
  * @param  pvParameters not used
  * @retval None
  */
void Main_task(void * pvParameters)
{
#ifdef SERIAL_DEBUG
  DebugComPort_Init();
#endif

  /*Initialize LCD and Leds */ 
  LCD_LED_Init();
  
  /* configure ethernet (GPIOs, clocks, MAC, DMA) */ 
  ETH_BSP_Config();

  /* Initilaize the LwIP stack */
  LwIP_Init();

  /* Initialize tcp echo server */
  //tcpecho_init();

  /* Initialize udp echo server */
  udpecho_init();

#ifdef USE_DHCP
  /* Start DHCPClient */
  xTaskCreate(LwIP_DHCP_task, (int8_t *)"DHCP", configMINIMAL_STACK_SIZE * 2, NULL,DHCP_TASK_PRIO, NULL);
#endif

#if 0
  /* Start toogleLed4 task : Toggle LED4  every 250ms */
  xTaskCreate(ToggleLed4, (int8_t *)"LED4", configMINIMAL_STACK_SIZE, NULL, LED_TASK_PRIO, NULL);
#endif

  for( ;; )
  {
    vTaskDelete(NULL);
  }
}

#if 0
/**
  * @brief  Toggle Led4 task
  * @param  pvParameters not used
  * @retval None
  */
void ToggleLed4(void * pvParameters)
{
  while(1)
  {   
    test = xnetif.ip_addr.addr;
    /*check if IP address assigned*/
    if (test !=0)
    {
      for( ;; )
      {
        /* toggle LED4 each 250ms */
        STM_EVAL_LEDToggle(LED4);
        vTaskDelay(250);
      }
    }
  }
}

#endif

/**
  * @brief  Initializes the STM324xG-EVAL's LCD and LEDs resources.
  * @param  None
  * @retval None
  */
void LCD_LED_Init(void)
{
#ifdef USE_LCD
  /* Initialize the STM324xG-EVAL's LCD */
  STM324xG_LCD_Init();
#endif

#if 0
  /* Initialize STM324xG-EVAL's LEDs */
  STM_EVAL_LEDInit(LED1);
  STM_EVAL_LEDInit(LED2);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
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
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: number of Ticks to delay.
  * @retval None
  */
void Delay(uint32_t nCount)
{
  vTaskDelay(nCount);
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

void myTraceCreate      (){
}

void myTraceSwitchedIn  (){
}

void myTraceSwitchedOut (){
}

void vApplicationTickHook()
{
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
