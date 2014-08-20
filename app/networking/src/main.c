#define USE_STDPERIPH_DRIVER
#include "stm32f4xx.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
#include "console.h"
#include "misc.h"
#include "enet.h"
#include "stdio.h"

#define PORTE_LED0 3
#define PORTE_LED1 4

void led_flash_task(void *pvParameters)
{
	// Configure LEDS appropiately
  	RCC->AHB1ENR   |= RCC_AHB1ENR_GPIOEEN;
  	GPIOE->MODER   |= (1 << (PORTE_LED0 * 2)) |
                    (1 << (PORTE_LED1 * 2));
	while (1) {
    	/* Toggle the LED. */
  		//GPIOE->ODR ^= 0x00000018;
  		//GPIOE->ODR ^= 0x00000010;
  		GPIOE->ODR ^= 0x00000008;
  		//printf("Changing LED status\n");

  		/* Wait one second. */
  		vTaskDelay(100);
	}
}

void networking_receive(void *pvParameters)
{
	while (1) {
		//printf("networking receive task\n");
		enet_process_rx_ring();
  		/* Wait one second. */
  		vTaskDelay(50);
	}

}

void networking_send(void *pvParameters)
{
	while (1) {
		//printf("networking send task\n");
	    uint8_t dummy_payload[64] = { 1,2,3,4,5,6,7,8 };
      	enet_send_udp_mcast(0xe000007c, 11333,
                          dummy_payload, sizeof(dummy_payload));
		
  		/* Wait one second. */
  		vTaskDelay(50);
	}

}

void led2_task(void *pvParameters)
{
	while (1) {
    	/* Toggle the LED. */
  		//GPIOE->ODR ^= 0x00000018;
  		GPIOE->ODR ^= 0x00000010;
  		//GPIOE->ODR ^= 0x00000008;
  		//printf("Changing LED status\n");

  		/* Wait one second. */
  		vTaskDelay(100);
	}	
}

void myTraceCreate      (){
}

void myTraceSwitchedIn  (){
}

void myTraceSwitchedOut	(){
}

inline float myTraceGetTick(){
	// 0xE000E014 -> Systick reload value
	// 0xE000E018 -> Systick current value
	return ((float)((*(unsigned long *)0xE000E014)-(*(unsigned long *)0xE000E018)))/(*(unsigned long *)0xE000E014);
}

inline unsigned long myTraceGetTimeMillisecond(){
	return (xTaskGetTickCountFromISR() + myTraceGetTick()) * 1000 / configTICK_RATE_HZ;
}

void vApplicationTickHook()
{
}

int main()
{
	// Initialize the console
	console_init();
	printf("Initializing console\n");

	/* Configures the priority grouping: 4 bits pre-emption priority */
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	printf("Initializing networking\n");
	enet_init();

	/* Create a task to flash the LED. */
	xTaskCreate(led_flash_task,
	            (signed portCHAR *) "LED Flash",
	            512 /* stack size */, NULL,
	            tskIDLE_PRIORITY + 5, NULL);

	xTaskCreate(networking_receive,
	            (signed portCHAR *) "Networking recv",
	            4096, NULL,
	            tskIDLE_PRIORITY + 5, NULL);

	/* Create a task to flash the LED. */
	xTaskCreate(networking_send,
	            (signed portCHAR *) "Networking send",
	            4096 /* stack size */, NULL,
	            tskIDLE_PRIORITY + 5, NULL);


	/* Start running the tasks. */
	vTaskStartScheduler();

  	/* We should never get here as control is now taken by the scheduler */
  	//for( ;; );

	return 0;
}