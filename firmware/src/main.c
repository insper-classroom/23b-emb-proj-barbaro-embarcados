/************************************************************************
* 5 semestre - Eng. da Computao - Insper
*
* 2021 - Exemplo com HC05 com RTOS
*
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// LEDs
#define LED_PIO      PIOD
#define LED_PIO_ID   ID_PIOD
#define LED_IDX      20
#define LED_IDX_MASK (1 << LED_IDX)

#define LED2_PIO      PIOD
#define LED2_PIO_ID   ID_PIOD
#define LED2_IDX      27
#define LED2_IDX_MASK (1 << LED2_IDX)

// Botão
#define BUT2_PIO      PIOC
#define BUT2_PIO_ID   ID_PIOC
#define BUT2_IDX      13
#define BUT2_IDX_MASK (1 << BUT2_IDX)
// Botão
#define BUT3_PIO      PIOD
#define BUT3_PIO_ID   ID_PIOD
#define BUT3_IDX      11
#define BUT3_IDX_MASK (1 << BUT3_IDX)
// Botão
#define BUT4_PIO      PIOD
#define BUT4_PIO_ID   ID_PIOD
#define BUT4_IDX      26
#define BUT4_IDX_MASK (1 << BUT4_IDX)
// Botão
#define BUT5_PIO      PIOA
#define BUT5_PIO_ID   ID_PIOA
#define BUT5_IDX      24
#define BUT5_IDX_MASK (1 << BUT5_IDX)
//Potenciometro
#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif
typedef struct {
	uint value;
} adcData;
/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)
TimerHandle_t xTimer;
QueueHandle_t xQueueADC;
/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
afec_callback_t callback);
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* RTOS application HOOK                                                */
/************************************************************************/

/* Called if stack overflow during execution */

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/* This function is called by FreeRTOS idle task */
extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/* This function is called by FreeRTOS each tick */
extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}
SemaphoreHandle_t xSemaphore;
/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/
static void AFEC_pot_callback(void) {
	adcData adc;
	adc.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueADC, &adc, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
void set_ledConecta(void);

void set_ledConecta(void){
	pio_set(LED2_PIO, LED2_IDX_MASK);
	pio_set(LED_PIO, LED_IDX_MASK);
}

void clear_ledConecta(void);

void clear_ledConecta(void){
	pio_clear(LED2_PIO, LED2_IDX_MASK);
	
}
void led_wait(int freq);

void led_wait(int freq){
	if(xSemaphoreTake(xSemaphore, 500 / portTICK_PERIOD_MS)){
		set_ledConecta();
	}else{
		set_ledConecta();
		delay_ms(freq);
		clear_ledConecta();
		delay_ms(freq);
	}

}
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
  /*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

  /* configura IRQ */
  afec_set_callback(afec, afec_channel, callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}
void vTimerCallback(TimerHandle_t xTimer) {
	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	afec_start_software_conversion(AFEC_POT);
}
void io_init(void) {

	// Ativa PIOs
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pmc_enable_periph_clk(BUT4_PIO_ID);
	pmc_enable_periph_clk(BUT5_PIO_ID);

	// Configura Pinos
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK,  PIO_DEFAULT | PIO_DEBOUNCE);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT4_PIO, PIO_INPUT, BUT4_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT5_PIO, PIO_INPUT, BUT5_IDX_MASK, PIO_PULLUP);
	
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	#if defined(__GNUC__)
	setbuf(stdout, NULL);
	#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	* emits one character at a time.
	*/
	#endif
}

uint32_t usart_puts(uint8_t *pstring) {
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART_COM))
	usart_serial_putchar(USART_COM, *(pstring+i++));
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
	uint timecounter = timeout_ms;
	uint32_t rx;
	uint32_t counter = 0;

	while( (timecounter > 0) && (counter < bufferlen - 1)) {
		if(usart_read(usart, &rx) == 0) {
			buffer[counter++] = rx;
		}
		else{
			timecounter--;
			vTaskDelay(1);
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen,
char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void config_usart0(void) {
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);

	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_init(void) {
	char buffer_rx[128];
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEbarbaro", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN1964", 100);
	
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
void task_bluetooth(void) {
	printf("Task Bluetooth started \n");

	printf("Inicializando HC05 \n");
	pio_clear(LED_PIO, LED_IDX_MASK);
	config_usart0();
	hc05_init();
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_callback);

	xTimer = xTimerCreate(/* Just a text name, not used by the RTOS
						kernel. */
						"Timer",
						/* The timer period in ticks, must be
						greater than 0. */
						100,
						/* The timers will auto-reload themselves
						when they expire. */
						pdTRUE,
						/* The ID is used to store a count of the
						number of times the timer has expired, which
						is initialised to 0. */
						(void *)0,
						/* Timer callback */
						vTimerCallback);
	xTimerStart(xTimer, 0);
	// configura LEDs e Botões
	io_init();

    char start_of_packet = '<';
    char end_of_packet = '>';
    char separator = ',';
	int handshake = 1;
	char handshake_msg;
	adcData adc;
	float value;
	char str[128];
	// Task não deve retornar.

	while(1) {
		if(handshake){
			usart_write(USART_COM,'h');
			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(1 / portTICK_PERIOD_MS);
			}
			led_wait(50);
			if(usart_read(USART_COM,&handshake_msg))
			if(handshake_msg=='h'){
				handshake=0;
				xSemaphoreGiveFromISR(xSemaphore, 0);
				printf("handshake recebido com sucesso");
			}
		}
		else{
			set_ledConecta();
			if(xQueueReceive(xQueueADC, &adc, portMAX_DELAY) == pdPASS){
				value = ((float)adc.value/(float)4000);
			}
			char message[128];
			int message_len = 0;
			// atualiza valor do botão
			sprintf(str, "%.2f", value);
			message[message_len++] = start_of_packet;
			message[message_len++] = (pio_get(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK) == 0) ? 'L' : '0';
			message[message_len++] = separator;

			
			message[message_len++] = (pio_get(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK) == 0) ? 'D' : '0';
			message[message_len++] = separator;

			
			message[message_len++] = (pio_get(BUT4_PIO, PIO_INPUT, BUT4_IDX_MASK) == 0) ? 'U' : '0';
			message[message_len++] = separator;
			
			message[message_len++] = (pio_get(BUT5_PIO, PIO_INPUT, BUT5_IDX_MASK) == 0) ? 'R' : '0';
			message[message_len++] = separator;
			message[message_len++] = str[0];
			message[message_len++] = str[1];
			message[message_len++] = str[2];
			message[message_len++] = str[3];
			message[message_len++] = end_of_packet;
			message[message_len] = '\0';
			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(1 / portTICK_PERIOD_MS);
			}
			usart_put_string(USART_COM, message);
			// envia fim de pacote
			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(1 / portTICK_PERIOD_MS);
			}
			// dorme por 500 ms
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	configure_console();
	xQueueADC = xQueueCreate(100, sizeof(adcData));
	if (xQueueADC == NULL)
	printf("falha em criar a queue xQueueADC \n");
	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);
   // cria semáforo binário
   xSemaphore = xSemaphoreCreateBinary();

   // verifica se semáforo foi criado corretamente
   if (xSemaphore == NULL)
   printf("falha em criar o semaforo \n");
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
