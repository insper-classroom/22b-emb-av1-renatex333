#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// Config do BUT1
#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)
// Config do BUT2
#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)
// Config do BUT3
#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

// Config Pinos de saída
#define IN1_PIO PIOD
#define IN1_PIO_ID ID_PIOD
#define IN1_PIO_IDX 30
#define IN1_PIO_IDX_MASK (1 << IN1_PIO_IDX)

#define IN2_PIO PIOA
#define IN2_PIO_ID ID_PIOA
#define IN2_PIO_IDX 6
#define IN2_PIO_IDX_MASK (1 << IN2_PIO_IDX)

#define IN3_PIO PIOC
#define IN3_PIO_ID ID_PIOC
#define IN3_PIO_IDX 19
#define IN3_PIO_IDX_MASK (1 << IN3_PIO_IDX)

#define IN4_PIO PIOA
#define IN4_PIO_ID ID_PIOA
#define IN4_PIO_IDX 2
#define IN4_PIO_IDX_MASK (1 << IN4_PIO_IDX)


/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

QueueHandle_t xQueueModo;
QueueHandle_t xQueueSteps;
SemaphoreHandle_t xSemaphoreRTT;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/** prototypes */
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
static void init_buts(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but1_callback(void) {
	uint32_t angulo = 180;
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueModo, &angulo, &xHigherPriorityTaskWoken);
}
void but2_callback(void) {
	uint32_t angulo = 90;
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueModo, &angulo, &xHigherPriorityTaskWoken);
}
void but3_callback(void) {
	uint32_t angulo = 45;
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueModo, &angulo, &xHigherPriorityTaskWoken);
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		xSemaphoreGiveFromISR(xSemaphoreRTT, 0);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
  //gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
  //gfx_mono_draw_string("oii", 0, 20, &sysfont);
	
	const double angulo_por_passo = 0.17578125;
	uint32_t angulo_lido = 0;
	uint32_t passos = 0;
	char str_oled[20];
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;

	for (;;)  {
    if (xQueueReceive(xQueueModo, &(angulo_lido), 0)){
	    passos = angulo_lido/angulo_por_passo;
			xQueueSend(xQueueSteps, &passos, &xHigherPriorityTaskWoken);
	    gfx_mono_draw_filled_rect(10, 10, 124, 10, GFX_PIXEL_CLR);
			sprintf(str_oled, "Angulo = %d graus", angulo_lido);
			gfx_mono_draw_string(str_oled, 10, 10, &sysfont);
    }
	}
}

static void task_motor(void *pvParameters){
	uint32_t n_passos = 0;
	for (;;)	{
		if (xQueueReceive(xQueueSteps, &n_passos, 0)){
			for (int i = 0; i < (n_passos/4); i++) {
				RTT_init(1000, 5, RTT_MR_ALMIEN);
				if (xSemaphoreTake(xSemaphoreRTT, 1000) == pdTRUE){
					pio_set(IN1_PIO, IN1_PIO_IDX_MASK);
					pio_clear(IN2_PIO, IN2_PIO_IDX_MASK);
					pio_clear(IN3_PIO, IN3_PIO_IDX_MASK);
					pio_clear(IN4_PIO, IN4_PIO_IDX_MASK);
				}
				RTT_init(1000, 5, RTT_MR_ALMIEN);
				if (xSemaphoreTake(xSemaphoreRTT, 1000) == pdTRUE){
					pio_clear(IN1_PIO, IN1_PIO_IDX_MASK);
					pio_set(IN2_PIO, IN2_PIO_IDX_MASK);
					pio_clear(IN3_PIO, IN3_PIO_IDX_MASK);
					pio_clear(IN4_PIO, IN4_PIO_IDX_MASK);
				}
				RTT_init(1000, 5, RTT_MR_ALMIEN);
				if (xSemaphoreTake(xSemaphoreRTT, 1000) == pdTRUE){
					pio_clear(IN1_PIO, IN1_PIO_IDX_MASK);
					pio_clear(IN2_PIO, IN2_PIO_IDX_MASK);
					pio_set(IN3_PIO, IN3_PIO_IDX_MASK);
					pio_clear(IN4_PIO, IN4_PIO_IDX_MASK);
				}
				RTT_init(1000, 5, RTT_MR_ALMIEN);
				if (xSemaphoreTake(xSemaphoreRTT, 1000) == pdTRUE){
					pio_clear(IN1_PIO, IN1_PIO_IDX_MASK);
					pio_clear(IN2_PIO, IN2_PIO_IDX_MASK);
					pio_clear(IN3_PIO, IN3_PIO_IDX_MASK);
					pio_set(IN4_PIO, IN4_PIO_IDX_MASK);
				}	
			}
		}
	}
}
/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

void init_buts(void) {
	// Ativa os PIOs
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	/* configura prioridade */
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4);
	
	// Inicializa como saídas
	pio_set_output(IN1_PIO, IN1_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(IN2_PIO, IN2_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(IN3_PIO, IN3_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(IN4_PIO, IN4_PIO_IDX_MASK, 0, 0, 0);
	
	// BUT1
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE , but1_callback);
	// BUT2
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE , but2_callback);
	// BUT3
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE , but3_callback);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	// Inicializa botões
	init_buts();
	// Inicializa semáforo
	xSemaphoreRTT = xSemaphoreCreateBinary();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	
	if (xTaskCreate(task_motor, "motor", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create motor task\r\n");
	}
	
	 xQueueModo = xQueueCreate(100, sizeof(uint32_t));
	 if (xQueueModo == NULL){
		printf("falha em criar a queue xQueueModo \n");
	 }
	 
	 xQueueSteps = xQueueCreate(100, sizeof(uint32_t));
	 if (xQueueSteps == NULL){
		 printf("falha em criar a queue xQueueSteps \n");
	 }

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
