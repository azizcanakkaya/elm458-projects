#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "string.h"

#define CBC 1
#define ECB 1
#define CTR 1
#include "aes.h"

/*************************************************
* function declarations
*************************************************/
void init_systick(uint32_t s, uint8_t cen);
int main(void);
void delay_ms(volatile uint32_t);
void unlock_flash();
void lock_flash();
void erase_flash_sector3();
void write_flash(uint32_t addr, uint32_t datai);
void read_flash(uint32_t addr);

void Decryption_rx();
void CommandLEDs();

/*************************************************
* variables
*************************************************/
volatile uint32_t tDelay;
volatile uint8_t x = 0x00;
volatile uint8_t y = 0x00;
volatile uint32_t mem_datao;
volatile uint8_t ButAct = 0;
volatile uint8_t identifier;
volatile uint8_t identifier_rx;
volatile uint32_t DataTemplate;
volatile uint32_t DataFrame = 0;
volatile uint32_t DataFrame_rx;
volatile uint32_t RC_in = 0x49;	//73
volatile uint32_t RC_mod = 0x100;	//256
volatile uint32_t Rcode = 0;
volatile uint32_t Rcode_rx = 0;

uint8_t key[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
uint8_t iv[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
uint8_t in[16];

#define KEY1 0x45670123
#define KEY2 0xCDEF89AB
#define KEYADDR 0x0800C000

volatile int tx_complete = 0;
volatile int rx_complete = 0;
volatile int bufpos_tx = 0;
volatile int bufpos_rx = 0;
volatile uint8_t DataFbuf_tx[16];
volatile uint8_t DataFbuf_rx[16];
volatile uint8_t c = 0;
volatile uint8_t count_tx = 0;

/*************************************************
* timer 2 interrupt handler
*************************************************/
uint8_t TIM2_IRQHandler(void)
{
    TIM2->SR = (uint16_t)(~(1 << 0));

    if(ButAct == 1){
    	x++; }		//40 ticks per sec
 }

/*************************************************
* external interrupt handler
*************************************************/
void EXTI0_IRQHandler(void)
{
	if (EXTI->PR & (1 << 0)){
		ButAct ^= 0x1;
		for(uint32_t j=0; j<500000; j++);

		if (ButAct == 1){
			y++;

			read_flash(KEYADDR);
			unlock_flash();
			RC_in = mem_datao;
			lock_flash();

			Rcode = RC_in % 0x100;
			//GPIOD->ODR ^= (uint16_t)(1 << 14);

		}

		EXTI->PR = (1 << 0);
	}
}

/*************************************************
* SysTick interrupt handler
*************************************************/
void SysTick_Handler(void)
{

	IWDG->KR |= 0xaaaa;	// Resetting IWDG timer

	__disable_irq();

	if(ButAct == 0){
			if((x >= 1) & (y >= 2) & (x < 40)){		//DOUBLE PRESS
				ButAct = 0;
	       	    x = 0;
	       	    y = 0;
	       	    ButAct = 0;
	        	//GPIOD->ODR ^= (uint16_t)(1 << 13);

	        	RC_in++;
	        	identifier = 0x02;						//DOOR LOCK
	        	DataFrame = ((Rcode << 24) | (DataTemplate << 0) | (identifier << 0));
	        	identifier = 0;

	        	// DataFrame Encryption
	        	uint8_t x0 = DataFrame & 0xFF;
	        	uint8_t x1 = (DataFrame >> 8) & 0xFF;
	        	uint8_t x2 = (DataFrame >> 16) & 0xFF;
	        	uint8_t x3 = (DataFrame >> 24) & 0xFF;
	        	in[0] = x0;
	        	in[1] = x1;
	        	in[2] = x2;
	        	in[3] = x3;
	        	for (int i = 4; i < 16; i++){
	        		in[i] = 0x00;
	        	}
	        	struct AES_ctx ctx;
//	        	AES_init_ctx_iv(&ctx, key, iv);
//	        	AES_CBC_encrypt_buffer(&ctx, in, 16);
	        	AES_init_ctx(&ctx, key);
	        	AES_ECB_encrypt(&ctx, in);

				unlock_flash();							//MEMORY WRITE
				erase_flash_sector3();
				write_flash(KEYADDR, RC_in);
				lock_flash();

				//Enable TX interrupt
				USART3->CR1 |= (1 << 7);
	       	}

			else if((x >= 1) & (x < 40) & (y < 2)){		//SINGLE PRESS
	    		x = 0;
	    		y = 0;
	    		ButAct = 0;
	   	    	//GPIOD->ODR ^= (uint16_t)(1 << 12);

	   	    	RC_in++;
	   	    	identifier = 0x01	;					//DOOR UNLCOK
	   	    	DataFrame = ((Rcode << 24) | (DataTemplate << 0) | (identifier << 0));
	   	    	identifier = 0;

	   	    	// DataFrame Encryption
	   	    	uint8_t x0 = DataFrame & 0xFF;
	   	    	uint8_t x1 = (DataFrame >> 8) & 0xFF;
	   	    	uint8_t x2 = (DataFrame >> 16) & 0xFF;
	   	    	uint8_t x3 = (DataFrame >> 24) & 0xFF;
	   	    	in[0] = x0;
	   	    	in[1] = x1;
	   	    	in[2] = x2;
	   	    	in[3] = x3;
	   	    	for (int i = 4; i < 16; i++){
	   	    		in[i] = 0x00;
	   	    	}
	   	    	struct AES_ctx ctx;
//	   	    	AES_init_ctx_iv(&ctx, key, iv);
//	   	    	AES_CBC_encrypt_buffer(&ctx, in, 16);
	   	    	AES_init_ctx(&ctx, key);
	   	    	AES_ECB_encrypt(&ctx, in);

				unlock_flash();							//MEMORY WRITE
				erase_flash_sector3();
				write_flash(KEYADDR, RC_in);
				lock_flash();

				//Enable TX interrupt
				USART3->CR1 |= (1 << 7); //tx interrupt
	    	 }

			else if(x >= 40){							//LONG PRESS 2S
				x = 0;
				y = 0;
				ButAct = 0;
	    	   	//GPIOD->ODR ^= (uint16_t)(1 << 15);

				RC_in++;
				identifier = 0x10	;					//DOOR UNLCOK
				DataFrame = ((Rcode << 24) | (DataTemplate << 0) | (identifier << 0));
				identifier = 0;

				// DataFrame Encryption
				uint8_t x0 = DataFrame & 0xFF;
				uint8_t x1 = (DataFrame >> 8) & 0xFF;
				uint8_t x2 = (DataFrame >> 16) & 0xFF;
				uint8_t x3 = (DataFrame >> 24) & 0xFF;
				in[0] = x0;
				in[1] = x1;
				in[2] = x2;
				in[3] = x3;
				for (int i = 4; i < 16; i++){
					in[i] = 0x00;
				}
				struct AES_ctx ctx;
//	   	    	AES_init_ctx_iv(&ctx, key, iv);
//	   	    	AES_CBC_encrypt_buffer(&ctx, in, 16);
				AES_init_ctx(&ctx, key);
				AES_ECB_encrypt(&ctx, in);

				unlock_flash();							//MEMORY WRITE
				erase_flash_sector3();
				write_flash(KEYADDR, RC_in);
				lock_flash();

				//Enable TX interrupt
				USART3->CR1 |= (1 << 7); //tx interrupt
	   		}
	}
	__enable_irq();
}

/*************************************************
* UART2 interrupt handler -- (RX)
*************************************************/

void USART2_IRQHandler(void)
{
	if (USART2->SR & (1 << 5)){
	    if (bufpos_rx < 16 ) {
	    	DataFbuf_rx[bufpos_rx] = USART2->DR;
	    	bufpos_rx++;
	    }
	    if (bufpos_rx == 16 ){
	    	//USART2->CR1 &= (uint32_t)~(1 << 5);
	    	bufpos_rx = 0;
	    	rx_complete = 1;
	    	Decryption_rx();
	    }
	}
}

/*************************************************
* UART3 interrupt handler -- (TX)
*************************************************/

void USART3_IRQHandler(void)
{
	for(int i = 0; i < 16; i++ ){
		DataFbuf_tx[i] = in[i];
	}

	if (USART3->SR & (1 << 7)) {
	        // clear interrupt
	        USART3->SR &= (uint32_t)~(1 << 7);


	        	if (bufpos_tx == 16) {
	        		// buffer is flushed out, disable tx interrupt
	        		tx_complete = 1;
	        		bufpos_tx = 0;
	        		USART3->CR1 &= (uint32_t)~(1 << 7);
	        	}
	        	else {
	        		USART3->DR = DataFbuf_tx[bufpos_tx];
	        		delay_ms(100); //0.01s
	        		bufpos_tx++;
	        		tx_complete = 0;
	        	}
	}
}

/*************************************************
* initialize SysTick
*************************************************/
void init_systick(uint32_t s, uint8_t cen)
{
    // Clear CTRL register
    SysTick->CTRL = 0x00000;
    // Main clock source is running with HSI by default which is at 8 Mhz.
    // SysTick clock source can be set with CTRL register (Bit 2)
    // 0: Processor clock/8 (AHB/8)
    // 1: Processor clock (AHB)
    SysTick->CTRL |= (0 << 2);
    // Enable callback (bit 1)
    SysTick->CTRL |= ((uint32_t)cen << 1);
    // Load the value
    SysTick->LOAD = s;
    // Set the current value to 0
    SysTick->VAL = 0;
    // Enable SysTick (bit 0)
    SysTick->CTRL |= (1 << 0);
}

/*************************************************
* MEMORY
*************************************************/

void unlock_flash(){
	FLASH -> KEYR = KEY1;
	FLASH -> KEYR = KEY2;
}

void lock_flash() {
    FLASH->CR |= FLASH_CR_LOCK; // bit 31
}

void erase_flash_sector3() {
    const uint32_t sec = 3;
    __disable_irq();
    while(FLASH->SR & FLASH_SR_BSY); // check if busy
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= (sec << 3); // SNB bit 3:6
    FLASH->CR |= FLASH_CR_STRT; // start
    while(FLASH->SR & FLASH_SR_BSY); // check if busy
    __enable_irq();
}

void write_flash(uint32_t addr, uint32_t datai){
    while(FLASH->SR & FLASH_SR_BSY); // check if busy
    FLASH->CR |= FLASH_CR_PG;
    FLASH->CR &= ~(0x3U << 8); // clear PSIZE bit 8:9
    FLASH->CR |= (0x2 << 8);   // program PSIZE
    *(volatile uint32_t*)addr = datai;
    while(FLASH->SR & FLASH_SR_BSY); // check if busy
}

void read_flash(uint32_t addr){
    while(FLASH->SR & FLASH_SR_BSY); // check if busy
    FLASH->CR |= FLASH_CR_PG;
    FLASH->CR &= ~(0x3U << 8); // clear PSIZE bit 8:9
    FLASH->CR |= (0x2 << 8);   // program PSIZE
    volatile uint32_t *bridge  = addr;
    mem_datao= *bridge;
    while(FLASH->SR & FLASH_SR_BSY); // check if busy
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{

	RCC->AHB1ENR |= 0x0000000B;
    // enable SYSCFG clock (APB2ENR: bit 14) | enable TIM2 clock (bit0)
    RCC->APB2ENR |= (1 << 14);				//for ext interrupt
    RCC->APB1ENR |= ((1 << 18) | (1 << 0));	// USART3 & timer
    RCC->APB1ENR |= (1 << 17); 				//USART2

    GPIOD->MODER &= 0x00FFFFFF;		//LEDs
    GPIOD->MODER |= 0x55000000;

    // set pin modes as alternate mode 7 (pins 2 and 3)
    GPIOA->MODER &= 0xFFFFFF0F; // clear old values
    GPIOA->MODER |= 0x000000A0; // Set pin 2/3 to alternate func. mode (0b10)
    GPIOD->MODER |= (2 << 16); // gpıod alt func
    // set pin modes as high speed
    GPIOA->OSPEEDR |= 0x000000A0; // Set pin 2/3 to high speed mode (0b10)
    GPIOD->OSPEEDR |= (2 << 16);
    // choose AF7 for USART2 in Alternate Function registers
    GPIOD->AFR[1] |= (0x7 << 0); // for pin PD8 for USART3 TX
    GPIOA->AFR[0] |= (0x7 << 12); // for pin PA3 for USART2 RX


    uint32_t source_adr = 0x44;		//68
    uint32_t dest_adr = 0x05;		//5
    DataTemplate = ((dest_adr << 16) | (source_adr << 8));


    // Light up LEDs for 1 sec
    GPIOD->ODR &= 0x0000;
    GPIOD->ODR |= 0xF000;
    delay_ms(1000000);
    GPIOD->ODR &= 0x0000;

    // * EXT INTERRUPT * //
    SYSCFG->EXTICR[0] |= 0x00000000;
    // Choose either rising edge trigger (RTSR) or falling edge trigger (FTSR)
    EXTI->RTSR |= 0x00001;   // Enable rising edge trigger on EXTI0
    EXTI->FTSR |= 0x00001;
    // Mask the used external interrupt numbers.
    EXTI->IMR |= 0x00001;    // Mask EXTI0
    // Set Priority for each interrupt request
    NVIC->IP[EXTI0_IRQn] = 0x10; // Priority level 1
    // enable EXT0 IRQ from NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);


    // * SysTick * //
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();
    // configure SysTick to interrupt every 21k ticks
    // when SysClk is configured to 168MHz,
    // SysTick will be running at 168Mhz/8 = 21Mhz speed
    // passing 21000 here will give us 1ms ticks
    // enable callback
    init_systick(16777215, 1);
    //10500000 => 0.5sn, 16777215 => max 0.8s


    // * TIMER * //
    // Timer clock runs at ABP1 * 2
    //   since ABP1 is set to /4 of fCLK
    //   thus 168M/4 * 2 = 84Mhz
    // set prescaler to 83999
    //   it will increment counter every prescalar cycles
    // fCK_PSC / (PSC[15:0] + 1)
    // 84 Mhz / 8399 + 1 = 10 khz timer clock speed
    TIM2->PSC = 8399;
    // Set the auto-reload value to 10000
    //   which should give 1 second timer interrupts
    TIM2->ARR = 500; // 0.05s
    // Update Interrupt Enable
    TIM2->DIER |= (1 << 0);
    // enable TIM2 IRQ from NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
    // Enable Timer 2 module (CEN, bit0)
    TIM2->CR1 |= (1 << 0);


    // * USART3 TX * //
    // usart3 tx enable, RE bit 2
    USART3->CR1 |= (1 << 3);
    USART3->BRR |= (22 << 4);
    USART3->BRR |= 13;
    // usar3 word length M, bit 12
    USART3->CR1 |= (0 << 12); // 0 - 1,8,n
    // usart3 parity control, bit 9
    USART3->CR1 |= (0 << 10); // 0 - no parity
    //usart3 number of stop bits
    USART3->CR2 |= (0 << 12);  // 0 - 1 stop bit
    // enable usart3 - UE, bit 13
    USART3->CR1 |= (1 << 13);
    NVIC_EnableIRQ(USART3_IRQn);


    // * USART2 RX * //
    //usart3 rx enable, TE bit 3
    USART2->CR1 |= (1 << 2);
    //baud rate initialization
    USART2->BRR |= (22 << 4);
    USART2->BRR |= 13;
    //usart3 word length M, bit 12
    USART2->CR1 |= (0 << 12); // 0 - 1,8,n
    //usart3 parity control, bit 9
    USART2->CR1 |= (0 << 10); // 0 - no parity
    //usart3 number of stop bits
    USART2->CR2 |= (0 << 12);  // 0 - 1 stop bit
    //enable usart2 - UE, bit 13
    USART2->CR1 |= (1 << 13);
    //enable rx interrupt
    USART2->CR1 |= (1 << 5);
    NVIC_EnableIRQ(USART2_IRQn);


    // * IWDG * //
    //Enable iwdg
    IWDG->KR |= 0xcccc;
    //remove iwdg register protection
    IWDG->KR |= 0X5555;
    //Arrange prescaler
    IWDG->PR |= (3 << 0); // writing "011", enables divider 32 & 4096ms
    //reload iwdg counter
    IWDG->RLR = 0xfff; // load to max value (4096ms)
    IWDG->KR |= 0xaaaa; //refresh the counter & disable wwdg


    // * RESET RollingCode * //
//    unlock_flash();
//    erase_flash_sector3();
//    write_flash(KEYADDR, RC_in);
//    lock_flash();

    // * SLEEP MODE * //
    __enable_irq();
    SCB->SCR |= (1 << 1); //Sleep on exit
    __WFI();

}

void delay_ms(volatile uint32_t s)
{
    tDelay = s;
    while(tDelay != 0){
    	tDelay--;
    }
}

void Decryption_rx()
{
	struct AES_ctx ctx;
	//AES_init_ctx_iv(&ctx, key, iv);
	//AES_CBC_decrypt_buffer(&ctx, DataFbuf_rx, 16);
	AES_init_ctx(&ctx, key);
	AES_ECB_decrypt(&ctx, DataFbuf_rx);

	for(int z=0; z<4; z++){
		DataFrame_rx |= (DataFbuf_rx[z] << (8*z));
	}

	CommandLEDs();
}

void CommandLEDs(){

	identifier_rx = (DataFrame_rx & 0xFF);
	Rcode_rx = ((DataFrame_rx >> 24) & 0xFF);
	//Rcode_rx = 5;

	if(Rcode_rx == Rcode){
		// identify the command
		if(identifier_rx == 1){
			GPIOD->ODR ^= (uint16_t)(1 << 12);	//GREEN
			identifier_rx = 0;
			DataFrame_rx = 0;
			//Rcode_rx = 0;
		}
		else if(identifier_rx == 2){
			GPIOD->ODR ^= (uint16_t)(1 << 13);	//ORANGE
			identifier_rx = 0;
			DataFrame_rx = 0;
			//Rcode_rx = 0;
		}
		else if(identifier_rx >= 16){
			GPIOD->ODR ^= (uint16_t)(1 << 15);	//BLUE
			identifier_rx = 0;
			DataFrame_rx = 0;
			//Rcode_rx = 0;
		}
	}
	else{
		//Error occured during transmission, iwdg reset
		for(int s=0; s<20; s++){
			GPIOD->ODR ^= (uint16_t)(1 << 14);	//red
			delay_ms(250000);
		}
		identifier_rx = 0;
		DataFrame_rx = 0;
		//Rcode_rx = 0;
		IWDG->KR |= 0X5555;
		IWDG->RLR = 0x01;
	}
	//Rcode_rx = 0;
}


