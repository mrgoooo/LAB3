/************************************************************************
 * SMS - cwiczenie proste 3: obsluga zlozonych czujnikow i urzadzen     *
 * wykonawczych (plytka ZL27ARM, biblioteka SPL)                        *
 *                                                                      *
 * Program:                                                             *
 *  1) komunikuje sie protokolem MODBUS RTU (USART1: PA9/PA10,          *
 *     115200 bodow, 8E1) ze stanowiskiem grzejaco-chlodzacym,          *
 *  2) realizuje prosty regulator temperatury T1 (grzalka G1 /          *
 *     wentylator W1 / oba elementy wylaczone),                         *
 *  3) mierzy temperature przetwornikiem z petla pradowa 4-20 mA        *
 *     (ADC1, kanal 8 = PB0, rezystor pomiarowy 160 Ohm),               *
 *  4) wyswietla temperature T1, temperature zadana, pomiar z petli     *
 *     pradowej oraz stan regulatora na wyswietlaczu LCD 2x16.          *
 ************************************************************************/
#include "modbus_conf.h"
#include "modbus.h"
#include "stm32f10x.h"
#include "lcd_hd44780.h"
#include "delay.h"
#include "misc.h"
#include "main.h"
#include <stdbool.h> 		// true, false
#include <stdio.h>

/* ------------------------------------------------------------------------
 * Parametry komunikacji MODBUS RTU ze stanowiskiem grzejaco-chlodzacym
 * ---------------------------------------------------------------------- */
#define STAND_ADDRESS     10      // !!! DO USTAWIENIA: adres MODBUS zapisany na stanowisku !!! adres modbus (trzebna napisać)
#define MB_BAUDRATE       115200  // predkosc transmisji; ramka 8E1 (8 bitow danych, parzystosc, 1 bit stopu)
#define MB_TIMEOUT        1000    // maksymalny czas oczekiwania na odpowiedz stanowiska [ms]

#define REG_T1            0       // Input Register 0   : pomiar temperatury T1 [setne czesci *C]
#define REG_W1            0       // Holding Register 0 : moc wentylatora W1 [promile]
#define REG_G1            4       // Holding Register 4 : moc grzalki G1 [promile]

/* ------------------------------------------------------------------------
 * Parametry regulatora. Temperatury wyrazone sa w setnych czesciach *C,
 * czyli w jednostkach stanowiska (2500 oznacza 25,00 *C) - dzieki temu
 * regulator liczy wylacznie na liczbach calkowitych.
 * ---------------------------------------------------------------------- */
#define SETPOINT_INIT     3500    // temperatura zadana po starcie programu: 35,00 *C
#define SETPOINT_MIN      2000    // dolne ograniczenie temperatury zadanej: 20,00 *C
#define SETPOINT_MAX      6000    // gorne ograniczenie temperatury zadanej: 60,00 *C
#define DEADBAND          50      // "okolice" temperatury zadanej (strefa nieczulosci): +/- 0,50 *C
#define POWER_ON          1000    // element wlaczony:  1000 promili = 100 % mocy
#define POWER_OFF         0       // element wylaczony

/* ------------------------------------------------------------------------
 * Petla pradowa 4-20 mA (przetwornik o zakresie -30..60 *C, rezystor
 * pomiarowy 160 Ohm, 12-bitowy ADC z napieciem odniesienia 3,3 V).
 * Wzor ze skryptu (Uc - wartosc odczytana z ADC):
 *     t = (Uc * 3,3/4095 - 0,64) * 90/2,56 - 30
 * po uproszczeniu do postaci t = LOOP_A * Uc - LOOP_B:
 *     LOOP_A = 3,3*90 / (4095*2,56) ~= 0,028331  [*C na jednostke ADC]
 *     LOOP_B = 0,64*90/2,56 + 30     = 52,5      [*C]
 * Stale wylicza kompilator - w programie zostaje jedno mnozenie i odejmowanie.
 * ---------------------------------------------------------------------- */
#define LOOP_A            (3.3f * 90.0f / (4095.0f * 2.56f))
#define LOOP_B            (0.64f * 90.0f / 2.56f + 30.0f)
// Prad ponizej 4 mA oznacza uszkodzenie czujnika lub instalacji. Prog przyjeto
// z zapasem na tolerancje elementow: 3,6 mA * 160 Ohm = 0,576 V -> 0,576/3,3*4095 ~= 715
#define LOOP_ADC_MIN      715

// Stan regulatora - wykorzystywany do sygnalizacji na LCD i diodach LED
typedef enum {
	REG_IDLE,      // temperatura w okolicach zadanej: grzalka i wentylator wylaczone
	REG_HEATING,   // temperatura za niska:  wlaczona grzalka G1
	REG_COOLING,   // temperatura za wysoka: wlaczony wentylator W1
	REG_ERROR      // blad komunikacji ze stanowiskiem
} REG_STATE;

void RCC_Config(void);
void GPIO_Config(void);
void NVIC_Config(void);
void TIM2_Config(void);
void TIM4_Config(void);
void ADC1_Config(void);
void USART1_Config(void);
void LCD_Refresh(int16_t setpoint);

MB_RESPONSE_STATE Stand_ReadInputRegister(uint16_t reg, uint16_t *value);
MB_RESPONSE_STATE Stand_WriteHoldingRegister(uint16_t reg, uint16_t value);

char KB2char(void);

uint8_t *resp;                // wskaznik na tresc ostatniej odpowiedzi MODBUS
uint16_t resplen;             // dlugosc tresci ostatniej odpowiedzi MODBUS
MB_RESPONSE_STATE respstate;  // stan ostatniej odpowiedzi MODBUS (do podgladu w debuggerze)

// Zmienne wspoldzielone z funkcjami obslugi przerwan (stad volatile):
volatile uint16_t adc_value = 0;                // ostatni wynik konwersji ADC1 - zapis w ADC1_2_IRQHandler
volatile int16_t temp_setpoint = SETPOINT_INIT; // temperatura zadana [0,01 *C] - zmiana w przerwaniu klawiatury

int16_t temp_t1 = 0;              // ostatni poprawny pomiar temperatury T1 [0,01 *C]
bool t1_valid = false;            // true, gdy temp_t1 pochodzi z ostatniego (udanego) odczytu
REG_STATE reg_state = REG_IDLE;   // biezacy stan regulatora

int main(void) {
	uint16_t raw;         // surowa zawartosc rejestru odczytanego ze stanowiska
	int16_t setpoint;     // kopia temperatury zadanej na czas jednego kroku regulatora
	uint16_t fan, heater; // wyznaczone sterowania: moc wentylatora W1 i grzalki G1 [promile]

	RCC_Config();		  // konfiguracja RCC
	GPIO_Config();    // konfiguracja GPIO
	LED(LEDALL,LED_OFF);
	NVIC_Config();		// konfiguracja NVIC
	TIM2_Config();		// konfigruacja TIM2
	TIM4_Config();		// konfiguracja TIM4
	ADC1_Config();		// konfiguracja ADC1
	USART1_Config();	// konfiguracja USART1
	MB_Config( MB_BAUDRATE );// konfiguracja MB (wyznacza czasy t1,5 i t3,5 dla danej predkosci)
	Communication_Mode(false, false); // przerwania USART1 wylaczone - wlaczy je automat MODBUS

	// Od tej chwili co 10us wywolywane sa: MB(), DelayTick() i TimeoutTick() (SysTick_Handler)
	SysTick_Config( 90 ); // 10us
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK_Div8 ); //kanał wejściowy 8 
	NVIC_SetPriority(SysTick_IRQn, 0 );

	LCD_Initialize(); // inicjalizacja LCD
	LCD_WriteText((unsigned char*)"SMS L3: start"); // znak, ze program doszedl do petli glownej

	while(1) {
		// ,,Regulator''
		setpoint = temp_setpoint; // kopia - oryginal moze zostac zmieniony w przerwaniu klawiatury
		fan = POWER_OFF;          // domyslnie oba elementy wylaczone: jest to stan bezpieczny,
		heater = POWER_OFF;       // ktory obowiazuje rowniez przy bledzie odczytu temperatury

		// Krok 1: pomiar temperatury T1 (Input Register 0 stanowiska)
		respstate = Stand_ReadInputRegister(REG_T1, &raw);
		if(respstate != RESPONSE_OK){
			// reakcja na blad komunikacji: brak wiarygodnego pomiaru, wiec regulator
			// niczego nie wlacza (sterowania pozostaja zerowe), a blad jest sygnalizowany
			t1_valid = false;
			reg_state = REG_ERROR;
		} else {
			temp_t1 = (int16_t)raw; // temperatura w setnych czesciach *C (ze znakiem)
			t1_valid = true;

			// Krok 2: wyznaczenie sterowania
			if(temp_t1 > setpoint + DEADBAND){        // temperatura zbyt wysoka -> wentylator W1
				fan = POWER_ON;
				reg_state = REG_COOLING;
			} else if(temp_t1 < setpoint - DEADBAND){ // temperatura za niska -> grzalka G1
				heater = POWER_ON;
				reg_state = REG_HEATING;
			} else {                                  // okolice temperatury zadanej -> nic
				reg_state = REG_IDLE;
			}
		}

		// Krok 3: przeslanie sterowan do stanowiska (Holding Register 4 i 0).
		// Zapis wykonywany jest w kazdym obiegu petli - rowniez po bledzie odczytu,
		// aby sprobowac wylaczyc grzalke i wentylator. Grzalka jest ustawiana pierwsza.
		if(Stand_WriteHoldingRegister(REG_G1, heater) != RESPONSE_OK)
			reg_state = REG_ERROR;
		if(Stand_WriteHoldingRegister(REG_W1, fan) != RESPONSE_OK)
			reg_state = REG_ERROR;

		// Krok 4: sygnalizacja stanu - diody LED jako proste narzedzie diagnostyczne
		LED(LED2, reg_state == REG_HEATING ? LED_ON : LED_OFF); // LED2: wlaczona grzalka G1
		LED(LED3, reg_state == REG_COOLING ? LED_ON : LED_OFF); // LED3: wlaczony wentylator W1
		LED(LED8, reg_state == REG_ERROR   ? LED_ON : LED_OFF); // LED8: blad komunikacji MODBUS
		LED(LED1, LED_TOGGLE);                                   // LED1: "puls" petli glownej
		LCD_Refresh(setpoint);

		// Stanowisko odswieza pomiary i sterowania raz na sekunde, wiec czestsze
		// wykonywanie regulatora nie ma sensu. (Wcisniecie klawisza skraca to opoznienie,
		// bo KB2char() korzysta z tego samego licznika - nowa wartosc zadana od razu
		// pojawia sie na LCD.)
		Delay_ms(500);
	}
}

/* Odczyt jednego rejestru typu Input Register (funkcja MODBUS 0x04).
 * Tresc zapytania : [adres rejestru HI][LO][liczba rejestrow HI][LO]
 * Tresc odpowiedzi: [liczba bajtow danych][wartosc HI][wartosc LO]
 * Adres urzadzenia, kod funkcji i CRC dodaje (oraz sprawdza) biblioteka MODBUS.
 * Zmienna *value jest ustawiana tylko wtedy, gdy zwrocono RESPONSE_OK. */
MB_RESPONSE_STATE Stand_ReadInputRegister(uint16_t reg, uint16_t *value){
	uint8_t request[4] = { reg >> 8, reg & 0xFF, 0x00, 0x01 }; // jeden rejestr o adresie reg

	MB_SendRequest(STAND_ADDRESS, FUN_READ_INPUT_REGISTER, request, 4);
	respstate = MB_GetResponse(STAND_ADDRESS, FUN_READ_INPUT_REGISTER, &resp, &resplen, MB_TIMEOUT);
	if(respstate == RESPONSE_OK)
		*value = (resp[1] << 8) | resp[2]; // resp[0] to liczba bajtow danych, dalej wartosc 16-bitowa
	return respstate;
}

/* Zapis jednego rejestru typu Holding Register (funkcja MODBUS 0x06).
 * Tresc zapytania : [adres rejestru HI][LO][wartosc HI][LO]
 * Poprawna odpowiedz stanowiska jest echem zapytania. */
MB_RESPONSE_STATE Stand_WriteHoldingRegister(uint16_t reg, uint16_t value){
	uint8_t request[4] = { reg >> 8, reg & 0xFF, value >> 8, value & 0xFF };

	MB_SendRequest(STAND_ADDRESS, FUN_WRITE_SINGLE_REGISTER, request, 4);
	respstate = MB_GetResponse(STAND_ADDRESS, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, MB_TIMEOUT);
	return respstate;
}


void USART1_Config(){
	USART_InitTypeDef USART_InitStruct;

	// Konfiguracja USART (przerwania wylaczone)
	// Stanowisko pracuje w konfiguracji 8E1 z predkoscia 115200 bodow.
	USART_InitStruct.USART_BaudRate = MB_BAUDRATE;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // bez sprzetowej kontroli przeplywu
	// W STM32 dlugosc slowa obejmuje bit parzystosci: 8 bitow danych + 1 bit parzystosci = 9 bitow
	USART_InitStruct.USART_WordLength = USART_WordLength_9b;
	USART_InitStruct.USART_Parity = USART_Parity_Even;       // kontrola parzystosci (E)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;      // jeden bit stopu
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // transmisja w obie strony
	USART_Init(USART1, &USART_InitStruct);

	// Przerwania odbiornika (RXNE - bufor odbiorczy niepusty) i nadajnika (TXE - bufor
	// nadawczy pusty) pozostaja wylaczone; wlacza je biblioteka MODBUS (Communication_Mode)
	// dokladnie wtedy, gdy czeka na znak lub ma cos do wyslania.
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

	// Priorytet przerwania USART1 jest juz ustawiony (NVIC_Config) - mozna wlaczyc uklad
	USART_Cmd(USART1, ENABLE);
}

/* Ponizsze piec funkcji to "sterowniki" wymagane przez biblioteke MODBUS
 * (w modbus.c maja one puste, slabe definicje - tutaj sa nadpisywane). */

// Przelaczanie USART1 w tryb nasluchiwania (rx), nadawania (tx) lub oczekiwania
void Communication_Mode(bool rx, bool tx){
	// jesli rx == true, wlacz przerwanie USART_IT_RXNE, w przeciwnym razie wylacz
	USART_ITConfig(USART1, USART_IT_RXNE, rx?ENABLE:DISABLE);
	// jesli tx == true, wlacz przerwanie USART_IT_TXE, w przeciwnym razie wylacz
	USART_ITConfig(USART1, USART_IT_TXE , tx?ENABLE:DISABLE);
}

// Wyslanie pojedynczego znaku (wywolywane, gdy bufor nadawczy USART1 jest pusty)
void Communication_Put(uint8_t ch){
	// wyslij znak na USART1
	USART_SendData(USART1, ch);
}

// Odebranie pojedynczego znaku (wywolywane z MB() po zgloszeniu odebrania znaku)
uint8_t Communication_Get(void){
	uint8_t tmp = 0;
	// odczytaj znak z USART1 (rzutowanie na uint8_t odcina 9. bit, czyli bit parzystosci)
	tmp = USART_ReceiveData(USART1);
	// ustaw flage, ze znak juz nie jest do odczytu
	SetCharacterReceived(false);
	return tmp;
}

// Start odliczania kwantow 50us (wywolywane m.in. po kazdym odebranym znaku)
void Enable50usTimer(void){
	// Licznik TIM4 pracuje bez przerwy, dlatego przed wlaczeniem przerwania jest zerowany,
	// a zalegla flaga przepelnienia - kasowana. Bez tego pierwszy "kwant" bylby krotszy
	// niz 50us (albo zerowy), a wiec czasy t1,5 i t3,5 bylyby zanizone.
	TIM_SetCounter(TIM4, 0);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	// wlacz przerwanie TIM_IT_Update
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

// Zatrzymanie odliczania kwantow 50us
void Disable50usTimer(void){
	// wylacz przerwanie TIM_IT_Update
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
}

void NVIC_Config(void){
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	// 2 bity priorytetu wywlaszczania + 2 bity podpriorytetu (wartosci 0..3)
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);

	// Klawiatura PD6 -> EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource6);
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Klawiatura PD7 -> EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource7);
	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Klawiatura PD8 -> EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource8);
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Klawiatura PD9 -> EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource9);
	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Klawiatura -- EXTI9_5_IRQn
	// Najnizszy priorytet w programie. Funkcja obslugi skanuje klawiature (KB2char),
	// co trwa do ok. 20 ms i wymaga dzialajacego Delay_ms(). Przerwanie SysTick
	// (priorytet wywlaszczania 0) musi wiec moc ja wywlaszczyc - w przeciwnym razie
	// licznik opoznienia nie bylby dekrementowany i doszloby do zakleszczenia.
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Przetwornik ADC -- ADC1_2_IRQn
	NVIC_ClearPendingIRQ(ADC1_2_IRQn);
	NVIC_EnableIRQ(ADC1_2_IRQn);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Przerwania SysTick (wywoluje MB()), USART1 i TIM4 wspolnie realizuja protokol
	// MODBUS, dlatego wszystkie trzy maja najwyzszy i JEDNAKOWY priorytet wywlaszczania (0):
	//  - wolniejsze przerwania (klawiatura, ADC) nie zaburzaja zaleznosci czasowych protokolu,
	//  - te trzy przerwania nie wywlaszczaja sie nawzajem, a operuja na wspolnych
	//    zmiennych biblioteki MODBUS (bufor ramki, flagi, liczniki kwantow 50us).
	// O kolejnosci obslugi jednoczesnych zgloszen decyduje podpriorytet: USART1 dostaje
	// NIZSZY (1) niz SysTick i TIM4 (0). Przerwanie USART1 potrafi byc zglaszane bez
	// przerwy - np. gdy znaki nadejda w trakcie nadawania, ustawia sie flaga przepelnienia
	// odbiornika, kasowana dopiero odczytem znaku. Z takiego stanu automat MODBUS wychodzi
	// po odliczeniu czasu t3,5, wiec USART1 nie moze zaglodzic przerwania TIM4 - inaczej
	// program zawieslby sie na stale.

	// USART1 -- USART1_IRQn
	NVIC_ClearPendingIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// TIM4 (timer 50us) -- TIM4_IRQn
	NVIC_ClearPendingIRQ(TIM4_IRQn);
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void RCC_Config(void) {
	ErrorStatus HSEStartUpStatus;                          // zmienna opisujaca rezultat
	// uruchomienia HSE
	// konfigurowanie sygnalow taktujacych
	RCC_DeInit();                                          // reset ustawien RCC
	RCC_HSEConfig(RCC_HSE_ON);                             // wlacz HSE
	HSEStartUpStatus = RCC_WaitForHSEStartUp();            // czekaj na gotowosc HSE
	if(HSEStartUpStatus == SUCCESS) {
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);//
		FLASH_SetLatency(FLASH_Latency_2);                   // zwloka Flasha: 2 takty
		// dla 72MHz
		RCC_HCLKConfig(RCC_SYSCLK_Div1);                     // HCLK = SYSCLK
		RCC_PCLK2Config(RCC_HCLK_Div1);                      // PCLK2 = HCLK
		RCC_PCLK1Config(RCC_HCLK_Div2);                      // PCLK1 = HCLK/2
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); // PLLCLK = HSE*9
		// czyli 8MHz * 9 = 72 MHz
		RCC_PLLCmd(ENABLE);                                  // wlacz PLL
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);  // czekaj na uruchomienie PLL
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);           // ustaw PLL jako zrodlo
		// sygnalu zegarowego
		while(RCC_GetSYSCLKSource() != 0x08);                // czekaj az PLL bedzie
		// sygnalem zegarowym systemu
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);                    // ADCCLK = PCLK2/6 = 12 MHz

		// konfiguracja sygnalow taktujacych uzywanych peryferii
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);// wlacz taktowanie timera TIM2
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);// wlacz taktowanie timera TIM2
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);// wlacz taktowanie timera TIM4
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);// wlacz taktowanie portu AFIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);// wlacz taktowanie ADC1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);// wlacz taktowanie portu GPIO A
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);// wlacz taktowanie portu GPIO B
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);// wlacz taktowanie portu GPIO C
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);// wlacz taktowanie portu GPIO D
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);// wlacz taktowanie portu GPIO E
		// wlacz taktowanie USART1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);// USART1 jest podlaczony do szyny APB2
	}
}

void GPIO_Config(void) {
	// konfigurowanie portow GPIO
	GPIO_InitTypeDef  GPIO_InitStructure;

	// Konfiguracja Tx USART1 (PA9) -- funkcja alternatywna, push-pull
	// (pinem steruje nadajnik USART1, a nie rejestr wyjsciowy portu)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Konfiguracja Rx USART1 (PA10) -- wejscie plywajace
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Konfiguracja wejscia analogowego (PB0) -- wejscie analogowe
	// PB0 to kanal 8 przetwornika ADC1 (ADC12_IN8); na tym pinie mierzone jest
	// napiecie odlozone na rezystorze 160 Ohm wlaczonym w petle pradowa 4-20 mA.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Konfiguracja LEDALL (PB8-PB15) -- wyjscie, push-pull
	GPIO_InitStructure.GPIO_Pin = LEDALL;             		// piny PB8-PB15
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      // wyjscie w trybie push-pull
	GPIO_Init(GPIOB, &GPIO_InitStructure);                // inicjalizacja portu B

	// Konfiguracja klawiatury (PD10-PD13) -- wyjscie, otwarty dren
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Konfiguracja klawiatury (PD6-PD9) -- wejscie plywajace
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13);
}

/* Wyswietlenie wszystkich informacji jednoczesnie. Obie linie sa zawsze zapisywane
 * w calosci (16 znakow), dzieki czemu stara tresc jest nadpisywana i nie trzeba
 * czyscic wyswietlacza (brak migotania). Uklad:
 *     T1=25.31 Tz=35.0     <- temperatura T1 ze stanowiska (MODBUS) i temperatura zadana
 *     4-20: 24.8*C G1      <- temperatura z petli pradowej 4-20 mA i stan regulatora
 * Stan regulatora: "G1" - grzanie, "W1" - chlodzenie, "--" - oba elementy wylaczone,
 * "ERR" - blad komunikacji MODBUS.
 * Argument setpoint to temperatura zadana [0,01 *C], dla ktorej regulator wyznaczyl
 * biezacy stan. Celowo nie jest tu czytana zmienna temp_setpoint: przerwanie klawiatury
 * moglo ja zmienic juz po wykonaniu kroku regulatora i LCD pokazalby nowa wartosc
 * zadana razem ze stanem obliczonym jeszcze dla starej. */
void LCD_Refresh(int16_t setpoint){
	static const char * const state_txt[] = { "-- ", "G1 ", "W1 ", "ERR" }; // indeksowane typem REG_STATE
	char line[17];                           // 16 znakow + konczace '\0'
	uint16_t adc = adc_value;                // kopia - oryginal jest zapisywany w przerwaniu ADC

	// Linia 1: obecna temperatura T1 oraz temperatura zadana (przeliczone na *C)
	if(t1_valid)
		snprintf(line, sizeof(line), "T1=%5.2f Tz=%4.1f", temp_t1 / 100.0f, setpoint / 100.0f);
	else // brak aktualnego pomiaru (blad komunikacji)
		snprintf(line, sizeof(line), "T1=--.-- Tz=%4.1f", setpoint / 100.0f);
	LCD_GoTo(0,0);
	LCD_WriteText((unsigned char*)line);

	// Linia 2: temperatura z petli pradowej 4-20 mA (znak '*' zastepuje symbol stopnia)
	if(adc >= LOOP_ADC_MIN)
		snprintf(line, sizeof(line), "4-20:%5.1f*C %s", LOOP_A * adc - LOOP_B, state_txt[reg_state]);
	else // prad ponizej 4 mA: przerwana petla lub uszkodzony czujnik
		snprintf(line, sizeof(line), "4-20: blad   %s", state_txt[reg_state]);
	LCD_GoTo(0,1);
	LCD_WriteText((unsigned char*)line);
}

// TIM2 okresowo (co 0,5 s) wyzwala konwersje ADC1 zdarzeniem CC2 - pomiar petli pradowej
void TIM2_Config(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;
	TIM_TimeBaseStructure.TIM_Period = 5000;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// konfiguracja kanalu timera
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = 1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_ITConfig ( TIM2, TIM_IT_CC2 , ENABLE );  //konfigracja przerwania wyzwalania ADC1
	TIM_Cmd(TIM2, ENABLE);
}

void TIM4_Config(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	// Konfiguracja timera TIM4 do odliczania kwantow 50us
	// TIM4 jest taktowany sygnalem TIMxCLK = 72 MHz (PCLK1 = 36 MHz, ale poniewaz
	// preskaler APB1 jest rozny od 1, sygnal dla timerow jest mnozony przez 2).
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;                 // 72 MHz / 72 = 1 MHz (takt co 1us)
	TIM_TimeBaseStructure.TIM_Period = 50-1;                    // 50 taktow -> przepelnienie co 50us
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // zliczanie w gore
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // dzielnik czestotliwosci = 1
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;            // brak powtorzen
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	// Przerwanie przepelnienia jest na razie wylaczone - wlacza je i wylacza biblioteka
	// MODBUS (Enable50usTimer / Disable50usTimer), gdy potrzebuje odmierzac t1,5 i t3,5.
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIM4, ENABLE);                                      // licznik pracuje bez przerwy
}

void ADC1_Config(void) {
	ADC_InitTypeDef  ADC_InitStructure;

	ADC_DeInit(ADC1);                                   // reset ustawien ADC1

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  // niezalezne dzialanie ADC 1 i 2
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // pomiar pojedynczego kanalu
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // pomiar automatyczny
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2; // start na przerwanie
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // pomiar wyrownany do prawej
	ADC_InitStructure.ADC_NbrOfChannel = 1;             // jeden kanal
	ADC_Init(ADC1, &ADC_InitStructure);                 // inicjalizacja ADC1

	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_41Cycles5); // ADC1, kanal 8 (PB0),
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	ADC_Cmd(ADC1, ENABLE);                              // aktywacja ADC1

	ADC_ResetCalibration(ADC1);                         // reset rejestru kalibracji ADC1
	while(ADC_GetResetCalibrationStatus(ADC1));         // oczekiwanie na koniec resetu
	ADC_StartCalibration(ADC1);                         // start kalibracji ADC1
	while(ADC_GetCalibrationStatus(ADC1));              // czekaj na koniec kalibracji
}

char KB2char(void){
	unsigned int GPIO_Pin_row, GPIO_Pin_col, i, j;
	const unsigned char KBkody[16] = {'1','2','3','A',\
			'4','5','6','B',\
			'7','8','9','C',\
			'*','0','#','D'};
	GPIO_SetBits(GPIOD, GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13);
	GPIO_Pin_row = GPIO_Pin_10;
	for(i=0;i<4;++i){
		GPIO_ResetBits(GPIOD, GPIO_Pin_row);
		Delay_ms(5);
		GPIO_Pin_col = GPIO_Pin_6;
		for(j=0;j<4;++j){
			if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_col) == 0){
				GPIO_ResetBits(GPIOD, GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13);
				return KBkody[4*i+j];
			}
			GPIO_Pin_col = GPIO_Pin_col << 1;
		}
		GPIO_SetBits(GPIOD, GPIO_Pin_row);
		GPIO_Pin_row = GPIO_Pin_row << 1;
	}
	GPIO_ResetBits(GPIOD, GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13);
	return 0;
}

/* Obsluga klawiatury - zmiana temperatury zadanej. Funkcja jest wywolywana
 * z EXTI9_5_IRQHandler po wykryciu zbocza opadajacego na liniach kolumn:
 *     'A': +1 *C     'B': -1 *C     'C': +5 *C     'D': -5 *C
 * Pozostale klawisze sa ignorowane. Nowa wartosc trafia na LCD i do regulatora
 * przy najblizszym obiegu petli glownej. */
void KB_Process(void){
	int16_t setpoint = temp_setpoint;

	switch(KB2char()){ // ustalenie, ktory klawisz wywolal przerwanie (skanowanie klawiatury)
	case 'A': setpoint += 100; break;
	case 'B': setpoint -= 100; break;
	case 'C': setpoint += 500; break;
	case 'D': setpoint -= 500; break;
	default: return;   // inny klawisz albo klawisz zostal juz puszczony
	}
	// ograniczenie temperatury zadanej do rozsadnego zakresu pracy stanowiska
	if(setpoint > SETPOINT_MAX) setpoint = SETPOINT_MAX;
	if(setpoint < SETPOINT_MIN) setpoint = SETPOINT_MIN;
	temp_setpoint = setpoint;
}

void LED(uint16_t led, enum LED_ACTION act) {
	switch(act){
	case LED_ON: GPIO_SetBits(GPIOB, led); break;
	case LED_OFF: GPIO_ResetBits(GPIOB, led); break;
	case LED_TOGGLE: GPIO_WriteBit(GPIOB, led, (GPIO_ReadOutputDataBit(GPIOB, led) == Bit_SET?Bit_RESET:Bit_SET));
	}
}
