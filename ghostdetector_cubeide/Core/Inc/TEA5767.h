/*
 * TEA5767.h
 *
 *  Created on: Dec 13, 2023
 *      Author: panba
 */

#ifndef INC_TEA5767_H_
#define INC_TEA5767_H_

// makra dla TEA5767
#define TEA5767_ADDR                     0xC0 // adres I2C (0b1100000 przewsuniete jeden bit w lewo, bo tak kaze transmit)
//https://www.st.com/resource/en/user_manual/um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf  strona 492

// 1 bajt danych
#define TEA5767_MUTE                     0x80 // tryb wyciszenia
#define TEA5767_SCAN                     0x40 // tryb skanowania

// 3 bajt danych
#define TEA5767_SCAN_UP                  0x80 // Skanowaie częstotliwości do góry
#define TEA5767_SCAN_DOWN                0x00 // Skanowanie do dołu
#define TEA5767_SCAN_STOP_HIGH           0x60 // Search stop level: Wysoki (ADC = 10)
#define TEA5767_SCAN_STOP_MID            0x40 // Search stop level: Sredni (ADC = 7)
#define TEA5767_SCAN_STOP_LOW            0x20 // Search stop level: Niski (ADC = 5)			!!
#define TEA5767_HI_INJECTION             0x10 // HLSI=1 (High side injection)
#define TEA5767_LO_INJECTION             0x00 // HLSI=0 (Low side injection)				(HLSI warunkuje sposob otrzymania kanalu radiowego dla pewnej czestotliwosci, str 29 dokumentacji)
#define TEA5767_MONO                     0x08 // Tryb dzwieku mono
#define TEA5767_STEREO                   0x00 // Tryb dzwieku stereo
#define TEA5767_MUTE_RIGHT               0x04 // Prawy kanal wyciszony
#define TEA5767_MUTE_LEFT                0x02 // Lewy kanal wyciszony

/// 4 bajt danych
#define TEA5767_STANDBY                  0x40 // Tryb Standby
#define TEA5767_JAPAN_BAND               0x20 // Japonskie pasmo (od 76MHz do 91MHz)
#define TEA5767_EUROPE_BAND              0x00 // Europejskie pasmo (od 87.5MHz do 108MHz)
#define TEA5767_XTAL                     0x10 // czestotliwosc zegara = 32.768kHz			(Mozna sie przyjrzec potem temu)
#define TEA5767_SOFT_MUTE                0x08 // Wyciszenie soft
#define TEA5767_HCC                      0x04 // High Cut Control on  (HCC bit = 1)			(idk co to jest)
#define TEA5767_HCC_OFF                  0x00 // High Cut Control off (HCC bit = 0)
#define TEA5767_SNC                      0x02 // Stereo Noise Cancelling jest on  (SNC bit = 1)
#define TEA5767_SNC_OFF                  0x00 // Stereo Noise Cancelling jest off (SNC bit = 0)

typedef struct {
	uint8_t HILO;            //HILO=1 -> HI and HILO=0 -> LO injection
	uint8_t Mute;            // jeśli nie zero -> MUTE
	uint8_t Mono;            // jeśli nie zero -> MONO
	uint8_t Band;            // jeśli nie zero -> użyj pasma Japońskiego (76..91MHz)
	uint8_t HCC;             // jeśli nie zero -> włącz High Cut Control
	uint8_t SNC;             // jeśli nie zero -> włącz Stereo Noise Cancelling
} TEA5767_SettingsTypeDef;

extern uint8_t txbuf[5];                             // dane wysyłane do TEA5767
extern uint8_t rxbuf[5];                             // dane odbierane z TEA5767
extern char freq[50];
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c2;
extern TEA5767_SettingsTypeDef tuner;

void TEA5767_write(void) {
	HAL_I2C_Master_Transmit(&hi2c2,TEA5767_ADDR, txbuf, 4, HAL_MAX_DELAY);
}

void TEA5767_read(void) {
	HAL_I2C_Master_Receive(&hi2c2,TEA5767_ADDR, rxbuf, 4, HAL_MAX_DELAY);
}

void TEA5767_SetFreq(uint8_t hilo, float freq) {	// Ustawia czestotliwosc
	uint16_t div;

	if (hilo == 1)
		div = (freq * 1000000 + 225000) / 8192;
	else
		div = (freq * 1000000 - 225000) / 8192;

	txbuf[0] = (div >> 8) & 0x3f; 											// and sprawia, ze zapisywane są tylko bity od 5 do 0 odpowiedzialne za PLL
	if (tuner.Mute) txbuf[0] |= TEA5767_MUTE;								// x|=3 to równoważnik x = x|3
	txbuf[1] = div & 0xff;													// caly drugi bit to PLL (zapis czestotliwosci)
	txbuf[2] = (hilo == 0) ? TEA5767_LO_INJECTION : TEA5767_HI_INJECTION;	// ? operator działa jak if, komenda warunkuje sposob zmiany czestotliwosc na kanal
	txbuf[3] = TEA5767_XTAL;												// 4 bajkt mowi o czestotliwosci zegara
	if (tuner.Band) txbuf[3] |= TEA5767_JAPAN_BAND;
	if (tuner.HCC)  txbuf[3] |= TEA5767_HCC;
	if (tuner.SNC)  txbuf[3] |= TEA5767_SNC;
	txbuf[4] = 0x00; 														// piaty bajt zawsze jest ten sam

	TEA5767_write();
}

uint8_t TEA5767_get_ADC(void) {
	return (rxbuf[3] & 0xf0) >> 4;
}

uint8_t TEA5767_hilo_optimal(float freq) { 									//ustawia optymalne HILO dla czestotliwosci
	uint8_t ADC_lo, ADC_hi;

	TEA5767_SetFreq(1,(freq + 450000) / 1000000.0);							// strojenie F=freq+450kHz z hilo high
	delay_us(30000); 														// przynajmniej 27ms
	TEA5767_read();
	ADC_hi = TEA5767_get_ADC();

	TEA5767_SetFreq(1,(freq - 450000) / 1000000.0);							// strojenie F=freq-450kHz z HILO low
	delay_us(30000); 														// przynajmniej 27ms
	TEA5767_read();
	ADC_lo = TEA5767_get_ADC();

	// Jesli (LevelHigh < LevelLow) to HILO=1, inaczej HILO=0
	return (ADC_hi < ADC_lo) ? 1 : 0;
}

void TEA5767_set_frequency(float freq) {	//wykrywa optymalne HILO na podstawie czestotliwosci i ustawia ją
	tuner.HILO = TEA5767_hilo_optimal(freq * 1000000.0);
	TEA5767_SetFreq(tuner.HILO,freq);
}

uint8_t TEA5767_get_ready(void) {			//zwraca jeden jeśli flaga READY to 1
	uint8_t buf = (rxbuf[0] & 0x80)>>7;
	return buf;
}


uint8_t TEA5767_get_blr(void) {				//zwraca jeden jesli flaga osiagniecia limitu pasma to 1
	return (rxbuf[0] & 0x40) >> 6;
}

float TEA5767_get_freq(void) {				//zwraca obecna czestotliwosc
	float current_freq;

	TEA5767_read();
	if (tuner.HILO)
		current_freq = ((rxbuf[1] | (rxbuf[0] & 0x3f) << 8) * 8192) - 225000;
	else
		current_freq = ((rxbuf[1] | (rxbuf[0] & 0x3f) << 8) * 8192) + 225000;

	return current_freq;
}


void TEA5767_scan(uint8_t scan_dir) {	//skanuje w zadanym kierunku
	float cur_freq;
	uint16_t div;

	cur_freq = TEA5767_get_freq();

	if (scan_dir == TEA5767_SCAN_UP)
		div = (cur_freq + 300000) / 8192;
	else
		div = (cur_freq - 300000) / 8192;

	txbuf[0]  = (div >> 8) & 0x3f;
	if (tuner.Mute) txbuf[0] |= TEA5767_MUTE;
	txbuf[0] |= TEA5767_SCAN;
	txbuf[1]  = div & 0xff;
	txbuf[2]  = (tuner.HILO == 0) ? TEA5767_LO_INJECTION : TEA5767_HI_INJECTION;
	txbuf[2] |= scan_dir;
	txbuf[2] |= TEA5767_SCAN_STOP_HIGH;
	txbuf[3]  = TEA5767_XTAL;
	if (tuner.Band) txbuf[3] |= TEA5767_JAPAN_BAND;
	if (tuner.HCC)  txbuf[3] |= TEA5767_HCC;
	if (tuner.SNC)  txbuf[3] |= TEA5767_SNC;
	txbuf[4]  = 0x00; // Zawsze piaty bajt taki sam

	TEA5767_write();
}

// do automatic search for station
// input: scan_dir - search direction
void TEA5767_auto_scan(uint8_t scan_dir) {
	float cur_freq;
	if (TEA5767_get_ready()) {
		if (TEA5767_get_blr()) {
			// Frequency band limit reached, wrap up
			if (scan_dir == TEA5767_SCAN_UP) {
				TEA5767_set_frequency(87.5);
				TEA5767_scan(scan_dir);
				delay_us(100);
			} else {
				TEA5767_set_frequency(108.0);
				TEA5767_scan(scan_dir);
				delay_us(100);
			}
		} else {
			// Start scan from current frequency
			cur_freq =TEA5767_get_freq() / 1000000 + 0.0005;
			TEA5767_set_frequency(cur_freq);
			delay_us(100);
		}
	}
}

void TEA5767_scanning(uint8_t scan_dir){

	TEA5767_scan(scan_dir);
	HAL_Delay(300);

	TEA5767_auto_scan(scan_dir);
	HAL_Delay(300);
		while (TEA5767_get_ready() == 0) {
			TEA5767_read();
			HAL_Delay(300);
			TEA5767_read();
			TEA5767_auto_scan(scan_dir);
		}
}

void delay_us (uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
}


#endif /* INC_TEA5767_H_ */
