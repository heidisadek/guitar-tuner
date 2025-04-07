/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
	

#include "main.h"
#include "config.h"
#include "fft.c"

#include <stdio.h>
#include <math.h>
#include <string.h>

//functions
void adc_func();
float Find_Dominant_Frequency();
void find_closest_note(float idx);
void check_string(float idx);
float lpf(uint16_t prev, uint16_t curr);
void closest_diff(int freq);
int diff_find(int freq);
void auto_tuning();
void manual_tuning();
int get_margin(); 
void print_buf();
void draw_bar();
void draw_mode();
void draw_manual();
void erase_manual();
void draw_needle(float error_given);
void draw_note();
void erase_note();
void draw_degree();
void erase_degree();
void erase_needle();

//global variables
uint8_t temp[IMG_ROWS * IMG_COLS];
int concert_pitch = 440;
char *notes[12] = {"A", "A#","B","C","C#","D","D#","E","F","F#","G","G#"};
int frequencies[6] = {82, 110, 147, 196, 247, 330}; 
uint16_t n = FFT_SIZE;
int samples_n = 0;
float adc_r[FFT_SIZE];
float adc_i[FFT_SIZE];
uint16_t adc_res;
char message[100];
uint16_t freq;
int max_idx;
uint16_t max_adc;
float max_adc_r;
int select;
float LPF_Beta = 0.04; // 0<ß<1
int8_t mode;
int8_t current_row = -1, current_col = -1;
uint8_t sel;
int deg; 
int note = 0;

#define ERASE 0x00
#define BAR_C 0xFF
#define MODE_C 0xAA
uint8_t NOTE_C = 0x99;
uint8_t DEGREE_C = 0x99;
uint8_t NEEDLE_C = 0x99;

int main(void)
{
  /* Reset of all peripherals. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DAC_Init();
	
	/* Initialize ROW outputs */
  HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET);

	mode = -1; //-1 -> automatic, 1 -> manual
 	
  while (1) {
		
		draw_bar();
		draw_mode();
		if (mode == 1) { //manual
			
			sprintf(message, "in manual mode\n");
			print_msg(message);
			current_row = -1, current_col = -1;
			HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET);
			HAL_Delay(10);

			current_row = -1, current_col = -1;
			HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET);
			HAL_Delay(10);

			current_row = -1, current_col = -1;
			HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);

			current_row = -1, current_col = -1;
			HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_SET);		
			HAL_Delay(10);

			current_row = -1, current_col = -1;	
			HAL_Delay(50);
			
			manual_tuning();
			
		} else { //auto
			sprintf(message, "in auto mode\n");
			print_msg(message);
			auto_tuning();
		}
	}
}

void auto_tuning() {
	while (samples_n < n) {
		adc_func();
		samples_n++;
	}
	FFT(adc_r, adc_i, n);
	freq =	Find_Dominant_Frequency();
	
	sprintf(message, "detected freq is: %d\n", freq);
	
	find_closest_note(freq);
		
	print_buf();
	
	samples_n = 0;
}

void manual_tuning() {
	while (samples_n < n) {
		adc_func();
		samples_n++;
	}
	FFT(adc_r, adc_i, n);
	freq = Find_Dominant_Frequency();
	check_string(freq);
			
	draw_bar();
	draw_mode();
	print_buf();
		
	samples_n = 0;
}

void adc_func() {
	HAL_ADC_Start(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 100);
  adc_res = HAL_ADC_GetValue(&hadc3);
	
	if (samples_n < 2) {
		adc_r[samples_n] = adc_res;
	} else {
		adc_r[samples_n] = lpf(adc_r[samples_n-1], adc_res);
	}
	adc_i[samples_n] = 0;
}

void print_buf() {
	print_msg("Snap!\r\n");
	print_msg("\r\nPREAMBLE!\r\n");
	uart_send_bin(temp, IMG_ROWS * IMG_COLS);
}

// Function to detect peak frequency
float Find_Dominant_Frequency() {
  uint16_t maxIndex = 0;
  float maxValue = 0.0f;

  // Compute magnitude of FFT output
  for (uint16_t i = 1; i < FFT_SIZE / 2; i++) {
    magnitude[i] = sqrtf(adc_r[i] * adc_r[i] + adc_i[i] * adc_i[i]);
    if (magnitude[i] > maxValue && i < 150) {
			
      maxValue = magnitude[i];
      maxIndex = i;
			
			max_adc_r = adc_r[i];
			max_idx = i;
    } else if (magnitude[i] > maxValue && i > 150) {
			 maxValue = magnitude[i];
			maxIndex = 0;
		}
  }
	return (maxIndex * SAMPLE_RATE) / FFT_SIZE;
}


void closest_diff(int freq) {
  float i = (log2(freq/concert_pitch))*12;
	float n = pow(2, (i/12));
	int degree = 4 + (i + 9) / 12;
	int freq_idx = diff_find(freq);
	if (freq_idx < 2) {
			degree = 2;
	} else if (freq_idx == 5) {
	    degree = 4;
	} else {
	    degree = 3;
	}

	select = freq_idx;
	deg = degree;

	check_string(freq);
}

int diff_find(int freq) {
    int min = 0;
    for (int i = 1; i < sizeof(frequencies)/sizeof(int); i++) {
        if (abs(freq-frequencies[i]) < abs(freq-frequencies[min])) {
            min = i;
        }
    }
    return min;
}

void check_string(float idx) {
	erase_needle();
	int ideal;
	int lowest, highest;
	//select = 0;
	switch (select) { 
		case 0:
			sprintf(message,"freq detected = %fHz\r\n", idx);
			print_msg(message);
			sprintf(message,"Tuning string 1 (E2)\r\n");
			print_msg(message);
			ideal = frequencies[0];
			lowest = 70;
			highest = 99;
			break;
		case 1:
			sprintf(message,"freq detected = %fHz\r\n", idx);
			print_msg(message);
			sprintf(message,"Tuning string 2 (A2)\r\n");
			print_msg(message);
			ideal = frequencies[1];
			lowest = 100;
			highest = 129;
			break;
		case 2:
			sprintf(message,"freq detected = %fHz\r\n", idx);
			print_msg(message);
			sprintf(message,"Tuning string 3 (D3)\r\n");
			print_msg(message);
			ideal = frequencies[2];
			lowest = 130;
			highest = 164;
			break;
		case 3:
			sprintf(message,"freq detected = %fHz\r\n", idx);
			print_msg(message);
			sprintf(message,"Tuning string 4 (G3)\r\n");
			print_msg(message);
			ideal = frequencies[3];
			lowest = 165;
			highest = 219;
			break;
		case 4:
			sprintf(message,"freq detected = %fHz\r\n", idx);
			print_msg(message);
			sprintf(message,"Tuning string 5 (B3)\r\n");
			print_msg(message);
			ideal = frequencies[4];
			lowest = 220;
			highest = 269;
			break;
		case 5:
			sprintf(message,"freq detected = %fHz\r\n", idx);
			print_msg(message);
			sprintf(message,"Tuning string 6 (E4)\r\n");
			print_msg(message);
			ideal = frequencies[5];
			lowest = 270;
			highest = 399;
			break;
		default:
			sprintf(message,"freq detected = %fHz\r\n", idx);
			print_msg(message);
			sprintf(message,"Tuning string 1 (E2)\r\n");
			print_msg(message);
			ideal = frequencies[0];
			lowest = 70;
			highest = 99;
			deg = 2;
			break;
	}

	float error;
	int range = (idx < ideal) ? (ideal-lowest):(highest-ideal);
	
	error = (idx - (ideal+get_margin()))/range;
	sprintf(message,"error = %f\r\n", error);
	print_msg(message);
	
	error = (error > 1) ? 1:error;
	error = (error < -1) ? -1:error;
	
	sprintf(message,"error = %f\r\n", error);
	print_msg(message);
	draw_needle(error);
	
}

void draw_needle(float error_given) { //starts at 32/2 (middle), 32+32*i+32/2 where i = 7 to 15
	float error;
	//case 1 : if error given is between -50% and 50%
	NOTE_C = 0x99;
	DEGREE_C = 0x99;
	NEEDLE_C = 0x99;
	if (error_given <= 0.5 && error_given >= -0.5) {
		if (error_given == 0) {
			NOTE_C = 0xFF;
			DEGREE_C = 0xFF;
			NEEDLE_C = 0xFF;
			
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin); //green
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //blue
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		}
		error = error_given*2;
		for (int i = 0; i < 8; i++) {
			float a = error*i;
			int err = a;
			temp[480-(32*i)+(32/2)+err] = NEEDLE_C;
			
		}
		erase_note();
		erase_degree();
		draw_note();
		draw_degree();
	} else if ((error_given <= -0.5 && error_given >= -1) || (error_given <= 1 && error_given >= 0.5)){
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin); //red
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		//case 2: between -100%->-50% amd 50%->100%
		error = error_given;
		int val = (error < 0) ? -1:1;
		float error_adjusted = (error*2) - val;
		for (int i = 0; i < 8; i++) {
			float a = error_adjusted*i;
			int err = a;
			temp[480+(val)*(32*(err-i*val))+(32/2)+i*val] = NEEDLE_C;
		}
		erase_note();
		erase_degree();
		draw_note();
		draw_degree();
	}
}

//https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
float lpf(uint16_t prev, uint16_t curr) {
	float y;
	y = prev + LPF_Beta * (curr - prev);
	return y;
}

			
void erase_note() {
	for (int i = 17; i < 32; i++) {
		for (int j = 0; j < 32; j++) {
			temp[32*i + j] = ERASE;
		}
	}
}

void erase_degree() {
	for (int i = 26; i < 31; i++) {
		for (int j = 24; j < 29; j++) {
			temp[32*i + j] = ERASE;
		}
	}
}

void draw_bar() { //cols 80, rows 110
	for (int i = 32+160; i < 32+192; i++) {
		if (i == 32+160+8) {
			for (int j = 0; j < 9; j++) {
				temp[j*32+i-j] = BAR_C;
			}
		} else if (i > (32+160+8) && i < (32+160+23)) {
			temp[i] = BAR_C;
		} else if (i == (32+160+23)){
			for (int j = 0; j < 9; j++) {
				temp[32*j+i+j] = BAR_C;
			}
		}
	}
}

void erase_needle() {
	for (int i = 32*2+160; i < 32*2+192; i++) {
		if (i == 32*2+160) {
			for(int j = 0; j < 9; j++) {
				for (int k = 0; k <= j; k++) {
					temp[32*j+i+8-k] = ERASE;
				}
			}
		} else if (i == 32*2+160+8) {
			for (int j = 0; j < 16; j++) {
				for (int k = 0; k < 9; k++) {
					temp[32*k+i+j] = ERASE;
				}
			}
		} else if (i == (32*2+160+23)) {
			for(int j = 0; j < 9; j++) {
				for (int k = 0; k <= j; k++) {
					temp[32*j+i+j-k] = ERASE;
				}
			}
		}
	}
}

void draw_note() {
	switch (select) { //select: 0 - E, 1 - A, 2 - D, 3 - G, 4 - B, 5 - E
		case 0:
			for (int i = 17; i < 30; i++) {
				temp[32+i*32+11] = NOTE_C;
				if (i == 17 || i == 29) {
					for (int j = 0; j < 8; j++) {
						temp[32+i*32+12+j] = NOTE_C;
					}
					
				} else if (i == 23) {
					for (int j = 0; j < 7; j++) {
						temp[32+i*32+12+j] = NOTE_C;
					}
				}
			}
			break;
		case 1:
			for (int i = 17; i < 30; i++) {
				if (i == 17) {
					for (int j = 1; j < 8; j++) {
						temp[32+i*32+12+j] = NOTE_C;
					}
				} else if (i == 18) {
					temp[32+i*32+12] = NOTE_C;
					temp[32+i*32+20] = NOTE_C;
				} else if (i == 24) {
					for (int j = 0; j < 11; j++) {
						temp[32+i*32+11+j] = NOTE_C;
					}
				} else {
					temp[32+i*32+11] = NOTE_C;
					temp[32+i*32+21] = NOTE_C;
				}
			}
				
			break;
		case 2:
			for (int i = 17; i < 30; i++) {
				temp[32+i*32+11] = NOTE_C;
				if (i == 17 || i == 29) {
					for (int j = 0; j < 7; j++) {
						temp[32+i*32+12+j] = NOTE_C;
					}
				} else {
					if (i == 18 || i == 28) {
						temp[32+i*32+19] = NOTE_C;
					} else {
						temp[32+i*32+20] = NOTE_C;
					}
				}
			}
			break;
		case 3:
			for (int i = 17; i < 30; i++) {
				if (i == 17) {
					for (int j = 1; j < 8; j++) {
						temp[32+i*32+12+j] = NOTE_C;
					}
				} else if (i == 18) {
					temp[32+i*32+12] = NOTE_C;
					temp[32+i*32+20] = NOTE_C;
					temp[32+(i+1)*32+20] = NOTE_C;
				} else if (i == 24) {
					temp[32+i*32+11] = NOTE_C;
					for (int j = 5; j < 8; j++) {
						temp[32+i*32+11+j] = NOTE_C;
					} 
				} else if (i == 25) {
					temp[32+i*32+11] = NOTE_C;
					temp[32+i*32+19] = NOTE_C;
				} else {
					if (i > 25 && i < 28) {
						temp[32+i*32+11] = NOTE_C;
						temp[32+i*32+20] = NOTE_C;
					} else if (i == 28) {
						temp[32+i*32+19] = NOTE_C;
						temp[32+i*32+11] = NOTE_C;
					} else if (i == 29) {
						for (int j = 1; j < 8; j++) {
							temp[32+i*32+11+j] = NOTE_C;
						} 
					} else {
						temp[32+i*32+11] = NOTE_C;
					}				
				}
			}
			break;
		case 4:
			for (int i = 17; i < 30; i++) {
				temp[32+i*32+11] = NOTE_C;
				if (i == 17 || i == 29) {
					for (int j = 0; j < 7; j++) {
						temp[32+i*32+12+j] = NOTE_C;
					}
				} else if (i == 23) {
					for (int j = 0; j < 7; j++) {
						temp[32+i*32+12+j] = NOTE_C;
					}
				} else {
					if (i == 18 || i == 22 || i == 24 || i == 28) {
						temp[32+i*32+19] = NOTE_C;
					} else {
						temp[32+i*32+20] = NOTE_C;
					}
				}
			}
			break;
		case 5:
			for (int i = 17; i < 30; i++) {
				temp[32+i*32+11] = NOTE_C;
				if (i == 17 || i == 29) {
					for (int j = 0; j < 8; j++) {
						temp[32+i*32+12+j] = NOTE_C;
					}
					
				} else if (i == 23) {
					for (int j = 0; j < 7; j++) {
						temp[32+i*32+12+j] = NOTE_C;
					}
				}
			}
			break;
		default:
			for (int i = 17; i < 30; i++) { 
				temp[i] = ERASE;
			}
			break;
	}
}

void draw_degree() {
	//2
	switch (deg) { //select: 0 - E, 1 - A, 2 - D, 3 - G, 4 - B, 5 - E
		case 2:
			temp[32*26+24] = DEGREE_C;
			temp[32*26+25] = DEGREE_C;
			temp[32*26+26] = DEGREE_C;
			temp[32*26+27] = DEGREE_C;
			temp[32*26+28] = DEGREE_C;
			
			temp[32*27+28] = DEGREE_C;
			
			temp[32*28+24] = DEGREE_C;
			temp[32*28+25] = DEGREE_C;
			temp[32*28+26] = DEGREE_C;
			temp[32*28+27] = DEGREE_C;
			temp[32*28+28] = DEGREE_C;
			
			temp[32*29+24] = DEGREE_C;
			
			temp[32*30+24] = DEGREE_C;
			temp[32*30+25] = DEGREE_C;
			temp[32*30+26] = DEGREE_C;
			temp[32*30+27] = DEGREE_C;
			temp[32*30+28] = DEGREE_C;
			break;
		case 3:
			temp[32*26+24] = DEGREE_C;
			temp[32*26+25] = DEGREE_C;
			temp[32*26+26] = DEGREE_C;
			temp[32*26+27] = DEGREE_C;
			temp[32*26+28] = DEGREE_C;
			
			temp[32*27+28] = DEGREE_C;
			
			temp[32*28+24] = DEGREE_C;
			temp[32*28+25] = DEGREE_C;
			temp[32*28+26] = DEGREE_C;
			temp[32*28+27] = DEGREE_C;
			temp[32*28+28] = DEGREE_C;
			
			temp[32*29+28] = DEGREE_C;
			
			temp[32*30+24] = DEGREE_C;
			temp[32*30+25] = DEGREE_C;
			temp[32*30+26] = DEGREE_C;
			temp[32*30+27] = DEGREE_C;
			temp[32*30+28] = DEGREE_C;
			break;
		case 4:
			temp[32*26+24] = DEGREE_C;
			temp[32*26+28] = DEGREE_C;

			temp[32*27+24] = DEGREE_C;
			temp[32*27+28] = DEGREE_C;

			temp[32*28+24] = DEGREE_C;
			temp[32*28+25] = DEGREE_C;
			temp[32*28+26] = DEGREE_C;
			temp[32*28+27] = DEGREE_C;
			temp[32*28+28] = DEGREE_C;
			
			temp[32*29+28] = DEGREE_C;
			
			temp[32*30+28] = DEGREE_C;
			break;
		default:
			temp[32*26+24] = DEGREE_C;
			temp[32*26+25] = DEGREE_C;
			temp[32*26+26] = DEGREE_C;
			temp[32*26+27] = DEGREE_C;
			temp[32*26+28] = DEGREE_C;
			
			temp[32*27+28] = DEGREE_C;
			
			temp[32*28+24] = DEGREE_C;
			temp[32*28+25] = DEGREE_C;
			temp[32*28+26] = DEGREE_C;
			temp[32*28+27] = DEGREE_C;
			temp[32*28+28] = DEGREE_C;
			
			temp[32*29+24] = DEGREE_C;
			
			temp[32*30+24] = DEGREE_C;
			temp[32*30+25] = DEGREE_C;
			temp[32*30+26] = DEGREE_C;
			temp[32*30+27] = DEGREE_C;
			temp[32*30+28] = DEGREE_C;
			break;
	}
}

void draw_mode() {
	//auto
	if (mode == -1) {
		//erase_manual();
		for(int i = 0; i < 156; i++) {
			temp[i] = ERASE;
		}
		//1
		temp[32+1] = MODE_C;
		temp[32+2] = MODE_C;
		temp[32+3] = MODE_C;
		temp[32+5] = MODE_C;
		temp[32+7] = MODE_C;
		temp[32+9] = MODE_C;
		temp[32+10] = MODE_C;
		temp[32+11] = MODE_C;
		temp[32+13] = MODE_C;
		temp[32+14] = MODE_C;
		temp[32+15] = MODE_C;
		//2
		temp[32+33] = MODE_C;
		temp[32+35] = MODE_C;
		temp[32+37] = MODE_C;
		temp[32+39] = MODE_C;
		temp[32+42] = MODE_C;
		temp[32+45] = MODE_C;
		temp[32+47] = MODE_C;
		//3
		temp[32+65] = MODE_C;
		temp[32+66] = MODE_C;
		temp[32+67] = MODE_C;
		temp[32+69] = MODE_C;
		temp[32+71] = MODE_C;
		temp[32+74] = MODE_C;
		temp[32+77] = MODE_C;
		temp[32+79] = MODE_C;
		//4
		temp[32+97] = MODE_C;
		temp[32+99] = MODE_C;
		temp[32+101] = MODE_C;
		temp[32+102] = MODE_C;
		temp[32+103] = MODE_C;
		temp[32+106] = MODE_C;
		temp[32+109] = MODE_C;
		temp[32+110] = MODE_C;
		temp[32+111] = MODE_C;
	}
	
	
	if (mode == 1) { //manual 
		for(int i = 0; i < 144; i++) {
			temp[i] = ERASE;
		}
		
		temp[32+1] = MODE_C;
		temp[32+2] = MODE_C;
		temp[32+4] = MODE_C;
		temp[32+5] = MODE_C;
		temp[32+33] = MODE_C;
		temp[32+35] = MODE_C;
		temp[32+37] = MODE_C;
		temp[32+65] = MODE_C;
		temp[32+69] = MODE_C;
		temp[32+97] = MODE_C;
		temp[32+101] = MODE_C;

		temp[32+7] = MODE_C;
		temp[32+8] = MODE_C;
		temp[32+9] = MODE_C;
		temp[32+39] = MODE_C;
		temp[32+41] = MODE_C;
		temp[32+71] = MODE_C;
		temp[32+72] = MODE_C;
		temp[32+73] = MODE_C;
		temp[32+103] = MODE_C;
		temp[32+105] = MODE_C;
		
		temp[32+11] = MODE_C;
		temp[32+14] = MODE_C;
		temp[32+43] = MODE_C;
		temp[32+44] = MODE_C;
		temp[32+46] = MODE_C;
		temp[32+75] = MODE_C;
		temp[32+77] = MODE_C;
		temp[32+78] = MODE_C;
		temp[32+107] = MODE_C;
		temp[32+110] = MODE_C;
		
		temp[32+16] = MODE_C;
		temp[32+18] = MODE_C;
		temp[32+48] = MODE_C;
		temp[32+50] = MODE_C;
		temp[32+80] = MODE_C;
		temp[32+82] = MODE_C;
		temp[32+112] = MODE_C;
		temp[32+113] = MODE_C;
		temp[32+114] = MODE_C;
		
		temp[32+20] = MODE_C;
		temp[32+21] = MODE_C;
		temp[32+22] = MODE_C;
		temp[32+52] = MODE_C;
		temp[32+54] = MODE_C;
		temp[32+84] = MODE_C;
		temp[32+85] = MODE_C;
		temp[32+86] = MODE_C;
		temp[32+116] = MODE_C;
		temp[32+118] = MODE_C;
		
		temp[32+24] = MODE_C;
		temp[32+56] = MODE_C;
		temp[32+88] = MODE_C;
		temp[32+120] = MODE_C;
		temp[32+121] = MODE_C;
		temp[32+122] = MODE_C;
	}
}

void find_closest_note(float idx) {
	if (idx >= 70 && idx < 100) { //case 0
		select = 0;
		deg = 2;
		check_string(idx);
	} if (idx >= 100 && idx < 130) {
		select = 1;
		deg = 2;
		check_string(idx);
	} if (idx >= 130 && idx < 165) {
		select = 2;
		deg = 3;
		check_string(idx);
	} if (idx >= 165 && idx < 220) {
		select = 3;
		deg = 3;
		check_string(idx);
	} if (idx >= 220 && idx < 270) {
		select = 4;
		deg = 3;
		check_string(idx);
	} if (idx >= 270 && idx < 400) {
		select = 5;
		deg = 4;
		check_string(idx);
	}
}

int get_margin() { 
	switch(select) {
		case 0:
			return 3;
			break;
		case 1:
			return 7;
			break;
		case 2:
			return 7;
			break;
		case 3:
			return 6;
			break;
		case 4:
			return 9;
			break;
			break;
		case 5:
			return 17;
			break;
		default:
			return 3;
			break;		
	}
}