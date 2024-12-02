/*!
    \file  main.c
    \brief USB CDC ACM device

    \version 2019-6-5, V1.0.0, demo for GD32VF103
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/


#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include "lio_lcd.h"
#include "lcd_light_gui.h"
#include <math.h>
#include "drivers.h"

#define GRAPH_HEIGHT    30
#define BITMASK 0xFFFFFFF8
#define RED 	0xF800
#define GREEN	0x07E0
#define BLUE	0x001F
#define TID     1000
#define DISPLAY_WIDTH 160
#define ANALOG_PORT              GPIOA
#define ANALOG_PIN               GPIO_PIN_3
#define ANALOG_CHANNEL           ADC_CHANNEL_3


void init_ADC_pin3();


void calculateAverageAcc(float acctbl[], float reduced_acctbl[], int num) {
    int group_size = num / DISPLAY_WIDTH;
    int i, j;

    for (i = 0; i < DISPLAY_WIDTH; i++) {
        float sum = 0;
        int start_index = i * group_size;
        for (j = 0; j < group_size; j++) {
            sum += acctbl[start_index + j];
        }
        reduced_acctbl[i] = sum / group_size;
    }
}

void drawCurve(float reduced_acctbl[], int baseline_y) {
    for (int i = 0; i < DISPLAY_WIDTH - 1; i++) {
        int x1 = i;
        int y1 = baseline_y - (int)((reduced_acctbl[i]-9.82) * 3);  // Skalning av accelerationen för att passa skärmen
        int x2 = i + 1;
        int y2 = baseline_y - (int)((reduced_acctbl[i + 1]-9.82) * 3);  // Skalning av accelerationen för att passa skärmen
        LCD_DrawLine(x1, y1, x2, y2, WHITE); // Rita linje mellan punkterna
    }
}





int main(void)
{
    int num = 0;
    float key1, key2, pKey = -1, mode = 0, power, newPower, count = 0, t = 0;
    float mass = 0, A = 0;
    float G = 9.82; 
    float acc = 0;
    float newMass = 0;
    float acctbl[4096] = {0};
    float reduced_acctbl[DISPLAY_WIDTH] = {0};
    int buttonPressed1 = 1, buttonPressed2 = 1, battery_percentage = 0;
    float maxAcceleration = 0;
    uint16_t analog_read = 0;


    /* The related data structure for the IMU, contains a vector of x, y, z floats*/
    mpu_vector_t vec, vec_temp;
    /* for lcd */
    uint16_t line_color;

    /* Initialize pins for I2C */
    uint32_t button;
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_I2C0);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
    gpio_init(ANALOG_PORT, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, ANALOG_PIN);
    init_ADC_pin3();
    /* Initialize the IMU (Notice that MPU6500 is referenced, this is due to the fact that ICM-20600
       ICM-20600 is mostly register compatible with MPU6500, if MPU6500 is used only thing that needs 
       to change is MPU6500_WHO_AM_I_ID from 0x11 to 0x70. */
    mpu6500_install(I2C0);
    
    /* Initialize LCD */
    lio_lcd_config_t lcd_conf = {0x00, 160, 128, 0, 0, RED | GREEN};

    lio_lcd_init(lcd_conf);

    lio_lcd_clear(0);
    
    lio_lcd_set_background(0);

    t5omsi();  // Initialize timer5 1kHz

      
    while(1){
        analog_read = ADC_RDATA(ADC0); 
        battery_percentage = (analog_read-2389)/17;

        count++;
        if(gpio_input_bit_get(GPIOA, GPIO_PIN_5) == 0 && buttonPressed1 == 0 && (mode == -1 || mode == 0)) {
            mass += 5;
            power = mass * 9.82;
            buttonPressed1 = 1; 
        }
        if(gpio_input_bit_get(GPIOA, GPIO_PIN_5) == 1 && buttonPressed1 == 1) {
            buttonPressed1 = 0;
        }
        if(gpio_input_bit_get(GPIOA, GPIO_PIN_6) == 0 && buttonPressed2 == 0) {
            if(mode == 0) {
                mode = 1;
                buttonPressed2 = 1;
            } else if(mode == 1) {
                mode = -1;
                buttonPressed2 = 1;
            } else if(mode == -1) {
                maxAcceleration = 0;
                mass = 0;
                num = 0;
                newMass = 0;
                for(int i = 0; i < 4096; i++) {
                    acctbl[i] = 0;
                }
                A = 0;
                mode = 0;
                buttonPressed2 = 1;
                lio_lcd_clear(0);
            }
        }
        if(gpio_input_bit_get(GPIOA, GPIO_PIN_6) == 1 && buttonPressed2 == 1) {
            buttonPressed2 = 0;
        }        

        if (mode == 1) {
            mpu6500_getAccel(&vec);
            acc = (vec.z / 4096) * 9.82;
            acctbl[num % 4096] = acc; // Se till att inte överskrida arraygränsen
            num = (num + 1) % 4096; // Håll 'num' inom gränsen av acctbl storlek
            if(acc > maxAcceleration) maxAcceleration = acc;
            if (num == 4095) { // När arrayen är full, beräkna och rita
                mode = -1;
                calculateAverageAcc(acctbl, reduced_acctbl, 4096);
                lio_lcd_clear(0); // Rensa skärmen
                drawCurve(reduced_acctbl, 60); // Rita kurvan från mitten av skärmen (y=64)
            }
        }

        if(mode == -1) {
            newPower = mass * maxAcceleration;
            newMass = (power + newPower) / G;
            if(num < 160) {
                for (int i = 0; i < num - 1; i++) {
                    int x1 = i;
                    int y1 = 60 - (float)((acctbl[i]-9.82) * 2);
                    int x2 = i + 1;
                    int y2 = 60 - (float)((acctbl[i + 1]-9.82) * 2);
                    LCD_DrawLine(x1, y1, x2, y2, WHITE);
                }
            } else {
                calculateAverageAcc(acctbl, reduced_acctbl, num);
                drawCurve(reduced_acctbl, 64); // Rita kurvan från mitten av skärmen (y=64)
            }
        }

        LCD_ShowNum(40, 60, num, 3, WHITE);
        LCD_ShowStr(0,0,"massa:",WHITE,0);
        LCD_ShowNum(20, 40, mass, 3, WHITE);
        LCD_ShowStr(0,0,"maxvikt:",WHITE,0);
        LCD_ShowNum(50, 40, newMass, 3, WHITE);
        LCD_ShowStr(0,0,"maxaccs:",WHITE,0);
        LCD_ShowNum(80, 40, maxAcceleration, 3, WHITE);
        LCD_ShowNum(80, 40, battery_percentage, 3, WHITE);
        //LCD_ShowChar(1,2,37,0,WHITE);
    }
}

void init_ADC_pin3(){
    
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC0);

    /* Select the clock frequency that will be used for the ADC core. Refer to README for more info on what to select. */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);

    /* Reset ADC0 configuration. GD32VF103 has two internal ADCs (ADC0, ADC1). */
    adc_deinit(ADC0);

    /* Set the ADCs to work independently. Refer to the manual for the different parallel modes available. */
    adc_mode_config(ADC_MODE_FREE);

    /* Set the conversion mode to continuous. Continious mode lets the ADC take measurements continiously without
       an external trigger. */
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);

    /* Sets where padding is applied to the measurement. Data alignment right puts padding bits above MSB */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    /* Selects how many channels to convert each time. This can be used to "queue" multiple channels. Here just one channel is selected. */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);

    /* Set the channel as the first "queued" conversion each time the ADC is activated. */
    adc_regular_channel_config(ADC0, 0, ANALOG_CHANNEL, ADC_SAMPLETIME_13POINT5);

    /* Since we are using continious conversion we do not want to use an external trigger. */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);

    /* Enable ADC.*/
    adc_enable(ADC0);

    /* Let ADC stabilize */
    delay_1ms(1);

    /* Calibrates the ADC against an internal source. */
    adc_calibration_enable(ADC0);

    /* Start converting */
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
}
