/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "fat/disk/spi_flash.h"
#include "utilities.h"
#include <FreeRTOS.h>
#include "task.h"
#include "queue.h"
#include "io.hpp"
#include "gpio.hpp"
#include "tasks.hpp"
#include "vector.hpp"
#include "printf_lib.h"
#include "examples/examples.hpp"
#include "lpc_pwm.hpp"


#include "LPC17xx.h"
#include "bio.h"
#include "adc0.h"
SemaphoreHandle_t sem = 0;
char pwm_freq[10];
int id = 0;
volatile double freq = 0;
volatile int dir = 0; // 0 anticlockwise
static int ijk = 0;
uint8_t reg = 0x3;
float Pi = 3.14159;
uint8_t bufferx[2] = {0};
uint8_t buffery[2] = {0};
uint16_t sensor_data[] =
{
        0,
        409,
        819,
        1228,
        1638,
        2047,
        2457,
        2866,
        3269,
        3685,
        4095
};
void des(void) {
LPC_PWM1->PCR &= ~(1 << (1 + 9));
    LPC_PINCON->PINSEL4 &= ~(3 << ((1)*2));
}
#if 1
extern "C"{
void receive_char(char data)
{
    //u0_dbg_printf("%c",data);
    printf("%c\n", data);
    if(data == 'x') {
        dir = 0;
        pwm_freq[id] = '\0';
        sscanf(pwm_freq, "%lf", &freq);
        printf("freq %lf\n", freq);
        id = 0;
        xSemaphoreGive(sem);
        return ;
    }
    if(data == 'y') {
        dir = 1;
         pwm_freq[id] = '\0';
         sscanf(pwm_freq, "%lf", &freq);
         printf("freq %lf\n", freq);
         id = 0;
         xSemaphoreGive(sem);
         return ;
     }

    pwm_freq[id] = data;
    id++;
}

#if 1
void UART2_IRQHandler(void)
{
    receive_char(LPC_UART2->RBR);
}
#endif

}
class uart2_3: public scheduler_task
{
    public:
        uart2_3(uint8_t priority):scheduler_task("uart2_3", 4096, priority)
        {

        }
#if 1
        bool init(void)
        {
            uint32_t uart2_clock = (0x1 << 24);
            uint32_t uart2_clk_mul = (0x1 << 16);
            uint32_t reset_uart2_clk = ~(0x3 << 16);
            uint32_t uart2_tx = (0x2 << 16); // port2.8
            uint32_t uart2_rx = (0x2 << 18); // port2.9
            // init your uart here

            // 1. initialize uart2 co-processor clock
            LPC_SC->PCONP |= uart2_clock;

            // 2. Enable uart peripheral clock div we are using div = 1
            LPC_SC->PCLKSEL1 = (LPC_SC->PCLKSEL1 & reset_uart2_clk)
                             | uart2_clk_mul;

            uint32_t enable_dlab = (0x1 << 7);
            //uint32_t disable_dlab = ~(0x1 << 7);

            uint32_t bit_8 = 0x3;
            // 4. set the baud rate
            // we want to set baud rate to 115200
            // I. first enable DLAB bit in uart2_lcr to access DLL and DLM divisor
            LPC_UART2->LCR = enable_dlab; // by default 0 is 1 stop bit so no need to set it
            {
                uint32_t baud_rate = 115200;

                uint16_t data = sys_get_cpu_clock()/(16*baud_rate)+0.5;

                LPC_UART2->DLL = data; // lsb 8 bit data
                LPC_UART2->DLM = data >> 8; // msb 8 bit
            }
            LPC_UART2->LCR = bit_8; //disable dlab and set 8 bit mode
            LPC_UART2->FCR = 1; //enable fifo

            //3. select uart2 rx, tx pin, need to add common of both the boards
            LPC_PINCON->PINSEL4 = (LPC_PINCON->PINSEL4 & ~(0x3 << 16))
                                 | uart2_tx;

            LPC_PINCON->PINSEL4 = (LPC_PINCON->PINSEL4 & ~(0x3 << 18))
                                 | uart2_rx;
            //LPC_UART2->IER = 0x1; // enable interrupt for RBR
            //NVIC_EnableIRQ(UART2_IRQn);


            // initialize the lsm303 sensor

            I2C2::getInstance().writeReg(0x3c, 0, 0x9c);

            I2C2::getInstance().writeReg(0x3c, 0x2, 0x0);
#if 0
            PWM pw1(PWM::pwm2, 250);
             pw1.set(37.5);
             //vTaskDelay(1000);
#endif
            return true;
        }
#endif

        char get_char2(void)
        {
            while(!(LPC_UART2->LSR & 0x1));

            return (LPC_UART2->RBR & 0xff);
        }

        void put_char2(char ch)
        {
            while(!(LPC_UART2->LSR & (0x1 << 5)));

            LPC_UART2->THR = ch; // will put into lsb 8 bit into THR

        }

        void put_string(char *p)
        {
            while(*p)
            {
                put_char2(*p);
                p++;
            }

        }

        bool run(void* p)
        {

            if (ijk == 0)
            {
                PWM pw1(PWM::pwm2, 50);
                pw1.set(7.5);
                vTaskDelay(1000);
            }


            // reading 1 values
            char data[10];


            Pi = 3.14159;
            memset(buffery,'\0', sizeof(buffery));
            memset(bufferx,'\0', sizeof(bufferx));
            memset(data, '\0', sizeof(data));
            int ii = 0;
            float ang[5]={0};
            volatile double angle =0;
            volatile uint8_t xh, xl, yh, yl;
           do{

                   reg = 0x3;
                   I2C2::getInstance().readRegisters(0x3c, reg, (uint8_t*)&xh, 1);
                   I2C2::getInstance().readRegisters(0x3c, reg+1, (uint8_t*)&xl, 1);

                   volatile int16_t valx = (int16_t)(xl | (int16_t)(xh << 8));
                   reg = 0x7;
                   I2C2::getInstance().readRegisters(0x3c, reg, (uint8_t*)&yh, 1);
                   I2C2::getInstance().readRegisters(0x3c, reg+1, (uint8_t*)&yl, 1);
                   volatile int16_t valy = (int16_t)(yl | (int16_t)(yh << 8));
                   valx = (float)valx/1100.0F  * 100;// converting gauss into micro-tesla
                   valy = (float)valy/1100.0F  * 100;
                    angle = atan2((valy), (valx)); // angle in radian

                   printf("angle %f \n", (angle*180)/Pi);

                   vTaskDelay(30);

                   I2C2::getInstance().writeReg(0x3c, 0x2, 0x0);

            }
            while(ii++ < 5);
           ii=0;

           if(ijk != 0){
               PWM pw2(PWM::pwm2, 250);
               pw2.set(37.5); // set it neutral position
               vTaskDelay(300);
           }
           ijk++;
            float avg = ang[0];
#if 1
            int d = 1;
            for(int ij = 1; ij < 5; ij++)
            {
                if((ang[ij] < avg-0.087) && (ang[ij] > avg+0.087))
                {
                    d--;
                    continue;
                }

                avg += ang[ij];
                avg /=(d+1);
                d++;

            }
#endif

            snprintf(data, sizeof(data)-1, "%f", angle);
            data[strlen(data)] = 'x';
            //data[strlen(data)+1] = '\0';
printf("sending\n");
            put_string(data); // send angle to ARMSVR
            printf("sent\n");
//            while(!xSemaphoreTake(sem, portMAX_DELAY));
            id = 0;
            while(1)
            {
                char data1 = get_char2();

                if(data1 == 'x') {
                        dir = 0;
                        pwm_freq[id] = '\0';
                        sscanf(pwm_freq, "%lf", &freq);
                        printf("freq %lf\n", freq);
                        break;
                    }
                    if(data1 == 'y') {
                        dir = 1;
                         pwm_freq[id] = '\0';
                         sscanf(pwm_freq, "%lf", &freq);
                         printf("freq %lf\n", freq);
                         break;
                     }
                    printf("data %c\n", data1);
                    pwm_freq[id] = data1;
                    id++;
            }

            // init pwm
            PWM pwm(PWM::pwm2, (uint32_t)freq); // init pwm with 0 Hz
           float duty_aclock = (2/((1/freq)*10)); // anticlockwise
           float duty_clock = (1/((1/freq)*10)); // clockwise
           //float duty_neutral = (1.5)/((1/freq)*10); // for 1.5 ms neutral

           if(dir == 1)
               pwm.set(duty_aclock);
           else
               pwm.set(duty_clock);

           printf("dir - %d\n", dir);


            vTaskDelay(30);

            des(); // end pwm
            //pwm.set(duty_neutral);


            return true;
        }
};
#endif

#if 0
void task2(void *p)
{
    // reading 1 values
   volatile uint8_t xh, xl, yh, yl;
    I2C2::getInstance().writeReg(0x3c, 0, 0x1c);
    I2C2::getInstance().writeReg(0x3c, 0x1, 0x20);
    I2C2::getInstance().writeReg(0x3c, 0x2, 0x0);
    while(1) {

        reg = 0x3;
        //I2C2::getInstance().writeReg(0x3c, 0x2, 0x0);
        I2C2::getInstance().readRegisters(0x3c, reg, (uint8_t*)&xh, 1);
        I2C2::getInstance().readRegisters(0x3c, reg+1, (uint8_t*)&xl, 1);
        vTaskDelay(30);

        volatile int16_t valx = (int16_t)(xl | (int16_t)(xh << 8));
        reg = 0x7;
        I2C2::getInstance().readRegisters(0x3c, reg, (uint8_t*)&yh, 1);
        I2C2::getInstance().readRegisters(0x3c, reg+1, (uint8_t*)&yl, 1);
        volatile int16_t valy = (int16_t)(yl | (int16_t)(yh << 8));
        valx = (float)valx/1100.0F  * 100;// converting gauss into micro-tesla
        valy = (float)valy/1100.0F  * 100;
        volatile double angle = (atan2((valy), (valx))*180)/Pi; // angle in radian
        printf("valy %x valx %x\n",valy, valx);
        if(angle < 0)
                   angle = 360 + angle;

        printf("angle %f \n", angle);

        vTaskDelay(30);

        I2C2::getInstance().writeReg(0x3c, 0x2, 0x0);
        vTaskDelay(300);
    }
}
#endif

#if 0
void task1(void *p)
{
    //int i = 50;
    PWM pwm(PWM::pwm2,50);
    while(1) {
       // pwm.set(7.5);  // anti clockwise
        //vTaskDelay(2000);
        pwm.set(10); // anticlockwise
        vTaskDelay(2000);
        pwm.set(5); // clockwise
        vTaskDelay(2000);

        //PWM pwm(PWM::pwm1, i);
        //i+50;
        }
}
#endif

int main(void)
{
    sem = xSemaphoreCreateBinary();


    // Let's initialize the PWM signal on PWM1-2
#if 0
xTaskCreate(task1, "task", 1024, NULL, 1, NULL);
vTaskStartScheduler();
#endif
    //xSemaphoreTake(sem, portMAX_DELAY);
    //scheduler_add_task(new terminalTask(1));
#if 1
    scheduler_add_task(new uart2_3(1));
    scheduler_start();
#endif
#if 0
    xQueueSend();
    xQueueReceive();
#endif
    return 0;
}
