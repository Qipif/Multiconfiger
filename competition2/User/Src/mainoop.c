#include "main.h"
#include "ad9833.h"
#include "myfft.h"
#include "mydds.h"
#include "oled.h"
#include "encoder.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

//ad9833
AD9833_Handler ad9833Single;

void main_init(void)
{
	OLED_Init();
	OLED_ShowString(1, 1, "Hip");

    //ad9833
    AD9833_Init(&ad9833Single, wave_square, 10000, 0, &hspi2, GPIOC, GPIO_PIN_0); //频率、相位
}


void main_loop(void)
{

}


