/*
 * 017rtc_lcd.c
 *
 *  Created on: 20 Nov 2023
 *      Author: Eliran.Malki
 */




#include<stdio.h>
#include "ds1307.h"
#include "lcd.h"



#define SYSTICK_TIM_CLK   16000000UL



void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSTRVR = (uint32_t*)0xE000E014;
	uint32_t *pSTCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSTRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSTRVR |= count_value;

    //do some settings
    *pSTCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSTCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSTCSR |= ( 1 << 0); //enables the counter

}


//static void mdelay(uint32_t cnt)
//{
//	for(uint32_t i=0 ; i < (cnt * 1000); i++);
//}


char* get_day_of_week(uint8_t i)
{
	char* days[] = { "Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};

	return days[i-1];
}


void number_to_string(uint8_t num , char* buf)
{

	if(num < 10){
		buf[0] = '0';
		buf[1] = num+48;
	}else if(num >= 10 && num < 99)
	{
		buf[0] = (num/10) + 48;
		buf[1]= (num % 10) + 48;
	}
}



//hh:mm:ss
char* time_to_string(RTC_time_t *rtc_time)
{
	static char buf[9];

	buf[2]= ':';
	buf[5]= ':';

	number_to_string(rtc_time->hours,buf);
	number_to_string(rtc_time->minutes,&buf[3]);
	number_to_string(rtc_time->seconds,&buf[6]);

	buf[8] = '\0';

	return buf;

}

//dd/mm/yy
char* date_to_string(RTC_date_t *rtc_date)
{
	static char buf[9];

	buf[2]= '/';
	buf[5]= '/';

	number_to_string(rtc_date->date,buf);
	number_to_string(rtc_date->month,&buf[3]);
	number_to_string(rtc_date->year,&buf[6]);

	buf[8]= '\0';

	return buf;

}

int main(void)
{

	RTC_time_t current_time;
	RTC_date_t current_date;

	printf("RTC test\n");

	lcd_init();

	lcd_print_string("RTC Test...");

	if(ds1307_init()){
		printf("RTC init has failed\n");
		while(1);
	}

	init_systick_timer(1);

	current_date.day = SUNDAY;
	current_date.date = 24;
	current_date.month = 12;
	current_date.year = 23;

	current_time.hours = 11;
	current_time.minutes = 59;
	current_time.seconds = 30;
	current_time.time_format = TIME_FORMAT_12HRS_PM;
	ds1307_set_current_date(&current_date);
	ds1307_set_current_time(&current_time);

	ds1307_get_current_time(&current_time);
	ds1307_get_current_date(&current_date);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_12HRS_PM){
		am_pm = (current_time.time_format) ? "PM" : "AM";
		printf("Current time is : %s %s\n",time_to_string(&current_time),am_pm); //print in format 10:30:00 PM
	}
	else{
		printf("Current time is : %s \n",time_to_string(&current_time));//print in format 10:30:00
	}

	// print the date in format 24/12/23 <SUNDAY>

	printf("Current date is : %s <%s>\n",date_to_string(&current_date),get_day_of_week(current_date.day));


	while(1);

	return 0;
}

void SysTick_Handler(void)
{
	RTC_time_t current_time;
	RTC_date_t current_date;
	ds1307_get_current_time(&current_time);
	ds1307_get_current_date(&current_date);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS){
		am_pm = (current_time.time_format) ? "PM" : "AM";
		printf("Current time is : %s %s\n",time_to_string(&current_time),am_pm); //print in format 10:30:00 PM
	}
	else{
		printf("Current time is : %s \n",time_to_string(&current_time));//print in format 10:30:00
	}

	// print the date in format 24/12/23 <SUNDAY>

	printf("Current date is : %s <%s>\n",date_to_string(&current_date),get_day_of_week(current_date.day));
}




