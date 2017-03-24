#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
//#include <time.h>
#include "ff.h"
#include "diskio.h"
#include "hd44780.h"
#include "uart_functions.h"
#define pi 3.14
#define diameter 1.167

#define CMD_GO_IDLE_STATE 0x00

#define R1_IDLE_STATE 0

#define TIMER_OFF 0b00000000
#define TIMER_ON  0b00000101

#define TIMER_TO_TACH (60UL * 10000000UL / 512UL)


char lcd_string_display[32];  //char array for both lines of 16x2 LCD
volatile char lcd_str1[16] = "S:      W:  .   "; //top line of LCD with set values
volatile char lcd_str2[16] = "LAP:            "; //bottom line of LCD with set values
char get_speed[3]; //char array for speed values
char get_pwr[4];   //char array for power consumption values

uint16_t hours=0;
uint16_t mins=0;
uint16_t secs=0;
uint16_t msecs=0;

volatile uint16_t count=0;    //Main revolution counter
volatile uint16_t rpm=0;   //Revolution per minute
volatile uint16_t rps=0;   //Revolution per second
char speed_data[10];

char speed_char[20];
char pow_char[20];

//volatile uint16_t second_count = 0;

FATFS fs;
FIL fil;
UINT bW;
char header_buffer[50];
char data_buffer[60];

//GPS variables
char frame[200];
char GNSSrunstatus[2];
char Fixstatus[2];
char UTCdatetime[19];
char latitude[11];
char logitude[12];
char altitude[9];
char speedOTG[7];
char course[7];
char fixmode[2];
char HDOP[5];
char PDOP[5];
char VDOP[5];
char satellitesinview[3];
char GNSSsatellitesused[3];
char GLONASSsatellitesused[3];
char cn0max[3];
char HPA[7];
char VPA[7];



void send_byte(uint8_t b)
{
	SPDR = b;
	// wait for byte to be shifted out
	while(!(SPSR & (1 << SPIF)));
	SPSR &= ~(1 << SPIF);
}


uint8_t recv_byte()
{
	/* send dummy data for receiving some */
	SPDR = 0xff;
	while(!(SPSR & (1 << SPIF)));
	SPSR &= ~(1 << SPIF);

	return SPDR;
}

uint8_t send_command(uint8_t command, uint32_t arg)
{
	uint8_t response;

	/* wait some clock cycles */
	recv_byte();

	/* send command via SPI */
	send_byte(0x40 | command);
	send_byte((arg >> 24) & 0xff);
	send_byte((arg >> 16) & 0xff);
	send_byte((arg >> 8) & 0xff);
	send_byte((arg >> 0) & 0xff);
	switch(command)
	{
		case CMD_GO_IDLE_STATE:
		send_byte(0x95);
		break;
		default:
		send_byte(0xff);
		break;
		
	}
	
	/* receive response */
	uint8_t i;
	for(i = 0; i < 10; ++i)
	{
		response = recv_byte();
		if(response != 0xff)
		break;
	}

	return response;
}



// Initialize the SPI port
//void SPI_Init(){
uint8_t SPI_Init(){
	
	//DDRB = 0b00000111;		// Set Pin0, Pin1 and Pin2 of PORTB as output
	//SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);	// SPI enabled in Master mode, clock rate set at fck/16
	DDRB |= (1<<DDB2);
	DDRB |= (1<<DDB1);
	DDRB |= (1<<DDB0);
	DDRB &= ~(1<<DDB3);
	DDRB |= (1<<DDB4);
	DDRB |= (1<<DDB5);
	DDRB |= (1<<DDB6);
	DDRB |= (1<<DDB7);

	//PORTB |= (1<<PORTB0);

	SPCR |= (0<<SPIE) | (1<<SPE) | (0<<DORD) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA) | (1<<SPR1) | (1<<SPR0);

	SPSR &= ~(1<<SPI2X);

	//reset card
	uint16_t i;
	uint8_t response;
	for(i = 0; ; ++i)
	{
		response = send_command(CMD_GO_IDLE_STATE, 0);
		if(response == (1 << R1_IDLE_STATE))
		break;

	}

	return 1;
}


void adc_init(void)
{
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	ADMUX |= (1<<REFS0);
	//MCUCR |= (1<<SM0);
}

uint16_t read_adc(int ch_sel){
	
	//ch_sel =  ch_sel & 0b00000111;
	//ADMUX |= ch_sel;
	if(ch_sel == 0){
		ADMUX |= 0b01000000;				//(0<<MUX0);
	}
	else if(ch_sel == 1){
		ADMUX |= 0b01000001;							//(1<<MUX0);
	}
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));
	return ADC;
}

/****************************************/



volatile uint16_t state[8] = {0,0,0,0,0,0,0,0}; //check state of buttons


uint8_t chk_buttons(uint8_t buttons) {
//static uint16_t state[8] = {0,0,0,0,0,0,0,0};
state[buttons] = (state[buttons] << 1) | (!bit_is_clear(PIND, buttons))|0xE000;

if(state[buttons] == 0xF000) 
return 1;

return 0;
}



uint16_t speed;
uint16_t r2;

/*************************************/
void tcnt1_init(void){        //initialize timer
	TCCR1B |= (1<<CS10)|(1<<CS12)|(1<<WGM12);
	OCR1A=7811;
	TIMSK |= (1<<OCIE1A);
	TIFR |= (1<<OCF1A);
}


void tcnt0_init(void){        //initialize timer

	TCCR0 |=(1<<CS02);
	OCR0=5;
	TIMSK |= (1<<TOIE0);
	TIFR |= (1<<TOV0);
}
/*************************************/

void tcnt2_init(void){        //initialize timer

	TIMSK |= (1<<TOIE2);
	TCCR2 |= (1<<CS22);
	TIFR |= (1<<TOV2);
}	



ISR(TIMER1_COMPA_vect){

	rps = count;
	rpm = rps*60;
	count = 0;

}


char print_sp_int[10];	
char print_sp_dec[10];
char print_pow_int[10];
char print_pow_dec[10];

ISR(TIMER2_OVF_vect){

	uint16_t adc_v,adc_i;
	float temp_v, temp_i;
	float current;
	float pow;
	
	float act_speed;
	float after_dec;
	int before_dec;
	int new_after_dec;
	int z;
	int temp;
	int k;
	int y;
	//char print_sp_int[10];	
	//char print_sp_dec[10];
	char point = '.';
	static uint8_t tme =0;
	tme++;
		
	if (tme > 225) {           //take the data when tme is greater than least 1 sec. 
	
	act_speed = (diameter * pi * rpm *60)/5280;
	before_dec = act_speed;
	
	after_dec = act_speed - before_dec;
	new_after_dec = after_dec * 100;
		
	itoa(before_dec,print_sp_int,10);
	
	itoa(new_after_dec,print_sp_dec,10);
	
	//itoa(act_speed, speed_data, 10); // convert to a string 
	
	z = 0;	
	for(z=0; z < strlen(print_sp_int); z++){
		
		lcd_str1[z+2] = print_sp_int[z];
	}
	
	
	temp = z;
	
	lcd_str1[temp+2] = '.';
	
	temp++;
	y=0;
	
	for(y=0; y<2; y++){
		
		lcd_str1[temp+2] = print_sp_dec[y];
		temp++;
	
	}
	
		
	sprintf(speed_char,"%s%c%s",print_sp_int,point,print_sp_dec);
	/*for(k=0; k++; k < strlen(print_dec)){
		//lcd_str1[]
		//print_dec[k] = '\0';
	}*/
	adc_v = read_adc(0);
	adc_i = read_adc(1);
	
	temp_v = adc_v/24;
	//temp_i = adc_i/24;
	
	current = (temp_v - 9.7)/10;
	
	before_dec = pow;
	
	after_dec = pow - before_dec;
	new_after_dec = after_dec * 10000;
		
	itoa(before_dec,print_pow_int,10);
	
	itoa(new_after_dec,print_pow_dec,10);
	
	
		
	/*z = 0;	
	for(z=0; z < strlen(print_pow_int); z++){
		
		lcd_str1[z+10] = print_pow_int[z];
	}
	
	
	temp = z;
	
	lcd_str1[temp+10] = '.';
	
	temp++;
	y=0;
	
	for(y=0; y<3; y++){
		
		lcd_str1[temp+10] = print_pow_dec[y];
		temp++;
	
	}	
	*/	
		
	sprintf(pow_char,"%s%c%s",print_pow_int,point,print_pow_dec);
		
		
	//lcd_str1[2] = speed_data[0]; //get 1st character send to position 2 line 1
	//lcd_str1[3] = speed_data[1]; //get 2nd character send to position 3 line 1
	//lcd_str1[4] = speed_data[2]; //get 3rd character send to position 5 line 1
	//lcd_str1[5] = speed_data[3];	
	itoa(r2,get_pwr, 10);
	lcd_str1[10] = get_pwr[0];  //get 1st character send to position 10 line 1
	lcd_str1[11] = get_pwr[1];  //get 2nd character send to position 11 line 1
	lcd_str1[12] = get_pwr[2];  //get 3rd character send to position 12 line 1
	lcd_str1[14] = get_pwr[3];  //get 4th character send to position 14 line 1
	
		
		
	
		
	tme = 0;
	
	}


}



unsigned int tachTimer;
//int rotation_count;
int rot_time;
ISR(INT0_vect){
	
	count++;

}


char gps_buff[200];
//char gps_lat[20];
//char gps_long[20];
//char gps_alt[20];

ISR(USART0_RX_vect){
	
	//int count=0;
/*	int f;
	for(f = 0; f < 200; f++){
		gps_buff[f] = '\0'
	
	}
*/
	//memset(gps_buff, '\0', sizeof(gps_buff));
	static uint8_t gps_int;
	static uint8_t i;
	gps_int = UDR0;
	gps_buff[i++] = gps_int;

	if(i == 95)
		i=0;

}


ISR(TIMER0_OVF_vect){
    char temp[6];
	
		msecs++;
		
		if(msecs == 1000){
			secs++;
			msecs=0;
		}

        if(secs == 60){    
            mins++;         //increment the minute
			secs=0;
		}
		if(mins == 60){    
            hours++;         //increment the minute
            //update_curr_time();
			mins=0;
		}

	
	/*itoa(mins,temp,10);
	
	lcd_str2[7] = temp[0];
	lcd_str2[8] = temp[1];
		
	lcd_str2[9] = ':';
	
	itoa(secs,temp,10);
	lcd_str2[10] = temp[0];
	lcd_str2[11] = temp[1];
	
	lcd_str2[12] = ':';
	
	itoa(msecs,temp,10);
	lcd_str2[13] = temp[0];
	lcd_str2[14] = temp[1];
	lcd_str2[15] = temp[2];
	*/
	//update_time();
}





static char *strtok_single(char *str, char const *delims)
{
    static char  *src = NULL;
    char  *p,  *ret = 0;

    if (str != NULL)
        src = str;

    if (src == NULL || *src == '\0')    // Fix 1
        return NULL;

    ret = src;                          // Fix 2
    if ((p = strpbrk(src, delims)) != NULL)
    {
        *p  = 0;
        src = ++p;
    }
    else
        src += strlen(src);

    return ret;
}



int main(void){

	uint8_t count = 0;
	int flag = 0;
	int comma_count = 0;
  	unsigned long tachometer;
	char count_data[5];
	
	SPI_Init();					// Initialize the SPI port
	
	f_mount(0, &fs);			// Mount the SD card
	disk_initialize(0);			// Initializing the SD card
	
	//_delay_ms(10)
	tcnt1_init();
	tcnt0_init();
	tcnt2_init();
	
	uart_init();
	
	//DDRD |= 0x00;	// for buttons
	//PORTE |= 0b00000010; 
	
	DDRD |= 0b00000000;	// for buttons
	PORTD |= 0b00000001;
	DDRE |= 0b00110000;
	PORTE |= 0b00110010; 
	
	EICRA |= _BV(ISC01); 
	EIMSK |= _BV(INT0);
	
	sei();
	
	
	lcd_init();    //initialize the LCD
	
	sei();
	
	
	// Open a existing csv file or create a new csv file on the SD Card
	f_open(&fil, "/logfile.csv", FA_WRITE | FA_CREATE_ALWAYS);
	
		// Print the header on the first line of the file
	sprintf(header_buffer,"Time,Energy(j),Speed(mph),Latitude,Longitude,Altitude\n");
	f_write(&fil, header_buffer, strlen(header_buffer)/*46*/, &bW);
	
	
	
	/***************UART PORTION************************/
	//uart_puts("AT+CGNSSEQ?");
	uart_puts("AT\r");
	
	_delay_ms(50);
	
	uart_puts("AT+CGNSPWR=1\r");
	
	_delay_ms(50);
	
	uart_puts("AT+CGNSINF\r");
	
	_delay_ms(50);

	
	while(flag != 1)
    {  
    	
	
		if ((PIND & 0x01) == 1) {      //button 1 keeps L8 LED on while pressed
			PORTB &= 0x7F;  
		}
   		
		else 
		{ 
			PORTB |= 0x80;
		}
		
		
		if (chk_buttons(1) == 1){     //button 2 increments lap counter
			count++;
		}
		
		if (chk_buttons(2) == 1) {     //button 3 resets lap counter
			count = 0;
		}
		
		if (chk_buttons(3) == 1) {     //button 4 ends prg
			flag = 1;
			count=0;
		}
		
	
		speed = rand() % 1000;
		r2 = rand() % 10000;	
		char *str1;
		
	
		strtok_single(gps_buff, " ");
  		strcpy(GNSSrunstatus, strtok_single(NULL, ","));// Gets GNSSrunstatus
  		strcpy(Fixstatus, strtok_single(NULL, ",")); // Gets Fix status
  		strcpy(UTCdatetime, strtok_single(NULL, ",")); // Gets UTC date and time
  		strcpy(latitude, strtok_single(NULL, ",")); // Gets latitude
  		strcpy(logitude, strtok_single(NULL, ",")); // Gets longitude
  		strcpy(altitude, strtok_single(NULL, ",")); // Gets MSL altitude
  		strcpy(speedOTG, strtok_single(NULL, ",")); // Gets speed over ground
  		strcpy(course, strtok_single(NULL, ",")); // Gets course over ground
  		strcpy(fixmode, strtok_single(NULL, ",")); // Gets Fix Mode
  		strtok_single(NULL, ",");
  		strcpy(HDOP, strtok_single(NULL, ",")); // Gets HDOP
  		strcpy(PDOP, strtok_single(NULL, ",")); // Gets PDOP
  		strcpy(VDOP, strtok_single(NULL, ",")); // Gets VDOP
  		strtok_single(NULL, ",");
  		strcpy(satellitesinview, strtok_single(NULL, ",")); // Gets GNSS Satellites in View
  		strcpy(GNSSsatellitesused, strtok_single(NULL, ",")); // Gets GNSS Satellites used
  		strcpy(GLONASSsatellitesused, strtok_single(NULL, ",")); // Gets GLONASS Satellites used
  		strtok_single(NULL, ",");
  		strcpy(cn0max, strtok_single(NULL, ",")); // Gets C/N0 max
  		strcpy(HPA, strtok_single(NULL, ",")); // Gets HPA
  		strcpy(VPA, strtok_single(NULL, "\r")); // Gets VPA
		
		int pwr_rand = 744;
		
		int k;
		/*for(k=0; k++; k < strlen(data_buffer)){
			data_buffer[k] = '\0';
		}*/
		//sprintf(data_buffer,"%d:%d:%d,%s,%s,%s,%s,%s\n",mins,secs,msecs,pow_char,speed_char,latitude,logitude,altitude);
		sprintf(data_buffer,"%d:%d:%d,%d,%s,%s,%s,%s\n",mins,secs,msecs,r2,speed_char,latitude,logitude,altitude);
		f_write(&fil, data_buffer, strlen(data_buffer), &bW);		// Writes the dataset in the file
	
		for(k=0; k++; k < strlen(data_buffer)){
			data_buffer[k] = '\0';
		}
	
		
		refresh_lcd(lcd_string_display);  //calling refresh lcd from hd44780
		uint8_t i;

		for(i=0; i<16; i++)     
		{
			lcd_string_display[i] = lcd_str1[i];  //fill line 1 characters
			lcd_string_display[i+16] = lcd_str2[i];   //fill line 2 characters
		}     

		//lap number increments when button pressed
		char buf[3] = "   "; //initialize array with spaces
		itoa(count,buf, 10); //integer to ascii conversion to char array
		
		//uart_puts("AT");
			
		
		/*for(k = 0; k<2; k++){
			gps_buff[k] = uart_getc();
		}*/
		
		
		//char temp_gps[4];
		/******************UART DATA PRINT**************************/
		/*lcd_str2[0] = 'G';
		lcd_str2[1] = 'P';
		lcd_str2[2] = 'S';
		//itoa(gps_buff,temp_gps,10);
		lcd_str2[4] = gps_buff[0];
		lcd_str2[5] = gps_buff[1];
		*/
		
		//itoa(count,count_data, 10);
		
		
		lcd_str2[4] = buf[0]; //get 1st character send to position 4 line 2
		if (count > 9){       //if count more than 1 digit
			lcd_str2[5] = buf[1];  //get 2nd character send to position 5 line 2
		}
		
		else {
			lcd_str2[5] = ' ';     //put space in position 5 line 2
		} 
	}
	
	// Close the file
	f_close(&fil);

	// Unmount the SD card
	f_mount(0,0);

	return 0;
	
}



