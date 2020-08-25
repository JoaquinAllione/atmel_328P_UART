/*
 * UART_ADC_POT.c
 *
 * Created: 01/08/2020 8:23:50
 * Author : Joaquin
 */ 

#define F_CPU 16000000
#define BAUD 9600
#include <avr/io.h>
#include <stdio.h>
#include "string.h"
#include "stdlib.h"
#include <avr/interrupt.h>
#include <stdint-gcc.h>

//=================RINGBUFFER================//

typedef struct
{
	int32_t indexRead;
	int32_t indexWrite;
	int32_t count;
	int32_t size;
	int8_t *pbuf;
}ringBufferData_struct;

void *ringBuffer_init(int32_t size){
	ringBufferData_struct *rb;
	rb = malloc(sizeof(ringBufferData_struct));
	rb->pbuf = malloc(size);
	rb->indexRead = 0;
	rb->indexWrite = 0;
	rb->count = 0;
	rb->size = size;

	return rb;
}

int8_t ringBuffer_putData(void *pRb, uint8_t data){
	ringBufferData_struct *rb = pRb;
	int8_t ret = 1;

	rb->pbuf[rb->indexWrite] = data;
	rb->indexWrite++;
	if(rb->indexWrite >= rb->size){
		rb->indexWrite = 0;
	}
	if(rb->count < rb->size){
		rb->count++;
		}else{
		rb->indexRead++;
		if(rb->indexRead >= rb->size){
			rb->indexRead = 0;
		}
		ret = 0;
	}
	return ret;
}

int8_t ringBuffer_getData(void *pRb, uint8_t *data){
	ringBufferData_struct *rb = pRb;
	int8_t ret = 1;

	if(rb->count){
		*data = rb->pbuf[rb->indexRead];
		rb->indexRead++;
		if(rb->indexRead >= rb->size){
			rb->indexRead = 0;
		}
		rb->count--;
		}else{
		ret = 0;
	}
	return ret;
}

int8_t ringBuffer_isFull(void *pRb){
	ringBufferData_struct *rb = pRb;

	return rb->count == rb->size;
}

int8_t ringBuffer_isEmply(void *pRb){
	ringBufferData_struct *rb = pRb;

	return rb->count == 0;
}

//===========================================//

//============UART - RINGBUFFER==============//

static void* pRingBufferRx;
static void* pRingBufferTx;

void UART_init(void){

	cli(); //desactiva todas las interrupciones

	UCSR0C &= ~(1<<UMSEL01); //configura la USART como UART
	UCSR0C &= ~(1<<UMSEL00);

	UCSR0C &= ~(1<<UPM01); //sin paridad
	UCSR0C &= ~(1<<UPM00);

	UCSR0C &= ~(1<<USBS0); //un bit de stop

	UCSR0C |= (1<<UCSZ00)|(1<<UCSZ01); //8 bits de datos
	UCSR0B &= ~(1<<UCSZ02);

	UCSR0A |= (1<<U2X0);
	UBRR0 = ((F_CPU/8)/BAUD)-1; //configura baudrate

	UCSR0B |= (1<<TXEN0); //Tx y Rx habilitados
	UCSR0B |= (1<<RXEN0);

	UCSR0B |= (1<<RXCIE0); //habilita las interrupciones por recepcion de datos
	
	sei(); //habilita las interrupciones glovales
	
}

void UART_ringBuffer_init(void){
	
	pRingBufferRx = ringBuffer_init(16);
	pRingBufferTx = ringBuffer_init(16);
	
	UART_init();

}

int32_t UART_ringBuffer_recDatos(uint8_t *pBuf, int32_t size){
	int32_t ret = 0;

	cli();
	while (!(ringBuffer_isEmply(pRingBufferRx))&&(ret < size))
	{
		ringBuffer_getData(pRingBufferRx, &pBuf[ret]);
		ret = 1;
	}
	sei();
	return ret;
}

//===========================================//

//===============WRITE UART==================//
void UART_write(unsigned char caracter){
	while(!(UCSR0A&(1<<5)));    // mientras el registro UDR0 esté lleno espera
	UDR0 = caracter;            //cuando el el registro UDR0 está vacio se envia el caracter
}

void UART_write_txt(char* cadena){			//cadena de caracteres de tipo char
	cli();
	while(*cadena !=0x00){				//mientras el último valor de la cadena sea diferente a el caracter nulo
		UART_write(*cadena);			//transmite los caracteres de cadena
		cadena++;						//incrementa la ubicación de los caracteres en cadena
		//para enviar el siguiente caracter de cadena
	}
	sei();
}
//===========================================//

#define LENGTH_BUFFER_INSTRUC 10
#define INICIO_DE_TRAMA 58
#define FINAL_DE_TRAMA (0x0A)

uint8_t buffer[3];
uint8_t buffer_instruccion[LENGTH_BUFFER_INSTRUC];
int8_t dato_recibido = 0;
int8_t index_buffer = 0;
int8_t trama_completa = 0;

void MefLeerTrama(void);
void clear_buffer_instruc(uint8_t *pBuf);

int16_t adc_value = 0;

#define CONV_init() ADCSRA |= (1<<ADSC) //inicia convercion

ISR(ADC_vect){
	adc_value = ADC;
	uint8_t buffer[4];
	itoa(adc_value, buffer,10);
	UART_write_txt(":P01:");
	UART_write_txt(buffer);
	UART_write(FINAL_DE_TRAMA);
}

void ADC_init(void){

	cli();

	ADMUX &= ~(1<<ADLAR); //ajusto a derecha
	
	ADMUX &= ~(1<<REFS1);
	ADMUX |= (1<<REFS0); //referencia externa

	ADMUX &= ~(1<<MUX3); //ADC0
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX0);

	ADCSRA |= (1<<ADEN);//ADC activado

	ADCSRA |= (1<<ADPS0); //divido por 128
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS2);

	ADCSRA |= (1<<ADIE);//interrupciones habilitadas

	sei();
}

int main(void)
{

	UART_ringBuffer_init();
	ADC_init();

	clear_buffer_instruc(buffer_instruccion);
	/* Replace with your application code */
	while (1)
	{
		dato_recibido = UART_ringBuffer_recDatos(buffer, sizeof(buffer));
		if((dato_recibido == 1)||(trama_completa == 1)){
			MefLeerTrama();
		}
	}
}

typedef enum{
	EST_MEF_LEER_TRAMA_INICIO_TRAMA = 0,
	EST_MEF_LEER_TRAMA_1ER_CARACTER,
	EST_MEF_LEER_TRAMA_2DO_CARACTER,
	EST_MEF_LEER_TRAMA_3ER_CARACTER,
	EST_MEF_LEER_TRAMA_FIN_TRAMA,
	EST_MEF_LEER_TRAMA_ANALIZANDO,
}estMefLeerTrama_enum;

void clear_buffer_instruc(uint8_t *pBuf){
	int8_t i;
	for(i=0; i<LENGTH_BUFFER_INSTRUC; i++){
		pBuf[i] = '\0';
	}
}

void MefLeerTrama(void){

	static estMefLeerTrama_enum estMefLeerTrama = EST_MEF_LEER_TRAMA_INICIO_TRAMA;
	
	switch(estMefLeerTrama){

		case EST_MEF_LEER_TRAMA_INICIO_TRAMA:

		if(buffer[0]==INICIO_DE_TRAMA){
			index_buffer = 0;
			buffer_instruccion[index_buffer] = buffer[0];
			index_buffer++;
			estMefLeerTrama = EST_MEF_LEER_TRAMA_1ER_CARACTER;
			}else{
			clear_buffer_instruc(buffer_instruccion);
			estMefLeerTrama = EST_MEF_LEER_TRAMA_INICIO_TRAMA;
		}
		break;

		case EST_MEF_LEER_TRAMA_1ER_CARACTER:

		if(buffer[0]=='P'){
			buffer_instruccion[index_buffer] = buffer[0];
			index_buffer++;
			estMefLeerTrama = EST_MEF_LEER_TRAMA_2DO_CARACTER;
			}else{
			clear_buffer_instruc(buffer_instruccion);
			estMefLeerTrama = EST_MEF_LEER_TRAMA_INICIO_TRAMA;
		}
		break;

		case EST_MEF_LEER_TRAMA_2DO_CARACTER:

		if(buffer[0]=='0'){
			buffer_instruccion[index_buffer] = buffer[0];
			index_buffer++;
			estMefLeerTrama = EST_MEF_LEER_TRAMA_3ER_CARACTER;
			}else{
			clear_buffer_instruc(buffer_instruccion);
			estMefLeerTrama = EST_MEF_LEER_TRAMA_INICIO_TRAMA;
		}
		break;

		case EST_MEF_LEER_TRAMA_3ER_CARACTER:

		if(buffer[0]=='1'){
			buffer_instruccion[index_buffer] = buffer[0];
			index_buffer++;
			estMefLeerTrama = EST_MEF_LEER_TRAMA_FIN_TRAMA;
			}else{
			clear_buffer_instruc(buffer_instruccion);
			estMefLeerTrama = EST_MEF_LEER_TRAMA_INICIO_TRAMA;
		}
		break;

		case EST_MEF_LEER_TRAMA_FIN_TRAMA:

		if(buffer[0]==FINAL_DE_TRAMA){
			buffer_instruccion[index_buffer] = buffer[0];
			estMefLeerTrama = EST_MEF_LEER_TRAMA_ANALIZANDO;
			trama_completa = 1;
			}else{
			clear_buffer_instruc(buffer_instruccion);
			estMefLeerTrama = EST_MEF_LEER_TRAMA_INICIO_TRAMA;
		}
		break;

		case EST_MEF_LEER_TRAMA_ANALIZANDO:

		trama_completa = 0;
		
		if (strcmp(buffer_instruccion, ":P01\n")==0)
		{
			CONV_init();
			clear_buffer_instruc(buffer_instruccion);
			estMefLeerTrama = EST_MEF_LEER_TRAMA_INICIO_TRAMA;
			break;
		}
		clear_buffer_instruc(buffer_instruccion);
		estMefLeerTrama = EST_MEF_LEER_TRAMA_INICIO_TRAMA;
		break;

		default:

		clear_buffer_instruc(buffer_instruccion);
		estMefLeerTrama = EST_MEF_LEER_TRAMA_INICIO_TRAMA;
		break;

	}
}

ISR(USART_RX_vect){
	uint8_t dato;
	dato = 	UDR0;
	ringBuffer_putData(pRingBufferRx, dato);
}