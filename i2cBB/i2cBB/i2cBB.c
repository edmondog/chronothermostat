#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>

#define F_CPU 8000000
#define BAUD 19200
#include <util/setbaud.h>
#define sbi(P, B) (P |= _BV(B))
#define cbi(s,b) (s &= ~(_BV(b)))

#define RS485DIR_DDR DDRD
#define RS485DIR_PORT PORTD
#define RS485DIR_BIT PD2

#define OUT_0 PB7
#define OUT_0_PORT PORTB
#define OUT_0_DDR DDRB
#define LED_1 PB6
#define LED_1_PORT PORTB
#define LED_1_DDR DDRB
#define ENABLE_0 PD7
#define ENABLE_0_PORT PORTD
#define ENABLE_0_DDR DDRD
#define ENABLE_1 PB0
#define ENABLE_1_PORT PORTB
#define ENABLE_1_DDR DDRB
#define ENABLE_2 PB1
#define ENABLE_2_PORT PORTB
#define ENABLE_2_DDR DDRB
#define ENABLE_3 PB2
#define ENABLE_3_PORT PORTB
#define ENABLE_3_DDR DDRB
#define FASE_1 PD6
#define FASE_1_PORT PORTD
#define FASE_1_PIN PIND
#define FASE_1_DDR DDRD
#define FASE_2 PD5
#define FASE_2_PORT PORTD
#define FASE_2_PIN PIND
#define FASE_2_DDR DDRD
#define PULS PB3
#define PULS_PORT PORTB
#define PULS_PIN PINB
#define PULS_DDR DDRB
#define SEG_A PC5
#define SEG_A_PORT PORTC
#define SEG_A_DDR DDRC
#define SEG_B PC4
#define SEG_B_PORT PORTC
#define SEG_B_DDR DDRC
#define SEG_C PC2
#define SEG_C_PORT PORTC
#define SEG_C_DDR DDRC
#define SEG_D PB4
#define SEG_D_PORT PORTB
#define SEG_D_DDR DDRB
#define SEG_E PB5
#define SEG_E_PORT PORTB
#define SEG_E_DDR DDRB
#define SEG_F PC3
#define SEG_F_PORT PORTC
#define SEG_F_DDR DDRC
#define SEG_G PC1
#define SEG_G_PORT PORTC
#define SEG_G_DDR DDRC
#define SEG_P PC0
#define SEG_P_PORT PORTC
#define SEG_P_DDR DDRC
unsigned char caratteri[20]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF};
volatile int pos=0;
volatile unsigned char pd;
unsigned char v0=0;
unsigned char v1=0;
unsigned char v2=0;
unsigned char v3=0;
unsigned char w0=0;
unsigned char w1=0;
unsigned char w2=0;
unsigned char w3=0;
unsigned char s0=0;
unsigned char s1=0;
unsigned char s2=0;
unsigned char s3=0;
unsigned long v_acc;
unsigned long w_acc;
int v,w,termo;
int setp=0;
unsigned char modo=0;
int pos0=0;
volatile unsigned int t0;
volatile unsigned char ttd=0;
volatile unsigned char d0=0;
volatile unsigned char d1=0;
volatile unsigned char d2=0;
volatile unsigned char d3=0;
unsigned char button_up=0;
unsigned char button_dn=0;
unsigned char button_old=0;
void cifra(unsigned char n, unsigned char c);

#define T_OUT 40000

// Port for the I2C
#define I2C_DDR DDRD
#define I2C_PIN PIND
#define I2C_PORT PORTD
#define DEV0 10010000
#define DEV1 10010010
#define I2C_R 1
#define I2C_W 0

// Pins to be used in the bit banging
#define I2C_CLK 4
#define I2C_DAT 3

#define I2C_DATA_HI()\
I2C_PORT |= (1 << I2C_DAT);\
I2C_DDR &= ~ (1 << I2C_DAT);
#define I2C_DATA_LO()\
I2C_PORT &= ~ (1 << I2C_DAT);\
I2C_DDR |= (1 << I2C_DAT);

#define I2C_CLOCK_HI()\
I2C_PORT |= (1 << I2C_CLK);\
I2C_DDR &= ~ (1 << I2C_CLK);
#define I2C_CLOCK_LO()\
I2C_PORT &= ~ (1 << I2C_CLK);\
I2C_DDR |= (1 << I2C_CLK);

void I2C_Init();

volatile unsigned char res=0,n=0,device=0;
volatile int t=0,tt=0,tv0=0,tv1=0;
volatile unsigned char i2c_step=0, clock_req=0;
volatile unsigned char tv1_valid=0, tv0_valid=0;
volatile unsigned char chrno=0,chrr,chrv1[16],chrv2[16],received;
unsigned char txbuf[32];
unsigned char p0=0;
unsigned char ptx=0;
unsigned char me;
unsigned char req=0;

ISR(USART_RX_vect)
{
	chrr=UDR0;
	if(chrr)
	{
		chrv1[chrno++]=chrr;
		chrno&=0x0f;
		if(chrr==13)
		{
			strncpy(chrv2,chrv1,chrno);
			chrno=0;
			received=1;
		}
	}
}

ISR(PCINT2_vect)
{
	if(bit_is_set(FASE_1_PIN,FASE_1))
	{
		if(bit_is_set(FASE_2_PIN,FASE_2) && bit_is_clear(pd,FASE_2)) // difu
				pos++;
		if(bit_is_set(pd,FASE_2) && bit_is_clear(FASE_2_PIN,FASE_2)) // difd
				pos--;
		if(pos>9999)
			pos=0;
		if(pos<0)
			pos=9999;
	}	
	pd=PIND;
}


ISR(TIMER0_COMPA_vect)
{
	switch(clock_req)
	{
	case 0:
		switch(i2c_step)
		{
		case 0:	// start
			I2C_DATA_LO();
			break;
		case 1:
			I2C_CLOCK_LO();
			break;
			
		case 2:	// DEV0 10010010
		case 5:
			I2C_DATA_HI();
			clock_req=1;
			break;
		case 8:
			if(device==1)
			{
				I2C_DATA_HI();
			}
			else
			{
				I2C_DATA_LO();
			}
			clock_req=1;
			break;
		case 3:
		case 4:
		case 6:
		case 7:
		case 9: // W negato
			I2C_DATA_LO();
			clock_req=1;
			break;

		case 10: // read ack
			I2C_DATA_HI();
			I2C_CLOCK_HI();
			break;
		case 11:
			//			c = I2C_PIN ;
			//			d = (1<< I2C_DAT);
			//			e=c&d;
			I2C_DATA_LO();
			I2C_CLOCK_LO();
			break;

		case 12: // c=0;
		case 13:
		case 14:
		case 15:
		case 16:
		case 17:
		case 18:
		case 19:
			I2C_DATA_LO();
			clock_req=1;
			break;

		case 20: // read ack
			I2C_DATA_HI();
			I2C_CLOCK_HI();
			break;
		case 21:
			//			c = (I2C_PIN & (1<< I2C_DAT));
			I2C_DATA_LO();
			I2C_CLOCK_LO();
			break;
			
		case 22: // STOP
			I2C_DATA_LO();
			break;
		case 23:
			I2C_CLOCK_HI();
			break;
		case 24:
			I2C_DATA_HI();
			break;
			
		case 25: // START
			I2C_DATA_LO();
			break;

		case 26:
			I2C_CLOCK_LO();
			break;

		case 27: // DEV0 10010011
		case 30:
		case 34: // Read
			I2C_DATA_HI();
			clock_req=1;
			break;
		case 33:
			if(device==1)
			{
				I2C_DATA_HI();
			}
			else
			{
				I2C_DATA_LO();
			}
			clock_req=1;
			break;
		case 28:
		case 29:
		case 31:
		case 32:
			I2C_DATA_LO();
			clock_req=1;
			break;
			
		case 35: // read ack
			I2C_DATA_HI();
			I2C_CLOCK_HI();
			break;
		case 36:
			//			c = (I2C_PIN & (1<< I2C_DAT));
			res=0;
			I2C_DATA_LO();
			I2C_CLOCK_LO();
			break;

		case 37:
		case 38:
		case 39:
		case 40:
		case 41:
		case 42:
		case 43:
		case 44:
			clock_req=10;
			break;
		case 45:
			I2C_DATA_LO();
			clock_req=1;
			break;
		case 46:
			t=res;
			t <<= 8;
			break;
			
		case 47: // 2nd byte temp
			res=0;
			I2C_DATA_HI();
			break;

		case 48:
		case 49:
		case 50:
		case 51:
		case 52:
		case 53:
		case 54:
		case 55:
			clock_req=10;
			break;

		case 56:
			I2C_DATA_LO();
			clock_req=1;
			break;
		case 57:
			t+=res;
			t >>= 4;
			tt+=t;
			n++;
			I2C_CLOCK_HI();
			break;
		case 58:
			I2C_DATA_HI();
			break;
		case 59:
			if(n==10)
			{
				if(device==1)
				{
					tv1=(tt>>4);
					tv1_valid=1;
					device=0;
				}
				else
				{
					tv0=(tt>>4);
					tv0_valid=1;
					device=1;
				}
				tt=0;
				n=0;
			}
			break;
		case 60:
			#define ENDSEQ 60
			break;
			
		}
		break;
		
	case 1:
		I2C_CLOCK_HI();
		i2c_step--;
		clock_req=2;
		break;
	case 2:
		I2C_CLOCK_LO();
		i2c_step--;
		clock_req=0;
		break;
	case 10:
		res<<=1;
		I2C_DATA_HI();
		I2C_CLOCK_HI();
		i2c_step--;
		clock_req=11;
		break;
	case 11:
		if((I2C_PIN & (1<< I2C_DAT)))
			res|=1;
		I2C_CLOCK_LO();
		i2c_step--;
		clock_req=0;
		break;
	}
	if(i2c_step!=ENDSEQ)
		i2c_step++;
	else
		i2c_step=0;

	t0++;
	ttd++;
	switch(ttd)
	{
		case 1:
			cifra(caratteri[d0],0);
			break;
		case 2:
			cifra(caratteri[d1],1);
			break;
		case 3:
			cifra(caratteri[d2],2);
			break;
		case 4:
			cifra(caratteri[d3],3);
			ttd=0;
			break;
	}

}

void cifra(unsigned char n, unsigned char c)
{
	sbi(ENABLE_0_PORT,ENABLE_0);
	sbi(ENABLE_1_PORT,ENABLE_1);
	sbi(ENABLE_2_PORT,ENABLE_2);
	sbi(ENABLE_3_PORT,ENABLE_3);
	
	if(n&1)
		sbi(SEG_A_PORT,SEG_A);
	else
		cbi(SEG_A_PORT,SEG_A);
	
	if(n&2)
		sbi(SEG_B_PORT,SEG_B);
	else
		cbi(SEG_B_PORT,SEG_B);
	
	if(n&4)
		sbi(SEG_C_PORT,SEG_C);
	else
		cbi(SEG_C_PORT,SEG_C);
	
	if(n&8)
		sbi(SEG_D_PORT,SEG_D);
	else
		cbi(SEG_D_PORT,SEG_D);
	
	if(n&16)
		sbi(SEG_E_PORT,SEG_E);
	else
		cbi(SEG_E_PORT,SEG_E);
	
	if(n&32)
		sbi(SEG_F_PORT,SEG_F);
	else
		cbi(SEG_F_PORT,SEG_F);
	
	if(n&64)
		sbi(SEG_G_PORT,SEG_G);
	else
		cbi(SEG_G_PORT,SEG_G);
	
	if(n&128)
		sbi(SEG_P_PORT,SEG_P);
	else
		cbi(SEG_P_PORT,SEG_P);

	switch(c)
	{
	case 0:
		cbi(ENABLE_0_PORT,ENABLE_0);
		break;
	case 1:
		cbi(ENABLE_1_PORT,ENABLE_1);
		break;
	case 2:
		cbi(ENABLE_2_PORT,ENABLE_2);
		break;
	case 3:
		cbi(ENABLE_3_PORT,ENABLE_3);
		break;
	}
}


void display()
{
	switch(modo)
	{
	case 0:
		if(bit_is_clear(PULS_PIN,PULS))
		{
			s0=setp%10;
			s1=(setp/10)%10;
			s2=(setp/100)%10;
			s3=(setp/1000)%10;
			d0=s0;
			d1=s1+10;
			d2=s2;
			d3=s3;
		}
		else
		{
			d0=v0;
			d1=v1+10;
			d2=v2;
			d3=v3;
		}
		if(pos!=pos0)
		{
			pos=0;
			t0=0;
			modo=1;
		}
		break;
		
	case 1:
		d0=pos%10;
		d1=(pos/10)%10;
		d2=(pos/100)%10;
		d3=(pos/1000)%10;
		if(button_dn)
		{
			if(pos==33) // modify setpoint
			{
				modo=2;
				pos=setp;
				t0=0;
			}
			else
			if(pos==44) // modify me
			{
				modo=4;
				pos=me;
				t0=0;
			}
			else
			if(pos==22) // show termo
			{
				modo=6;
				pos=me;
				t0=0;
			}
			else
			{
				modo=0;
			}
			pos0=pos;
		}
		if(t0>T_OUT)
		{
			modo=0;
			pos0=pos;
		}
		if(pos!=pos0)
		{
			t0=0;
			pos0=pos;
		}
		break;
	
	case 2: // aspetta il rilascio del pulsante
		if(button_up)
		{
			modo++;
		}
		break;

	case 3:
		d0=pos%10;
		d1=((pos/10)%10)+10;
		d2=(pos/100)%10;
		d3=(pos/1000)%10;
		if(button_dn)
		{
			modo=0;
			setp=pos;
			eeprom_update_word (0,setp);
		}
		if(t0>T_OUT)
		{
			modo=0;
			pos0=pos;
		}
		if(pos!=pos0)
		{
			t0=0;
			pos0=pos;
		}
		break;

	case 4: // aspetta il rilascio del pulsante
		if(button_up)
		{
			modo++;
		}
		break;

	case 5:
		if(pos<0)
			pos=0;
		if(pos>255)
			pos=0;
		d0=pos%10;
		d1=((pos/10)%10)+10;
		d2=(pos/100)%10;
		d3=(pos/1000)%10;
		if(button_dn)
		{
			modo=0;
			me=pos;
			eeprom_update_word(2,pos);
		}
		if(t0>T_OUT)
		{
			modo=0;
			pos0=pos;
		}
		if(pos!=pos0)
		{
			t0=0;
			pos0=pos;
		}
		break;
	case 6: // aspetta il rilascio del pulsante
		if(button_up)
		{
			modo++;
		}
		break;

	case 7:
		d0=termo%10;
		d1=((termo/10)%10)+10;
		d2=(termo/100)%10;
		d3=(termo/1000)%10;
		if(button_dn || t0>T_OUT)
		{
			modo=0;
		}
		break;

	}
}


void Add_tx(unsigned char car)
{
	if(((p0+1)&31)!=ptx)
	{
		txbuf[p0++]=car;
		p0&=31;
	}
}

void Add_uint_tx(unsigned int i)
{
	if(i==0)
		Add_tx('0');
	else
	{
		if(i<10)
			Add_tx('0'+i);
		else
		{
			if(i<100)
			{
				Add_tx('0'+i%10);
				Add_tx('0'+i/10);
			}
			else
			{
				if(i<1000)
				{
					Add_tx('0'+i%10);
					Add_tx('0'+(i/10)%10);
					Add_tx('0'+(i/100));
				}
				else
				{
					if(i<10000)
					{
						Add_tx('0'+i%10);
						Add_tx('0'+(i/10)%10);
						Add_tx('0'+(i/100)%10);
						Add_tx('0'+(i/1000));
					}
					else
						Add_tx('E');
				}
			}
		}
	}
}
void Spool_tx(void)
{
	// 485
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	#if USE_2X
	UCSR0A |= (1 << U2X0);
	#else
	UCSR0A &= ~(1 << U2X0);
	#endif
	/* Set frame format: 8data */
	UCSR0C = (3<<UCSZ00);
	sbi(UCSR0B,TXEN0);
	sbi(UCSR0B,RXEN0);
	sbi(UCSR0B,RXCIE0);

	if(bit_is_set(UCSR0A,UDRE0))
	{
		if(ptx!=p0)
		{
			sbi(RS485DIR_PORT,RS485DIR_BIT);
			UDR0=txbuf[ptx++];
			ptx&=31;
		}
		else
		{
			if(bit_is_set(UCSR0A,TXC0))
			{
				cbi(RS485DIR_PORT,RS485DIR_BIT);
				sbi(UCSR0A,TXC0);
			}
		}
		if(received)
		{
			if(chrv2[0]=='@' && chrv2[1]=='z' && chrv2[2]!=13 && chrv2[3]==13)
			{
				// scrive l'id della scheda (solo una scheda deve essere collegata)
				eeprom_update_byte(2,chrv2[2]);
				me=chrv2[2];
				Add_tx(10);
				Add_tx(13);
				Add_tx('M');
				Add_tx('e');
				Add_tx('=');
				Add_tx(me);
				Add_tx(10);
				Add_tx(13);
			}
			if(chrv2[0]=='@' && chrv2[1]==me && (chrv2[2]=='R' || chrv2[2]=='r') && chrv2[3]=='0')
			{
				// temperatura
				Add_tx(req+'0');
				Add_tx(v3+'0');
				Add_tx(v2+'0');
				Add_tx(v1+'0');
				Add_tx(v0+'0');
				Add_tx(s3+'0');
				Add_tx(s2+'0');
				Add_tx(s1+'0');
				Add_tx(s0+'0');
				Add_tx(13);
			}
			if(chrv2[0]=='@' && chrv2[1]==me && (chrv2[2]=='R' || chrv2[2]=='r') && chrv2[3]=='1')
			{
				// setp
				Add_tx(s3+'0');
				Add_tx(s2+'0');
				Add_tx(s1+'0');
				Add_tx(s0+'0');
				Add_tx(13);
			}
			if(chrv2[0]=='@' && chrv2[1]==me && (chrv2[2]=='W' || chrv2[2]=='w') && chrv2[3]=='1' && chrv2[4]!=13 && chrv2[5]!=13 && chrv2[6]!=13 && chrv2[7]!=13 && chrv2[8]==13)
			{
				//
				s3=chrv2[4]-'0';
				s2=chrv2[5]-'0';
				s1=chrv2[6]-'0';
				s0=chrv2[7]-'0';
				setp=((chrv2[4]-'0')*1000)+((chrv2[5]-'0')*100)+((chrv2[6]-'0')*10)+(chrv2[7]-'0');
				eeprom_update_word (0,setp);
				Add_tx('O');
				Add_tx('K');
				Add_tx(13);
			}
			received=0;
		}
	}
}
void Flush(void)
{
	p0=0;
}
#include <avr/io.h>

int main(void)
{
	int val0,val1;
	// I/O
	cbi(LED_1_PORT,LED_1);
	sbi(LED_1_DDR, LED_1);
	// IO SETUP
	sbi(OUT_0_DDR,OUT_0);
	sbi(SEG_A_DDR,SEG_A);
	sbi(SEG_B_DDR,SEG_B);
	sbi(SEG_C_DDR,SEG_C);
	sbi(SEG_D_DDR,SEG_D);
	sbi(SEG_E_DDR,SEG_E);
	sbi(SEG_F_DDR,SEG_F);
	sbi(SEG_G_DDR,SEG_G);
	sbi(SEG_P_DDR,SEG_P);
	sbi(ENABLE_0_DDR,ENABLE_0);
	sbi(ENABLE_1_DDR,ENABLE_1);
	sbi(ENABLE_2_DDR,ENABLE_2);
	sbi(ENABLE_3_DDR,ENABLE_3);
	
	
	cbi(FASE_1_DDR,FASE_1);
	cbi(FASE_2_DDR,FASE_2);
	sbi(FASE_1_PORT,FASE_1);
	sbi(FASE_2_PORT,FASE_2);
	cbi(PULS_DDR,PULS);
	sbi(PULS_PORT,PULS);

	// IO INTERRUPT
	sbi(PCICR,PCIE2);
	sbi(PCMSK2,PCINT21);
//	sbi(PCMSK2,PCINT22);
	
	I2C_Init();
	// TIMER0
	sbi(TCCR0A,WGM01);
	cbi(TCCR0B,CS00);
	sbi(TCCR0B,CS01);
	cbi(TCCR0B,CS02);
	OCR0A=0xF0;
	sbi(TIMSK0,OCIE0A);

// 485
	cbi(RS485DIR_PORT,RS485DIR_BIT);
	sbi(RS485DIR_DDR, RS485DIR_BIT);
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	#if USE_2X
		UCSR0A |= (1 << U2X0);
	#else
		UCSR0A &= ~(1 << U2X0);
	#endif
	/* Set frame format: 8data */
	UCSR0C = (3<<UCSZ00);
	sbi(UCSR0B,TXEN0);
	sbi(UCSR0B,RXEN0);
	sbi(UCSR0B,RXCIE0);

	me=eeprom_read_word (2);

	setp=eeprom_read_word (0);
	if(setp>999 || setp<0)
		setp=130;

	s0=setp%10;
	s1=(setp/10)%10;
	s2=(setp/100)%10;
	s3=(setp/1000)%10;

	sei();

	val0=0;
	val1=0;
	
    while(1)
    {

		Spool_tx();
		if(tv0_valid)
		{
			val0=tv0;
			tv0_valid=0;
		}
		
		if(tv1_valid)
		{
			val1=tv1;
			tv1_valid=0;
			termo=val1;		
		}

		v0=val0%10;
		v1=(val0/10)%10;
		v2=(val0/100)%10;
		v3=(val0/1000)%10;

		if((val0+3)<setp)
		{
			sbi(LED_1_PORT,LED_1);
			req=1;
		}			
		if(val1>300 && (val0+3)<setp)
		{
			sbi(OUT_0_PORT,OUT_0);
		}
		if((val0)>setp)
		{
			cbi(LED_1_PORT,LED_1);
			req=0;
		}
		if(val1<290 || (val0)>setp)
		{
			cbi(OUT_0_PORT,OUT_0);
		}

		if(bit_is_clear(PULS_PIN,PULS)&& button_old==0)
			button_dn=1; // difu
		else
			button_dn=0;
		if(bit_is_set(PULS_PIN,PULS)&& button_old==1)
			button_up=1; // difd
		else
			button_up=0;

		if(bit_is_clear(PULS_PIN,PULS))
			button_old=1;
		else
			button_old=0;

		display();
		
	}
}





// Inits bitbanging port, must be called before using the functions below
//
void I2C_Init()
{
	I2C_PORT &= ~ ((1 << I2C_DAT) | (1 << I2C_CLK));

	I2C_CLOCK_HI();
	I2C_DATA_HI();

}

