#define F_OSC 16000000		           /* czêstotliwoœæ 16 MHz */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>

const volatile unsigned int mInit[] 	= {0x07,0x1A,0xEE}; // g³ówna sekwencja inicjuj¹ca
const volatile unsigned int mInitIgn[]	= {0xED,0x80,0x86}; // id urz¹dzenia wymagane przy zap³onie
const volatile unsigned int mInit2[] 	= {0x00,0x1E,0xEC}; // druga sekwencja inicjuj¹ca


const volatile unsigned int mPlay[] 	= {0x1B,0xE0,0x01,0x08,0x10};  
const volatile unsigned int mInfo[] 	= {0x1E,0xEF};  
const volatile unsigned int mPwrDown[] 	= {0x19,0x22}; 

const volatile unsigned int mNEXT[] 	= {0x1B,0x2D,0x40,0x01};
const volatile unsigned int mPREV[] 	= {0x1B,0x2D,0x00,0x01};
const volatile unsigned int mDINC[] 	= {0x1A,0x50,0x41};
const volatile unsigned int mDDEC[] 	= {0x1A,0x50,0x01};
const volatile unsigned int mRND[] 		= {0x19,0x52};
const volatile unsigned int mFF[] 		= {0x19,0x29};
const volatile unsigned int mFR[] 		= {0x19,0x2F}; 



#define mState_REC 		0
#define mState_SEND		1

#define mWaiting 		0
#define mMelCmdInit 	1
#define mMelCmdPlay 	2
#define mMelCmdInfo 	3

#define mMelCmdPwrD		5
#define mMelCmdNext		6
#define mMelCmdPrev		7
#define mMelCmdDinc		8
#define mMelCmdDdec		9
#define mMelCmdRND		10
#define mMelCmdFF		11
#define mMelCmdFR		12

volatile unsigned int mPlayInfo[] = {0x00,0x02,0x00,0x01,0x80,0x01,0xC7,0x0A,0x02};
volatile unsigned int mPlayInfoLen = 8;

volatile unsigned int mInitDeviceID[] = {0xE8};

volatile unsigned int mCartInfo[] = {0x00,0xFC,0xFF,0x4A,0xFC,0xFF};
volatile unsigned int mCartInfoLen = 5;


volatile unsigned int mClock=7, mDataIn=0, mDataOut=0, mByteC=0, mState=mState_REC, mCmdFound=mWaiting;
volatile unsigned int mDataBuff[7] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


void init_extint0(void) // definiowanie przerwania INT0
{
	GICR |= 1<<INT0; 	// wlaczenie przerwania INT0
	MCUCR |= 1<<ISC01; 	// przerwanie aktywowane na zbocze opadaj¹ce
}


void init_extint1(void)  // definiowanie przerwania INT1
{
	GICR |= 1<<INT1;    // wlaczenie przerwania INT1
	MCUCR |= 1<<ISC11;  // przerwanie aktywowane na zbocze opadaj¹ce
}

void init_timer(void)  // definiowanie licznika Timer1
{
	TIMSK|=(1<<TICIE1)|(1<<TOIE1);  // ustawienie przerwania od przepe³nienia i na zbocze rosn¹ce
 	TCCR1B|=(1<<ICES1); 	    // przerwanie wywo³ywane zboczem rosn¹cym dla CAPT_vect
}

void setoutput_mdata(void)
{
	DDRD |= (1<<PIND2);         // mDATA ustawiona jako wyjœcie
	PORTD |= (1 << PD2);        // mDATA przyjmuje stan logiczny 1
}

void mPopulateBuffer(void)      // bufor ko³owy
{
	mDataBuff[0]=mDataBuff[1];
	mDataBuff[1]=mDataBuff[2];
	mDataBuff[2]=mDataBuff[3];
	mDataBuff[3]=mDataBuff[4];
	mDataBuff[4]=mDataBuff[5];
	mDataBuff[5]=mDataBuff[6];
	mDataBuff[6]=mDataIn;
}

//funkcja przerwania przy przepe³nieniu licznika Timer1
ISR(TIMER1_OVF_vect)
{
	mClock=7; // reset licznika

	TCCR1B&=~(1<<CS11); // zatrzymanie licznika

	if (mState == mState_SEND) 
	{
		mState = mState_REC;
		DDRD &= ~(1<<PIND2); // wejscie
		PORTD |= (1<<PD2); // podciagniecie napiecia
	}

	mPopulateBuffer(); 

	switch (mCmdFound)
	{
		case (mWaiting): 


			// g³ówna sekwencja inicjuj¹ca
			if ((mDataBuff[4]==mInit[0])&&(mDataBuff[5]==mInit[1])&&(mDataBuff[6]==mInit[2]))
				mCmdFound = mMelCmdInit;
			else

			// druga sekwencja inicjuj¹ca
			if ((mDataBuff[4]==mInitIgn[0])&&(mDataBuff[5]==mInitIgn[1])&&(mDataBuff[6]==mInitIgn[2]))
				mCmdFound = mMelCmdInit;
			else


            // CDC
            if ((mDataBuff[2]==(mInitDeviceID[0]+1))&&(mDataBuff[3]==mPlay[0])&&(mDataBuff[4]==mPlay[1])&&(mDataBuff[5]==mPlay[2])&&(mDataBuff[6]==mPlay[3]))
					mCmdFound = mMelCmdPlay;
            else

            // przeslanie Id urz¹dzenia
            if ((mDataBuff[4]==mInitDeviceID[0])&&(mDataBuff[5]==mInfo[0])&&(mDataBuff[6]==mInfo[1]))
					mCmdFound = mMelCmdInfo;
            else

            // wylaczenie lub zatrzymanie
            if ((mDataBuff[4]==mInitDeviceID[0])&&(mDataBuff[5]==mPwrDown[0])&&(mDataBuff[6]==mPwrDown[1]))
					mCmdFound = mMelCmdPwrD;
            else

            /* przyciski */
            if ((mDataBuff[2]==mInitDeviceID[0])&&(mDataBuff[3]==mNEXT[0])&&(mDataBuff[4]==mNEXT[1])&&(mDataBuff[5]==mNEXT[2])&&(mDataBuff[6]==mNEXT[3]))
					mCmdFound = mMelCmdNext;
            else
            if ((mDataBuff[2]==mInitDeviceID[0])&&(mDataBuff[3]==mPREV[0])&&(mDataBuff[4]==mPREV[1])&&(mDataBuff[5]==mPREV[2])&&(mDataBuff[6]==mPREV[3]))
					mCmdFound = mMelCmdPrev;
            else
            if ((mDataBuff[3]==mInitDeviceID[0])&&(mDataBuff[4]==mDINC[0])&&(mDataBuff[5]==mDINC[1])&&(mDataBuff[6]==mDINC[2]))
					mCmdFound = mMelCmdDinc;
            else
            if ((mDataBuff[3]==mInitDeviceID[0])&&(mDataBuff[4]==mDDEC[0])&&(mDataBuff[5]==mDDEC[1])&&(mDataBuff[6]==mDDEC[2]))
					mCmdFound = mMelCmdDdec;
            else
            if ((mDataBuff[4]==mInitDeviceID[0])&&(mDataBuff[5]==mRND[0])&&(mDataBuff[6]==mRND[1]))
					mCmdFound = mMelCmdRND;
            else
            if ((mDataBuff[4]==mInitDeviceID[0])&&(mDataBuff[5]==mFF[0])&&(mDataBuff[6]==mFF[1]))
					mCmdFound = mMelCmdFF;
            else
            if ((mDataBuff[4]==mInitDeviceID[0])&&(mDataBuff[5]==mFR[0])&&(mDataBuff[6]==mFR[1]))
					mCmdFound = mMelCmdFR;
            else

		break;

		default: break;
	} 

#ifndef LOGONLY 
	switch (mCmdFound) 
	{
		case (mWaiting): break; 


		case (mMelCmdInit):
			if (mDataIn == mInitDeviceID[0]) 
			{
					mState = mState_SEND;
					mDataOut = mInitDeviceID[0]; 


						mCmdFound = mWaiting; 


					setoutput_mdata(); 

			}
		 break;

		case (mMelCmdPwrD):
					mState = mState_SEND;
					mDataOut = 0x00; 
					mCmdFound = mWaiting; 
					mPlayInfo[1]=0x02; 
					mPlayInfo[8]=0x02; 
					setoutput_mdata(); 

		break;

		case (mMelCmdNext):
					mCmdFound = mWaiting; 
					mPlayInfo[5]++; 
		 break;
		case (mMelCmdPrev):
					mCmdFound = mWaiting; 
					mPlayInfo[5]--; 
		 break;
		case (mMelCmdDinc):
					mCmdFound = mWaiting; 
					mPlayInfo[3]++;
					mPlayInfo[5]=0x01;
		 break;
		case (mMelCmdDdec):
					mCmdFound = mWaiting; 
					mPlayInfo[3]--;
					mPlayInfo[5]=0x01;
		 break;
 		case (mMelCmdRND):
					mCmdFound = mWaiting; 
		 break;
 		case (mMelCmdFF):
					mCmdFound = mWaiting; 
		 break;
 		case (mMelCmdFR):
					mState = mState_SEND;
					mDataOut = 0x00; 
					mCmdFound = mWaiting; 
					setoutput_mdata(); 

					mPlayInfo[8]=0x08; 
					mPlayInfo[1]=0x08;
		 break;

		case (mMelCmdPlay):
					mState = mState_SEND;
					mDataOut = mPlayInfo[mByteC]; 
					mByteC++;
					if (mByteC > mPlayInfoLen)
					{
						mCmdFound = mWaiting; 
						mByteC=0;
					}
					setoutput_mdata(); 
		 break;

		 case (mMelCmdInfo):
					mState = mState_SEND;
					mDataOut = mCartInfo[mByteC]; 
					mByteC++;
					if (mByteC > mCartInfoLen)
					{
						mCmdFound = mWaiting; 
						mByteC=0;
					}
					setoutput_mdata(); 
		 break;

			default: break;
	}
#endif

}
//funkcja przerwania przy zboczu rosn¹cym licznika Timer1
ISR(TIMER1_CAPT_vect)
{

	TCNT1 = 0xFF9C;     // 50ms // 0xFF38 100mS max // iloœæ impulsów 65535
	TCCR1B|=(1<<CS11);  // preskaler na 8

	// odbieranie danych przy zboczu narastajacym
	if (PIND & (1<<PIND2))  // sprawdzanie stanu logicznego na wejsciu PIND2
		mDataIn |= (1 << mClock); // ustawienie 1
	else
		mDataIn &= ~(1 << mClock); // ustawienie 0

	mClock--;

}
// Zewnetrzne przerwanie 1 ISR  == CLOCK ==
SIGNAL(SIG_INTERRUPT1)
{
	if (mState == mState_SEND)
	{
	  	if (( mDataOut & (1 << mClock) ))  // sprawdzanie stanu logicznego 
			PORTD |= (1 << PD2); // ustawienie 1
		else
			PORTD &= ~(1 << PD2); // ustawienie  0
	} 
}

int main(void)
{
	init_extint0(); // zewnetrzne przerwanie INT0 (ustawienie)
	init_extint1(); // zewnetrzne przerwanie INT1 (ustawienie)
	init_timer();   // inicjowanie licznika


	// ustawienia portów
	DDRD &= ~(1<<PIND2);  // wejœcie (mDATA)
	PORTD |= (1<<PD2);    // podciagniecie oznacza 1 na wejsciu
	DDRD &= ~(1<<PIND3);  // wejœcie (mCLOCK)
	PORTD |= (1<<PD3);    // podciagniecie oznacza 1 na wejsciu
	DDRC &= ~(1<<PINC0);  // wejœcie (mBUSY)
	PORTC |= (1<<PC0);    // podciagniecie oznacza 1 na wejsciu

	// w³¹czenie przerwañ
	sei();

	// inicjacja radia przy uruchomieniu (podanie 0 logicznego na mBUSY przez 1 sekunde)
	DDRC |= (1<<PINC0);   // wyjœcie
	PORTC &= ~(1<<PC0);   // 0 na wyjsciu
	_delay_ms(1000);      // czas trwania ok 1 sekundy
	DDRC &= ~(1<<PINC0);  // ponowne ustawienie mBUSY jako wejœcie
	PORTC |= (1<<PC0);    // 1 na wejœciu

 	while (1)
    {
    }

}


