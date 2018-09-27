#define F_OSC 16000000		           /* czêstotliwoœæ 16 MHz */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>

const volatile unsigned int mInit[] 	= {0x07,0x1A,0xEE}; // g³ówna sekwencja inicjuj¹ca
const volatile unsigned int mInitIgn[]	= {0xED,0x80,0x86}; // id urz¹dzenia wymagane przy zap³onie
const volatile unsigned int mInit2[] 	= {0x00,0x1E,0xEC}; // secondary init request

// these always begin with device ID, ex. 0xE8 (CD-CHGR)
const volatile unsigned int mPlay[] 	= {0x1B,0xE0,0x01,0x08,0x10};  // this is cd/track/play info
const volatile unsigned int mInfo[] 	= {0x1E,0xEF};  // cd-cartrigde information
const volatile unsigned int mPwrDown[] 	= {0x19,0x22}; // powerdown, respond 00 and stop playing!

const volatile unsigned int mNEXT[] 	= {0x1B,0x2D,0x40,0x01};
const volatile unsigned int mPREV[] 	= {0x1B,0x2D,0x00,0x01};
const volatile unsigned int mDINC[] 	= {0x1A,0x50,0x41};
const volatile unsigned int mDDEC[] 	= {0x1A,0x50,0x01};
const volatile unsigned int mRND[] 		= {0x19,0x52};
const volatile unsigned int mFF[] 		= {0x19,0x29};
const volatile unsigned int mFR[] 		= {0x19,0x2F}; // Fast Forward and power up? respond 00 !



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
	mClock=7; // reset bit counter

	TCCR1B&=~(1<<CS11); // stop timer

	if (mState == mState_SEND) // if SEND state relase DATA line
	{
		mState = mState_REC;
		DDRD &= ~(1<<PIND2); // input
		PORTD |= (1<<PD2); // enable pull-up
	}

	mPopulateBuffer(); // populate circular buffer

	switch (mCmdFound)
	{
		case (mWaiting): // Are we waiting for Melbus Command?


			// main Melbus init
			if ((mDataBuff[4]==mInit[0])&&(mDataBuff[5]==mInit[1])&&(mDataBuff[6]==mInit[2]))
				mCmdFound = mMelCmdInit;
			else

			// secondary init
			if ((mDataBuff[4]==mInitIgn[0])&&(mDataBuff[5]==mInitIgn[1])&&(mDataBuff[6]==mInitIgn[2]))
				mCmdFound = mMelCmdInit;
			else


            // CDC
            if ((mDataBuff[2]==(mInitDeviceID[0]+1))&&(mDataBuff[3]==mPlay[0])&&(mDataBuff[4]==mPlay[1])&&(mDataBuff[5]==mPlay[2])&&(mDataBuff[6]==mPlay[3]))
					mCmdFound = mMelCmdPlay;
            else

            // cd-c/md-c cartridge info request
            if ((mDataBuff[4]==mInitDeviceID[0])&&(mDataBuff[5]==mInfo[0])&&(mDataBuff[6]==mInfo[1]))
					mCmdFound = mMelCmdInfo;
            else

            // powerdown or stop playing request!
            if ((mDataBuff[4]==mInitDeviceID[0])&&(mDataBuff[5]==mPwrDown[0])&&(mDataBuff[6]==mPwrDown[1]))
					mCmdFound = mMelCmdPwrD;
            else

            /* buttons */
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
	} // end switch mWaiting

#ifndef LOGONLY // only sniffer, don't send any commands!
	switch (mCmdFound) // process commands
	{
		case (mWaiting): break; // ops, just quit!


		case (mMelCmdInit):
			if (mDataIn == mInitDeviceID[0]) // wait for Melbus device id
			{
					mState = mState_SEND;
					mDataOut = mInitDeviceID[0]; // send device id


						mCmdFound = mWaiting; // return to cmd search mode


					setoutput_mdata(); // PD2 output

			}
		 break;

		case (mMelCmdPwrD):
					mState = mState_SEND;
					mDataOut = 0x00; // respond to powerdown;
					mCmdFound = mWaiting; // return to cmd search mode
					mPlayInfo[1]=0x02; // STOP
					mPlayInfo[8]=0x02; // STOP
					setoutput_mdata(); // PD2 output

		break;

		case (mMelCmdNext):
					mCmdFound = mWaiting; // return to cmd search mode
					mPlayInfo[5]++; // 5
		 break;
		case (mMelCmdPrev):
					mCmdFound = mWaiting; // return to cmd search mode
					mPlayInfo[5]--; // 5
		 break;
		case (mMelCmdDinc):
					mCmdFound = mWaiting; // return to cmd search mode
					mPlayInfo[3]++;
					mPlayInfo[5]=0x01;
		 break;
		case (mMelCmdDdec):
					mCmdFound = mWaiting; // return to cmd search mode
					mPlayInfo[3]--;
					mPlayInfo[5]=0x01;
		 break;
 		case (mMelCmdRND):
					mCmdFound = mWaiting; // return to cmd search mode
		 break;
 		case (mMelCmdFF):
					mCmdFound = mWaiting; // return to cmd search mode
		 break;
 		case (mMelCmdFR):
					mState = mState_SEND;
					mDataOut = 0x00; // respond to powerup / fastforward
					mCmdFound = mWaiting; // return to cmd search mode
					setoutput_mdata(); // PD2 output

					mPlayInfo[8]=0x08; // Enter PLAY MODE!
					mPlayInfo[1]=0x08;
		 break;

		case (mMelCmdPlay):
					mState = mState_SEND;
					mDataOut = mPlayInfo[mByteC]; // send cmd answer
					mByteC++;
					if (mByteC > mPlayInfoLen)
					{
						mCmdFound = mWaiting; // return to cmd search mode
						mByteC=0;
					}
					setoutput_mdata(); // PD2 output
		 break;

		 case (mMelCmdInfo):
					mState = mState_SEND;
					mDataOut = mCartInfo[mByteC]; // send cmd answer
					mByteC++;
					if (mByteC > mCartInfoLen)
					{
						mCmdFound = mWaiting; // return to cmd search mode
						mByteC=0;
					}
					setoutput_mdata(); // PD2 output
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

	// read DATA on rising CLOCK pulse
	if (PIND & (1<<PIND2))  // if Melbus DATA is logical 1
		mDataIn |= (1 << mClock); // set bit to 1
	else
		mDataIn &= ~(1 << mClock); // set bit to 0

	mClock--;

}

// Zewnetrzne przerwanie 0 ISR  == DATA ==
SIGNAL(SIG_INTERRUPT0)
{

}

// Zewnetrzne przerwanie 1 ISR  == CLOCK ==
SIGNAL(SIG_INTERRUPT1)
{
	if (mState == mState_SEND)
	{
	  	if (( mDataOut & (1 << mClock) ))  // if Melbus DATA is logical 1
			PORTD |= (1 << PD2); // set 1
		else
			PORTD &= ~(1 << PD2); // set  0
	} // end mState_SEND
}

int main(void)
{
	init_extint0(); // zewnetrzne przerwanie INT0 (ustawienie)
	init_extint1(); // zewnetrzne przerwanie INT1 (ustawienie)
#ifdef DEBUG
	init_rs232();   // init UART
#endif
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


