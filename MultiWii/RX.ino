//for RC
volatile uint8_t  RcFrameData = 0;
//


/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

#if defined(SPEKTRUM)
#include <wiring.c>  //Auto-included by the Arduino core... but we need it sooner.
#endif

//RAW RC values will be store here
#if defined(SBUS)
volatile uint16_t rcValue[RC_CHANS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // interval [1000;2000]
#elif defined(SPEKTRUM)
volatile uint16_t rcValue[RC_CHANS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1501, 1502}; // interval [1000;2000]
#elif defined(SERIAL_SUM_PPM)
volatile uint16_t rcValue[RC_CHANS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // interval [1000;2000]
#else
volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#endif

#if defined(SERIAL_SUM_PPM) //Channel order for PPM SUM RX Configs
static uint8_t rcChannel[RC_CHANS] = {SERIAL_SUM_PPM};
#elif defined(SBUS) //Channel order for SBUS RX Configs
// for 16 + 2 Channels SBUS. The 10 extra channels 8->17 are not used by MultiWii, but it should be easy to integrate them.
static uint8_t rcChannel[RC_CHANS] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11,12,13,14,15,16,17};
static uint16_t sbusIndex=0;
#elif defined(SPEKTRUM)
static uint8_t rcChannel[RC_CHANS] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11,12,13};
#endif

#define FAILSAFE_DETECT_TRESHOLD  990

/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver()
{
	int chan;
	/******************    Configure each rc pin for PCINT    ***************************/
	// Init PPM SUM RX
	#if defined(SERIAL_SUM_PPM)
	pinMode(13,INPUT);
	PPM_PIN_INTERRUPT;
	#endif
	// Init Sektrum Satellite RX
	#if defined (SPEKTRUM)
	SerialOpen(SPEK_SERIAL_PORT,115200);
	#endif
	// Init SBUS RX
	#if defined(SBUS)
	SerialOpen(SBUS_SERIAL_PORT,100000);
	#endif
	#if !defined(RCSERIAL)
	for (chan = 0; chan < RC_CHANS; chan++) rcData[chan] = rcValue[rcChannel[chan]];
	#endif
}

/**************************************************************************************/
/***************                PPM SUM RX Pin reading             ********************/
/**************************************************************************************/
// 32u4  ICP1 = PD4/ADC8 = pin 4     ICP3 = PC7/OC4A = pin 13
// mega  ICP1 = PDç = NC   ICP3 = PE7 = NC   ICP4 = PL0 = Digital pin 49   ICP5 = PL1 = Digital pin 48
// 328   ICP1 = PB0 = pin 8

// attachInterrupt fix for promicro
#if defined(PROMICRO) && defined(SERIAL_SUM_PPM)
ISR(INT6_vect){rxInt();}
#endif

// Read PPM SUM RX Data
#if defined(SERIAL_SUM_PPM)
#define SYNCPULSEWIDTH 3000		// Sync pulse must be more than 3ms long
#if defined(ICP_PIN13)
volatile uint16_t icp_value;
volatile uint16_t PPMStart;		// Sync pulse timer
volatile uint8_t chan;	        // Channel number
volatile uint8_t num_chan = 4;	// Number of channels in PPM

void rxInt()
{
	uint16_t diff;
	icp_value = ICR3;
	diff = (icp_value - PPMStart) >> 1;
	PPMStart = icp_value;
	if (diff > SYNCPULSEWIDTH)
	{
		num_chan = chan - 1;
		chan = 0;
	}
	else
	{
		if(900<diff && diff<2200 && chan<RC_CHANS)
		{
			rcValue[chan] = diff;
			if (chan >= num_chan) RcFrameData = 0x01;
		}
		chan++;
	}
}
#else
void rxInt()
{
	uint16_t now,diff;
	static uint16_t last = 0;
	static uint8_t chan = 0;
	static uint8_t num_chan = 4;
	
	now = micros();
	diff = now - last;
	last = now;
	if(diff > SYNCPULSEWIDTH)
	{
		num_chan = chan - 1;
		chan = 0;
	}
	else
	{
		if(900<diff && diff<2200 && chan<RC_CHANS)
		{ //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
			rcValue[chan] = diff;
			if (chan >= num_chan) RcFrameData = 0x01;
		}
		chan++;
	}
}
#endif
#endif

/**************************************************************************************/
/***************                   SBUS RX Data                    ********************/
/**************************************************************************************/
#if defined(SBUS)
void  readSBus()
{
	#define SBUS_SYNCBYTE 0x0F // Not 100% sure: at the beginning of coding it was 0xF0 !!!
	static uint16_t sbus[25]={0};
	while(SerialAvailable(SBUS_SERIAL_PORT))
	{
		int val = SerialRead(SBUS_SERIAL_PORT);
		if(sbusIndex==0 && val != SBUS_SYNCBYTE)
		continue;
		sbus[sbusIndex++] = val;
		if(sbusIndex==25){
			sbusIndex=0;
			// Failsafe: there is one Bit in the SBUS-protocol (Byte 25, Bit 4) whitch is the failsafe-indicator-bit
			#if defined(FAILSAFE)
			if (!((sbus[23] >> 3) & 0x0001))
			if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0; // clear FailSafe counter
			else
			continue;
			#endif
			// now the two Digital-Channels
			if ((sbus[23]) & 0x0001)       rcValue[16] = 2000; else rcValue[16] = 1000;
			if ((sbus[23] >> 1) & 0x0001)  rcValue[17] = 2000; else rcValue[17] = 1000;
			rcValue[0]  = ((sbus[1]|sbus[2]<< 8) & 0x07FF)/2+SBUS_OFFSET; // Perhaps you may change the term "/2+976" -> center will be 1486
			rcValue[1]  = ((sbus[2]>>3|sbus[3]<<5) & 0x07FF)/2+SBUS_OFFSET;
			rcValue[2]  = ((sbus[3]>>6|sbus[4]<<2|sbus[5]<<10) & 0x07FF)/2+SBUS_OFFSET;
			rcValue[3]  = ((sbus[5]>>1|sbus[6]<<7) & 0x07FF)/2+SBUS_OFFSET;
			rcValue[4]  = ((sbus[6]>>4|sbus[7]<<4) & 0x07FF)/2+SBUS_OFFSET;
			rcValue[5]  = ((sbus[7]>>7|sbus[8]<<1|sbus[9]<<9) & 0x07FF)/2+SBUS_OFFSET;
			rcValue[6]  = ((sbus[9]>>2|sbus[10]<<6) & 0x07FF)/2+SBUS_OFFSET;
			rcValue[7]  = ((sbus[10]>>5|sbus[11]<<3) & 0x07FF)/2+SBUS_OFFSET; // & the other 8 + 2 channels if you need them
			//The following lines: If you need more than 8 channels, max 16 analog + 2 digital. Must comment the not needed channels!
			//rcValue[8]  = ((sbus[12]|sbus[13]<< 8) & 0x07FF)/2+SBUS_OFFSET;
			//rcValue[9]  = ((sbus[13]>>3|sbus[14]<<5) & 0x07FF)/2+SBUS_OFFSET;
			//rcValue[10] = ((sbus[14]>>6|sbus[15]<<2|sbus[16]<<10) & 0x07FF)/2+SBUS_OFFSET;
			//rcValue[11] = ((sbus[16]>>1|sbus[17]<<7) & 0x07FF)/2+SBUS_OFFSET;
			//rcValue[12] = ((sbus[17]>>4|sbus[18]<<4) & 0x07FF)/2+SBUS_OFFSET;
			//rcValue[13] = ((sbus[18]>>7|sbus[19]<<1|sbus[20]<<9) & 0x07FF)/2+SBUS_OFFSET;
			//rcValue[14] = ((sbus[20]>>2|sbus[21]<<6) & 0x07FF)/2+SBUS_OFFSET;
			//rcValue[15] = ((sbus[21]>>5|sbus[22]<<3) & 0x07FF)/2+SBUS_OFFSET;
			RcFrameData = 0x01;
		}
	}
}
#endif


/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/
#if defined(SPEKTRUM)

void readSpektrum()
{
	int16_t Temp;
	if (spekFrameFlags != 0x01) return;
	if ((!f.ARMED) &&
	#if defined(FAILSAFE) || (SPEK_SERIAL_PORT != 0)
	(failsafeCnt > 5) &&
	#endif
	( SerialPeek(SPEK_SERIAL_PORT) == '$')) {
		while (SerialAvailable(SPEK_SERIAL_PORT))
		{
			serialCom();
			delay (10);
		}
		return;
	} //End of: Is it the GUI?
	while (SerialAvailable(SPEK_SERIAL_PORT) > SPEK_FRAME_SIZE)
	{ // More than a frame?  More bytes implies we weren't called for multiple frame times.  We do not want to process 'old' frames in the buffer.
		for (uint8_t i = 0; i < SPEK_FRAME_SIZE; i++) {SerialRead(SPEK_SERIAL_PORT);}  //Toss one full frame of bytes.
	}
	if (spekFrameFlags == 0x01) {   //The interrupt handler saw at least one valid frame start since we were last here.
		if (SerialAvailable(SPEK_SERIAL_PORT) == SPEK_FRAME_SIZE)  // size = 16
		{  //A complete frame? If not, we'll catch it next time we are called.
			SerialRead(SPEK_SERIAL_PORT); SerialRead(SPEK_SERIAL_PORT);  //Eat the header bytes
			for (uint8_t b = 2; b < SPEK_FRAME_SIZE; b += 2) {
				uint8_t bh = SerialRead(SPEK_SERIAL_PORT);
				uint8_t bl = SerialRead(SPEK_SERIAL_PORT);
				uint8_t spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);
				if (spekChannel < RC_CHANS)
				rcValue[spekChannel] = 988 + ((((uint16_t)(bh & SPEK_CHAN_MASK) << 8) + bl) SPEK_DATA_SHIFT);
				/**/
				if (spekChannel == 12) debug[0] = rcValue[12];
				if (spekChannel == 13) debug[1] = rcValue[13];
				/**/
			}
			spekFrameFlags = 0x00;
			RcFrameData = 0x01;
			#if defined(FAILSAFE)
			if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // Valid frame, clear FailSafe counter
			#endif
		}
		else
		{ //Start flag is on, but not enough bytes means there is an incomplete frame in buffer.  This could be OK, if we happened to be called in the middle of a frame.  Or not, if it has been a while since the start flag was set.
			uint32_t spekInterval = (timer0_overflow_count << 8) * (64 / clockCyclesPerMicrosecond()) - spekTimeLast;
			if (spekInterval > 2500) {spekFrameFlags = 0;}  //If it has been a while, make the interrupt handler start over.
		}
	}
}
#endif


/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/

#if defined(SERIAL_SUM_PPM)
static uint16_t rcData4Values[RC_CHANS][4], rcDataMean[RC_CHANS];
static uint8_t rc4ValuesIndex = 0;
#endif

bool computeRC()
{
	uint8_t chan;
	#if defined(SBUS)
	readSBus();
	if (RcFrameData == 0x01)
	{
		RcFrameData = 0x00;
		for (chan = 0; chan < RC_CHANS; chan++) rcData[chan] = rcValue[rcChannel[chan]];
		return true;
	}
	#elif defined(SPEKTRUM)
	readSpektrum();
	if (RcFrameData == 0x01)
	{
		RcFrameData = 0x00;
		for (chan = 0; chan < RC_CHANS; chan++) rcData[chan] = rcValue[rcChannel[chan]];
		return true;
	}
	#elif defined(SERIAL_SUM_PPM)
	if (RcFrameData == 0x01)
	{
		RcFrameData = 0x00;
		rc4ValuesIndex++;
		if (rc4ValuesIndex == 4) rc4ValuesIndex = 0;
		#if !defined(RAWPPMSUM)
		for (chan = 0; chan < RC_CHANS; chan++)
		{
			uint8_t a;
			rcData4Values[chan][rc4ValuesIndex] = rcValue[rcChannel[chan]];
			rcDataMean[chan] = 0;
			for (a=0;a<4;a++) rcDataMean[chan] += rcData4Values[chan][a];
			rcDataMean[chan]= (rcDataMean[chan]+2)>>2;
			if ( rcDataMean[chan] < (uint16_t)rcData[chan] -3)  rcData[chan] = rcDataMean[chan]+2;
			if ( rcDataMean[chan] > (uint16_t)rcData[chan] +3)  rcData[chan] = rcDataMean[chan]-2;
		}
		#else
		for (chan = 0; chan < RC_CHANS; chan++) rcData[chan] = rcValue[rcChannel[chan]];
		#endif
		#if defined(FAILSAFE)
		if(rcValue[rcChannel[THROTTLE]]<FAILSAFE_DETECT_TRESHOLD) return false;
		else if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;
		#endif
		return true;
	}
	#endif
	return false;
}




