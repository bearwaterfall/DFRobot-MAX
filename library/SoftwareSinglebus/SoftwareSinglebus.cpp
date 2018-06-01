#include <SoftwareSinglebus.h>
#include <Arduino.h>

#if MEGAUNO ==TRUE
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay_basic.h>
#endif

#if MEGAUNO == TRUE
#define _READDELAY_	10
#define _POLLDELAY_	20
#define _RECEIVEDELAY_	200
#define __PARAGRAPHGUARD__ 50
#else
#define _READDELAY_	10
#define _RECEIVEDELAY_	100
#define _POLLDELAY_	5
#define __PARAGRAPHGUARD__ 200
#endif

typedef struct _DELAY_TABLE{
	long baud;
	unsigned short rx_delay_centering;
	unsigned short rx_delay_intrabit;
	unsigned short rx_delay_stopbit;
	unsigned short tx_delay;
} DELAY_TABLE;

#if  MEGAUNO == TRUE
#if F_CPU == 16000000			//rxcenter   rxintra    rxstop    tx
static const DELAY_TABLE table = { 57600, 10, 37, 37, 33, };
const int XMIT_START_ADJUSTMENT = 5;
#elif F_CPU == 8000000
static const DELAY_TABLE table = { 14400, 30, 75, 75, 72, };
const int XMIT_START_ADJUSTMENT = 4;
#elif F_CPU == 20000000
static const DELAY_TABLE table = { 14400, 96, 197, 197, 194, };
const int XMIT_START_ADJUSTMENT = 6;
#else
#error This version of SIM808SoftSerial supports only 20, 16 and 8MHz processors
#endif
#else
	const int XMIT_START_ADJUSTMENT = 0;			//delay gain...
	static const DELAY_TABLE table = { 14400, 70, 140, 140, 145, };
#endif
uint8_t SoftwareSinglebus::rxBuffer[BUFFER_LENGTH];
uint8_t SoftwareSinglebus::rxBufferIndex = 0;

uint8_t SoftwareSinglebus::txAddress = 0;
uint8_t SoftwareSinglebus::transmitting = 0;
uint8_t SoftwareSinglebus::txBufferIndex = 0;
uint8_t SoftwareSinglebus::txBufferLength = 0;
uint8_t SoftwareSinglebus::txBuffer[_MAX_RX_BUFF];

SoftwareSinglebus *SoftwareSinglebus::active_object=0;
uint8_t SoftwareSinglebus::isInitlaze = false;
//char SoftwareSinglebus::_receive_buffer[_MAX_RX_BUFF];
volatile uint8_t SoftwareSinglebus::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSinglebus::_receive_buffer_head = 0;

#if MEGAUNO == TRUE
void SoftwareSinglebus::tunedDelay(uint16_t delay_us){
	/*uint8_t tmp = 0;
	asm volatile("sbiw    %0, 0x01 \n\t"
		"ldi %1, 0xFF \n\t"
		"cpi %A0, 0xFF \n\t"
		"cpc %B0, %1 \n\t"
		"brne .-10 \n\t"
		: "+r" (delay_us), "+a" (tmp)
		: "0" (delay_us)
		);*/
	_delay_loop_2(delay_us);
}
#else
	inline void SoftwareSinglebus::tunedDelay(uint16_t delay_us){
		while (delay_us--){
			asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop");
			asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop");
		}
	}
#endif

#if MEGAUNO == TRUE
	void SoftwareSinglebus::tx_pin_write(uint8_t pin_state){
		if (pin_state == LOW){
			*transmitPortRegister &= ~transmitBitMask;
		}
		else{
			*transmitPortRegister |= transmitBitMask;
		}
	}

	uint8_t SoftwareSinglebus::rx_pin_read(){
		return *receivePortRegister & receiveBitMask;
	}			
#else
void SoftwareSinglebus::tx_pin_write(uint8_t pin_state){
	if (pin_state == LOW){
		//*transmitPortRegister &= ~transmitBitMask;
		(GPIO_Desc[transmitPin].P)->DOUT &= ~GPIO_Desc[transmitPin].bit;
	}else{
	//	*transmitPortRegister |= transmitBitMask;
		(GPIO_Desc[transmitPin].P)->DOUT |= GPIO_Desc[transmitPin].bit;
	}
}



uint8_t SoftwareSinglebus::rx_pin_read(){
	return ((GPIO_Desc[transmitPin].P)->PIN & GPIO_Desc[transmitPin].bit) ? HIGH : LOW;
}
#endif

void SoftwareSinglebus::recv(){
	uint8_t checkLocal = 0x00;
	uint16_t checkSum = 0x00;
#if GCC_VERSION<40302
#if MEGAUNO == TRUE
	asm volatile(
		"push r18 \n\t"
		"push r19 \n\t"
		"push r20 \n\t"
		"push r21 \n\t"
		"push r22 \n\t"
		"push r23 \n\t"
		"push r26 \n\t"
		"push r27 \n\t"
		::);
#endif
#endif
	uint8_t dta = 0;
	int oldTail = 0;//, thisHead;
	bool morebytes = true;	
	bool cb_t = true;

#if MEGAUNO == TRUE
	setRxIntMsk(false);
#endif	
	do{
		if (!rx_pin_read()){
			tunedDelay(_transmit_delay_centering);
			for (uint8_t i = 0x1; i; i <<= 1){	//receive one byte or every 8 bit         
				tunedDelay(_transmit_delay_intrabit);
				uint8_t notify = ~i;
				if (rx_pin_read()){
					dta |= i;
				}
				else{
					dta &= notify;
				}
			}
			this->_receive_buffer[oldTail++] = dta;
			morebytes = false;
			tunedDelay(/*_transmit_delay_stopbit*/_transmit_delay_intrabit);//140
			for (uint8_t i = 0; i < 100; i++){
				tunedDelay(1);
				if (!rx_pin_read()){
					morebytes = true;
					break;
				}
			}
			if (morebytes == false){
				
				for (char i = 0; i < oldTail-1; i++){
					checkSum += this->_receive_buffer[i];
				}			
				checkSum = checkSum & 0XFF;
				checkLocal = this->_receive_buffer[oldTail-1];
				if (checkSum == checkLocal){	//received 0x0a		
					isReceive = true;
					if (transmitting){	//center have send READ commond and transmitting=1			
						localType_t = TRANSMIT;
						receivedLength = oldTail;					
					}
					else if (cbIndex){	//if register any callback function and do the following logic...
						uint8_t addr_l = 0x00;
						uint8_t strs[30];
						uint8_t lengthNUM = 0;
						uint8_t _state_t = false;
						
						for (uint8_t m = 0; m < oldTail; m++){
							if ((_receive_buffer[m] == 0x55)/* && (_receive_buffer[m + 1] == 0xaa)*/){
								lengthNUM = oldTail - m - 1;
								memcpy(strs, &_receive_buffer[m], lengthNUM);	//copy all data to strs,and set _state_t=true,to do next...
								_state_t = true;
								break;
							}
						}

						if (_state_t){	//55 aa 0x01(addr) 0x01(len) 0x05(interrupt) 0x01(register) 0xXX(data) 0x0d 0x0a 
							for (uint8_t j = 0; j < cbIndex; j++){	//find the service who have registered
								if (this->callbackfun[j].addr_t == strs[2]){
									uint8_t sendAck[6] = { 0x55, 0xaa, 0x00, 0x00, 0xff, 0x00 };
									this->callbackfun[j].length_t = strs[3];
									this->callbackfun[j].reg_t = strs[4];
									this->callbackfun[j].register_t = strs[5];	//register start address
									this->callbackfun[j].index_t = 0;

									memcpy(this->callbackfun[j].data, &strs[6], strs[3]);
									
									tx_pin_write(HIGH);
									checkSum = 0x1fe + this->callbackfun[j].addr_t;
									sendAck[2] = this->callbackfun[j].addr_t;
									sendAck[5] = checkSum & 0xff;
									this->writeBytes(sendAck, 6);

									interrupts();
									this->callbackfun[j].callback_t(callbackfun[j].arg);			//do call back function
									noInterrupts();																			
									cb_t = false;
								}
							}
						}
						if (cb_t){
							localType_t = TRANSMIT;
							receivedLength = oldTail;
						}
					}
					else{
						break;
					}
				}else{		//check error
					isReceive = false;
				}
			}
			tx_pin_write(HIGH);
		}else{
			//morebytes = false;
			break;
		}
	}while (morebytes);
	
#if MEGAUNO == TRUE
	setRxIntMsk(true);
#endif
#if GCC_VERSION < 40302
#if MEGAUNO == TRUE
	asm volatile(
		"pop r27 \n\t"
		"pop r26 \n\t"
		"pop r23 \n\t"
		"pop r22 \n\t"
		"pop r21 \n\t"
		"pop r20 \n\t"
		"pop r19 \n\t"
		"pop r18 \n\t"
		::);
#endif
#endif
}

void SoftwareSinglebus::handle_interrupt(){
	if (active_object)
	{
#if MEGAUNO == FALSE
		noInterrupts();
#endif
		active_object->recv();	
#if MEGAUNO == FALSE
		interrupts();
#endif
		
	}
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
	SoftwareSinglebus::handle_interrupt();
}
#endif
#if defined(PCINT1_vect)
ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect, ISR_ALIASOF(PCINT0_vect));
#endif

SoftwareSinglebus::SoftwareSinglebus(uint8_t _transmitPin, bool inverse_logic):
_transmit_delay_centering(0),
_transmit_delay_intrabit(0),
_transmit_delay_stopbit(0),
_transmit_delay(0),
_buffer_overflow(false),
_inverse_logic(inverse_logic){
	isInitlaze = true;
	setTransmitpin(_transmitPin);
}

SoftwareSinglebus::~SoftwareSinglebus(){
	end();
}

#if MEGAUNO == TRUE
void SoftwareSinglebus::setTransmitpin(uint8_t _tx){
	transmitPin = _tx;
	cbIndex = 0;	
	//uno int=0 digital=2	
	//setTxMode(false);
	setTX(_tx);
	setRX(_tx);
}

void SoftwareSinglebus::setTX(uint8_t tx)
{
	// First write, then set output. If we do this the other way around,
	// the pin would be output low for a short while before switching to
	// output hihg. Now, it is input with pullup for a short while, which
	// is fine. With inverse logic, either order is fine.
	digitalWrite(tx, _inverse_logic ? LOW : HIGH);
	pinMode(tx, OUTPUT);
	transmitBitMask = digitalPinToBitMask(tx);
	uint8_t port = digitalPinToPort(tx);
	transmitPortRegister = portOutputRegister(port);
}

void SoftwareSinglebus::setRX(uint8_t rx)
{
	pinMode(rx, INPUT);
	if (!_inverse_logic)
		digitalWrite(rx, HIGH);  // pullup for normal logic!
	transmitPin = rx;
	receiveBitMask = digitalPinToBitMask(rx);
	uint8_t port = digitalPinToPort(rx);
	receivePortRegister = portInputRegister(port);
}

#else
void SoftwareSinglebus::setTransmitpin(uint8_t _tx){
	transmitPin = _tx;
	cbIndex = 0;
}
#endif

#if MEGAUNO == TRUE
uint16_t subtract_cap(uint16_t num, uint16_t sub) {
	if (num > sub)
		return num - sub;
	else
		return 1;
}

void SoftwareSinglebus::begin(){

	uint16_t bit_delay = (F_CPU / 57600) / 4;
	_transmit_delay = subtract_cap(bit_delay,15/4);
	if (digitalPinToPCICR(transmitPin)) {
#if GCC_VERSION > 40800
		_transmit_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);
		_transmit_delay_intrabit = subtract_cap(bit_delay, 23 / 4);
		_transmit_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (37 + 11) / 4);
#else
		_transmit_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4);
		_transmit_delay_intrabit = subtract_cap(bit_delay, 11 / 4);
		_transmit_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (44 + 17) / 4);
#endif
	}

	if (digitalPinToPCICR(transmitPin)){
		*digitalPinToPCICR(transmitPin) |= _BV(digitalPinToPCICRbit(transmitPin));
		_pcint_maskreg = digitalPinToPCMSK(transmitPin);
		_pcint_maskvalue = _BV(digitalPinToPCMSKbit(transmitPin));
		tunedDelay(_transmit_delay);
	}
	/*_transmit_delay_centering = table.rx_delay_centering;
	_transmit_delay_intrabit = table.rx_delay_intrabit;
	_transmit_delay_stopbit = table.rx_delay_stopbit;
	_transmit_delay = table.tx_delay;*/
	//Serial.println(_transmit_delay);
	//_transmit_delay -= 5;
	active_object = this;	
	setRxIntMsk(true);
}

void SoftwareSinglebus::setRxIntMsk(bool enable)
{
	if (enable)
		*_pcint_maskreg |= _pcint_maskvalue;
	else
		*_pcint_maskreg &= ~_pcint_maskvalue;
}
#else
void SoftwareSinglebus::begin(){
	_transmit_delay_centering = table.rx_delay_centering;
	_transmit_delay_intrabit = table.rx_delay_intrabit;
	_transmit_delay_stopbit = table.rx_delay_stopbit;
	_transmit_delay = table.tx_delay;
//	Set up transmit pin to interrupts after init...
	active_object = this;
	pinMode(transmitPin, OUTPUT);
	if (!_inverse_logic)
		digitalWrite(transmitPin, HIGH);
	attachInterrupt(0, SoftwareSinglebus::handle_interrupt, 3);
}
#endif

void SoftwareSinglebus::end(){
	active_object = NULL;
}

char SoftwareSinglebus::checkBus(){
	uint8_t m = _POLLDELAY_;
	for (; m; m--){
		if (digitalRead(transmitPin) == LOW){
			return false;
		}
		asm volatile("nop"); asm volatile("nop"); asm volatile("nop"); asm volatile("nop");
	}
	return true;
}

//write a byte to bus
size_t SoftwareSinglebus::write(uint8_t b){
	if (transmitting){
		if (txBufferLength >= TX_BUFFER_LENGTH){	//32Bytes
			return 0;
		}
		txBuffer[txBufferIndex] = b;
		++txBufferIndex;
		txBufferLength = txBufferIndex;
	}
	return 1;
}

//write bytes and just check BUS once
#if MEGAUNO == TRUE
size_t SoftwareSinglebus::writeBytes(uint8_t *str, uint8_t num){
	if (_transmit_delay == 0){
		return 0;
	}
	if (this->checkBus() == false)	return 0;
	
	volatile uint8_t *reg = transmitPortRegister;
	uint8_t reg_mask = transmitBitMask;
  uint8_t inv_mask = ~transmitBitMask;
	uint8_t oldSREG = SREG;
	uint8_t startDelay = _transmit_delay;
	cli();
	setRxIntMsk(false);
	setTX(transmitPin);
	while (num--){
		uint8_t b = *str;
		//str++;
		
		tx_pin_write(_inverse_logic ? HIGH : LOW);
		tunedDelay(_transmit_delay /*+ XMIT_START_ADJUSTMENT*/);
/*
		for (byte mask = 0x01; mask; mask <<= 1){
			if ((*str)&mask)	tx_pin_write(HIGH);
			else tx_pin_write(LOW);
			tunedDelay(_transmit_delay);
		}
		*/
		for(uint8_t i=8;i>0;--i){
			if(b&1)
				*reg  |= reg_mask;
			else
				*reg &= inv_mask;
			tunedDelay(_transmit_delay);
			b >>= 1;
		}
		
		str++;
		tx_pin_write(HIGH);

		tunedDelay(_transmit_delay);
	}
	interrupts();
	SREG = oldSREG;
	sei();
	setRxIntMsk(true);
	setRX(transmitPin);
	return 1;
}
#else
size_t SoftwareSinglebus::writeBytes(uint8_t *str, uint8_t num){
	if (_transmit_delay == 0){
		return 0;
	}
	if (this->checkBus() == false)	return;
	while (num--){
		uint8_t b = *str;
		str++;
		noInterrupts(); //turn off interrupts for a clean txmit
		tx_pin_write(_inverse_logic ? HIGH : LOW);
		tunedDelay(_transmit_delay + XMIT_START_ADJUSTMENT);

		if (_inverse_logic){
			for (byte mask = 0x01; mask; mask <<= 1){
				if (b&mask)	tx_pin_write(LOW);
				else tx_pin_write(HIGH);
				tunedDelay(_transmit_delay);
			}
			tx_pin_write(LOW);
		}
		else{
			for (byte mask = 0x01; mask; mask <<= 1){
				if (b&mask)	tx_pin_write(HIGH);
				else tx_pin_write(LOW);
				tunedDelay(_transmit_delay);
			}
			tx_pin_write(HIGH);
		}
		interrupts();
		tunedDelay(_transmit_delay);
	}
	tunedDelay(_transmit_delay-12);
	return 1;
}
#endif

void SoftwareSinglebus::flush(){
	if (!isListening())	return;
	noInterrupts();
	_receive_buffer_head = _receive_buffer_tail = 0;
	interrupts();
}

int SoftwareSinglebus::peek(){
	if (!isListening())
		return -1;
	// Empty buffer?
	if (_receive_buffer_head == _receive_buffer_tail)
		return -1;
	// Read from "head"
	return _receive_buffer[_receive_buffer_head];
}

//slave address
void SoftwareSinglebus::beginTransmission(uint8_t address, idenFrame state_t){
	transmitting = 1;
	txAddress = address;
	ident_ = state_t;
	rxBufferIndex = 0;
	receivedLength = 0;
	isLastTimes = 0;
	delay(3);
}

/*
*transmitting is a switch,to control callback register function.when center will read peripheral,transmitting=1,at this time
*if something coming,will not callback register function,because center want them.
*/

uint8_t SoftwareSinglebus::endTransmission(void){
	uint8_t ret = 0;
	uint8_t checkSum = 0x00;
	uint8_t tx_str[20];
	tx_str[0] = 0x55;	//frame hander
	tx_str[1] = 0xaa;
	tx_str[2] = txAddress;	//slave address
	tx_str[3] = txBufferIndex;	//frame length or will read data length
	tx_str[4] = ident_;	//frame 
	if (ident_ == READ){
		tx_str[3] = txBuffer[1];	//read data len.
		tx_str[5] = txBuffer[0];	//register
		for (char i = 0; i < 6; i++){
			checkSum += tx_str[i];
		}
		tx_str[6] = checkSum & 0xff;
		ret = this->writeBytes(tx_str, 7);
		if(this->isLastTimes == 3){
			ident_ = VOIDLE;
		}
		this->localType_t = BEGIN;
	}
	else if (ident_ == WRITE){
		memcpy(&tx_str[5], txBuffer, txBufferIndex);
		tx_str[3] -= 1;
		
		for (char i = 0; i < txBufferIndex + 5; i++){
			checkSum += tx_str[i];
		}
		tx_str[txBufferIndex + 5] = checkSum;// &0xff;
		ret = this->writeBytes(tx_str, txBufferLength + 6);
		transmitting = 0;
	}
	else{
		ident_ = VOIDLE;
		transmitting = 0;
	}
	txBufferIndex = 0;
	txBufferLength = 0;
	// indicate that we are done transmitting
	delayMicroseconds(500);	
	
	return ret;
}

uint8_t SoftwareSinglebus::requestFrom(uint8_t addr, uint8_t quantity){
	if (quantity > BUFFER_LENGTH){
		quantity = BUFFER_LENGTH;
	}
	uint8_t read = readFrom(addr,quantity);
}

//wait timeout,break if timeout and read false
uint8_t SoftwareSinglebus::readFrom(uint8_t address_t, uint8_t length){
	unsigned long m;
	
	for(;;){
		m = millis();
		while ((millis() - m) < _READDELAY_){
			delay(2);
			if (this->localType_t == TRANSMIT){	//received data	55 aa 01 01 01 7f 0d 0a
				for (char i = 0; i < receivedLength; i++){
					if ((_receive_buffer[i] == 0x55)/* && (_receive_buffer[i + 1] == 0xaa)*/){	//find frame hander
						i += 2;
						if (_receive_buffer[i] == address_t){
							if (length > (receivedLength - i - 5))
								length = receivedLength - i - 5;
							if (length > _receive_buffer[i + 1])
								length = _receive_buffer[i + 1];
							memcpy(rxBuffer, &_receive_buffer[i + 4], length);
							receivedLength = length;	
							break;
						}
					}
				}
				this->localType_t = END;
				return true;
			}
		}
		if(isLastTimes < 3){
			isLastTimes++;
			#if 0
			Serial.print("isLastTimes---------------------------------");
			Serial.println(isLastTimes);
			#endif
			delayMicroseconds(100);
			this->endTransmission();
		}else{
			//Serial.println("read error!!!");
			break;
		}
	}
	//delay(1);
	return false;
}

int SoftwareSinglebus::read(){
	int value = -1;
	if (rxBufferIndex < receivedLength){
		value = rxBuffer[rxBufferIndex++];
		//++rxBufferIndex;
	}
	return value;
}

int SoftwareSinglebus::read(uint8_t addr_t){
	uint8_t value = 0x00;
	uint8_t index_t = 0x00; 
	if (cbIndex == 0)
		return -1;
	for (uint8_t i = 0; i < cbIndex; i++){
		if (this->callbackfun[i].addr_t == addr_t){
			if (this->callbackfun[i].index_t < this->callbackfun[i].length_t){
				value = this->callbackfun[i].data[this->callbackfun[i].index_t];
				this->callbackfun[i].index_t += 1;
			}
		}
	}
	return value;
}

int SoftwareSinglebus::read(uint8_t addr_t, uint8_t reg_){
	uint8_t value = 0x00;
	uint8_t index_t = 0x00;
	if (cbIndex == 0)
		return -1;
	for (uint8_t i = 0; i < cbIndex; i++){
		if (this->callbackfun[i].addr_t == addr_t){
			if (this->callbackfun[i].index_t < this->callbackfun[i].length_t){
				reg_ = this->callbackfun[i].reg_t;
				value = this->callbackfun[i].data[this->callbackfun[i].index_t];
				this->callbackfun[i].index_t += 1;
			}
		}
	}
	return value;
}

int SoftwareSinglebus::available(){
	return	(receivedLength - rxBufferIndex);
}

int SoftwareSinglebus::available(uint8_t addr_t){
	uint8_t value = 0x00;
	uint8_t index_t = 0x00;
	if (cbIndex == 0)
		return -1;
	for (uint8_t i = 0; i < cbIndex; i++){
		if (this->callbackfun[i].addr_t == addr_t){
			value = this->callbackfun[i].length_t - this->callbackfun[i].index_t;
		}
	}
	return value;
}

/*
*register callback function
*/
char SoftwareSinglebus::registerCB(uint8_t add, void(*cb)(void*), void* arg){
	if (cbIndex == 5){
		return false;
	}
	this->callbackfun[cbIndex].addr_t = add;
	this->callbackfun[cbIndex].callback_t = cb;
	this->callbackfun[cbIndex].index_t = 0;
	this->callbackfun[cbIndex].length_t = 0;
	this->callbackfun[cbIndex].arg = arg;
	cbIndex++;
	return true;
}







