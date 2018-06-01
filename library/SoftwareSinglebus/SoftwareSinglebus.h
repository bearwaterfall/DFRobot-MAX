#ifndef SoftwareSinglebus_h
#define SoftwareSinglebus_h

#include <inttypes.h>
#include <Stream.h>

/******************************************************************************
* Definitions
******************************************************************************/

#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || (__AVR_ATmega2560__)
#define MEGAUNO TRUE
#endif

#define CBMAX	5
#define _MAX_RX_BUFF 32 // RX buffer size
#define BUFFER_LENGTH 30
#define TX_BUFFER_LENGTH	50
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

extern char debugbuffer[20];

enum singlebusType{
	IDLE,
	BEGIN,
	TRANSMIT,
	END
};

enum idenFrame{
	VOIDLE,
	READ,
	WRITE,
	INITIALIZE,
	INTERRUPT
};

class SoftwareSinglebusMgr{
public:
	virtual void manageMsg(void);
};

class callbackfun_t{
public:
	uint8_t addr_t;		//service id
	uint8_t reg_t;
	uint8_t index_t;	//
	uint8_t length_t;	//
	uint8_t register_t;
	uint8_t data[20];	//receive data buffer
	void* arg;
	void(*callback_t)(void*);	//callback function pointer
};

class SoftwareSinglebus : public Stream{
private:
	idenFrame	ident_;
	// per object data
	uint8_t transmitPin;

	uint8_t transmitBitMask;
	volatile uint8_t *transmitPortRegister;

#if MEGAUNO == TRUE
	uint8_t receiveBitMask;
	volatile uint8_t *receivePortRegister;

	volatile uint8_t *_pcint_maskreg;
	uint8_t _pcint_maskvalue;

	void setTX(uint8_t transmitPin);
	void setRX(uint8_t receivePin);

	void setRxIntMsk(bool enable) __attribute__((__always_inline__));
#endif

	uint16_t _transmit_delay_centering;
	uint16_t _transmit_delay_intrabit;
	uint16_t _transmit_delay_stopbit;
	uint16_t _transmit_delay;

	uint16_t _buffer_overflow : 1;
	uint16_t _inverse_logic : 1;

	singlebusType localType_t;
	uint8_t receivedLength;
	static uint8_t rxBuffer[];
	static uint8_t rxBufferIndex;
	
	// static data
	static uint8_t transmitting;
	static uint8_t txAddress;
	static uint8_t txBufferIndex;
	static uint8_t txBufferLength;
	static uint8_t txBuffer[];

	static volatile uint8_t _receive_buffer_tail;
	static volatile uint8_t _receive_buffer_head;
	static SoftwareSinglebus *active_object;
	//uint8_t _pcint_maskvalue;

	//uint8_t cbNum_t;
	uint8_t cbIndex;
	callbackfun_t callbackfun[5];
	// private methods
	void recv();
	uint8_t rx_pin_read();
	void tx_pin_write(uint8_t pin_state);
	void setTransmitpin(uint8_t _tx);

	// private static method for timing
	static inline void tunedDelay(uint16_t delay_us);
	char checkBus();
	uint8_t readFrom(uint8_t address,uint8_t length);
	uint8_t isReceive;
	uint8_t isLastTimes;
#if MEGAUNO == TRUE
	//void setRxIntMsk(bool enable);
#endif

public:
	// public methods
	SoftwareSinglebus(uint8_t _transmitPin, bool inverse_logic = false);
	~SoftwareSinglebus();
	void begin();
	void init( uint8_t addr, void*);
	bool listen();
	void end();
	bool isListening() { return this == active_object; }
	bool overflow() { bool ret = _buffer_overflow; _buffer_overflow = false; return ret; }
	int peek();

	void beginTransmission(uint8_t address, idenFrame state_t);//传递设备地址
    size_t write(uint8_t b);
	uint8_t endTransmission(void);
	uint8_t requestFrom(uint8_t, uint8_t);

	size_t writeBytes(uint8_t *str, uint8_t num = 1);
	
	virtual int read();
	int read(uint8_t addr_t);
	int read(uint8_t addr_t, uint8_t reg_);
	virtual int available();
	int available(uint8_t addr_t);
	virtual void flush();

	using Print::write;
	//char debugRev;
	//static char data_t;
	char _receive_buffer[_MAX_RX_BUFF*2];
	// public only for easy access by interrupt handlers
	static inline void handle_interrupt();

	static uint8_t isInitlaze;
	char registerCB(uint8_t add, void(*cb)(void*), void* arg);
};

#endif
