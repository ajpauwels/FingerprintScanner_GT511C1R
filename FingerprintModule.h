#ifndef FINGERPRINT_MODULE_H
#define FINGERPRINT_MODULE_H

/* Includes */
#include <Arduino.h>

/* Symbolic constants */
// Symbolically define the serial interface to easily switch between Serial (HW or SW)
#define COMMS Serial1

// The maximum number of times to re-try for a response packet before failing
// Each try introduces a waiting period defined by WAITTIME
// The maximum execution time for a command in milliseconds is therefore TIMEOUT * WAITTIME
#define TIMEOUT 11

// The amount of time to wait between each response retry, in milliseconds
#define WAITTIME 500

// Commonly used bytes for all packets
#define DEVICE_ID_MSB 0x00
#define DEVICE_ID_LSB 0x01

// Commonly used command packet bytes
#define CMD_START_CODE_1 0x55
#define CMD_START_CODE_2 0xAA

// Commonly used response packet bytes
#define RES_START_CODE_1 0x55
#define RES_START_CODE_2 0xAA

// Commonly used data packet bytes
#define DATA_START_CODE_1 0x5A
#define DATA_START_CODE_2 0xA5

// Define packet sizes in bytes
#define CMD_PKT_SIZE  12
#define RESP_PKT_SIZE 12
#define DATA_PKT_MAX_SIZE 51846	// The maximum possible size of a data packet
#define DATA_PKT_ADD 6			// The size of the non-variable part of the data packet

// Uncomment if you want debug messages printed to the USB serial monitor
#define DEBUG

/* Enumerations */
// Command codes
enum COMMAND {
	CMD_OPEN = 0x01,				// Initialize the fingerprint module
	CMD_CLOSE = 0x02,				// Terminate the fingerprint module
	CMD_USB_INTERNAL_CHECK = 0x03,	// Check if the connected USB device is valid (only for USB comms)
	CMD_CHANGE_BAUDRATE = 0x04,		// Change the UART baudrate
	CMD_SET_IAP_MODE = 0x05,		// Enter IAP mode (for firmware upgrade)

	CMD_CMOS_LED = 0x12,			// Control the CMOS LED

	CMD_GET_ENROLL_COUNT = 0x20,	// Get the number of enrolled fingerprints
	CMD_CHECK_ENROLLED = 0x21,		// Check if given ID is enrolled
	CMD_ENROLL_START = 0x22,		// Start enrolling
	CMD_ENROLL1 = 0x23,				// 1st enrollment template (ENROLL_START -> ENROLL1)
	CMD_ENROLL2 = 0x24,				// 2nd enrollment template (ENROLL_START -> ENROLL1 -> ENROLL2)
	CMD_ENROLL3 = 0x25,				// 3rd enrollment template (ENROLL_START -> ENROLL1 -> ENROLL2 -> ENROLL3)
	CMD_IS_PRESS_FINGER = 0x26,		// Check if a finger is on the sensor

	CMD_ACK = 0x30,					// Acknowledge response (OK)
	CMD_NACK = 0x31,				// Non-acknowledge response (ERROR)

	CMD_DELETE_ID = 0x40,			// Delete fingerprint with specified ID
	CMD_DELETE_ALL = 0x41,			// Delete all fingerprints

	CMD_VERIFY = 0x50,				// Verify if captured print matches template with specified ID (1:1)
	CMD_IDENTIFY = 0x51,			// Identify captured fingerprint (1:N)
	CMD_VERIFY_TEMPLATE = 0x52,		// Verify the given fingerprint template matches the template with specified ID (1:1)
	CMD_IDENTIFY_TEMPLATE = 0x53,	// Identify the given fingerprint template (1:N)

	CMD_CAPTURE_FINGER = 0x60,		// Capture a fingerprint image if finger is pressed and store in RAM
	CMD_MAKE_TEMPLATE = 0x61,		// Make template based off of previous CAPTURE_FINGER call and transmit
	CMD_GET_IMAGE = 0x62,			// Transmit fingerprint image captured with CAPTURE_FINGER
	CMD_GET_RAW_IMAGE = 0x63,		// Capture image (regardless of whether finger is placed) and transmit

	CMD_GET_TEMPLATE = 0x70,		// Retrieve template with specified ID
	CMD_SET_TEMPLATE = 0x71,		// Set template with specified ID to be new uploaded template
};

// Whether or not the command and subsequent reply was successful
enum RESPONSE {
	ACK = 0x30,
	NACK = 0x31
};

// Error codes for when response packet is NACK or no packet was received
enum RESPONSE_ERROR {
	NACK_NOT_RECVD = 0x0001,				// No response packet was received
	NACK_INVALID_ENROLLMENT_STAGE = 0x0002,	// The stage of enrollment is not between 0 and 2

	NACK_INVALID_POS = 0x1003,				// Specified ID not between 0-19
	NACK_IS_NOT_USED = 0x1004,				// Specified ID is not in use
	NACK_IS_ALREADY_USED = 0x1005,			// Specified ID is already in use
	NACK_COMM_ERR = 0x1006,					// Communications error
	NACK_VERIFY_FAILED = 0x1007,			// A 1:1 verification failed
	NACK_IDENTIFY_FAILED = 0x1008,			// A 1:N identification failed
	NACK_DB_IS_FULL = 0x1009,				// Database is full
	NACK_DB_IS_EMPTY = 0x100A,				// Database is empty
	NACK_BAD_FINGER = 0x100C,				// Fingerprint quality is too low
	NACK_ENROLL_FAILED = 0x100D,			// Enrollment failed
	NACK_IS_NOT_SUPPORTED = 0x100E,			// The command is not supported
	NACK_DEV_ERR = 0x100F,					// Device error
	NACK_INVALID_PARAM = 0x1011,			// Invalid parameter
	NACK_FINGER_IS_NOT_PRESSED = 0x1012,	// Finger is not pressed

	NACK_BAD_HEADER = 0x1013,				// Packet header is incorrect
	NACK_BAD_ID = 0x1014,					// Device ID in packet does not match desired device ID
	NACK_BAD_CHKSUM = 0x1015				// Given checksum does not match computed checksum
};

// The different states the fingerprint module can be in during enrolling
enum ENROLL_STATE {
	START,
	CAPTURE,
	ENROLL,
	COMPLETE,
	REMOVE_FINGER
};

/* Type definitions */
// Check if byte, word, and dword are defined, define them if not
#ifndef byte
typedef unsigned char byte;
#endif

#ifndef word
typedef uint16_t word;
#endif

#ifndef dword
typedef uint32_t dword;
#endif

// Used in enrollSequence, defines a type for a lambda function given to write to an output
typedef void (*writeFunc)(const char* str);

/* Class definition */
class FingerprintModule {
	private:
		byte mRespPkt[RESP_PKT_SIZE];		// Buffer to hold the response packet
		byte mDataPkt[DATA_PKT_MAX_SIZE];	// Buffer to hold data packets
		bool mRespStatus;					// Holds whether an ACK or NACK was received
		dword mRespParam;					// Holds the response parameter: either an error code or a response param
		uint8_t mEnrollmentStage;			// Used during enrollment, keeps track of if this is the first, second, or third fingerprint image

		word flipEndianness(word);
		dword flipEndianness(dword);
		void split(word, byte*);
		void split(dword, byte*);
		word computeCheckSum(byte*, uint32_t);
		bool send(word, dword param = 0x00000000, bool isBigEndian = true);
		bool sendDataPkt();
		bool recvResponsePkt();
		bool recvDataPkt(uint32_t size);

	public:
		FingerprintModule();
		~FingerprintModule();

		dword getResponseParam();
		dword getErrorCode();
		bool getResponseStatus();
		String strFromError(word);

		bool enrollSequence(uint32_t, writeFunc out = 0x00);

		bool open(bool errChk = true);
		bool close();
		bool powerCMOS(bool);
		bool changeBaudrate(uint32_t);
		bool getEnrollCount();
		bool isIDEnrolled(uint32_t);
		bool startEnrollment(uint32_t);
		bool createEnrollmentTemplate();
		bool isFingerPressed();
		bool captureFingerprint(bool highQual = false);
		bool deleteID(uint32_t);
		bool deleteAll();
		bool verify(uint32_t);
		bool identify();
		bool verifyTemplate(uint32_t, byte[]);
		bool identifyTemplate(byte[]);
};

#endif
