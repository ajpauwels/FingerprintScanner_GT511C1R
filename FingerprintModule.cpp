/**
 * Library to run the GT-511C1R fingerprint module from ADH Technology.
 *
 * Notes:
 *	-	Each time this class is instantiated, it statically allocates both the response and data
 *		packet buffers. The response packet buffer is set to an easily manageable 12 bytes, but
 *		the data packet buffer is allocated to its maximum possible size of 51,840 bytes.
 *		This means special attention should be made that there be enough RAM available to store
 *		this rather large object.
 *	-	This library gives you public access to the response and data packet arrays. Mutual
 *		exclusion is not guaranteed, and any changes made using these pointers will permanently
 *		change the buffer. It's recommended to do a copy into your own data buffer, and to ensure
 *		no interrupts will attempt to modify the buffers while a packet is being received.
 *	-	To enroll a fingerprint, follow this general flow (from the datasheet):
 *			1.	Call startEnroll(id) with the ID you'd like to enroll and ensure it succeeds
 *			2.	Call captureFingerprint until it succeeds, or until an error that isn't NACK_FINGER_IS_NOT_PRESSED
 *				occurs, in which case either try restarting the enrollment or resetting the device
 *			3.	Call createEnrollmentTemplate() to generate a template from the recorded finerprint. The
 *				library will keep track of which enrollment you're on for you.
 *
 * @author Alexandre Pauwels
 *
 * Last updated: 06/08/2016
 */

// Includes
#include "FingerprintModule.h"

// BEGIN PUBLIC

/**
 * Initializes the fingerprint module by doing the following:
 *	- Open serial communications
 */
FingerprintModule::FingerprintModule() {
	COMMS.begin(9600);
	while(!COMMS);
}

/**
 * Destroys the fingerprint module by doing the following:
 *	- Close serial communications
 */
FingerprintModule::~FingerprintModule() {
	COMMS.end();
}

/**
 * Retrieves a double-word containing the response parameter
 * provided by the module. Use only if the latest response was
 * successful, otherwise will return the previous response's
 * parameter.
 *
 * @return A double-word (4 bytes) containing the parameter in big-endian format
 */
dword FingerprintModule::getResponseParam() {
	return mRespParam;
}

/**
 * Retrieves the error code from a bad response. This function is
 * exactly the same as getResponseParam(), but is provided here to
 * make the library's function calls easier to understand in your code.
 *
 * @return A double-word (4 bytes) containing the error code from the last reponse
 */
dword FingerprintModule::getErrorCode() {
	return mRespParam;
}

/**
 * Retrieves whether the microcontroller successfully received the latest
 * response request. Use in conjunction with getErrorCode() to get a good idea
 * of what failed.
 *
 * @return True if the last response was successfully received, false otherwise
 */
bool FingerprintModule::getResponseStatus() {
	return mRespStatus;
}

/**
 * Accepts an error code and returns a string containing the companying error
 * message.
 *
 * @param word A word (2 bytes) containing the error code
 *
 * @return A string explaining the error
 */
String FingerprintModule::strFromError(word errCode) {
	switch (errCode) {
		case NACK_NOT_RECVD:
			return F("no response was received");
			break;

		case NACK_INVALID_ENROLLMENT_STAGE:
			return F("the enrollment stage is not between 0 and 2, restart the enrollment");
			break;

		case NACK_INVALID_POS:
			return F("the given ID is not between 0 and 19");
			break;

		case NACK_IS_NOT_USED:
			return F("there is no enrollment for the given ID");
			break;

		case NACK_IS_ALREADY_USED:
			return F("the given ID is already in use");
			break;

		case NACK_COMM_ERR:
			return F("the given checksum does not match the computed checksum");
			break;

		case NACK_VERIFY_FAILED:
			return F("could not match the fingerprint to the specified enrollment ID");
			break;

		case NACK_IDENTIFY_FAILED:
			return F("the given fingerprint did not match any enrollments");
			break;

		case NACK_DB_IS_FULL:
			return F("the maximum number of enrolled fingerprints has already been reached");
			break;

		case NACK_DB_IS_EMPTY:
			return F("there are no enrolled templates on the device");
			break;

		case NACK_BAD_FINGER:
			return F("the recorded fingerprint is of too low quality to be used");
			break;

		case NACK_ENROLL_FAILED:
			return F("failed to enroll the fingerprint");
			break;

		case NACK_IS_NOT_SUPPORTED:
			return F("did not recognize the given command");
			break;

		case NACK_DEV_ERR:
			return F("the fingerprint sensor has experienced a fatal error");
			break;

		case NACK_INVALID_PARAM:
			return F("the given parameter was invalid");
			break;

		case NACK_FINGER_IS_NOT_PRESSED:
			return F("no finger was detected pressed on the device");
			break;

		case NACK_BAD_HEADER:
			return F("the sent packet's header was not recognized");
			break;

		case NACK_BAD_ID:
			return F("the sent packet's device ID was incorrect (should be 0x0001)");
			break;

		case NACK_BAD_CHKSUM:
			return F("the sent packet's checksum did not match the checksum computed by the sensor");
			break;

		default:
			return F("unrecognized error");
			break;
	}
}

/**
 * This is a blocking function which will enroll one fingerprint
 * to the specified ID. The enrollment is emulated as a state-machine
 * and performs error-checking along the way to recover from any
 * bad input or communications errors. If the error is unrecoverable,
 * the function returns and the error can be retrieved using getErrorCode().
 * If the second argument (a function pointer) is provided, it will be called
 * with a char array of size 16 providing a basic user-oriented message, e.g.
 * "Place finger". This way, any output device can be attached to this function
 * and used to provide the user with instructions.
 *
 * @param id The ID of the fingerprint to enroll
 * @param out A pointer to a function taking in a const char* (optional)
 *
 * @return True on enrollment success, false otherwise
 */
bool FingerprintModule::enrollSequence(uint32_t id, writeFunc out) {
	bool success = true;					// Indicates whether the enrollment was successful
	bool done = false;						// Indicates whether or not to exit the state machine
	ENROLL_STATE state = START;				// Stores the current state of the state machine
	bool usingStream = (out != nullptr);	// True if an output function was given

	if (usingStream) {
		out("Beginning enroll");
	}

	while (!done) {
		switch (state) {
			// Begin enrollment for the specified ID, end execution on error
			case START:
				if (startEnrollment(id)) {
					state = REMOVE_FINGER;
				} else {
					success = false;
					done = true;
				}
				break;

			// Capture the image of a fingerprint
			case CAPTURE:
				// Give user instruction to place finger
				if (usingStream) {
					out("Place finger");
				}

				// Error out if CMOS could not light
				if (!powerCMOS(true)) {
					success = false;
					done = true;
				}

				// Try and capture a fingerprint, if comms have broke down return
				if (captureFingerprint(true)) {
					state = ENROLL;
				} else {
					if (mRespParam == NACK_COMM_ERR) {
						success = false;
						done = true;
					}
				}
				break;

			// Enroll a captured fingerprint
			case ENROLL:
				// Error out if CMOS won't turn off
				if (!powerCMOS(false)) {
					success = false;
					done = true;
				}

				// Try and enroll, reset on failure
				if (createEnrollmentTemplate()) {
					if (mEnrollmentStage == 3) {
						state = COMPLETE;
					} else {
						state = REMOVE_FINGER;
					}
				} else {
					if (mRespParam == NACK_ENROLL_FAILED || mRespParam == NACK_BAD_FINGER) {
						state = CAPTURE;
					} else {
						success = false;
						done = true;
					}
				}
				break;

			// End the enrollment process
			case COMPLETE:
				done = true;
				break;

			// Used to ensure the user has removed his finger before another capture
			case REMOVE_FINGER:
				int i;
				// Give user instruction to remove finger
				if (usingStream) {
					out("Remove finger");
				}

				for (i = 0; i < 5 && !isFingerPressed(); ++i);

				// If the finger is not pressed, move on, however if there's a comm error, end execution
				if (i == 5) {
					if (mRespParam == NACK_FINGER_IS_NOT_PRESSED) {
						state = CAPTURE;
					} else {
						success = false;
						done = true;
					}
				}
				break;

			default:
				success = false;
				done = true;
				break;
		}
	}

	// Indicate success or failure if output stream used
	if (usingStream) {
		if (success) {
			out("Success!");
		} else {
			out("Failed to enroll");
		}
	}

	return success;
}

/**
 * Initializes the fingerprint module. Should only be called on creation
 * of the fingerprint module. Parameter determines if the library should
 * request additional information from the fingerprint module to perform
 * more thorough error checking. This is recommended.
 *
 * @param errChk True to perform error checking (default), false otherwise
 *
 * @return True if open succeeds, false otherwise
 */
bool FingerprintModule::open(bool errChk) {
	bool success;	// Indicates whether or not the open successfully completed

	// Send the open command, wait a bit for the response, and retrieve response packet
	send(CMD_OPEN, errChk);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	// If further error checking was requested, check the data packet for a non-zero serial ID
	if (errChk && mRespStatus) {
		uint8_t i;	// Loop counter

		success = recvDataPkt(24);

		if (success) {
			// Iterate through the serial ID as long as all of its bytes are 0
			for (i = 8; i < 24 && (mDataPkt[i] == 0x00); ++i);
			success &= (i != 24);
		}
	} else {
		success = mRespStatus;
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("Open operation failed: "));
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.println(F("Open operation succeeded"));
		}
	#endif

	return success;
}

/**
 * Sends the close command. Does not do anything to the fingerprint module but
 * does receive an ACK.
 *
 * @return True if succeeds, false otherwise
 */
bool FingerprintModule::close() {
	// Send the close command, wait a bit for the response, and retrieve response packet
	send(CMD_CLOSE);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("Close operation failed: "));
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.println(F("Close operation succeeded"));
		}
	#endif

	return mRespStatus;
}

/**
 * Turns the CMOS LED on or off. Parameter is true for on, false
 * for off. Returns true on success.
 *
 * @param on True if CMOS LED should be turned, false for off
 *
 * @return True if the operation succeeded, false otherwise
 */
bool FingerprintModule::powerCMOS(bool on) {
	// Send the close command, wait a bit for the response, and retrieve response packet
	send(CMD_CMOS_LED, on);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("CMOS operation failed: "));
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.println(F("CMOS operation succeeded"));
		}
	#endif

	return mRespStatus;
}

/**
 * Changes the serial speed at which communications are done. Module
 * is initialized to 9600 bps on initial power-on.
 * NOTE: Could not successfully test this function, broken for now.
 *
 * @param baud The baudrate to switch to
 *
 * @return True if the operation succeeded, false otherwise
 */
bool FingerprintModule::changeBaudrate(uint32_t baud) {
	// Send the close command, wait a bit for the response, and retrieve response packet
	send(CMD_CHANGE_BAUDRATE, baud);
	COMMS.flush();
	COMMS.end();
	COMMS.begin(baud);
	while(!COMMS);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("Baudrate change operation failed: "));
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.println(F("Baudrate change operation succeeded"));
		}
	#endif

	return mRespStatus;
}

/**
 * Gets the number of enrolled fingerprints stored in the module.
 *
 * @return True if the operation succeeds, false otherwise
 */
bool FingerprintModule::getEnrollCount() {
	// Send the get command, wait a bit for the response, and retrieve response packet
	send(CMD_GET_ENROLL_COUNT);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("Get enrollment count operation failed: "));
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.println(F("Get enrollment count operation succeeded"));
		}
	#endif

	return mRespStatus;
}

/**
 * Takes in an enrollment ID and checks to see if that ID has been enrolled
 * with the fingerprint module. This function will return false both if no
 * response was received, and if the ID was not enrolled. If the return is
 * false, you MUST use getErrorCode() to determine whether a communications
 * error happened or the ID simply wasn't enrolled or was invalid.
 *
 * @param uint32_t The ID to check
 *
 * @return True if the ID is enrolled, false on comms error or ID not enrolled
 */
bool FingerprintModule::isIDEnrolled(uint32_t id) {
	// Send the command, wait a bit for the response, then retrieve the response
	send(CMD_CHECK_ENROLLED, id);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("Checked to see if ID "));
			Serial.print(id);
			Serial.print(F(" was enrolled: "));
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.print(F("The ID "));
			Serial.print(id);
			Serial.println(F(" is registered with the fingerprint module"));
		}
	#endif

	return mRespStatus;
}

/**
 * Takes in an ID and begins an enrollment for that ID. In order
 * for the enrollment to successfully start, there must be less than
 * 20 enrolled templates, the requested enrollment ID must be between
 * 0 and 19 inclusive, and the enrollment ID must be available.
 * Check the error code if this call fails. Resets the mEnrollmentStage
 * member variable to 1 if successful.
 *
 * @param id The enrollment ID to begin enrollment for
 *
 * @return True if the enrollment has started, false otherwise
 */
bool FingerprintModule::startEnrollment(uint32_t id) {
	// Send the command and retrieve response after allowing for a delay
	send(CMD_ENROLL_START, id);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	// Reset the enrollment stage
	if (mRespStatus) {
		mEnrollmentStage = 0;
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("Attempted to begin enrollment for ID "));
			Serial.print(id);
			Serial.print(" but could not continue: ");
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.print(F("Successfully began enrollment for ID "));
			Serial.println(id);
		}
	#endif

	return mRespStatus;
}

/**
 * Creates the template for the appropriate stage of enrollment. Keeps
 * track of whether this is the first, second, or third enrollment using
 * member variable mEnrollmentStage. This function will increment the stage
 * on success.
 *
 * @return True on success, false if the enrollment failed
 */
bool FingerprintModule::createEnrollmentTemplate() {
	switch (mEnrollmentStage) {
		case 0:
			send(CMD_ENROLL1);
			break;

		case 1:
			send(CMD_ENROLL2);
			break;

		case 2:
			send(CMD_ENROLL3);
			break;

		default:
			return false;
	}
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	if (mRespStatus) {
		++mEnrollmentStage;
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("Attempted to enroll image #"));
			Serial.print(mEnrollmentStage);
			Serial.print(F(" but was unsuccessful: "));
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.print(F("Successfully registered image #"));
			Serial.print(mEnrollmentStage);
			Serial.println(F(" of the enrollment"));
		}
	#endif

	return mRespStatus;
}

/**
 * Checks to see if a finger is pressed on the sensor.
 *
 * @return True if there is a finger pressed on the sensor, false if there isn't
 *		   or there's a communications error (check the error code)
 */
bool FingerprintModule::isFingerPressed() {
	send(CMD_IS_PRESS_FINGER);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	if (mRespStatus && mRespParam != 0) {
		mRespParam = NACK_FINGER_IS_NOT_PRESSED;
		mRespStatus = false;
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("Checked to see if a finger is pressed on the sensor: "));
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.println(F("A finger is pressed on the sensor"));
		}
	#endif

	return mRespStatus;
}

/**
 * Tells the sensor to capture a fingeprint image, convert it,
 * and store it for use in an enrollment. Parameter used to specify
 * whether the sensor should use a low-quality but fast image or
 * higher-quality but slow image.
 *
 * @param True for a higher-quality image, false otherwise; defaults to false
 *
 * @return True if image successfully captured, false otherwise (check error code)
 */
bool FingerprintModule::captureFingerprint(bool highQual) {
	// Send the command, wait for a bit, and then receive the response
	send(CMD_CAPTURE_FINGER, highQual);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("Attempted to capture a "));
			Serial.print(highQual ? F("high") : F("low"));
			Serial.print(F(" quality fingerprint image for enrollment but failed: "));
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.println(F("Successfully captured a fingerprint image for enrollment"));
		}
	#endif

	return mRespStatus;
}

/**
 * Deletes the template with the given ID from the module. Returns false
 * if there was a comm error or if that ID does not exist in the system,
 * use getErrorCode() to determine the issue.
 *
 * @param id The ID of the template to remove
 *
 * @return True on success, false otherwise
 */
bool FingerprintModule::deleteID(uint32_t id) {
	send(CMD_DELETE_ID, id);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("Attempted to delete template with ID #"));
			Serial.print(id);
			Serial.print(F(": "));
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.print(F("Successfully deleted template with ID #"));
			Serial.println(id);
		}
	#endif

	return mRespStatus;
}

/**
 * Deletes all templates from the fingerprint module. Returns false
 * if there was a comm error or if the module did not contain
 * any fingerprint templates. Use getErrorCode() for more details.
 *
 * @return True on success, false otherwise
 */
bool FingerprintModule::deleteAll() {
	send(CMD_DELETE_ALL);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("Attempted to clear the module's fingerprint database: "));
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.println(F("Successfully deleted all templates from the fingerprint database."));
		}
	#endif

	return mRespStatus;
}

/**
 * Performs a 1:1 verification that the captured fingerprint matches the template
 * with the given ID. This function must be called directly after a successful
 * captureFinger() call.
 *
 * @param id The ID of the template to compare the captured fingerprint to
 *
 * @return True if the captured fingerprint matches the template, false otherwise
 */
bool FingerprintModule::verify(uint32_t id) {
	send(CMD_VERIFY, id);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print(F("Failed to verify that the captured fingerprint matches ID #"));
			Serial.print(id);
			Serial.print(F(": "));
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.print(F("The captured fingerprint matches ID #"));
			Serial.println(id);
		}
	#endif

	return mRespStatus;
}

/**
 * Performs a 1:N identification of the captured fingerprint. If successful, this function
 * will take a captured fingerprint and will store the ID of the template it matches
 * (number between 0 and 19). This function must be called directly after a successful
 * captureFinger() call.
 *
 * @return True if the captured fingerprint matches a template, false otherwise
 */
bool FingerprintModule::identify() {
	send(CMD_IDENTIFY);
	for (int i = 0; i < TIMEOUT && !recvResponsePkt(); ++i) {
		delay(WAITTIME);
	}

	#ifdef DEBUG
		if (!mRespStatus) {
			Serial.print("Attempted to identify the captured fingerprint: ");
			Serial.println(strFromError(mRespParam));
		} else {
			Serial.print("Successfully identified the captured fingerprint to be ID #");
			Serial.println(mRespParam);
		}
	#endif

	return mRespStatus;
}

/**
 * Takes in a module-generated template and an ID of a template on the module and
 * checks to see if they match. This is a 1:1 template verification.
 * The template should be 506 bytes in size.
 *
 * @param id The ID of the template on the module to check with the given template
 * @param templ An array of 506 bytes representing the template to verify
 *
 * @return True if the given template matches the given ID, false otherwise
 */
bool FingerprintModule::verifyTemplate(uint32_t id, byte templ[]) {
	bool success;	// Indicates whether the given template matches the template with the given ID

	return success;
}

// END PUBLIC

// BEGIN PRIVATE

/**
 * Sends the specified command and parameters to the fingerprint
 * module. An optional third argument can be used to specify whether the
 * parameter and command arguments are little endian or big endian. The
 * function assumes big endian.
 *
 * @param cmd 			A byte representing the command code
 * @param param 		Four bytes containing the parameters to the command
 * @param isBigEndian 	True if cmd and param are big-endian, false if little-endian, defaults to true
 *
 * @return True if all 12 bytes were successfully sent, false otherwise
 */
bool FingerprintModule::send(word cmd, dword param, bool isBigEndian) {
	byte pkt[12];		// Byte array containing the 12 bytes of the command packet
	byte paramArr[4];	// Array containing each byte of the parameter
	byte cmdArr[2];		// Array containing each byte of the command
	byte chkSumArr[2];	// Array containing each byte of the checksum

	// Build out each byte of the command packet, starting with the header
	pkt[0] = CMD_START_CODE_1;
	pkt[1] = CMD_START_CODE_2;
	pkt[2] = DEVICE_ID_LSB;
	pkt[3] = DEVICE_ID_MSB;

	// If the given parameter and command are big endian, flip them
	if (isBigEndian) {
		param = flipEndianness(param);
		cmd = flipEndianness(cmd);
	}

	// Split the parameter and command into arrays and add them to the packet
	split(param, paramArr);
	split(cmd, cmdArr);
	pkt[4] = paramArr[0];
	pkt[5] = paramArr[1];
	pkt[6] = paramArr[2];
	pkt[7] = paramArr[3];
	pkt[8] = cmdArr[0];
	pkt[9] = cmdArr[1];

	// Compute, flip, and split the checksum and add it to the packet
	word chkSum = computeCheckSum(pkt, 10);
	chkSum = flipEndianness(chkSum);
	split(chkSum, chkSumArr);
	pkt[10] = chkSumArr[0];
	pkt[11] = chkSumArr[1];

	// Debug prints the completed packet being sent
	#ifdef DEBUG
		Serial.print(F("Sending command packet: "));
		for (uint8_t i = 0; i < 12; ++i) {
			Serial.print(pkt[i], HEX);
			Serial.print(F(" "));
		}
		Serial.println();
	#endif

	// Send the completed packet to the fingerprint reader via the serial interface
	uint32_t bytesSent = COMMS.write(pkt, 12);

	// Return true if all 12 bytes were sent
	return (bytesSent == 12);
}

/**
 * Attempts to receive a response packet from the fingerprint module
 * and places it in the response packet buffer. If there is previous
 * unreceived data in the serial buffer, this data is thrown out until
 * a response packet is found and retrieved. If a complete 12-byte
 * response packet is received, returns true; otherwise, returns false
 *
 * @return True if receive was successful
 */
bool FingerprintModule::recvResponsePkt() {
	byte buff[12];					// Temporary storage buffer for the packet as it's being received
	byte done = false;				// Indicates the loop to stop iterating through the serial receive buffer
	word givenChkSum = 0x0000;		// Stores the received packet's given checksum

	// Retrieve and store a response packet if possible
	while (COMMS.available() && !done) {
		byte incomingByte;

		incomingByte = COMMS.read();

		if (incomingByte == 0x55 && COMMS.read() == 0xAA) {
			uint8_t i;			// Loop counter

			// Set the first 2 bytes of the response packet
			buff[0] = 0x55;
			buff[1] = 0xAA;

			// Start the loop at the 3rd byte and read the response
			for (i = 2; i < 12 && COMMS.available(); ++i) {
				buff[i] = COMMS.read();
			}

			// If we successfully read the remaining 10 response bytes, indicate receive done successfully and grab checksum
			if (i == 12) {
				done = true;
				givenChkSum = (buff[11] << 8) | buff[10];
			}
		}
	}

	// If the buffer ran out before receiving a response packet, update error params indicating no reception
	if (!done) {
		mRespStatus = false;
		mRespParam = NACK_NOT_RECVD;
	}
	// If the computed checksum does match the given one, update error params with error code
	else if (computeCheckSum(buff, 10) != givenChkSum) {
		mRespStatus = false;
		mRespParam = NACK_COMM_ERR;
	}
	// If the response was a NACK, update error params with error code
	else if (buff[8] == 0x31) {
		mRespStatus = false;
		mRespParam = (buff[7] << 24) | (buff[6] << 16) | (buff[5] << 8) | buff[4];
	}
	// If response succeeded, update response param and copy into module's response buffer
	else {
		mRespStatus = true;
		mRespParam = (buff[7] << 24) | (buff[6] << 16) | (buff[5] << 8) | buff[4];

		for (uint8_t i = 0; i < 12; ++i) {
			mRespPkt[i] = buff[i];
		}
	}

	// Debug prints the received response packet to USB serial
	#ifdef DEBUG
		if (!done) {
			Serial.println(F("Did not receive a complete response packet"));
		} else {
			Serial.print(F("Received response packet: "));
			for (uint32_t i = 0; i < 12; ++i) {
				Serial.print(buff[i], HEX);
				Serial.print(F(" "));
			}
			Serial.println();
		}
	#endif

	return done;
}

/**
 * Attempts to receive a data packet from the fingerprint module
 * and places it in the data packet buffer. If there is previous
 * unreceived data in the serial buffer, this data is thrown out until
 * a data packet is found and retrieved. If a complete data packet is
 * received, returns true; otherwise, returns false
 *
 * @param The size of the data being received, without counting packet metadata
 *
 * @return True if receive was successful
 */
bool FingerprintModule::recvDataPkt(uint32_t size) {
	word givenChkSum = 0x0000;			// The received packet's given check sum
	uint32_t totalPktSize = size + 6;	// The total size of the data packet with metadata
	byte done = false;					// Indicates the loop to stop iterating through the serial receive buffer

	// Retrieve and store a data packet if possible
	while (COMMS.available() && !done) {
		byte incomingByte;

		incomingByte = COMMS.read();

		if (incomingByte == 0x5A && COMMS.read() == 0xA5) {
			uint8_t i;			// Loop counter

			// Set the first 2 bytes of the response packet
			mDataPkt[0] = 0x5A;
			mDataPkt[1] = 0xA5;

			// Start the loop at the 3rd byte and read the response
			for (i = 2; i < totalPktSize && COMMS.available(); ++i) {
				mDataPkt[i] = COMMS.read();
			}

			// If we successfully read the remaining bytes, indicate receive done successfully
			if (i == totalPktSize) {
				done = true;
				givenChkSum = (mDataPkt[totalPktSize - 1] << 8) | mDataPkt[totalPktSize - 2];
			}
		}
	}

	// Check the checksum and indicate failure if incorrect
	if (done && computeCheckSum(mDataPkt, totalPktSize - 2) != givenChkSum) {
		done = false;
	}

	// Debug prints the received response packet to USB serial
	#ifdef DEBUG
		if (!done) {
			Serial.println(F("Did not receive a complete data packet"));
		} else {
			Serial.print(F("Received data packet: "));
			for (uint32_t i = 0; i < totalPktSize; ++i) {
				Serial.print(mDataPkt[i], HEX);
				Serial.print(F(" "));
			}
			Serial.println();
		}
	#endif

	return done;
}

/**
 * Takes in a byte array and computes its check-sum up to the given size.
 *
 * @param arr The byte array to compute the checksum of
 * @param size The size of the array
 *
 * @return A word containing the checksum in big-endian format
 */
word FingerprintModule::computeCheckSum(byte* arr, uint32_t size) {
	word chkSum = 0x0000;

	for (uint32_t i = 0; i < size; ++i) {
		chkSum += arr[i];
	}

	return chkSum;
}

/**
 * Takes a word (2 bytes) and flips it so the LSB is the MSB and vice-versa.
 *
 * @param flipThis The word to flip
 *
 * @return A word which contains the argument with its endianness switched
 */
word FingerprintModule::flipEndianness(word flipThis) {
	word flipped = 0x0000;

	flipped = ((flipThis & 0x00FF) << 8) | ((flipThis & 0xFF00) >> 8);

	return flipped;
}

/**
 * Takes a double word (4 bytes) and flips it so the LSB is the MSB and vice-versa.
 *
 * @param flipThis The double world to flip
 *
 * @return A double word which contains the argument with its endianness switched
 */
dword FingerprintModule::flipEndianness(dword flipThis) {
	dword flipped = 0x00000000;

	flipped = ((flipThis & 0x000000FF) << 24) | ((flipThis & 0xFF000000) >> 24) |
			  ((flipThis & 0x0000FF00) << 8)  | ((flipThis & 0x00FF0000) >> 8);

	return flipped;
}

/**
 * Accepts a word (2 bytes) and splits up each byte into elements of
 * a byte array. The MSB is placed at index 0.
 *
 * @param splitThis The word to split into an array
 * @param dest The byte array to place the bytes into
 *
 * @return The byte array containing the bytes of the word
 */
void FingerprintModule::split(word splitThis, byte* dest) {
	dest[0] = (splitThis >> 8) & 0xFF;
	dest[1] = splitThis & 0xFF;
}

/**
 * Accepts a double word (4 bytes) and splits up each byte into
 * elements of a byte array. The MSB is placed at index 0 (i.e. big-endian)
 *
 * @param splitThis The double word to split into an array
 * @param dest The byte array to place the bytes into
 *
 * @return The byte array containing the bytes of the double word
 */
void FingerprintModule::split(dword splitThis, byte* dest) {
	dest[0] = (splitThis >> 24) & 0xFF;
	dest[1] = (splitThis >> 16) & 0xFF;
	dest[2] = (splitThis >>  8) & 0xFF;
	dest[3] = splitThis & 0xFF;
}
