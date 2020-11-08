/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================*/

#include "Arduino.h"
#include "DMP.h"

DMP::DMP(MPU6050 * mpu){
    this->mpu = mpu;
}

uint8_t DMP::initialize(){
	// reset device
	DEBUG_PRINTLN(F("\n\nResetting MPU6050..."));
	mpu->reset();
	delay(30); // wait after reset

	// enable sleep mode and wake cycle
	/*Serial.println(F("Enabling sleep mode..."));
	setSleepEnabled(true);
	Serial.println(F("Enabling wake cycle..."));
	setWakeCycleEnabled(true);*/

	// disable sleep mode
	mpu->setSleepEnabled(false);

	// get MPU hardware revision
	mpu->setMemoryBank(0x10, true, true);
	mpu->setMemoryStartAddress(0x06);
	#ifdef DEBUG
		Serial.println(F("Checking hardware revision..."));
		Serial.print(F("Revision @ user[16][6] = "));
		Serial.println(readMemoryByte(), HEX);
		Serial.println(F("Resetting memory bank selection to 0..."));
	#endif
	mpu->setMemoryBank(0, false, false);

	// check OTP bank valid
	DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
	DEBUG_PRINT(F("OTP bank is "));
	DEBUG_PRINTLN(getOTPBankValid() ? F("valid!") : F("invalid!"));

	// setup weird slave stuff (?)
	DEBUG_PRINTLN(F("Setting slave 0 address to 0x7F..."));
	mpu->setSlaveAddress(0, 0x7F);
	DEBUG_PRINTLN(F("Disabling I2C Master mode..."));
	mpu->setI2CMasterModeEnabled(false);
	DEBUG_PRINTLN(F("Setting slave 0 address to 0x68 (self)..."));
	mpu->setSlaveAddress(0, 0x68);
	DEBUG_PRINTLN(F("Resetting I2C Master control..."));
	mpu->resetI2CMaster();
	delay(20);
	DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
	mpu->setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

	DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
	mpu->setIntEnabled(1<<MPU6050_INTERRUPT_FIFO_OFLOW_BIT|1<<MPU6050_INTERRUPT_DMP_INT_BIT);

	DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
	mpu->setRate(4); // 1khz / (1 + 4) = 200 Hz

	DEBUG_PRINTLN(F("Setting external frame sync to TEMP_OUT_L[0]..."));
	mpu->setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

	DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
	mpu->setDLPFMode(MPU6050_DLPF_BW_42);

	DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
	mpu->setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

	// load DMP code into memory banks
	DEBUG_PRINT(F("Writing DMP code to MPU memory banks ("));
	DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
	DEBUG_PRINTLN(F(" bytes)"));
	if (!mpu->writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) return 1; // Failed
	DEBUG_PRINTLN(F("Success! DMP code written and verified."));

	// Set the FIFO Rate Divisor int the DMP Firmware Memory
	unsigned char dmpUpdate[] = {0x00, MPU6050_DMP_FIFO_RATE_DIVISOR};
	mpu->writeMemoryBlock(dmpUpdate, 0x02, 0x02, 0x16); // Lets write the dmpUpdate data to the Firmware image, we have 2 bytes to write in bank 0x02 with the Offset 0x16

	//write start address MSB into register
	mpu->setDMPConfig1(0x03);
	//write start address LSB into register
	mpu->setDMPConfig2(0x00);

	DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
	mpu->setOTPBankValid(false);

	DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
	mpu->setMotionDetectionThreshold(2);

	DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
	mpu->setZeroMotionDetectionThreshold(156);

	DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
	mpu->setMotionDetectionDuration(80);

	DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
	mpu->setZeroMotionDetectionDuration(0);
	DEBUG_PRINTLN(F("Enabling FIFO..."));
	mpu->setFIFOEnabled(true);

	DEBUG_PRINTLN(F("Resetting DMP..."));
	mpu->resetDMP();

	DEBUG_PRINTLN(F("DMP is good to go! Finally."));

	DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
	mpu->setDMPEnabled(false);

	DEBUG_PRINTLN(F("Setting up internal 42-byte (default) DMP packet buffer..."));
	dmpPacketSize = 42;

	DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
	mpu->resetFIFO();
	mpu->getIntStatus();

	return 0; // success
}

bool DMP::packetAvailable() {
    return mpu->getFIFOCount() >= getFIFOPacketSize();
}

uint8_t DMP::getAccel(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[28] << 24) | ((uint32_t)packet[29] << 16) | ((uint32_t)packet[30] << 8) | packet[31]);
    data[1] = (((uint32_t)packet[32] << 24) | ((uint32_t)packet[33] << 16) | ((uint32_t)packet[34] << 8) | packet[35]);
    data[2] = (((uint32_t)packet[36] << 24) | ((uint32_t)packet[37] << 16) | ((uint32_t)packet[38] << 8) | packet[39]);
    return 0;
}

uint8_t DMP::getAccel(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[28] << 8) | packet[29];
    data[1] = (packet[32] << 8) | packet[33];
    data[2] = (packet[36] << 8) | packet[37];
    return 0;
}

uint8_t DMP::getAccel(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    v -> x = (packet[28] << 8) | packet[29];
    v -> y = (packet[32] << 8) | packet[33];
    v -> z = (packet[36] << 8) | packet[37];
    return 0;
}

uint8_t DMP::getQuaternion(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[0] << 24) | ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3]);
    data[1] = (((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) | ((uint32_t)packet[6] << 8) | packet[7]);
    data[2] = (((uint32_t)packet[8] << 24) | ((uint32_t)packet[9] << 16) | ((uint32_t)packet[10] << 8) | packet[11]);
    data[3] = (((uint32_t)packet[12] << 24) | ((uint32_t)packet[13] << 16) | ((uint32_t)packet[14] << 8) | packet[15]);
    return 0;
}

uint8_t DMP::getQuaternion(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 8) | packet[1]);
    data[1] = ((packet[4] << 8) | packet[5]);
    data[2] = ((packet[8] << 8) | packet[9]);
    data[3] = ((packet[12] << 8) | packet[13]);
    return 0;
}

uint8_t DMP::getQuaternion(Quaternion *q, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    uint8_t status = getQuaternion(qI, packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}

uint8_t DMP::getGyro(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[16] << 24) | ((uint32_t)packet[17] << 16) | ((uint32_t)packet[18] << 8) | packet[19]);
    data[1] = (((uint32_t)packet[20] << 24) | ((uint32_t)packet[21] << 16) | ((uint32_t)packet[22] << 8) | packet[23]);
    data[2] = (((uint32_t)packet[24] << 24) | ((uint32_t)packet[25] << 16) | ((uint32_t)packet[26] << 8) | packet[27]);
    return 0;
}

uint8_t DMP::getGyro(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[16] << 8) | packet[17];
    data[1] = (packet[20] << 8) | packet[21];
    data[2] = (packet[24] << 8) | packet[25];
    return 0;
}

uint8_t DMP::getGyro(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    v -> x = (packet[16] << 8) | packet[17];
    v -> y = (packet[20] << 8) | packet[21];
    v -> z = (packet[24] << 8) | packet[25];
    return 0;
}

uint8_t DMP::getLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
    v -> x = vRaw -> x - gravity -> x*8192;
    v -> y = vRaw -> y - gravity -> y*8192;
    v -> z = vRaw -> z - gravity -> z*8192;
    return 0;
}

uint8_t DMP::getLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
    return 0;
}

uint8_t DMP::getGravity(int16_t *data, const uint8_t* packet) {
    /* +1g corresponds to +8192, sensitivity is 2g. */
    int16_t qI[4];
    uint8_t status = getQuaternion(qI, packet);
    data[0] = ((int32_t)qI[1] * qI[3] - (int32_t)qI[0] * qI[2]) / 16384;
    data[1] = ((int32_t)qI[0] * qI[1] + (int32_t)qI[2] * qI[3]) / 16384;
    data[2] = ((int32_t)qI[0] * qI[0] - (int32_t)qI[1] * qI[1]
	       - (int32_t)qI[2] * qI[2] + (int32_t)qI[3] * qI[3]) / (2 * 16384L);
    return status;
}

uint8_t DMP::getGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

uint8_t DMP::getEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}


uint8_t DMP::getYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan2(gravity -> y , gravity -> z);
    if (gravity -> z < 0) {
        if(data[1] > 0) {
            data[1] = PI - data[1]; 
        } else { 
            data[1] = -PI - data[1];
        }
    }
    return 0;
}

uint8_t DMP::processFIFOPacket(const unsigned char *dmpData) {
    /*for (uint8_t k = 0; k < dmpPacketSize; k++) {
        if (dmpData[k] < 0x10) Serial.print("0");
        Serial.print(dmpData[k], HEX);
        Serial.print(" ");
    }
    Serial.print("\n");*/
    //Serial.println((uint16_t)dmpPacketBuffer);
    return 0;
}
uint8_t DMP::readAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed) {
    uint8_t status;
    uint8_t buf[dmpPacketSize];
    for (uint8_t i = 0; i < numPackets; i++) {
        // read packet from FIFO
        mpu->getFIFOBytes(buf, dmpPacketSize);

        // process packet
        if ((status = processFIFOPacket(buf)) > 0) return status;
        
        // increment external process count variable, if supplied
        if (processed != 0) (*processed)++;
    }
    return 0;
}

uint16_t DMP::getFIFOPacketSize() {
    return dmpPacketSize;
}