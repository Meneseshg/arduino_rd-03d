/*
   RD-03D Library
   Copyright (c) 2024 javier-fg

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
*/

#include <Arduino.h>
#include <math.h>
#include "RD03D.h"

//
// --- TargetData Class Implementation ---
//
TargetData::TargetData() : idNum(0), x(0), y(0), speed(0), distanceRes(0), distance(0), angle(0), _isValid(false) {}

void TargetData::clearValues(){
  x = 0;
  y = 0;
  speed = 0;
  distanceRes = 0;
  distance = 0;
  angle = 0;
  timestamp = 0;
  _isValid = false;
}

bool TargetData::setValues(int16_t _x, int16_t _y, int16_t _speed, uint16_t _distanceRes) {

  // If distanceResolution is zero, the measurement is not valid
  if(_distanceRes == 0){
    clearValues();
    return false;
  }else
    _isValid = true;

  // Save values
  x = _x;
  y = _y;
  speed = _speed;
  distanceRes = _distanceRes;
  timestamp = millis();

  // Compute distance and angle based on x and y values.
  distance = sqrt(sq(x) + sq(y));
  angle = atan2((float)y, (float)x) * 180.0 / PI;

// #ifdef RD03D_LOGGER_DEBUG
//   printInfo();
// #endif

  // If distance is more than theoretical, measurement not valid
  if( distance > MAX_DISTANCE){
    clearValues();
    return false;
  }

  return _isValid;
}

void TargetData::printInfo(){

  Serial.print("Target-");
  Serial.print(idNum);
  Serial.print(": ");

  if (!isValid() ){
    Serial.println("-- no valid data --");
    return;
  }

  // Serial.print("Distance: ");
  // Serial.print(distance / 10.0);
  // Serial.print(" cm, Angle: ");
  // Serial.print(angle);
  // Serial.print("°, X: ");
  // Serial.print(x);
  // Serial.print(" mm, Y: ");
  // Serial.print(y);
  // Serial.print(" mm, Speed: ");
  // Serial.print(speed);
  // Serial.print(" cm/s, Resolution: ");
  // Serial.println(distanceRes);

  Serial.print(distance / 10.0);
  Serial.print(" cm, ");
  Serial.print(angle);
  Serial.print("°, (");
  Serial.print(x / 10.0);
  Serial.print(",");
  Serial.print(y / 10.0);
  Serial.print("), ");
  Serial.print(speed);
  Serial.println(" cm/s");

}


//
// --- RD03D Class Implementation ---
//

// Data frame markers observed from the module (the datasheet does not document the protocol).
const uint8_t RD03D::FRAME_HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};

RD03D::RD03D(uint8_t rxPin, uint8_t txPin, uint32_t baudRate, HardwareSerial* serial, size_t bufferSize)
  : _rxPin(rxPin), _txPin(txPin), _baudRate(baudRate), _bufferSize(bufferSize), _mode(SINGLE_TARGET),
    _serial(serial), _bufferRxIndex(0), _bufferTxIndex(0), _targetCount(0),
    _rxState(RX_HEADER), _headerMatch(0)
{
  // Allocate the buffer for incoming data.
  _bufferRx = new uint8_t[_bufferSize];
  _bufferTx = new uint8_t[30];

  // Assign target ID incrementally
  for(uint8_t i = 0 ; i < MAX_TARGETS; i++){
    _targets[i].idNum = i + 1;
  }
}

bool RD03D::initialize( RD03DMode mode ) {

  // A larger RX buffer keeps the UART from overflowing at 256000 baud between
  // calls to tasks(). It MUST be configured before begin().
  _serial->setRxBufferSize(512);
  _serial->begin(_baudRate, SERIAL_8N1, _rxPin, _txPin);

  delay(1000); // The module needs time to boot before it accepts commands.

  // Set mode
  _mode = mode;

  // Send the detection-mode command to the radar module.
  cmd_buffer_rx_clean();
  if (_mode == SINGLE_TARGET)
    cmd_send_buffer(CMD_TARGET_DETECTION_SINGLE, sizeof(CMD_TARGET_DETECTION_SINGLE));
  else
    cmd_send_buffer(CMD_TARGET_DETECTION_MULTI, sizeof(CMD_TARGET_DETECTION_MULTI));

  // The module does not reliably acknowledge this command, so a missing ACK is
  // NOT treated as a failure (the reference implementation doesn't check one
  // either). We just give it time to apply and flush whatever it sends back.
  cmd_receive_ack();
  delay(200);
  cmd_buffer_rx_clean();

  // Start the parser hunting for a frame header.
  _rxState       = RX_HEADER;
  _headerMatch   = 0;
  _bufferRxIndex = 0;

  return true;
}

bool RD03D::tasks() {

  bool res = false;

  // Continuously read bytes from the serial port.
  while (_serial->available()) {

    uint8_t incomingByte = _serial->read();

    switch (_rxState) {

      // Hunt for the 4-byte header (0xAA 0xFF 0x03 0x00) so a frame can never
      // start mid-stream. This is the key fix: the old parser only looked for
      // the footer, so a stray 0x55 0xCC in the payload (or a dropped byte)
      // permanently misaligned every subsequent frame.
      case RX_HEADER:
        if (incomingByte == FRAME_HEADER[_headerMatch]) {
          _bufferRx[_headerMatch++] = incomingByte;
          if (_headerMatch == sizeof(FRAME_HEADER)) {
            _bufferRxIndex = _headerMatch;   // payload starts right after the header
            _rxState = RX_PAYLOAD;
          }
        } else {
          // Restart the match, allowing this byte to be a fresh header start.
          _headerMatch = (incomingByte == FRAME_HEADER[0]) ? 1 : 0;
          if (_headerMatch) _bufferRx[0] = incomingByte;
        }
        break;

      // Collect payload until the footer (0x55 0xCC). After every frame we go
      // back to hunting the header, so a corrupt/truncated frame self-heals on
      // the next real header instead of poisoning the stream forever.
      case RX_PAYLOAD:
        _bufferRx[_bufferRxIndex++] = incomingByte;

        if (_bufferRxIndex >= 6 &&
            _bufferRx[_bufferRxIndex - 2] == 0x55 &&
            _bufferRx[_bufferRxIndex - 1] == 0xCC) {
          if (processFrame()) res = true;
#ifdef RD03D_LOGGER_DEBUG
          Serial.print("# RX FRAME: ");
          printHex(_bufferRx, _bufferRxIndex);
#endif
          _headerMatch   = 0;
          _bufferRxIndex = 0;
          _rxState       = RX_HEADER;
        }
        // Guard against a runaway frame (lost footer / corruption): resync.
        else if (_bufferRxIndex >= _bufferSize) {
#ifdef RD03D_LOGGER_DEBUG
          Serial.println("# RX FRAME - OVERFLOW, resync");
#endif
          _headerMatch   = 0;
          _bufferRxIndex = 0;
          _rxState       = RX_HEADER;
        }
        break;
    }
  }

  return res;
}

bool RD03D::processFrame() {

  int16_t tx;
  int16_t ty;
  int16_t tspeed;
  uint16_t tdistRes;

  // Frame layout: [4-byte header][N * 8 target bytes][2-byte footer].
  // Validate the geometry before trusting it: the payload must be a whole
  // number of 8-byte targets. This rejects frames truncated by a stray footer
  // so we never parse misaligned garbage.
  if (_bufferRxIndex < 4 + 8 + 2) return false;        // need at least one target
  size_t payloadLen = _bufferRxIndex - 4 - 2;          // strip header and footer
  if (payloadLen % 8 != 0) return false;               // corrupt / truncated frame
  size_t nTargets = payloadLen / 8;
  if (nTargets > MAX_TARGETS) nTargets = MAX_TARGETS;

  // Clear EVERY slot first. The old code only touched slots the loop reached,
  // so a short frame left stale data in higher slots that still reported
  // isValid() == true — phantom targets. Clearing up front prevents that.
  for (uint8_t t = 0; t < MAX_TARGETS; t++)
    _targets[t].clearValues();

  _targetCount = 0;

  // Each target occupies 8 bytes starting at index 4.
  for (uint8_t t = 0; t < nTargets; t++) {

    size_t i = 4 + (size_t)t * 8;

    // X in mm (sign-magnitude: bit15 set = positive, magnitude in bits 14:0)
    if(_bufferRx[i + 1] & 0x80)
      tx = (int16_t)(_bufferRx[i] | (_bufferRx[i + 1] << 8)) - 32768;
    else
      tx = 0 - (int16_t)(_bufferRx[i] | (_bufferRx[i + 1] << 8));

    // Y in mm
    if(_bufferRx[i + 3] & 0x80)
      ty = (int16_t)(_bufferRx[i+2] | (_bufferRx[i + 3] << 8)) - 32768;
    else
      ty = 0 - (int16_t)(_bufferRx[i+2] | (_bufferRx[i + 3] << 8));

    // SPEED in cm/s
    if(_bufferRx[i + 5] & 0x80)
      tspeed = (int16_t)(_bufferRx[i+4] | (_bufferRx[i + 5] << 8)) - 32768;
    else
      tspeed = 0 - (int16_t)(_bufferRx[i+4] | (_bufferRx[i + 5] << 8));

    // DISTANCE resolution in mm (little-endian)
    tdistRes = (uint16_t)(_bufferRx[i + 6] | (_bufferRx[i + 7] << 8));

    if(_targets[t].setValues(tx, ty, tspeed, tdistRes))
      _targetCount++;
  }

  return _targetCount > 0;
}

TargetData* RD03D::getTarget(uint8_t target_num) {

  if(target_num >= MAX_TARGETS)
    target_num = MAX_TARGETS -1;

  return &_targets[target_num];
}

// Return a pointer to the internal targets array (for multi-target iteration).
TargetData* RD03D::getTargets() {
  return _targets;
}


// Function to print buffer contents
void RD03D::printHex(const uint8_t *buffer, size_t size) {

  for (int i = 0; i < size; i++) {
      Serial.print("0x");
      if (buffer[i] < 0x10) Serial.print("0");  // Add leading zero for single-digit hex values
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
  }

  Serial.println();
}

void RD03D::cmd_send_buffer(const uint8_t *buffer, size_t size){

  _serial->write(buffer, size);

#ifdef RD03D_LOGGER_DEBUG
  Serial.print("# SEND CMD: ");
  printHex(buffer, size);
#endif

}

// Wait for the reception of a frame (ends with 0x04030201). Return the number of bytes read.     
uint8_t RD03D::cmd_receive_ack(){

  unsigned long int timeout = millis() + TIMEOUT_RX;

  _bufferRxIndex = 0;

  // Continuously read bytes from the serial port.
  while (millis() < timeout) {

    if (_serial->available()){

      uint8_t incomingByte = _serial->read();
      _bufferRx[_bufferRxIndex++] = incomingByte;
  
      // Prevent buffer overflow.
      if (_bufferRxIndex >= _bufferSize) {
  #ifdef RD03D_LOGGER_DEBUG
        Serial.println("# RX ACK - BUFFER OVERFLOW");
  #endif
        _bufferRxIndex = 0;
        return 0;
      }
  
      // Check if the bytes match the frame terminator (0x04030201).
      if (_bufferRxIndex > 5 && _bufferRx[_bufferRxIndex - 4] == 0x04 && _bufferRx[_bufferRxIndex - 3] == 0x03 && _bufferRx[_bufferRxIndex - 2] == 0x02 && _bufferRx[_bufferRxIndex - 1] == 0x01) {
  #ifdef RD03D_LOGGER_DEBUG
        Serial.print("# RX ACK: ");
        printHex(_bufferRx, _bufferRxIndex);
  #endif
        return _bufferRxIndex;
      }

    }
  }

#ifdef RD03D_LOGGER_DEBUG
  Serial.println("# RX ACK - TIMEOUT");
#endif
  return 0;
}


void RD03D::cmd_buffer_rx_clean(){
  unsigned long int t = millis();
  _bufferRxIndex = 0;
  while (_serial->available() && millis() - t < TIMEOUT_RX)
    _serial->read();
  _bufferRxIndex = 0;
#ifdef RD03D_LOGGER_DEBUG
  Serial.println("# CLEAR RX BUFFER");
#endif
}