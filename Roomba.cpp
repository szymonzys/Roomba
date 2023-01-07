// Roomba.cpp
//
// Copyright (C) 2010 Mike McCauley
// $Id: Roomba.cpp,v 1.1 2010/09/27 21:58:32 mikem Exp mikem $

#include "Roomba.h"

Roomba::Roomba(HardwareSerial* serial, Baud baud)
{
  _serial = serial;
  _baud = baudCodeToBaudRate(baud);
  _pollState = PollStateIdle;
}

void Roomba::serialwrite(char c)
{
    _serial->write(c);
	delay(50);
}

// Resets the
void Roomba::reset()
{
    serialwrite(7);
}

// Start OI
// Changes mode to passive
void Roomba::start()
{
    _serial->begin(_baud);
    serialwrite(128);
}

uint32_t Roomba::baudCodeToBaudRate(Baud baud)
{
    switch (baud)
    {
	case Baud300:
	    return 300;
	case Baud600:
	    return 600;
	case Baud1200:
	    return 1200;
	case Baud2400:
	    return 2400;
	case Baud4800:
	    return 4800;
	case Baud9600:
	    return 9600;
	case Baud14400:
	    return 14400;
	case Baud19200:
	    return 19200;
	case Baud28800:
	    return 28800;
	case Baud38400:
	    return 38400;
	case Baud57600:
	    return 57600;
	case Baud115200:
	    return 115200;
	default:
	    return 57600;
    }
}

void Roomba::baud(Baud baud)
{
    serialwrite(129);
    serialwrite(baud);

    _baud = baudCodeToBaudRate(baud);
    _serial->begin(_baud);
}

void Roomba::safeMode()
{
  serialwrite(131);
}

void Roomba::fullMode()
{
  serialwrite(132);
}

void Roomba::power()
{
  serialwrite(133);
}

void Roomba::dock()
{
  serialwrite(143);
}

void Roomba::demo(Demo demo)
{
  serialwrite(136);
  serialwrite(demo);
}

void Roomba::cover()
{
  serialwrite(135);
}

void Roomba::coverAndDock()
{
  serialwrite(143);
}

void Roomba::spot()
{
  serialwrite(134);
}

void Roomba::drive(int16_t velocity, int16_t radius)
{
  serialwrite(137);
  serialwrite((velocity & 0xff00) >> 8);
  serialwrite(velocity & 0xff);
  serialwrite((radius & 0xff00) >> 8);
  serialwrite(radius & 0xff);
}

void Roomba::driveDirect(int16_t leftVelocity, int16_t rightVelocity)
{
  serialwrite(145);
  serialwrite((rightVelocity & 0xff00) >> 8);
  serialwrite(rightVelocity & 0xff);
  serialwrite((leftVelocity & 0xff00) >> 8);
  serialwrite(leftVelocity & 0xff);
}

void Roomba::leds(uint8_t leds, uint8_t powerColour, uint8_t powerIntensity)
{
  serialwrite(139);
  serialwrite(leds);
  serialwrite(powerColour);
  serialwrite(powerIntensity);
}

void Roomba::digitLedsRaw(uint8_t digit3, uint8_t digit2, uint8_t digit1, uint8_t digit0)
{
  serialwrite(163);
  serialwrite(digit3);
  serialwrite(digit2);
  serialwrite(digit1);
  serialwrite(digit0);
}

void Roomba::digitLedsASCII(uint8_t digit3, uint8_t digit2, uint8_t digit1, uint8_t digit0)
{
  serialwrite(164);
  serialwrite(digit3);
  serialwrite(digit2);
  serialwrite(digit1);
  serialwrite(digit0);
}

void Roomba::digitalOut(uint8_t out)
{
  serialwrite(147);
  serialwrite(out);
}

// Sets PWM duty cycles on low side drivers
void Roomba::pwmDrivers(uint8_t dutyCycle0, uint8_t dutyCycle1, uint8_t dutyCycle2)
{
  serialwrite(144);
  serialwrite(dutyCycle2);
  serialwrite(dutyCycle1);
  serialwrite(dutyCycle0);
}

// Sets low side driver outputs on or off
void Roomba::drivers(uint8_t out)
{
  serialwrite(138);
  serialwrite(out);
}

// Modulates low side driver 1 (pin 23 on Cargo Bay Connector)
// with the given IR command
void Roomba::sendIR(uint8_t data)
{
  serialwrite(151);
  serialwrite(data);
}

// Define a song
// Data is 2 bytes per note
void Roomba::song(uint8_t songNumber, const uint8_t* data, int len)
{
    serialwrite(140);
    serialwrite(songNumber);
    serialwrite(len >> 1); // 2 bytes per note
    _serial->write(data, len);
}

void Roomba::playSong(uint8_t songNumber)
{
  serialwrite(141);
  serialwrite(songNumber);
}

// Start a stream of sensor data with the specified packet IDs in it
void Roomba::stream(const uint8_t* packetIDs, int len)
{
  serialwrite(148);
  _serial->write(packetIDs, len);
}

// One of StreamCommand*
void Roomba::streamCommand(StreamCommand command)
{
  serialwrite(150);
  serialwrite(command);
}

// Use len=0 to clear the script
void Roomba::script(const uint8_t* script, uint8_t len)
{
  serialwrite(152);
  serialwrite(len);
  _serial->write(script, len);
}

void Roomba::playScript()
{
  serialwrite(153);
}

// Each tick is 15ms
void Roomba::wait(uint8_t ticks)
{
  serialwrite(155);
  serialwrite(ticks);
}

void Roomba::waitDistance(int16_t mm)
{
  serialwrite(156);
  serialwrite((mm & 0xff00) >> 8);
  serialwrite(mm & 0xff);
}

void Roomba::waitAngle(int16_t degrees)
{
  serialwrite(157);
  serialwrite((degrees & 0xff00) >> 8);
  serialwrite(degrees & 0xff);
}

// Can use the negative of an event type to wait for the inverse of an event
void Roomba::waitEvent(EventType type)
{
  serialwrite(158);
  serialwrite(type);
}

// Reads at most len bytes and stores them to dest
// If successful, returns true.
// If there is a timeout, returns false
// Blocks until all bytes are read
// Caller must ensure there is sufficient space in dest
bool Roomba::getData(uint8_t* dest, uint8_t len)
{
  while (len-- > 0)
  {
    unsigned long startTime = millis();
    while (!_serial->available())
    {
      // Look for a timeout
      if (millis() > startTime + ROOMBA_READ_TIMEOUT)
        return false; // Timed out
    }
    *dest++ = _serial->read();
  }
  return true;
}

bool Roomba::getSensors(uint8_t packetID, uint8_t* dest, uint8_t len)
{
  serialwrite(142);
  serialwrite(packetID);
  return getData(dest, len);
}

bool Roomba::getSensorsList(uint8_t* packetIDs, uint8_t numPacketIDs, uint8_t* dest, uint8_t len)
{
  serialwrite(149);
  serialwrite(numPacketIDs);
  _serial->write(packetIDs, numPacketIDs);
  return getData(dest, len);
}

// Simple state machine to read sensor data and discard everything else
bool Roomba::pollSensors(uint8_t* dest, uint8_t len)
{
    while (_serial->available())
    {
	uint8_t ch = _serial->read();
	switch (_pollState)
	{
	    case PollStateIdle:
		if (ch == 19)
		    _pollState = PollStateWaitCount;
                    _pollChecksum = ch;
		break;

	    case PollStateWaitCount:
                _pollChecksum += ch;
                _pollSize = ch;
		_pollCount = 0;
		_pollState = PollStateWaitBytes;
		break;

	    case PollStateWaitBytes:
		_pollChecksum += ch;
		if (_pollCount < len)
		    dest[_pollCount] = ch;
		if (++_pollCount >= _pollSize)
		    _pollState = PollStateWaitChecksum;
		break;

	    case PollStateWaitChecksum:
		_pollChecksum += ch;
		_pollState = PollStateIdle;
		return (_pollChecksum == 0);
		break;
	}
    }
    return false;
}

// Returns the number of bytes in the script, or 0 on errors
// Only saves at most len bytes to dest
// Calling with len = 0 will return the amount of space required without actually storing anything
uint8_t Roomba::getScript(uint8_t* dest, uint8_t len)
{
  serialwrite(154);

  unsigned long startTime = millis();
  while (!_serial->available())
  {
    // Look for a timeout
    if (millis() > startTime + ROOMBA_READ_TIMEOUT)
      return 0; // Timed out
  }

  int count = _serial->read();
  if (count > 100 || count < 0)
    return 0; // Something wrong. Cant have such big scripts!!

  // Get all the data, saving as much as we can
  uint8_t i;
  for (i = 0; i < count; i++)
  {
    startTime = millis();
    while (!_serial->available())
    {
      // Look for a timeout
      if (millis() > startTime + ROOMBA_READ_TIMEOUT)
        return 0; // Timed out
    }
    uint8_t data = _serial->read();
    if (i < len)
      *dest++ = data;
  }

  return count;
}
