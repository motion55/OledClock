/*

	HelloWorld.ino

	Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

	Copyright (c) 2016, olikraus@gmail.com
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification, 
	are permitted provided that the following conditions are met:

	* Redistributions of source code must retain the above copyright notice, this list 
		of conditions and the following disclaimer.
		
	* Redistributions in binary form must reproduce the above copyright notice, this 
		list of conditions and the following disclaimer in the documentation and/or other 
		materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
	CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
	INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
	CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
	SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
	NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
	STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
	ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

*/

#include <Arduino.h>
#include <U8g2lib.h>

#if defined(ESP8266)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <Time.h>
#include <TimeLib.h>

Adafruit_BMP280 bme; // I2C
boolean Use_bmp280;
float Temperature, Pressure;
float Ave_Temperature, Ave_Pressure;
int Count = 0;
#define	AVE_SAMPLES	128

char ssid[] = "BST";	//  your network SSID (name)
char pass[] = "";		// your network password

unsigned int localPort = 2390;      // local port to listen for UDP packets

const char* ntpServerName = "ntp.pagasa.dost.gov.ph";
IPAddress timeServerIP(121, 58, 193, 100);	//IP address of "ntp.pagasa.dost.gov.ph"

const int NTP_PACKET_SIZE = 48;	// NTP time stamp is in the first 48 bytes of the message

//buffer to hold incoming and outgoing packets
byte packetBuffer[NTP_PACKET_SIZE];

//A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

/*
	U8glib Example Overview:
		Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
		Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
		U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.
		
*/

U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ D8, /* dc=*/ D3, /* reset=*/ D4);


void setup(void) {
	Serial.begin(115200);

	DS3231_setup();

	if (!bme.begin(0x76))
	{
		Serial.println("Could not find a valid BMP280 sensor, check wiring!");
		Use_bmp280 = false;
	}
	else
	{
		Ave_Temperature = bme.readTemperature();
		Temperature = Ave_Temperature * AVE_SAMPLES;
		Ave_Pressure = bme.readPressure();
		Pressure = Ave_Pressure * AVE_SAMPLES;
		Count = 0;
		Use_bmp280 = true;
	}
	
	u8g2.begin();
	u8g2.setContrast(128);
	u8g2.clearBuffer();					// clear the internal memory
	u8g2.setFont(u8g2_font_courR14_tf);	// choose a suitable font
	u8g2.drawStr(10,32,"Connecting");	// write something to the internal memory
	u8g2.sendBuffer();					// transfer internal memory to the display
	for (int i = 0; i<400; i++)
	{
		if (WiFi.status() == WL_CONNECTED)
		{
			Serial.println(F("WiFi connected"));
			Serial.println(F("IP address: "));
			Serial.println(WiFi.localIP());
			break;
		}
		delay(50);
	}

	Serial.println(F("Starting UDP"));
	udp.begin(localPort);
	Serial.print(F("Local port: "));
	Serial.println(udp.localPort());

	IPAddress addr;
	if (WiFi.hostByName(ntpServerName, addr))
	{
		timeServerIP = addr;
	}

	setSyncProvider(getNtpTime);

	webserver_setup();
}

char TimeText[] = "00:00:00am\0";
//                 01234567890


void loop(void) {
	UpdateTime();
	if (Use_bmp280)
	{
		Ave_Temperature = Temperature / AVE_SAMPLES;
		Temperature -= Ave_Temperature;
		Temperature += bme.readTemperature();

		Ave_Pressure = Pressure / AVE_SAMPLES;
		Pressure -= Ave_Pressure;
		Pressure += bme.readPressure();
	}

	if (Count < 4)
	{
		Count++;
	}
	else
	{
		u8g2.clearBuffer();					// clear the internal memory
		u8g2.setFont(u8g2_font_courR14_tf);	// choose a suitable font
		u8g2.drawStr(10, 20, TimeText);
		String Pressure_Str(Ave_Pressure/100.0f,2); Pressure_Str += "hPa";
		if (Ave_Pressure < 100000.0f) Pressure_Str = " " + Pressure_Str;
		u8g2.setFont(u8g2_font_courR12_tf);	// choose a suitable font
		u8g2.drawStr(20, 36, Pressure_Str.c_str());
		String Temperature_Str(Ave_Temperature,1); Temperature_Str += "C";
		if (Ave_Temperature < 10.0f) Temperature_Str = " " + Temperature_Str;
		u8g2.setFont(u8g2_font_courR24_tf);	// choose a suitable font
		u8g2.drawStr(20, 63, Temperature_Str.c_str());
		u8g2.sendBuffer();					// transfer internal memory to the display
	}
	my_delay_ms(50);
}

/*///////////////////////////////////////////////////////////////////////////*/

void UpdateTime(void)
{
	time_t tm = now();

	int hour = hourFormat12(tm);
	if (hour < 10)
	{
		TimeText[0] = ' ';
		TimeText[1] = '0' + hour;
	}
	else
	{
		TimeText[0] = '1';
		TimeText[1] = '0' + (hour - 10);
	}

	int min = minute(tm);
	int min10 = min / 10;
	TimeText[3] = '0' + min10;
	TimeText[4] = '0' + min - (min10 * 10);

	int sec = second(tm);
	int sec10 = sec / 10;
	TimeText[6] = '0' + sec10;
	TimeText[7] = '0' + sec - (sec10 * 10);

	if (isAM(tm))
	{
		TimeText[8] = 'a';
		TimeText[9] = 'm';
	}
	else
	{
		TimeText[8] = 'p';
		TimeText[9] = 'm';
	}
}

const int timeZone = 8 * SECS_PER_HOUR;     // PHT
int packet_delay = 0;

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address)
{
	if (WiFi.status() == WL_CONNECTED)
	{
		Serial.println(F("sending NTP packet..."));
		// set all bytes in the buffer to 0
		memset(packetBuffer, 0, NTP_PACKET_SIZE);
		// Initialize values needed to form NTP request
		// (see URL above for details on the packets)
		packetBuffer[0] = 0b11100011;   // LI, Version, Mode
		packetBuffer[1] = 0;     // Stratum, or type of clock
		packetBuffer[2] = 6;     // Polling Interval
		packetBuffer[3] = 0xEC;  // Peer Clock Precision
								 // 8 bytes of zero for Root Delay & Root Dispersion
		packetBuffer[12] = 49;
		packetBuffer[13] = 0x4E;
		packetBuffer[14] = 49;
		packetBuffer[15] = 52;

		// all NTP fields have been given values, now
		// you can send a packet requesting a timestamp:
		udp.beginPacket(address, 123); //NTP requests are to port 123
		udp.write(packetBuffer, NTP_PACKET_SIZE);
		udp.endPacket();

		packet_delay = 1500;		 // wait to see if a reply is available
	}
}

time_t getNtpTime()
{
	Serial.println(F("Transmit NTP Request"));
	sendNTPpacket(timeServerIP); // send an NTP packet to a time server

	return 0;
}

void my_delay_ms(int msec)
{
	uint32_t delay_val = msec;
	uint32_t endWait = millis();
	uint32_t beginWait = endWait;
	while (endWait - beginWait < delay_val)
	{
		webserver_loop();

		int size = udp.parsePacket();
		if (packet_delay > 0)
		{
			if (size >= NTP_PACKET_SIZE)
			{
				Serial.println(F("Receive NTP Response"));
				udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
				unsigned long secsSince1900;
				// convert four bytes starting at location 40 to a long integer
				secsSince1900 = (unsigned long)packetBuffer[40] << 24;
				secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
				secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
				secsSince1900 |= (unsigned long)packetBuffer[43];

				time_t tm = secsSince1900 - 2208988800UL + timeZone;
				setTime(tm);
				DS3231_setTime(tm);
				packet_delay = 0;
			}
		}
		delay(1);
		endWait = millis();
	}
	if (packet_delay > 0)
		packet_delay -= msec;
}

