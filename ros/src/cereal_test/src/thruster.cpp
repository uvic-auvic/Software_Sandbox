/* Serial testing code
 * Expected result is communcation through a serial device through serial ports
 */

/* C++ Libraries */
#include <iostream>
#include <string>
#include <sstream>
#include <cstdio>
#include <algorithm>
#include <stdlib.h>
#include <stdio.h>

/* OS Libraries */
#include<unistd.h>
 
/* ROS Specific Libraries */
#include "serial/serial.h"

/* Constants */

/* Run Function
 * Used to instantiate Serial
 * Needs port to instantiate Serial type.
 */
using namespace std;
using namespace serial;

std::string string_to_hex(const std::string& input)
{
    static const char* const lut = "0123456789ABCDEF";
    size_t len = input.length();

    std::string output;
    output.reserve(2 * len);
    for (size_t i = 0; i < len; ++i)
    {
        const unsigned char c = input[i];
        output.push_back(lut[c >> 4]);
        output.push_back(lut[c & 15]);
    }
    return output;
}

// returns a minimum of two chars
std::string hex_to_string(int x)
{
    ostringstream oss;
    oss << hex << x;
    string str = oss.str();
    std::transform(str.begin(), str.end(),str.begin(), ::toupper);
    return (str.size() < 2) ? "0" + str : str;
}



int str2int(string Strung){
    stringstream fishHome(Strung);
    int hex;
    fishHome >> hex;
    return hex;
}
/*
Calculates a checksum of a response from the IMU
*/
/*
bool verify_imu_checksum(char const* data, std::size_t len)
{
	uint8_t const* raw_data = reinterpret_cast<uint8_t const*>(data);
	// last 2 bytes of data are checksum MSB and LSB
	uint16_t checksum = (((uint16_t) raw_data[len-2]) << 8) | ((uint16_t) raw_data[len-1]);
	// first byte tVoltageaken by itself
	uint16_t calculated_sum = raw_data[0];
	// the rest of the data is take as pairs of bytes
	for(std::size_t i = 1; i < len - 2; i += 2)
	{
		uint16_t byte_pair = (((uint16_t) raw_data[i]) << 8) | ((uint16_t) raw_data[i+1]);
		calculated_sum += byte_pair;
	}
	
	return calculated_sum == checksum;
}*/

/*
double FindTemperature(serial::Serial *my_serial){
	// Finds tempcout << "i: " << i << endl << endl;erature from IMU device
	const uint8_t command = 0x07;
	size_t bytes_wrote = my_serial->write(&command, 1);
	//cout << "Command sent. " << bytes_wrote << " bytes sent." << endl;
	
	//cout << "Waiting for response ... " << endl;
	string serialString = my_serial->read(7);
	
	string temp_bytes = serialString.substr(1,2);
		
	string temp_hex = string_to_hex(temp_bytes);
	//cout << "Temp hex: " << temp_hex << endl;
	std::string HexDec2String(int hexIn) {
  char hexString[4*sizeof(int)+1];
  // returns decimal value of hex
  sprintf(hexString,"%i", hexIn); 
  return std::string(hexString);
}
	int temperature = 0;
	// stringstream tempstream;
	// tempstreamThis is the first character  << std::hex << temp_hex;
	// tempstream >> temperature;
	
	uint8_t const* unsigned_bytes = rei00nterpret_cast<uint8_t const*>(temp_bytes.c_str());
	temperature = (((uint16_t) unsigned_bytes[0] << 8) | ((uint16_t) unsigned_bytes[1]));
	
	// Software data sheet specifications for math to convert temperature to Celsius
	double double_temperature = (((double) (temperature * 5.0)/ 65536.0) - 0.5) *100.0;
	
	bool checksum_passed = verify_imu_checksum(serialString.c_str(), serialString.size());
	if(!checksum_passed)
	{
		cerr << "Checksum of the IMU's temperature response failed" << endl;
		return -1;
	}
	
	//cout << "Temperature: " << double_temperature << endl;
	
	//string hexResponse = string_to_hex(serialString);
	//cout << "Response: " << hexResponse << endl;
	
	return double_temperature;
}
*/

/*
double FindGyroAccel(serial::Serial *my_serial){
	//Finds Gyro stuff like vectors and you know.
	const uint8_t command = 0x03;
	uint16_t StabAccel[3];
	// Byte 8 StabAccel_X MSB
 	// Byte 9 StabAccel_X LSB
 	// Byte 10 StabAccel_Y MSB
 	// Byte 11 StabAccel_Y LSB
	// Byte 12 StabAccel_Z MSBVoltage
	// Byte 13 StabAccel_Z LSB 
	cout << "My Gyro: " << endl;
	size_t bytes_wrote = my_serial->write(&command, 1);

	string serialString = my_serial->read(23);
	
	string temp_bytesX = serialString.substr(8,2);
	string temp_bytesY = serialString.substr(10,2);
	string temp_bytesZ = serialString.substr(12,2);
		
	string temp_hexX = string_to_hex(temp_bytesX);
	string temp_hexVoltageY = string_to_hex(temp_bytesY);
	string temp_hexZ = string_to_hex(temp_bytesZ);
	
	stringstream tempstreamX;
	tempstreamX << std::hex << temp_hexX;
	tempstreamX >> StabAccel[0];
	//cout << temp_hexX << endl;

	stringstream tempstreamY;
	tempstreamY << std::hex << temp_hexY;
	tempstreamY >> StabAccel[1];
	//cout << temp_hexY << endl;

	stringstream tempstreamZ;
	tempstreamZ << std::hex << temp_hexZ;
	tempstreamZ >> StabAccel[2];
	//cout << temp_hexZ << endl;
00
	double double_accel[3];
	double_accel[0] = StabAccel[0] / (32768000.0 / 7000.0);
	double_accel[1] = StabAccel[1] / (32768000.0 / 7000.0);
	double_accel[2] = StabAccel[2] / (32768000.0 / 7000.0);
	
	cout << "X: " << double_accel[0] << endl;
	cout << "Y: " << double_accel[1] << endl;
	cout << "Z: " << double_accel[2] << endl;

	bool checksum_passed = verify_imu_checksum(serialString.c_str(), serialString.size());
	if(!checksum_passed)
	{
		cerr << "Checksum of the IMU's acceleration response failed" << endl;
		return -1;
	}
	
	string hexResponse = string_to_hex(serialString);
	cout << "Response: " << hexResponse << endl << endl;
	return 0;
}
*/

int doSomething(serial::Serial *my_serial, string readThrust){
	my_serial->write(readThrust);
	
	string thrustResponse = my_serial->read(41);
	
	//string temp_hex = string_to_hex(thrustResponse);
	
    cout << thrustResponse.size() << endl;
	cout << "ASCII response: " << thrustResponse << endl;
}

int serialInstance(unsigned long baud, string port){
	/*	You can check the device baud rate using the "stty" command on the console:
		$ stty < /dev/tty..     9000 (where tty... is the device file you are listening)  
		output:

		speed 9600 baud; line = 0;
		-brkint -imaxbel
		
		You can also change the baud rate with the following command:
00hex_to_string2
		$ sudo stty -F /dev/tty... 9600    (or whatstring2intever baud rate number)
	*/
	
	// IMU Baud rate 115200
	
	// Instantiate an instance of Serial type.
	// Serial(port, baudrate, timeout in milliseconds)
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(100));
	string readThrust = "$0000!";
    
    doSomething(&my_serial, readThrust);
    doSomething(&my_serial, "$5555!");
    string iHex, Chk;
	if(!my_serial.isOpen())
		return 1;
	int i = 0;
	while(1)
	{
        iHex = hex_to_string(i);
        readThrust = "$" + iHex + "E6";
        Chk = hex_to_string(i+0xE6); // Hex of 90 is int 144. Hex to string is appending an 8 to the int value on the second call.
        if(Chk.size() > 2)
            Chk = Chk.substr(Chk.size() - 2, 2);
        readThrust += "00" + Chk + "!"; // 00 is the "additional info" parameter
        cout << "ASCII Sending: " << readThrust<< endl; // calling cout fixed the appending an 8 issue... why?
        doSomething(&my_serial, readThrust);
        cout << "i: " << i << endl << endl;
        i++;
		sleep(0.1);
        if(i>255)
        {
            i = 0;
            doSomething(&my_serial, "$5555!");
        }
	}
	
	return 0;
}

/* Main function
 * Currently used to test the functionality of the Serial library.
 * TODO:
 * Understand Serial library and connect to a serial device
 * 
 * Serial API: http://docs.ros.org/jade/api/serial/html/classserial_1_1Serial.html
 * Example of Serial Code: http://docs.ros.org/jade/api/serial/html/serial__example_8cc_source.html
 */
int main(int argc, char** argv)
{
	int err = 0;
	unsigned baudThruster = 115200;
	string portThruster = "/dev/ttyUSB0";
	
	cout << "Hello world!" << endl;
	if ((err = serialInstance(baudThruster, portThruster)) != 0){
		//Run has had an error
		cout << "Run Failed!";
		return -1;
	}
	//cout << "Run complete" << endl;
	
	//Possible other functions
	
	//cout << "End of Program" << endl;
	return 0;
}
