/* Serial testing code
 * Expected result is communcation through a serial device through serial ports
 */

/* C++ Libraries */
#include <iostream>
#include <string>
#include <sstream>
#include <cstdio>

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

/*
Calculates a checksum of a response from the IMU
*/
bool verify_imu_checksum(char const* data, std::size_t len)
{
	uint8_t const* raw_data = reinterpret_cast<uint8_t const*>(data);
	// last 2 bytes of data are checksum MSB and LSB
	uint16_t checksum = (((uint16_t) raw_data[len-2]) << 8) | ((uint16_t) raw_data[len-1]);
	// first byte taken by itself
	uint16_t calculated_sum = raw_data[0];
	// the rest of the data is take as pairs of bytes
	for(std::size_t i = 1; i < len - 2; i += 2)
	{
		uint16_t byte_pair = (((uint16_t) raw_data[i]) << 8) | ((uint16_t) raw_data[i+1]);
		calculated_sum += byte_pair;
	}
	
	return calculated_sum == checksum;
}

double FindTemperature(serial::Serial *my_serial){
	/* Finds temperature from IMU device
	*/
	const uint8_t command = 0x07;
	size_t bytes_wrote = my_serial->write(&command, 1);
	//cout << "Command sent. " << bytes_wrote << " bytes sent." << endl;
	
	//cout << "Waiting for response ... " << endl;
	string serialString = my_serial->read(7);
	
	string temp_bytes = serialString.substr(1,2);
		
	string temp_hex = string_to_hex(temp_bytes);
	//cout << "Temp hex: " << temp_hex << endl;
	
	int temperature = 0;
	/*
	stringstream tempstream;
	tempstream << std::hex << temp_hex;
	tempstream >> temperature;*/
	
	uint8_t const* unsigned_bytes = reinterpret_cast<uint8_t const*>(temp_bytes.c_str());
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

double total_accel_x = 0, total_accel_y = 0, total_accel_z = 0;
unsigned accel_reads = 0;
port
double FindGyroAccel(serial::Serial *my_serial){
	//Finds Gyro stuff like vectors and you know.
	const uint8_t command = 0x03;
	uint16_t StabAccel[3];
	/*
	Byte 8 StabAccel_X MSB
 	Byte 9 StabAccel_X LSB
 	Byte 10 StabAccel_Y MSB
 	Byte 11 StabAccel_Y LSB
	Byte 12 StabAccel_Z MSB
	Byte 13 StabAccel_Z LSB 
	*/
	cout << "My Gyro: " << endl;
	size_t bytes_wrote = my_serial->write(&command, 1);

	string serialString = my_serial->read(23);
	
	string temp_bytesX = serialString.substr(8,2);
	string temp_bytesY = serialString.substr(10,2);
	string temp_bytesZ = serialString.substr(12,2);
		
	string temp_hexX = string_to_hex(temp_bytesX);
	string temp_hexY = string_to_hex(temp_bytesY);
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

int serialInstance(unsigned long baud, string port){
	/*	You can check the device baud rate using the "stty" command on the console:
		$ stty < /dev/tty..      (where tty... is the device file you are listening)  
		output:

		speed 9600 baud; line = 0;
		-brkint -imaxbel
		
		You can also change the baud rate with the following command:

		$ sudo stty -F /dev/tty... 9600    (or whatever baud rate number)
	*/
	
	// IMU Baud rate 115200
	
	// Instantiate an instance of Serial type.
	// Serial(port, baudrate, timeout in milliseconds)
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(10000));
	const uint8_t command = 0x06;
	my_serial.write(&command, 1);
	my_serial.read(5);
	int i = 0;
	while(1)
	{
		cout << "Publish number: " << i << endl;
		double temperature = FindTemperature(&my_serial);
		cout<<"My Temperature: " << temperature << endl << endl;
		double gyro = FindGyroAccel(&my_serial);
		cout << endl << endl;
		i++;
		
		sleep(1);
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
	unsigned baudIMU = 115200;
	string portIMU = "/dev/ttyUSB0";
	
	//cout << "Hello world!" << endl;
	if ((err = serialInstance(baudIMU, portIMU)) != 0){
		//Run has had an error
		cout << "Run Failed!";
		return -1;
	}
	//cout << "Run complete" << endl;
	
	//Possible other functions
	
	//cout << "End of Program" << endl;
	return 0;
}
