#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termio.h>
#include <termios.h>
#include <vector>
#include <fstream>
#include <algorithm>
#include <stdint.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace std;

/*
	Serial RS-232 bridge pin I/O
	
          1 2 3 4 5
	-------------
	\ * * * * * /
	 \ * * * * /
          ---------
           6 7 8 9

	1) DCD "Data Carrier Detect" (input)
	2) RXD "Receive Data"        (input)
	3) TXD "Transmit Data"       (output)
	4) DTR "Data Terminal Relay" (output)
	5) SG  "Signal Ground"       (comm)
	6) DSR "Data Set Relay"      (input)
	7) RTS "Request Set Relay"   (output)
	8) CTS "Clean To Send"       (input)
	9) RI  "Ring Indicator"      (input)


*/

#define HIGH TIOCMBIS
#define LOW TIOCMBIC

enum PIN 
{
       LE  = TIOCM_LE,       //DSR (data set ready/line enable)
       DTR = TIOCM_DTR,      //DTR (data terminal ready)
       RTS = TIOCM_RTS,      //RTS (request to send)
       TX  = TIOCM_ST,       //Secondary TXD (transmit)
       RX  = TIOCM_SR,       //Secondary RXD (receive)
       CTS = TIOCM_CTS,      //CTS (clear to send)
       DCD = TIOCM_CAR,      //DCD (data carrier detect)
       CAR = TIOCM_CD,       //see TIOCM_CAR
       RNG = TIOCM_RNG,      //RNG (ring)
       RI  = TIOCM_RI,       //see TIOCM_RNG
       DSR = TIOCM_DSR,      //DSR (data set ready)
}
_PIN;

class RS232 
{
	public:
	int handle;

	void setup(string _device) 
	{ 
		handle = open(_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	}

	void set_pin( int _out, int _state )
	{
		int flags = _out;
		ioctl(handle,_state,&flags);
	}

	int get_pin( int _in )
	{
		int state;
	        ioctl(handle, TIOCMGET, &state);
		if (state & _in)
		{
			return 1;
		}
		else {
			return 0;
		}
	}

	void exit() { close(handle); }
};

class SerialIO {
	public:
	int baudrate;
	char type;
	char destination[2];
	char *message;
	char *localUsb;
	int fd;

	int io_setup(string serialport, int baud) 
	{
		struct termios toptions;
		fd = open(serialport.c_str(), O_RDWR | O_NOCTTY);
		if (fd == -1) {
			cout << "setup: failed open port: "<< serialport << endl;
			return -1;
		}

		if (tcgetattr(fd, &toptions) < 0) {
			cout << "setup: Couldn't get term attributes"<<endl;
			return -1;
		}
		speed_t brate = baud;
		switch (baud) {
		case 4800:
			brate = B4800;
			break;
		case 9600:
			brate = B9600;
			break;
		case 19200:
			brate = B19200;
			break;
		case 38400:
			brate = B38400;
			break;
		case 57600:
			brate = B57600;
			break;
		case 115200:
			brate = B115200;
			break;
		case 460800:
			brate = B460800;
			break;
		}

		cfsetispeed(&toptions, brate);
		cfsetospeed(&toptions, brate);

		toptions.c_cflag &= ~PARENB;
		toptions.c_cflag &= ~CSTOPB;
		toptions.c_cflag &= ~CSIZE;
		toptions.c_cflag |= CS8;

		toptions.c_cflag &= ~CRTSCTS;
		toptions.c_cflag |= CREAD | CLOCAL;
		toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
		toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		toptions.c_oflag &= ~OPOST;

		toptions.c_cc[VMIN] = 1;
		toptions.c_cc[VTIME] = 10;
		if (tcsetattr(fd, TCSANOW, &toptions) < 0) {
			cout << "setup: Couldn't set term attributes" << endl;
			return -1;
		}
		return fd;
	}

	int io_write(int fd, const char* str) 
	{
		int len = strlen(str);
		int n = write(fd, str, len);
		if (n != len)
			return -1;
		return n;
	}

	int io_writebyte(int fd, uint8_t b) 
	{
		int n = write(fd, &b, 1);
		if (n != 1)
			return -1;
		return 0;
	}

	int io_read_until(int fd, char buf[10][20], char until) 
	{
		char b[1];
		int i = 0, j = 0;
		do {
			do {
				int n = read(fd, b, 1);
				if (n == -1)
					return -1;
				if (n == 0) {
					usleep(10 * 1000);
					continue;
				}
				buf[j][i] = b[0];
				i++;
				if(b[0] == until)
					break;
			} while (b[0] != ',');
			buf[j][i] = 0;
			i = 0;
			j++;
		} while (b[0] != until);
		return i;
	}

	char io_read(int fd) 
	{
		char b[1];
		int n = read(fd, b, 1);
		if (n == -1){
			cout << "error read serial"<<endl;
		}
		return b[0];
	}

	int io_available(int fd) 
	{
		int nbytes = 0;
		ioctl(fd, FIONREAD, &nbytes);
		return nbytes;
	}

	void stop()
	{
		close(fd);
	}
	
};
