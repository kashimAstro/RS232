#include <iostream>
#include "RS232.h"

int main(int argc, char ** argv)
{
	if(argc < 2) 
	{
		cout << "parameter: device" << endl;
		exit(0);
	}

	RS232 rs;
	rs.setup(argv[1]);
	rs.set_pin(RTS,HIGH);

	while(1) {
		cout <<"State pin CTS: "<< rs.get_pin(CTS) << endl;
		sleep(1);
	}

	rs.exit();
	return 0;
}
