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

	while(1) {
		rs.set_pin(RTS,HIGH);
		cout <<"State pin RTS: "<< rs.get_pin(RTS) << endl;
		sleep(1);
		rs.set_pin(RTS,LOW);
		cout <<"State pin RTS: "<< rs.get_pin(RTS) << endl;
		sleep(1);
	}

	rs.exit();
	return 0;
}
