#include <iostream>
#include <string>
#include <sstream>
#include <cstdio>

/* OS Libraries */
#include<unistd.h>
 
/* ROS Specific Libraries */
#include "serial/serial.h"
#include <nodelet/nodelet.h>

namespace example_pkg
{
	class GrabberNodelet : public nodelet::Nodelet
	{
		public:
			virtual void onInit();
	};
}