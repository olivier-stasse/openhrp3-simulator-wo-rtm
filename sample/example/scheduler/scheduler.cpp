// -*- mode: c++; indent-tabs-mode: t; tab-width: 2; c-basic-offset: 2; -*-
#include <simscheduler.hh>

int main(int argc, char* argv[]) 
{
	SimScheduler aScheduler;

	aScheduler.init(argc,argv);
	// ==================  main loop   ======================
	aScheduler.mainLoop();
	return 0;
}
