// -*- mode: c++; indent-tabs-mode: t; tab-width: 2; c-basic-offset: 2; -*-
#include <schedulerproject.hh>

int main(int argc, char* argv[]) 
{
	SchedulerProject aScheduler;

	aScheduler.init(argc,argv);
	// ==================  main loop   ======================
	aScheduler.mainLoop();
	return 0;
}
