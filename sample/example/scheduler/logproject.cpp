#include <logproject.hh>

LogProject::LogProject()
{
}

void LogProject::setNameAndPath(std::string &bodyname,
				std::string &logpath)
{
  logpath_  = logpath;
  bodyname_ = bodyname;
  buildLogFilename();
}

void LogProject::buildLogFilename()
{
  logfilename_ = logpath_ + bodyname_ + ".log";
  std::ofstream aof;
  aof.open(logfilename_.c_str(),std::ofstream::out);
  aof.close();
  start_ = true;
}

void LogProject::printStart(std::ofstream &aof,
			    OpenHRP::SensorState &sstate)
{
  aof << "%time " ;
  // Dump q
  for(unsigned int i=0;i<sstate.q.length();i++)
    {
      aof << "q"<< i << " ";
    }
  // Dump dq
  for(unsigned int i=0;i<sstate.dq.length();i++)
    {
      aof << "dq" << i << " ";
    }
  // Dump u
  for(unsigned int i=0;i<sstate.u.length();i++)
    {
      aof << "u" << i << " ";
    }
  // Dump force
  for (unsigned int i=0;i<sstate.force.length();i++)
    {
      for(unsigned int j=0;j<6;j++)
	aof << "F" << i << "_" << j << " ";
    }
  // Dump rateGyro
  for (unsigned int i=0;i<sstate.rateGyro.length();i++)
    {
      for(unsigned int j=0;j<3;j++)
	aof << "RG"<<i << "_" <<j<< " ";
    }
  // Dump accel
  for (unsigned int i=0;i<sstate.accel.length();i++)
    {
      for(unsigned int j=0;j<3;j++)
	aof << "A"<< i << "_"<< j<< " ";
    }
  
  // Range sensor
  for (unsigned int i=0;i<sstate.range.length();i++)
    {
      for(unsigned int j=0;j<sstate.range[i].length();j++)
	aof << "SR" << i<<"_" <<j<< " ";
    }
  
  // Joint data value.
  for(unsigned int i=0;i<7;i++)
    {
      for(unsigned int j=0;j<jointData[i].size();j++)
	{
	  aof << "JD" << i << "_" << j << " ";
	}
    }
  aof << std::endl;

}

void LogProject::saveLog(double time,
			 OpenHRP::SensorState &sstate)
{
  std::ofstream aof;
  aof.open(logfilename_.c_str(),std::ofstream::app);

  if (start_)
    {
      printStart(aof,sstate);
      start_=false;
    }

  aof << time << " ";
  // Dump q
  for(unsigned int i=0;i<sstate.q.length();i++)
    {
      aof << sstate.q[i] << " ";
    }
  // Dump dq
  for(unsigned int i=0;i<sstate.dq.length();i++)
    {
      aof << sstate.dq[i] << " ";
    }
  // Dump u
  for(unsigned int i=0;i<sstate.u.length();i++)
    {
      aof << sstate.u[i] << " ";
    }
  // Dump force
  for (unsigned int i=0;i<sstate.force.length();i++)
    {
      for(unsigned int j=0;j<6;j++)
	aof << sstate.force[i][j]<< " ";
    }
  // Dump rateGyro
  for (unsigned int i=0;i<sstate.rateGyro.length();i++)
    {
      for(unsigned int j=0;j<3;j++)
	aof << sstate.rateGyro[i][j]<< " ";
    }
  // Dump accel
  for (unsigned int i=0;i<sstate.accel.length();i++)
    {
      for(unsigned int j=0;j<3;j++)
	aof << sstate.accel[i][j]<< " ";
    }
  
  // Range sensor
  for (unsigned int i=0;i<sstate.range.length();i++)
    {
      for(unsigned int j=0;j<sstate.range[i].length();j++)
	aof << sstate.range[i][j]<< " ";
    }
  
  // Joint data value.
  for(unsigned int i=0;i<7;i++)
    {
      for(unsigned int j=0;j<jointData[i].size();j++)
	{
	  aof << jointData[i][j] << " ";
	}
    }
  aof << std::endl;

  aof.close();
}
