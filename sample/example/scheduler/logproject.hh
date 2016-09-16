#ifndef _LOG_PROJECT_HH_
#define _LOG_PROJECT_HH_

#include <string>
#include <fstream>
#include <vector>

#include <hrpCorba/DynamicsSimulator.hh>

class LogProject
{
public:
  LogProject();
  void setNameAndPath(std::string &bodyname,
		   std::string &logpath);

  void saveLog(double time, OpenHRP::SensorState &sstate);
  std::vector<double> jointData[7];

protected:
  void buildLogFilename();
  void printStart(std::ofstream &aof,
		  OpenHRP::SensorState &sstate);
private:
  std::string logpath_;
  std::string bodyname_;
  std::string logfilename_;
  bool start_;

};

#endif /* _LOG_PROJECT_HH_ */
