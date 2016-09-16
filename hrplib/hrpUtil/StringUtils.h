#ifndef _HRP_UTILS_STRING_UTILS_HH_
#define _HRP_UTILS_STRING_UTILS_HH_

#include <string>
#include <sstream>

namespace hrp {

  template <typename To>
    bool stringTo(To& val, const char* str)
    {
      if (str == 0) { return false; }
      
      std::stringstream s;
      if ((s << str).fail()) { return false; }
      if ((s >> val).fail()) { return false; }
      return true;
    }
   
  template<>
    bool stringTo<std::string>(std::string& val, const char* str)
    {
      if (str == 0) { return false; }
      val = str;
      return true;
    }
  bool isEscaped(const std::string& str, std::string::size_type pos)
  {
    --pos;
    unsigned int i;
    for (i = 0; (pos >= 0) && str[pos] == '\\'; --pos, ++i) ;
    // If the number of \ is odd, delimiter is escaped.
    return (i % 2) == 1;
  }

  void eraseHeadBlank(std::string& str)
  {
    if (str.empty()) return;
    while (str[0] == ' ' || str[0] == '\t') str.erase(0, 1);
  }

  void eraseTailBlank(std::string& str)
  {
    if (str.empty()) return;
    while ((str[str.size() - 1] == ' ' || str[str.size() - 1] == '\t') &&
	   !isEscaped(str, str.size() - 1))
      str.erase(str.size() - 1, 1);
  }

  void eraseBothEndsBlank(std::string& str)
  {
    eraseHeadBlank(str);
    eraseTailBlank(str);
  }


  void toLower(std::string& str)
  {
    std::transform(str.begin(), str.end(), str.begin(),
		   (int (*)(int))std::tolower); 
  }

  std::string normalize(std::string& str)
    {
      eraseBothEndsBlank(str);
      toLower(str);
      return str;
    }

  typedef std::vector<std::string> vstring;

  vstring split(const std::string& input,
		const std::string& delimiter,
		bool ignore_empty=false)
  {
    typedef std::string::size_type size;
    vstring results;
    size delim_size = delimiter.size();
    size found_pos(0), begin_pos(0), pre_pos(0), substr_size(0);
     
    if (input.empty()) return results;

    while (1)
      {
	//    REFIND:
	found_pos = input.find(delimiter, begin_pos);
	if (found_pos == std::string::npos) 
	  {
	    std::string substr(input.substr(pre_pos));
	    eraseHeadBlank(substr);
	    eraseTailBlank(substr);
	    if (!(substr.empty() && ignore_empty)) results.push_back(substr);
	    break;
	  }
	substr_size = found_pos - pre_pos;
	if (substr_size >= 0)
	  {
	    std::string substr(input.substr(pre_pos, substr_size));
	    eraseHeadBlank(substr);
	    eraseTailBlank(substr);
	    if (!(substr.empty() && ignore_empty)) results.push_back(substr);
	  }
	begin_pos = found_pos + delim_size;
	pre_pos   = found_pos + delim_size;
      }
    return results;
  }

}
#endif
