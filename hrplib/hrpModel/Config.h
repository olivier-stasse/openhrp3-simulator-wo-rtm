
#ifndef HRPMODEL_CONFIG_H_INCLUDED
#define HRPMODEL_CONFIG_H_INCLUDED

#define HRPMODEL_VERSION_MAJOR 
#define HRPMODEL_VERSION_MINOR 
#define HRPMODEL_VERSION_MICRO 

#define OPENHRP_DIR "/home/ostasse/devel/ros-indigo-1/install"
#define OPENHRP_SHARE_DIR "/home/ostasse/devel/ros-indigo-1/install/"
#define OPENHRP_RELATIVE_SHARE_DIR ""

// for Windows DLL export 
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
# ifdef HRPMODEL_MAKE_DLL
#   define HRPMODEL_API __declspec(dllexport)
# else 
#   define HRPMODEL_API __declspec(dllimport)
# endif
#else 
# define HRPMODEL_API
#endif /* Windows */

#endif
