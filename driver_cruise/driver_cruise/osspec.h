#ifndef __OSSPEC__H__
#define __OSSPEC__H__

/* Files Extensions */
#define TRKEXT	"xml"
#define DLLEXT	"dll"
#define PARAMEXT ".xml"

#ifndef _WIN32
#error Hey ! Where is _WIN32 ??
#endif

#include <windows.h>
#include <string.h>
#ifdef _WIN32
//#	pragma warning( disable : 4514 4100 4130 4611 4115 4189 4505 4127 4244 4701 4706 4711 4702)
#	define uint unsigned int
#	define uchar unsigned char
#	define execlp _execlp
#	define strncasecmp strnicmp
#	define strcasecmp stricmp
#	undef WINGDIAPI
#	define WINGDIAPI __declspec( dllexport )
#	undef APIENTRY
#	define APIENTRY __stdcall
#	undef CALLBACK
#	define CALLBACK __stdcall
#	undef XMLPARSEAPI
#	define XMLPARSEAPI __declspec( dllexport )
#else // !WIN32
#endif // _WIN32

#define M_PI (3.1415926535897932384626433832795)

//#define DEBUG_OUT

#endif /* __OSSPEC__H__ */
