// stdafx.h: Standard system Include files can be included in this header file.
// Or often used but rarely changed
//Project-specific Include file
//

#pragma once

// If you have a platform that must be prioritized, modify the definition below.
// Refer to MSDN for the latest information on corresponding values for different platforms.
#ifndef WINVER // Allows the use of specific features of Windows XP (inclusive) and later versions.
#define WINVER 0x0501 // Change this to the appropriate value for other versions of Windows.
#endif

#ifndef _WIN32_WINNT // Allows the use of specific features of Windows XP (inclusive) and later versions.
#define _WIN32_WINNT 0x0501 // Change this to the appropriate value for other versions of Windows.
#endif

#ifndef _WIN32_WINDOWS // Allows the use of specific features of Windows 98 (inclusive) and later versions.
#define _WIN32_WINDOWS 0x0410 // Change this to the appropriate value for Windows Me (inclusive) and later versions.
#endif

#ifndef _WIN32_IE // Allow the use of specific functions of IE 6.0 (inclusive) and later versions.
#define _WIN32_IE 0x0600 // Change this to the appropriate value for other versions of IE.
#endif

#define WIN32_LEAN_AND_MEAN // Exclude uncommonly used members from Windows headers
#include <stdio.h>
#include <tchar.h>
#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS // Explicitly define part of the CString constructor

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN // Exclude uncommonly used members from Windows headers
#endif

#include <afx.h>
#include <afxwin.h> // MFC core and standard components
#include <afxext.h> // MFC extended functions
#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h> //Internet Explorer 4 common controls supported by MFC
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h> // Windows common controls supported by MFC
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <iostream>



// TODO: See here for other headers your program may need
