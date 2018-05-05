#pragma once

#include "resource.h"

#include <stdio.h>

#pragma warning(disable : 4305)
#pragma warning(disable : 4996)

#define ErrorMessage(x)	MessageBoxA(NULL, x, "ERROR", MB_ICONERROR)

#if defined(_DEBUG) || defined(DEBUG)

extern void TRACE(LPCSTR pszFormat, ...);

/* Debug */
#define TRACE0(x)				TRACE(x)
#define TRACE1(x, a)            TRACE(x, a)
#define TRACE2(x, a, b)         TRACE(x, a, b)
#define TRACE3(x, a, b, c)      TRACE(x, a, b, c)
#else
/* Release */
#define TRACE
#define TRACE0(x)
#define TRACE1(x, a)
#define TRACE2(x, a, b)
#define TRACE3(x, a, b, c)
#endif
