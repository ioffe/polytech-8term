#pragma once


#define WIDEN(s) ___DO_WIDEN(s)
#define ___DO_WIDEN(s) L##s

#define STRINGIZE(x) ___DO_STRINGIZE(x)
#define ___DO_STRINGIZE(x) #x

#define WSTRINGIZE(x) ___DO_WSTRINGIZE(x)
#define ___DO_WSTRINGIZE(x) L#x

#define JOIN(x, y) ___DO_JOIN(x, y)
#define ___DO_JOIN(x, y) ___DO_JOIN2(x, y)
#define ___DO_JOIN2(x, y) x##y

#define CALL_MEMFUN(obj, memfunptr) ((obj).*(memfunptr))
