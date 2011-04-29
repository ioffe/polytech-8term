#pragma once

#include <stdio.h>

/******************************************************************************/
// Функции strtrunc и wcstrunc
// Удаляют набор символов bad_chars в строке str
/******************************************************************************/
inline void strtrunc ( LPSTR str, LPCSTR bad_chars )
{
  LPSTR p1 = str;
  LPSTR p2 = str+strlen(str);

  while ( p1 != p2 )
  {
    if ( strchr ( bad_chars, *p1 ) == 0 )
    {
      p1++;
    }
    else
    {
      memmove ( p1, p1+1, p2-p1-1 );
      
      p2--;
    }
  }
  
  *p2 = '\0';
}

inline void strchange ( LPSTR str, LPCSTR bad_chars, char s )
{
  LPSTR p1 = str;
  LPSTR p2 = str+strlen(str);

  while ( p1 != p2 )
  {
    if ( strchr ( bad_chars, *p1 ) != 0 )
      *p1 = s;
    
    p1++;
  }    
}

inline void wcstrunc ( LPWSTR str, LPCWSTR bad_chars )
{
  LPWSTR p1 = str;
  LPWSTR p2 = str+wcslen(str);

  while ( p1 != p2 )
  {
    if ( wcschr ( bad_chars, *p1 ) == 0 )
    {
      p1++;
    }
    else
    {
      memmove ( p1, p1+1, (p2-p1-1)*sizeof(wchar_t) );
      
      p2--;
    }
  }
  
  *p2 = L'\0';
}

inline void wcstruncex ( LPWSTR str, LPCWSTR bad_chars )
{
  LPWSTR p1 = str;
  LPWSTR p2 = str+wcslen(str);

  bool mark = *p1 == L'\"';
  
  while ( p1 != p2 )
  {
    if ( wcschr ( bad_chars, *p1 ) == 0 || mark )
    {
      p1++;
    }
    else
    {
      memmove ( p1, p1+1, (p2-p1-1)*sizeof(wchar_t) );
      
      p2--;
    }

    if ( *p1 == L'\"' )
      mark = mark == true ? false : true;
  }
  
  *p2 = L'\0';
}

inline wchar_t * wcschrex( wchar_t * str, wchar_t symb )
{
  LPWSTR p1 = str;
  LPWSTR p2 = str+wcslen(str);

  bool mark = *p1 == L'\"';
  
  while ( p1 != p2 )
  {
    if ( *p1 == symb && !mark )
      return p1;

    p1++;

    if ( *p1 == L'\"' )
      mark = mark == true ? false : true;
  }
  
  return 0;
}

inline _bstr_t ExpandString ( LPCSTR str ) 
{
  char buffer[10240] ; 
  ExpandEnvironmentStringsA ( str, buffer, sizeof(buffer) - 1 ) ; 
  return buffer ; 
}  

inline _bstr_t ExpandString ( LPCWSTR str ) 
{
  wchar_t buffer[10240] ; 
  ExpandEnvironmentStringsW ( str, buffer, sizeof(buffer)/sizeof(wchar_t) - 1 ) ; 
  return buffer ; 
}  

inline _bstr_t FormatString ( LPCWSTR lpszFormat, ... ) 
{
   wchar_t buf[1024] ;
   va_list argptr;
   va_start(argptr, lpszFormat);
   int ret = _vsnwprintf ( buf, 1024, lpszFormat, argptr) ;
   if ( ret == -1 )
      buf[1023] = 0 ;
   va_end(argptr);

   return buf ; 
}   

