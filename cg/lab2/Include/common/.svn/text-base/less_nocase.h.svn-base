#pragma once 

#include <string>
#include <comutil.h>


struct less_nocase 
{
   bool operator () ( std::string const & p1, std::string const & p2 ) const
   {
      return _stricmp ( p1.c_str(), p2.c_str() ) < 0 ;
   }

   bool operator () ( std::wstring const & p1, std::wstring const & p2 ) const
   {
      return _wcsicmp ( p1.c_str(), p2.c_str() ) < 0 ;
   }

   bool operator () ( _bstr_t const & p1, _bstr_t const & p2 ) const
   {
      return _wcsicmp ( p1, p2 ) < 0 ;
   }

   bool operator () ( LPCSTR p1, LPCSTR p2 ) const
   {
      return _stricmp ( p1, p2 ) < 0 ;
   }

   bool operator () ( LPCWSTR p1, LPCWSTR p2 ) const
   {
      return _wcsicmp ( p1, p2 ) < 0 ;
   }
} ; 
