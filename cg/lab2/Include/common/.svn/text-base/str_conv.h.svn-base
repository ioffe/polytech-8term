#pragma once

#include <locale>
#include <string>
#include <iostream>

namespace util
{
   inline std::string wide2ansi( std::wstring const & source, std::locale const & loc = std::locale::classic() )
   {
      std::ctype< wchar_t > const & ct = std::use_facet< std::ctype< wchar_t > >(loc);

      std::string result( source.size(), char() );

      std::string::iterator dest( result.begin() );
      std::wstring::const_iterator it( source.begin() );
      std::wstring::const_iterator end( source.end() );

      for ( ; it != end; ++it, ++dest )
         *dest = ct.narrow( *it );

      return result;
   }

   inline std::wstring ansi2wide( std::string const & source, std::locale const & loc = std::locale::classic() )
   {
      std::ctype< char > const & ct = std::use_facet< std::ctype< char > >(loc);

      std::wstring result( source.size(), wchar_t() );

      std::wstring::iterator dest( result.begin() );
      std::string::const_iterator it( source.begin() );
      std::string::const_iterator end( source.end() );

      for ( ; it != end; ++it, ++dest )
         *dest = ct.widen( *it );

      return result;
   }

   inline std::wstring variant2wide( _variant_t const & v )
   {
      return std::wstring( LPCWSTR( _bstr_t( v ) ) );
   }

   inline std::string variant2ansi( _variant_t const & v )
   {
      return std::string( LPCSTR( _bstr_t( v ) ) );
   }
}