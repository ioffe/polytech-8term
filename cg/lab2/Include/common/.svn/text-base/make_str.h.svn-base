#pragma once

#include <string>
#include <sstream>

namespace util
{
   template < class Char >
   struct make_str_t
   {
      typedef
         std::basic_string< Char, std::char_traits< Char >, std::allocator< Char > >
         string_t;

      typedef
         std::basic_ostringstream< Char, std::char_traits< Char >, std::allocator< Char > >
         ostringstream_t;

      template < class T >
      make_str_t & operator <<( T const & t )
      {
         oss_ << t;
         return *this;
      }

      operator string_t() const
      {
         return oss_.str();
      }

      operator Char const *() 
      {
         str_ = oss_.str();
         return str_.c_str();
      }

   private:
      ostringstream_t oss_;
      string_t str_;
   };

   typedef make_str_t< char >    make_str;
   typedef make_str_t< wchar_t > make_wstr;
}