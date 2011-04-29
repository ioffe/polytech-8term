#pragma once

#include <memory>
#include <sstream>

namespace std
{
   inline basic_ostream< wchar_t > & operator <<( basic_ostream< wchar_t > & oss, string const & str )
   {
      oss << wstring( str.begin(), str.end() );
      return oss;
   }
}

namespace printer
{
   template < class Printer, class Char = char >
      struct stream
   {
      stream( Printer printer, std::ios_base::fmtflags flags = 0 ) 
         : impl_( new stream_impl( printer, flags ) )
      {}

      template < class T >
         stream & operator <<( T const & x )
      {
         (*impl_) << x;
         return *this;
      }
            
      typedef
         std::basic_ostream< Char >
         ostream_type;

      stream & operator <<( ostream_type & (*func)( ostream_type & ) )
      {
         (*impl_) << func;
         return *this;
      }

   private:
      struct stream_impl
      {
      private:
         typedef
            std::basic_ostringstream< Char, std::char_traits< Char >, std::allocator< Char > >
            str_stream_type;

      public:
         stream_impl( Printer printer, std::ios_base::fmtflags flags )
            : printer_( printer )
         {
            stream_.setf( flags );
         }

         ~stream_impl( )
         { 
            printer_( stream_.str() );
         }

         template < class T >
            stream_impl & operator <<( T const & x )
         {
            stream_ << x;
            return *this;
         }

         stream_impl & operator <<( size_t x )
         {
            stream_ << unsigned int( x );
            return *this;
         }

         stream_impl & operator <<( str_stream_type & (&func)( str_stream_type & ) )
         {
            return func( stream_ );
         }

      private:
         stream_impl( stream_impl const & );

      private:
         str_stream_type   stream_;
         Printer           printer_;
      };

      std::auto_ptr< stream_impl > impl_;
   };
}
