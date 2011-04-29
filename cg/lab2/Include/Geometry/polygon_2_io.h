#pragma once

#include <ios>
#include <cstdio>
#include <iterator>
#include <string>
#include <sstream>
#include <fstream>

#include "geometry/polygon_2_fwd.h"
#include "geometry/point_io.h"
#include "common/io_utils.h"

namespace cg
{
   template < class Traits, class Char >
      std::basic_ostream< Char > & operator <<(std::basic_ostream< Char > &out, polygon_2_t< Traits > const & p )
   {
      if ( !p.size() )
      {
         out << "empty\n";
         return out;
      }

      out.precision( 32 );

      for ( size_t i = 0; i < p.size(); ++i )
      {
         for (size_t j = 0; j != p[i].size(); ++j)
            out << p[i][j];
         out << std::endl;
      }

      out << std::endl;

      return out;
   }

   template < class Traits, class Char >
      std::basic_ostream< Char > & operator <<(std::basic_ostream< Char > &out, std::vector< polygon_2_t< Traits > > const & v )
   {   
      for ( size_t i = 0; i < v.size(); ++i )
         out << v[i];

      return out;
   }

   template < class Traits, class Char >
      std::basic_istream< Char > & operator >>( std::basic_istream< Char > &in, polygon_2_t< Traits > & p )
   {   
      p.clear();

      std::string s = util::first_nonempty_line( in );

      for ( ; in && !s.empty(); )
      {
         if ( s != "empty" )
         {
            std::istringstream iss( s );
            p.add_contour( std::istream_iterator< cg::point_2 >( iss ), std::istream_iterator< cg::point_2 >() );
         }

         getline( in, s );
         boost::trim( s );
      }
     
      return in;
   }

   template < class Traits, class Char >
      std::basic_istream< Char > & operator >>( std::basic_istream< Char > &in, std::vector< polygon_2_t< Traits > > & v )
   {
      v.clear();

      while ( in )
      {
         util::skip_spaces( in );
         if ( in )
         {
            v.push_back( polygon_2_t< Traits >() );
            in >> v.back();
         }
      }
      
      return in;
   }

   // Writes polygon into temporary file.
   // Returns path to created file, or empty string on failure.
   template < class Traits >
   std::string dump_polygon( polygon_2_t< Traits > const & poly )
   {
      // TODO: Using not standard _tempnam().

      std::string fileName(_tempnam(NULL, "polygon_"));
      if (!fileName.empty())
      {
         std::ofstream ofs(fileName.c_str());
         if (ofs)
            ofs << poly;
         else
            fileName.clear();
      }

      return fileName;
   }
}
