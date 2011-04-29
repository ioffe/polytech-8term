#pragma once

#include <sstream>

#include "common/indexer.h"
#include "common/io_utils.h"

#include "Geometry/point_io.h"
#include "Geometry/Triangulation/cgal_triangulation.h"

namespace cg            {
namespace triangulation {

   namespace details
   {
      template < class Char, class Triangulation>
      std::basic_ostream< Char > & write( std::basic_ostream< Char > & out, Triangulation const & trg )
      {
         typedef Triangulation trg_t;

         out.precision( 32 );

         util::indexer< typename trg_t::vertex_handle > indexer;
         for ( typename trg_t::vertices_iterator it = trg.vertices_begin(); it != trg.vertices_end(); ++it )
         {
            indexer( it );
            out << trg.construct( it ) << '\n';
         }

         for ( typename trg_t::vertices_iterator it = trg.vertices_begin(); it != trg.vertices_end(); ++it )
         {
            int const a = indexer( it );

            typename trg_t::edge_circulator ec = trg.incident_edges( it ), ec_end = ec;
            std::vector< int > edges;
            do 
            {
               typename trg_t::vertex_handle p = ec->first->vertex( ec->first->cw( ec->second ) );
               typename trg_t::vertex_handle q = ec->first->vertex( ec->first->ccw( ec->second ) );

               int const b = indexer( typename trg_t::vertex_handle( it ) == p ? q : p );
               if ( a < b )
                  edges.push_back( b );
            } while ( ++ec != ec_end );

            std::sort( edges.begin(), edges.end() );
            for ( size_t i = 0; i < edges.size(); ++i )
               out << a << ' ' << edges[i] << '\n';
         }

         return out;
      }

      template < class Char, class Triangulation >
      std::basic_istream< Char > & read( std::basic_istream< Char > & in, Triangulation & trg )
      {
         typedef Triangulation trg_t;

         trg.clear();

         std::vector< typename trg_t::vertex_handle > handles;

         std::string s = util::first_nonempty_line( in );

         while ( in && !s.empty() )
         {
            std::istringstream iss( s );
            typename trg_t::vertex_type v;

            if ( iss >> v )
               handles.push_back( trg.insert( v ) );
            else
            {
               int p, q;
               if ( std::istringstream( s ) >> p >> q )
                  trg.insert( handles[p], handles[q] );
            }

            getline( in, s );
            boost::trim( s );
         }

         return in;
      }
   }

   template < class Char, class Traits >
   std::basic_ostream< Char > & operator <<( std::basic_ostream< Char > & out,
                                             cg::triangulation::cgal_triangulation< Traits > const & trg )
   {
      return details::write( out, trg );
   }

   template < class Char, class Traits >
   std::basic_istream< Char > & operator >>( std::basic_istream< Char > & in,
                                             cg::triangulation::cgal_triangulation< Traits > & trg )
   {
      return details::read( in, trg );
   }

   template < class Char, class Traits >
   std::basic_ostream< Char > & operator <<( std::basic_ostream< Char > & out,
                                             cg::triangulation::cgal_triangulation_with_height< Traits > const & trg )
   {
      return details::write( out, trg );
   }

   template < class Char, class Traits >
   std::basic_istream< Char > & operator >>( std::basic_istream< Char > & in,
                                             cg::triangulation::cgal_triangulation_with_height< Traits > & trg )
   {
      return details::read( in, trg );
   }

}}