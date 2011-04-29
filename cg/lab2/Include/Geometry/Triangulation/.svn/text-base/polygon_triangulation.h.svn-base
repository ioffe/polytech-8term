#pragma once

#include <queue>

#include "cgal_triangulation.h"
#include "Geometry/polygon_2.h"

namespace cg {
namespace triangulation {

   namespace details
   {
      template < class face_handle >
      void bfs( std::queue< face_handle > & q, bool flag )
      {
         while ( !q.empty() )
         {
            face_handle f = q.front();
            q.pop();

            for ( size_t i = 0; i < 3; ++i )
            {
               face_handle ff = f->neighbor( i );
               if ( !f->is_constrained( i ) && ff->info() != flag )
               {
                  ff->info() = flag;
                  q.push( ff );
               }
            }
         }
      }

      template < class trg_t > 
         void mark_polygon_inside( trg_t & trg, cg::polygon_2 const & poly )
      {
         typedef typename trg_t::face_handle face_handle;
         typedef typename trg_t::vertex_handle vertex_handle;
         typedef typename trg_t::edge_circulator edge_circulator;

         for ( typename trg_t::faces_iterator it = trg.faces_begin(); it != trg.faces_end(); ++it )
            it->info() = false;

         std::queue< trg_t::face_handle > queue;

         for ( size_t i = 0; i != poly.size(); ++i )
         {
            cg::contour_2 const & cnt = poly[i];
            for ( size_t j = 0; j != cnt.size(); ++j )
            {
               vertex_handle p = trg.insert( cnt[j] );
               vertex_handle q = trg.insert( cnt[ cg::next( j, cnt.size() ) ] );
               edge_circulator beg = trg.incident_edges( trg.insert( cnt[j] ) );
               edge_circulator cur = beg; 

               do
               {
                  if ( cur->first->vertex( cur->second ) == q )
                  {
                     if ( !cur->first->info() )
                     {
                        cur->first->info() = true;
                        queue.push( cur->first );
                     }

                     break;
                  }
               } while ( ++cur != beg );
            }
         }
         
         bfs( queue, true );

         queue.push( trg.incident_edges( trg.infinite_vertex() )->first );
         bfs( queue, false );
      }
   }

   template < class Traits >
      void triangulate( cg::polygon_2_t< Traits > const & poly, std::vector< cg::point_2 > & vertices, std::vector< cg::point_3i > & faces )
   {
      typedef cgal_triangulation< cgal_triangulation_traits< double, cg::Empty, bool > > trg_t;
      trg_t trg;
      add_polygon( trg, poly );
      details::mark_polygon_inside( trg, poly );

      struct face_filter { bool operator ()( trg_t::face_handle f ) { return f->info(); } };
      get_data( trg, face_filter(), vertices, faces );
   }

   template< class PointType >
      void triangulate( std::vector< PointType > const & pts, std::vector< size_t > const & indices,
                        std::vector< cg::point_3i > & faces )
   {
      std::vector< cg::point_2 > poly;
      for ( size_t i = 0; i < indices.size(); ++i )
         poly.push_back( pts[indices[i]] );

      typedef cgal_triangulation< cgal_triangulation_traits< double, cg::Empty, bool > > trg_t;
      trg_t trg;
      add_contour( trg, poly.begin(), poly.end() );
      details::mark_polygon_inside( trg, cg::polygon_2( poly.begin(), poly.end() ) );

      struct face_filter { bool operator ()( trg_t::face_handle f ) { return f->info(); } };
      details::indexate_vertices( trg, indices );
      get_faces( trg, face_filter(), faces );
   }

}}