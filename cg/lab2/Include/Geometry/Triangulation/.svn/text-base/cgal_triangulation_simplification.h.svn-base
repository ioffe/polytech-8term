#pragma once

namespace cg            {
namespace triangulation {

   template < class Traits, class OutIter, class IsUntouchable >
   void remove_tvertices(  cgal_triangulation< Traits > & trg, typename Traits::scalar_type const eps, OutIter out,
                           IsUntouchable & is_untouchable )
   {
      typedef cgal_triangulation< Traits >               trg_t;
      typedef typename trg_t::face_handle                face_t;
      typedef typename edge_f< trg_t >::type             edge_t;
      typedef std::map< edge_t, trg_t::vertex_handle >   t_vertices_t;
      t_vertices_t t_vertices;

      for ( trg_t::faces_iterator it = trg.faces_begin(); it != trg.faces_end(); ++it )
      {
         for ( size_t i = 0; i != 3; ++i )
         {
            trg_t::vertex_handle const v0 = it->vertex( i );
            trg_t::vertex_handle const v1 = it->vertex( it->cw( i ) ), v2 = it->vertex( it->ccw( i ) );
            if ( !trg.is_constrained( v1, v2 ) )
               continue;

            edge_t const edge = v1 < v2 ? std::make_pair( v1, v2 ) : std::make_pair( v2, v1 );
            if ( t_vertices.find( edge ) != t_vertices.end() )
               continue;

            typename trg_t::cg_segment_2 const seg( trg.construct( edge.first ), trg.construct( edge.second ) );
            typename trg_t::cg_point_2 const & p = trg.construct( v0 );

            if ( cg::le( cg::distance( p, seg ), eps ) )
            {
               t_vertices[ edge ] = it->vertex( i );
               break;
            }
         }
      }

      for ( t_vertices_t::const_iterator it = t_vertices.begin(); it != t_vertices.end(); ++it )
      {
         typename trg_t::vertex_handle vh[] = 
         {
            it->second,
            it->first.first,
            it->first.second
         };
         trg.remove_constraint( vh[1], vh[2] );
         trg.insert( vh[0], vh[1] );
         trg.insert( vh[0], vh[2] );
         if ( is_untouchable( vh[1], vh[2] ) )
         {
            is_untouchable.remove( vh[1], vh[2] );
            is_untouchable.insert( vh[0], vh[1] );
            is_untouchable.insert( vh[0], vh[1] );
         }
         *out++ = trg.construct( vh[0] );
      }
   }

   template < class Traits, class OutIter >
   void remove_tvertices(  cgal_triangulation< Traits > & trg, typename Traits::scalar_type const eps, OutIter out )
   {
      remove_tvertices( trg, eps, out, details::empty_property_t() );
   }

   template < class Traits >
   void remove_tvertices( cgal_triangulation< Traits > & trg, typename Traits::scalar_type const eps )
   {
      remove_tvertices( trg, eps, util::null_iterator() );
   }

   template < class Triangulation, class OutIter, class IsUntouchable >
   void remove_redundant_points_on_constraints( Triangulation & trg, typename Triangulation::scalar_type eps,
      OutIter out, IsUntouchable & is_untouchable )
   {      
      typedef Triangulation trg_t;

      typedef typename trg_t::vertex_handle  vertex_handle;
      typedef typename trg_t::vertex_type    vertex_type;
      typedef typename trg_t::scalar_type    scalar_type;

      typedef std::vector< vertex_handle >   vertex_handles;
      vertex_handles all_vertices;
      for ( trg_t::vertices_iterator it = trg.vertices_begin(); it != trg.vertices_end(); ++it )
         all_vertices.push_back( it );

      vertex_handles incident_vertices;
      for ( vertex_handles::const_iterator it = all_vertices.begin(); it != all_vertices.end(); ++it )
      {           
         vertex_handle vh = *it;

         incident_vertices.clear();

         typename trg_t::vertex_circulator vc_begin = trg.incident_vertices( vh );
         typename trg_t::vertex_circulator vc = vc_begin;
         do
         {
            if ( trg.is_constrained( vh, vc ) )
               incident_vertices.push_back( vc );
         } while ( ++vc != vc_begin );

         if ( incident_vertices.size() != 2 )
            continue;

         vertex_type const & p0 = trg.construct( vh );
         vertex_type const & p1 = trg.construct( incident_vertices[0] );
         vertex_type const & p2 = trg.construct( incident_vertices[1] );

         cg::segment_t< scalar_type, vertex_type::dimension >  seg( p1, p2 );
         scalar_type const dist_to_seg = cg::distance( p0, seg );

         if ( cg::le( fabs( dist_to_seg ), eps ) 
               && !is_untouchable( vh, incident_vertices[0] )
               && !is_untouchable( vh, incident_vertices[1] ) )
         {
            trg.remove_constraint( vh, incident_vertices[0] );
            trg.remove_constraint( vh, incident_vertices[1] );
            trg.delete_vertex( vh );
            insert( trg, incident_vertices[0], incident_vertices[1], util::null_iterator(), is_untouchable );
            *out++ = p0;
         }
      }
   }

   template < class Triangulation, class OutIter >
   void remove_redundant_points_on_constraints( Triangulation & trg, typename Triangulation::scalar_type eps,
      OutIter out )
   {
      remove_redundant_points_on_constraints( trg, eps, out, details::empty_property_t() );
   }

   template < class Triangulation >
   void remove_redundant_points_on_constraints( Triangulation & trg, typename Triangulation::scalar_type eps )
   {
      remove_redundant_points_on_constraints( trg, eps, util::null_iterator() );
   }

   template < class Triangulation, class SrcProperty, class DstProperty >
   void eliminate_dup_points(    Triangulation const & src, Triangulation & dst,
      typename Triangulation::scalar_type eps, SrcProperty const & src_property, DstProperty & dst_property )
   {
      typedef Triangulation                  trg_t;
      typedef typename trg_t::vertex_type    vertex_type;
      typedef typename trg_t::vertex_handle  vertex_handle;
      typedef typename trg_t::face_handle    face_handle;

      {
         cg::DuplicatePointsEliminator< vertex_type > eliminator( eps );

         for ( trg_t::edges_iterator it = src.edges_begin(); it != src.edges_end(); ++it )
         {
            face_handle const & face = it->first;
            int v = it->second;
            vertex_handle vh[2] = 
            {
               face->vertex( face->cw( v ) ),
               face->vertex( face->ccw( v ) )
            };
            if ( src.is_constrained( vh[0], vh[1] ) )
            {
               vertex_type v[2];
               for ( size_t i = 0; i != 2; ++i )
               {
                  v[i] = *eliminator.insert( src.construct( vh[i] ) ).first;
               }
               if ( v[0] != v[1] )
               {
                  if ( src_property( vh[0], vh[1] ) )
                  {
                     std::vector< edge_f< trg_t >::type > new_edges;
                     insert( dst, v[0], v[1], std::back_inserter( new_edges ), dst_property );
                     for ( size_t i = 0; i != new_edges.size(); ++i )
                        dst_property.insert( new_edges[i] );
                  }
                  else
                  {
                     dst.insert( v[0], v[1] );
                  }
               }
            }
         }
      }
      {
         cg::DuplicatePointsEliminator< vertex_type > eliminator( eps );
         std::vector< trg_t::cg_segment_2 > new_edges, new_prop;         
         for ( trg_t::vertices_iterator vIt = dst.vertices_begin(); vIt != dst.vertices_end(); ++vIt )
         {
            typename trg_t::vertex_circulator vc_begin = dst.incident_vertices( vIt );
            typename trg_t::vertex_circulator vc = vc_begin;
            do 
            {
               if (dst.is_constrained(vIt, vc))
               {
                  trg_t::vertex_type v1 = *eliminator.insert(dst.construct(vIt)).first;
                  trg_t::vertex_type v2 = *eliminator.insert(dst.construct(vc)).first;
                  if (v1 != v2)
                  {
                     new_edges.push_back( trg_t::cg_segment_2( v1, v2 ) );
                     if (dst_property(vIt, vc))
                     {
                        new_prop.push_back( trg_t::cg_segment_2( v1, v2 ) );
                        dst_property.remove(vIt, vc);
                     }
                  }
               }
            } while (++vc != vc_begin);
         }
         dst.clear();
         for (size_t i = 0; i != new_edges.size(); ++i)
         {
            dst.insert(new_edges[i].P0(), new_edges[i].P1());
         }
         for (size_t i = 0; i != new_prop.size(); ++i)
         {
            dst_property.insert(dst.insert(new_prop[i].P0()), dst.insert(new_prop[i].P1()));
         }
      }
   }

   template < class Triangulation >
   void eliminate_dup_points( Triangulation const & src, Triangulation & dst, typename Triangulation::scalar_type eps )
   {
      eliminate_dup_points( src, dst, eps, details::empty_property_t(), details::empty_property_t() );
   }

}}