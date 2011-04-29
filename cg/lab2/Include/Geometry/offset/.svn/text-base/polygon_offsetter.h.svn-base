#pragma once

#include <boost/optional.hpp>

#include "geometry\segments_intersections.h"
#include "geometry\clockwise.h"

#include "graph.h"
#include "graph_fragmentator.h"
#include "equidistant_point.h"
#include "dist_to_seg.h"
#include "dcel.h"
#include "points_map.h"

namespace cg {
namespace polyoffset
{
   template< typename point_type >
      struct polygon_offsetter
   {
      typedef     point_type                                               point_type;
      typedef     std::vector< point_type >                                points_type;

      typedef     points_type                                              point_contour_type;
      typedef     std::vector< points_type >                               point_contours_type;

      typedef     typename point_contour_type::iterator                    points_iterator;
      typedef     typename point_contours_type::iterator                   contours_iterator;

      typedef     size_t                                                   point_index_type;
      typedef     std::vector< point_index_type >                          point_indices_type;

      typedef     typename point_type :: scalar_type                       scalar_type;

      typedef     std::vector< point_index_type >                          contour_type;
      typedef     std::vector< contour_type >                              contours_type;

   private:
      struct vertex_data_type
      {
         vertex_data_type( point_index_type id )
            : id_( id )
         {}

         vertex_data_type( point_index_type id, point_index_type contour_id )
            : id_( id )
            , contour_point_( contour_id )
         {}

         point_index_type id( ) const { return id_; }

         bool              has_contour_point( ) const { return  contour_point_; }
         point_index_type  contour_point( )     const { return *contour_point_; }

      private:
         point_index_type id_;
         boost::optional< point_index_type > contour_point_;
      };

   private:
      typedef     Graph< vertex_data_type >                                graph_type;

   private:
      typedef     typename graph_type :: vertex_index_type                 vertex_index_type;
      typedef     std::vector< vertex_index_type >                         vertex_indices_type;
      typedef     typename graph_type :: edge_index_type                   edge_index_type;
      typedef     typename graph_type :: vertex_index_pair_type            edge_type;              
      
      typedef     PointsMap< point_type, vertex_index_type >               points_map_type;

   public:
      class point_iterator
      {
         typedef polygon_offsetter <point_type> Offsetter;
         Offsetter const * offs;
         contour_type::const_iterator iter;
      public:
         point_iterator(contour_type::const_iterator & it, Offsetter const * offsetter)
            : iter(it)
            , offs(offsetter)
         { }

         point_iterator    & operator ++ ()        { iter++; return *this; }
         point_type const  & operator *  ()  const { return offs->point(*iter); }

         int operator -              (point_iterator const & ofss)       { return iter - ofss.iter; }
         point_iterator & operator = (point_iterator const & ofss)       { iter = offs.iter; return *this; }
         bool operator ==            (point_iterator const & ofss) const { return ofss.iter == iter; }
         bool operator !=            (point_iterator const & ofss) const { return ofss.iter != iter; }
      };

   public:
      template< typename FwdIter >
         polygon_offsetter( FwdIter p, FwdIter q, scalar_type eps, scalar_type min_angle = cg::pi / 16)
         : contour_( p, q )
         , eps_ ( eps )
         , min_angle_( min_angle )
      {}

   private:
      struct is_contour_invalid
      {
         is_contour_invalid( points_type const & points, scalar_type d )
            : points_( points )
            , d_ ( d )
         {}

         bool operator( )( contour_type const & c ) const
         {
            points_type c_points( 100 );

            c_points.clear( );         
            for( contour_type :: const_iterator it = c.begin( ); it != c.end( ); ++it )
               c_points.push_back( points_.at( *it ) );

            return  d_ > 0 &&  cg::clockwise_ordered( c_points.begin( ), c_points.end( ) ) 
               || d_ < 0 && !cg::clockwise_ordered( c_points.begin( ), c_points.end( ) );         
         }
      private:
         points_type const & points_;
         scalar_type d_;
      };

   private:
      void del_fl( points_type & cont )
      {
         if ( cont.size( ) < 3 )
            return;
         size_t cur = 0;
         for ( points_type::iterator it = cont.begin( ); it != cont.end( ); )
         {
            size_t const size = cont.size( );
            
            const size_t       prev     = ( ( size - 1 ) + cur ) % size;
            const size_t       next     = ( size + 1 + cur) % size;
            point_type const & prev_p   = cont.at( prev );
            point_type const & cur_p    = *it;
            point_type const & next_p   = cont.at( next );
            
            if ( cg::norm_sqr( prev_p - next_p ) < 1e2 * eps_ * eps_ )
            {
               it = cont.erase( it );           
               continue;
            }
            else
            {
               const point_type seg01 ( next_p - cur_p );
               const point_type seg12 ( prev_p - cur_p );
               const double edge_len = cg::distance_sqr( prev_p, cur_p );
               const double edge_len2 = cg::distance_sqr( next_p, cur_p );
         
               const double alpha = cg::angle( seg01, seg12 );

               if ( cg::eq( prev_p, next_p ) )
               {
                  it = cont.erase( it );
                  cur++;              
                  continue;
               }
               if ( alpha < cg::grad2rad( 20.0 ) || alpha > 2 * cg::pi - cg::grad2rad( 20.0 ) )
               {
                  it = cont.erase( it );
                  continue;
               }
            }
            ++cur;
            ++it;
         }
      }
      
      void del_fl_s( point_contours_type & p_contours )
      {
         for ( point_contours_type::iterator it = p_contours.begin( ); it != p_contours.end( ); )
         {
            del_fl( *it );
            if ( it->size( ) < 3 )
               it = p_contours.erase( it );
            else
               ++it;
         }
      }

      void set_point_contours( point_contours_type & p_contours )
      {
         static points_type c( 400 );

         int i = 0;         
         for( contours_type::const_iterator it = contours_.begin( ); it != contours_.end( ); ++it )
         {  
            c.clear( );
            for( contours_type::value_type::const_iterator i = it->begin( ); i != it->end( ); ++i )
               c.push_back( point( *i ) );

            p_contours.push_back( c );
         }
      }

   public:

      contours_iterator begin( )
      {
         return p_contours_.begin( );
      }

      contours_iterator end( )
      {
         return p_contours_.end( );
      }

      point_contours_type const & operator( )( scalar_type d )
      {
         if( points_map_.get( ) != NULL )
            clear( );      
         
         init_points_map( d );
         add_inner_contour( d );
         graph_.kill_dupe_edges( );
         process_intersections( );
         remove_vertices_by_dist( d );
         DCEL< graph_type, points_type > dcel( graph_, points_ ); 
         dcel( contours_ ); 

         contours_.erase( std::remove_if( contours_.begin( ), contours_.end( ), is_contour_invalid( points_, d ) ),
                           contours_.end( ) );
         
         set_point_contours( p_contours_ );
         del_fl_s( p_contours_ );

         return p_contours_;     
      }

      contours_type const &  contours( ) const { return contours_; }

      point_type const & point( vertex_index_type vi )  const
      { 
         return points_.at( graph_.vertex( vi ).id( ) ); 
      }

      contours_type::size_type nContours() const 
      { 
         return contours_.size();
      }

      point_iterator c_begin(contours_type::size_type cnt) const
      {
         return point_iterator(contours_[cnt].begin(), this);
      }

      point_iterator c_end(contours_type::size_type cnt) const
      {
         return point_iterator(contours_[cnt].end(), this);
      }

      point_index_type const & point_ancestor( vertex_index_type vi )  const
      { 
         return points_.at( graph_.vertex( vi ).contour_id( ) ); 
      }

      graph_type  const & graph( )  const { return graph_;  }
      points_type const & points( ) const { return points_; }

   private:

      void init_points_map( scalar_type d )
      {
         cg::rectangle_2 rect = cg::rectangle_2::bounding(contour_.begin(), contour_.end()) ;

         rect.inflate( 2 * abs( d ) );

         const int size = ceil( cg::sqrt( ( double ) contour_.size( ) ) );
         Assert( size >= 1 );
         
         point_type const diag( rect.XY( ) - rect.xy( ) );
         const double unit = std::max( diag.x, diag.y ) / size;
         const point_2i ext( ceil( diag / unit ) );
         cg::aa_transform const tform( rect.xy( ), point_2( unit, unit ) );

         points_map_.reset( new points_map_type( tform, ext, eps_ ) );
      }        

      void  filter_contour( points_type & cont )
      {
         size_t const size = cont.size( );      
         bool del;
         do {

            del = false;
            size_t cur = 0;
            for ( points_type::iterator it = cont.begin( ); it != cont.end( ); ++it )
            {
               size_t const size = cont.size( );
               if ( size < 3 )
                  return;

               const size_t       prev     = ( ( size - 1 ) + cur ) % size;
               const size_t       next     = ( size + 1 + cur) % size;
               point_type const & prev_p   = cont.at( prev );
               point_type const & cur_p    = *it;
               point_type const & next_p   = cont.at( next );
               
               if ( cg::norm_sqr( prev_p - next_p ) < 1 )
               {
                  it = cont.erase( it );
                  del = true;            
               }
               else 
               {
                  const point_type seg01 ( next_p - cur_p );
                  const point_type seg12 ( prev_p - cur_p );
                  const edge_len = cg::distance_sqr( prev_p, cur_p );
                  const edge_len2 = cg::distance_sqr( next_p, cur_p );
         
                  const double alpha = cg::angle( seg01, seg12 );

                  if ( cg::eq( prev_p, next_p ) )
                  {
                     it = cont.erase( it, it + 1 );
                     cur++;
                     del = true;
                     continue;
                  }

                  if ( alpha > 2 * cg::pi - cg::grad2rad( min_angle_ ) )
                  {
                     it = cont.erase( it );
                     del = true;
                  }
               }
               ++cur;
            }
         } while ( del );
      }

      // add inner ( offsetted contour. It has intersections, bridges, leafs )
      void add_inner_contour( scalar_type d )
      {
         const size_t contour_size = contour_.size( );
         vertex_indices_type all_points;
         
         for ( point_index_type cur = 0; cur != contour_size; ++cur )
         {
            const size_t       prev     = ( ( contour_size - 1 ) + cur ) % contour_size;
            const size_t       next     = ( cur + 1 ) % contour_size;
            point_type const & prev_p   = contour_.at( prev );
            point_type const & cur_p    = contour_.at( cur  );
            point_type const & next_p   = contour_.at( next );

            const point_type eq_point = equidistant_point( prev_p, cur_p, next_p, d ) ;

            Assert( cg::norm( eq_point - cur_p ) > abs( d ) - eps_ );

            const point_type seg01 ( next_p - cur_p );
            const point_type seg12 ( prev_p - cur_p );
      
            const double alpha = cg::angle( seg01, seg12 );
            
            if ( ( alpha > cg::pi + min_angle_ && d >= 0 ) || ( alpha < cg::pi - min_angle_ && d <= 0 ) )
            {  
               const scalar_type abs_d       = abs( d );
               const point_type diff    = eq_point - cur_p;
               Assert( cg::eq( cg::norm( diff ), abs_d, eps_ )  );
               const point_type l_perp  = cg::normalized_safe( point_type( -diff.y, diff.x ) );
               const double k           = tan( ( cg::pi - alpha ) / 4 ) * abs_d;

               all_points.push_back( add_vertex( eq_point - l_perp * k, &cur ) );            
               all_points.push_back( add_vertex( eq_point + l_perp * k, &cur ) );
            }
            else if ( ( alpha > cg::pi && d >= 0 ) || ( alpha < cg::pi && d <= 0 ) )
            {
               const double     abs_d       = abs( d );
               const double     beta        = alpha / 2 - cg::pi / 2;
               const point_type dir_ort     = cg::normalized_safe( eq_point - cur_p );
               const double     offset_dist = abs_d / cos( beta );
               all_points.push_back( add_vertex( cur_p + dir_ort * offset_dist, &cur ) );
            }
            else 
            {
               all_points.push_back( add_vertex( eq_point, &cur ) );
            }
         }

         const size_t size = all_points.size( );
      
         for ( point_index_type cur = 0; cur != size; ++cur )
         {
            const size_t next = ( cur + 1 ) % size;
            
            const vertex_index_type cur_out = all_points.at( cur );
            const vertex_index_type cur_in  = all_points.at( next );
            
            if ( cur_in == cur_out )
               continue; 

            graph_.add_edge( cur_out, cur_in );
         }
      }

   private:
      typedef  size_t                                                         segment_id_type;
      enum segment_qual_type { EDGE, BISECTOR, CONTOUR };

      struct segment_type
         : cg::segment_2
      {
         typedef  cg::segment_2                                                  seg_type;
         typedef  cg::point_2                                                    seg_point_type;

         segment_type( point_type const & p1, point_type const & p2, segment_qual_type qual, segment_id_type id )
               : cg::segment_2( seg_point_type( p1 ), seg_point_type( p2 ) )
               , qual_( qual )
               , id_ ( id )
         {}

         segment_qual_type qual( )          const { return qual_; }
         segment_id_type   id( )            const { return id_; }      

         segment_qual_type qual_;

      private:
         segment_id_type id_;
      };

      typedef    std::vector< segment_type >                                     segments_type;

      void process_intersections( )
      {
         typedef  GraphFragmentator< graph_type >                                               fragmentator_type;
         typedef  cg::SegmentsIntersections< segment_type, segments_type >                      intrs_type;
         
         segments_type segments;

         add_edges( segments );
         add_contour( segments );
         add_bisectors( segments );

         const intrs_type intrs( segments, eps_ );
         
         fragmentator_type fragmentator( graph_ );

         for( intrs_type :: const_iterator it = intrs.begin( ); it != intrs.end( ); ++it )
         {
            segment_type const & s1 = segments.at( it->id1( ) );
            segment_type const & s2 = segments.at( it->id2( ) );
            if( s1.qual( ) == EDGE && s2.qual( ) == EDGE )
            {
               const vertex_index_type vid = add_vertex( point_type( it->p( ) ) );
               fragmentator.add_vertex_on_edge( vid, ( edge_index_type ) s1.id( ) );
               fragmentator.add_vertex_on_edge( vid, ( edge_index_type ) s2.id( ) );
            }
         }
         fragmentator( points_ );
      }

   private:
      void add_edges( segments_type & segments ) const
      {
         const edge_index_type size = graph_.edges_count( );
         for( edge_index_type ei = 0; ei != size; ++ei )
         {
            if( graph_.is_edge_removed( ei ) )
               continue;

            edge_type const & edge = graph_.edge( ei );
            point_type const & p1 = points_.at( graph_.vertex( edge.first  ).id( ) );
            point_type const & p2 = points_.at( graph_.vertex( edge.second ).id( ) );
            segments.push_back( segment_type( p1, p2, EDGE, ( segment_id_type ) ei ) );
         }
      }

      void add_contour( segments_type & segments ) const
      {
         const point_index_type size = contour_.size( );
         for( point_index_type i = 0; i != size; ++i )
            segments.push_back( segment_type( contour_.at( i ), contour_.at( ( i + 1 ) % size ), 
                                             CONTOUR, ( segment_id_type ) i ) );
      }

      void add_bisectors( segments_type & segments ) const
      {
         const vertex_index_type size = graph_.vertices_count( );
         for( vertex_index_type i = 0; i != size; ++i )
         {
            point_type const & p1 = contour_.at( graph_.vertex( i ).contour_point( ) );
            point_type const & p2 = points_ .at( graph_.vertex( i ).id( ) );
            segments.push_back( segment_type( p1, p2, BISECTOR, ( segment_id_type ) i ) );
         }     
      }

   private:
      vertex_index_type add_vertex( point_type const & p, point_index_type * id = NULL )
      {
         std::pair< vertex_index_type, bool > const
            res = points_map_->insert( p, points_.size( ) );
     
         if( res.second )
         {
            if( id ) 
               graph_.add_vertex( vertex_data_type( res.first, *id ) );
            else
               graph_.add_vertex( vertex_data_type( res.first ) );
            points_.push_back( p );
         }
         return res.first;
      }

   private:
      void remove_vertices_by_dist( scalar_type d )
      {  
         segments_type segments;
         add_contour( segments );
         const scalar_type abs_d = cg::abs( d );
         cg::RasterizedSegments< segments_type > rast_segments( segments );

         for( vertex_index_type i = 0, size = graph_.vertices_count( ); i != size; ++i )
         {
            point_type const & p = points_.at( graph_.vertex( i ).id( ) );
            const point_type dp( abs_d, abs_d );

            cg::rectangle_2 const rect( p - dp, p + dp );
            DistToSegLessThan< segments_type > proc( segments, p, abs_d - 2 * eps_ );
             
            cg::visit( rast_segments.grid( ), rect, proc );
            if( proc ) 
               graph_.remove_vertex( i );
         }
      }

   private:
      void clear( )
      {
         points_.clear( );
         contours_.clear( );
         graph_.clear( );
      }
    
   // DATA
   private:
      // Исходный контур
      const points_type contour_;

      // Точки графа
      points_type points_;  

   private:
      contours_type        contours_;
      point_contours_type  p_contours_;

   private:
      graph_type graph_;

   private:
      std::auto_ptr< points_map_type > points_map_;

      // Epsilon для сравнения
   private:
      const scalar_type eps_;
      const scalar_type min_angle_;
   };
}}
