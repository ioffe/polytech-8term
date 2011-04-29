#pragma once 

#include "triangle_plane_intersection.h"
#include "Geometry/lerp.h"

namespace section_grid
{

   namespace settings
   {
      struct AllLines
      {
         static bool closed_lines_only( ) { return false; }
      };

      struct ClosedLinesOnly
      {
         static bool closed_lines_only( ) { return true; }
      };
   }

template <typename T, typename Settings = settings::ClosedLinesOnly > 
   class section
{
   typedef std::pair< size_t, size_t >             vertex_id;
   typedef std::set< vertex_id >                   vertices_type;
   typedef std::map< vertex_id, vertices_type >    graph_type;

   static vertex_id make_vertex_id( size_t a, size_t b )
   {
      return vertex_id( std::max( a, b ), std::min( a, b ) );
   }

   static bool valid( vertex_id vertex )
   {
      return vertex.first != -1 && vertex.second != -1; 
   }

   static double angle( point_2 const & v1, point_2 const & v2 )
   {
      double norm_prod = cg::sqrt( (v1*v1)*(v2*v2) );
      double sin_a = ( (-v1) ^ v2 ) / norm_prod;
      double cos_a = ( (-v1) * v2 ) / norm_prod;

      cos_a = cg::clamp(-1., 1., cos_a);

      double res = acos( cos_a );

      if ( sin_a * cos_a > 0 )
      {
         if ( sin_a < 0 || cos_a < 0 )
         {
            res += cg::pi;
         }
      }
      else
      {
         if ( sin_a < 0 || cos_a > 0 )
         {
            res = 2 * cg::pi - res;
         }
         else
            res = cg::pi - res;
      }

      return res;
   }

   point_3 interpolate_point( vertex_id idx, double level ) const
   {
      point_3 p0 = grid_.at_p3( grid_.num2pos( idx.first ) );
      point_3 p1 = grid_.at_p3( grid_.num2pos( idx.second ) );

      if ( p0.z > p1.z )
         std::swap( p0, p1 );

      return cg::Lerp<double, point_3>( p0.z, p1.z, p0, p1 )( level );
   }

   vertex_id find_leftmost( graph_type const & graph, double level ) const
   {
      point_2 min_p( std::numeric_limits<double>::max( ), std::numeric_limits<double>::max( ) );
      vertex_id idx(-1, -1);

      for ( graph_type::const_iterator it = graph.begin( ); it != graph.end( ); ++it )
      {
         point_3 point = interpolate_point( it->first, level );
         if ( point.x < min_p.x || point.x == min_p.x && point.y < min_p.y )
         {
            idx = it->first;
            min_p = point;            
         }
      }

      return idx;
   }

   vertex_id get_next_point( vertex_id idx, graph_type const & graph, double level, point_2 const & prev_vector, point_2 & vec ) const
   {
      point_2 p0 = interpolate_point( idx, level );
      double ang = std::numeric_limits< double >::max( );
      vertex_id res(-1, -1);

      graph_type::const_iterator git = graph.find( idx );
      Assert( git != graph.end( ) );

      for ( vertices_type::const_iterator it = git->second.begin( ); it != git->second.end( ); ++it )
      {
         point_2 p1 = interpolate_point( *it, level );

         double a = angle( prev_vector, p1 - p0 );
         if ( a < ang )
         {
            ang = a;
            res = *it;
            vec = p1 - p0;
         }
      }

      return res;
   }

private:
   const T& grid_;
public:

   section(const T& grid)
      : grid_(grid)
   {}

   //
   void getsection( double level, std::vector< std::vector< point_2 > > & segments )
   {
      segments.clear();

      graph_type graph;

      for (size_t i = 0; i < grid_.get_width() - 1; i++)
      {
         for (size_t j = 0; j < grid_.get_height() - 1; j++)
         {
            point_3 p00 = grid_.at_p3(point_2i(    i,     j));
            point_3 p10 = grid_.at_p3(point_2i(i + 1,     j));
            point_3 p01 = grid_.at_p3(point_2i(    i, j + 1));
            point_3 p11 = grid_.at_p3(point_2i(i + 1, j + 1));

            cg::triangle_3 tr1(p00, p10, p11);
            cg::triangle_3 tr2(p00, p11, p01);

            size_t side_1;
            size_t side_2;
            if (sections::gen::triangle_section(tr1, level, side_1, side_2))
            {
               size_t indices[3] = 
               { 
                  grid_.pos2num( point_2i( i, j ) ),        // 00
                  grid_.pos2num( point_2i( i + 1, j ) ),    // 10
                  grid_.pos2num( point_2i( i + 1, j + 1 ) ) // 11
               };

               vertex_id v1 = make_vertex_id( indices[( side_1 + 1 ) % 3], indices[( side_1 + 2 ) % 3] );
               vertex_id v2 = make_vertex_id( indices[( side_2 + 1 ) % 3], indices[( side_2 + 2 ) % 3] );

               graph[v2].insert( v1 );
            }

            if (sections::gen::triangle_section(tr2, level, side_1, side_2))
            {
               size_t indices[3] = 
               { 
                  grid_.pos2num( point_2i( i, j ) ),         // 00
                  grid_.pos2num( point_2i( i + 1, j + 1 ) ), // 11
                  grid_.pos2num( point_2i( i, j + 1 ) )      // 01
               };

               vertex_id v1 = make_vertex_id( indices[( side_1 + 1 ) % 3], indices[( side_1 + 2 ) % 3] );
               vertex_id v2 = make_vertex_id( indices[( side_2 + 1 ) % 3], indices[( side_2 + 2 ) % 3] );

               graph[v2].insert( v1 );
            }
         }
      }

      vertex_id current_point( -1, -1 );
      while ( valid( current_point = find_leftmost( graph, level ) ) )
      {
         segments.push_back( std::vector< cg::point_2 >( ) );
         std::vector< cg::point_2 > & current_polyline = segments.back( );

         point_2 prev_vec( 1, 0 );
         vertex_id start_point = current_point;

         while ( graph.find( current_point ) != graph.end( ) )
         {
            point_2 vec;

            current_polyline.push_back( interpolate_point( current_point, level ) );

            vertex_id next_point = get_next_point( current_point, graph, level, prev_vec, vec );

            graph[current_point].erase( next_point );
            if ( graph[current_point].empty( ) )
               graph.erase( current_point );

            current_point = next_point;
            prev_vec = vec;
         }

         if ( start_point == current_point )
            current_polyline.push_back( current_polyline.front( ) );
         else
         {
            if ( Settings::closed_lines_only( ) )
               segments.pop_back( );
         }
      }

      return;
   }
};

} // end of namespace