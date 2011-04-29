#pragma once

namespace cg
{
   template< typename graph_type >
      struct GraphFragmentator
   {
      typedef     typename graph_type :: vertex_index_type            vertex_index_type;
      typedef     typename graph_type :: edge_index_type              edge_index_type;
      typedef     typename graph_type :: edge_index_type              edge_index_type;
      typedef     typename graph_type :: vertex_index_pair_type       edge_type;

   private:
      typedef     std::vector< vertex_index_type >                    vertex_indices_type;
      typedef     std::vector< vertex_indices_type >                  vertices_per_edge_type;

   public:
      GraphFragmentator( graph_type & graph )
         : graph_ ( graph )
         , vertices_per_edge_( graph.edges_count( ) )
      {
         std::for_each( vertices_per_edge_.begin(), vertices_per_edge_.end( ), 
            boost::bind( &vertex_indices_type::reserve, _1, 10 ) );
      }

      void add_vertex_on_edge( vertex_index_type vi, edge_index_type ei )
      {
         vertices_per_edge_.at( ei ).push_back( vi );
      }

   private:
      template< typename points_type >
         struct vertices_on_edge_less_type
      {
         typedef   typename points_type :: value_type   point_type;

         vertices_on_edge_less_type( vertex_index_type out,      vertex_index_type in, 
                                     graph_type const & graph,   points_type const & points )
            : graph_ ( graph )
            , points_( points )
            , dir_   ( points_.at( graph_.vertex( in  ).id( ) ) - points_.at( graph_.vertex( out ).id( ) ) )
         {}

         bool operator( )( vertex_index_type v1, vertex_index_type v2 ) const
         {
            point_type const & p1 = points_.at( graph_.vertex( v1 ).id( ) );
            point_type const & p2 = points_.at( graph_.vertex( v2 ).id( ) );
            return p1 * dir_ < p2 * dir_;
         }

      private:
         graph_type   const & graph_;
         points_type  const & points_;

      private:
         point_type dir_;
      };

   public:
      template< typename points_type >
         void operator( )( points_type const & points )
      {
         for( edge_index_type ei = 0, size = vertices_per_edge_.size( ); ei != size; ++ei )
         {
            vertex_indices_type & indices = vertices_per_edge_.at( ei );
            if( indices.empty( ) )
               continue;            

            edge_type const & edge = graph_.edge( ei );
            add_vertex_on_edge( edge.first,  ei );
            add_vertex_on_edge( edge.second, ei );
         
            std::sort( indices.begin( ), indices.end( ) );
            indices.erase( std::unique( indices.begin( ), indices.end( ) ), indices.end( ) );
            if( indices.size( ) == 2 )
               continue;

            graph_.remove_edge( ei );

            std::sort( indices.begin( ), indices.end( ), 
               vertices_on_edge_less_type< points_type >( edge.first, edge.second, graph_, points ) );
         
            for( size_t i = 1, size = indices.size( ); i != size; ++i )
               graph_.add_edge( indices.at( i - 1 ), indices.at( i ) );

         }
         graph_.erase_removed_edges( );
         graph_.kill_dupe_edges( );
      }

   private:
      graph_type  & graph_;

   private:
      vertices_per_edge_type vertices_per_edge_;
   };
}
