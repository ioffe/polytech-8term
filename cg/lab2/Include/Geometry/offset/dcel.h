#pragma once

#include <stack>
#include <boost/optional.hpp>

// DCEL. Purpose: pick out all contours placed in graph structure
namespace cg {
namespace polyoffset
{
   template < typename graph_type, typename points_type >
   struct DCEL
   {
      typedef graph_type                                    graph_type;

      typedef points_type                                   points_type;
      typedef typename points_type :: value_type            point_type;

      typedef typename graph_type::edge_index_type          edge_index_type;
      typedef std::vector< edge_index_type >                edge_indices_type;

      typedef typename graph_type::vertex_index_type        vertex_index_type;
      typedef typename graph_type::vertex_index_pair_type   vertex_index_pair_type;
      
   private:
      struct edge_type
      {
         edge_type( vertex_index_type out, vertex_index_type in )
            : out_( out )
            , in_ ( in  )
         {}

         vertex_index_type in( )  const { return in_;  }
         vertex_index_type out( ) const { return out_; } 

         edge_index_type next( )  const { return *next_; }
         edge_index_type twin( )  const { return *twin_; }

         bool has_next( ) const { return next_; }
         bool has_twin( ) const { return twin_; }

         void set_next ( edge_index_type next ) { Assert( !next_ ); next_ = next; }
         void set_twin ( edge_index_type twin ) { Assert( !twin_ ); twin_ = twin; }
         void set_alpha( double alpha )         { alpha_ = alpha; }

         double alpha( vertex_index_type vi ) const 
         { 
            if( vi == out_ )
               return alpha_; 

            Assert( vi == in_ );
            return cg::norm_2pi( alpha_ - cg::pi );
         }

         vertex_index_type out( ) { return out_; }
         vertex_index_type in ( ) { return in_;  }

      private:
         vertex_index_type                  out_;
         vertex_index_type                  in_;
         boost::optional< edge_index_type > next_;
         boost::optional< edge_index_type > twin_;
         double                             alpha_;
      };

   private:
      typedef     std::vector< edge_type >                  edges_type;

   public:
      DCEL( graph_type const & graph, points_type const & points )
      {
         const edge_index_type e_count = graph.edges_count( );
         for( edge_index_type i = 0; i != e_count; ++i )
         {
            if( graph.is_edge_removed( i ) )
               continue;

            const vertex_index_pair_type  p = graph.edge( i );
            
            edge_index_type const id1 = edges_.size( );
            edges_.push_back( edge_type( p.first,  p.second ) );

            edge_index_type const id2 = edges_.size( );
            edges_.push_back( edge_type( p.second, p.first  ) );

            edges_.at( id1 ).set_twin( id2 );
            edges_.at( id2 ).set_twin( id1 );
         }

         calc_alphas( graph, points );
         link_edges( graph );
      }

   private:
      typedef  std::stack< edge_index_type >    stack_type;
      typedef  std::vector< bool >              bool_vector_type;

      // pick out contour placed in stack. ( the first edge is a twin of the last one ) 
      template < typename contours_type >
         void extract_contour( stack_type & st, contours_type & contours, bool lines, const edge_index_type * last = 0 ) const 
      {
         typedef     contours_type :: value_type     contour_type;
         contour_type * p_cont = 0;

         while( !st.empty( ) )
         {
            edge_index_type top = st.top( );
            st.pop( );

            if( last != 0 && top == *last )
               break;

            if( p_cont == 0 )
            {
               contours.push_back( contour_type( ) );
               p_cont = &contours.back( );
            }

            p_cont->push_back( edges_.at( top ).in( ) );
         }

         if( p_cont != 0 && ( p_cont->size( ) < (lines ? 2U : 3U) ) )
            contours.pop_back( );
      }
      
   public:
      // pick out all contours placed in graph
      template< typename contours_type >
         void operator( )( contours_type & contours, bool lines = false ) const
      {
         const edge_index_type edges_size = edges_.size( );

         bool_vector_type is_used ( edges_size, false );
         bool_vector_type cur_used( edges_size, false );
         
         typedef     contours_type :: value_type     contour_type;
         typedef     contours_type :: iterator       contour_interator;
         
         for ( edge_index_type e = 0; e != edges_size; ++e )
         {
            if ( is_used.at( e ) )
               continue;

            std::stack< edge_index_type > st;
            cur_used = std::vector< bool >( edges_size, false );
            
            edge_index_type cur_edge = e;
            while ( !is_used.at( cur_edge ) && !cur_used.at( cur_edge ) )
            {
               edge_type const & c_edge = edges_.at( cur_edge );
               if ( !lines && cur_used.at( c_edge.twin( ) ) )
               {
                  const edge_index_type twin = c_edge.twin( );
                  extract_contour( st, contours, lines, &twin );
               }
               else
               {
                  st.push( cur_edge );
               }
              
               is_used .at( cur_edge ) = true;
               cur_used.at( cur_edge ) = true;
               cur_edge = c_edge.next( );
            }

            if ( !st.empty( ) )
               extract_contour( st, contours, lines );
         }

   #ifdef _DEBUG
         for ( contours_type::iterator it = contours.begin( ); it != contours.end( ); ++it )
            Assert( it->size( ) > (lines ? 1U : 2U) );
   #endif
      } 

   private:
      // deterimne slope angles of each edges of DCEL
      void calc_alphas( graph_type const & graph, points_type const & points )
      {
         for( edges_type::iterator it = edges_.begin(); it != edges_.end( ); ++it )
         {
            const vertex_index_type v_in  = graph.vertex( it->in( )  ).id( );
            const vertex_index_type v_out = graph.vertex( it->out( ) ).id( );
            
            double const alpha = cg::angle( point_type(1, 0), points.at(v_in) - points.at(v_out) );
            it->set_alpha( alpha );
         }
      }

      // comparator. Compares two edges for less
      struct edge_less_type
      {
      public:
         edge_less_type( edges_type const & edges, vertex_index_type v )
            : edges_( &edges )
            , v_( v )
         {}

         bool operator( )( edge_index_type e1, edge_index_type e2 ) const 
         {
            return edges_->at( e1 ).alpha( v_ ) < edges_->at( e2 ).alpha( v_ );
         }     

      private:
         edges_type const * edges_;
         vertex_index_type  v_;     
      };
      
      // fill rsds structure: link edges with vertices
      void link_edges( graph_type const & graph )
      {
         const vertex_index_type v_size = graph.vertices_count( );
         std::vector< edge_indices_type >  per_vertex_edge_indices( v_size );

         for( edge_index_type i = 0, size = edges_.size( ); i != size; ++i )
         {
            edge_type const & edge = edges_.at( i );
            per_vertex_edge_indices.at( edge.in ( ) ).push_back( i );
            per_vertex_edge_indices.at( edge.out( ) ).push_back( i );
         }

         for ( vertex_index_type v = 0; v != v_size; ++v )
         {
            edge_indices_type & indices = per_vertex_edge_indices.at( v );
            
            Assert( indices.size( ) % 2 == 0 );

            if( indices.empty( ) )
               continue;
            
            std::sort( indices.begin( ), indices.end( ), edge_less_type( edges_, v ) );         
            link_vertex_edges( indices.begin( ), indices.end( ), v );
         }
      }

      template< typename FwdIter >
         void link_vertex_edges( FwdIter begin, FwdIter end, vertex_index_type v )
      {
         Assert( begin != end );
         FwdIter it = begin;

         boost::optional< edge_index_type > next, next2;
         while( true )
         {
            edge_type & edge = edges_.at( *it );
            if( edge.out( ) == v )
            {
               if( next )
                  next2 = *next;

               next = *it;
            }
            else
            {
               if( edge.has_next( ) )
                  break;
               else if( next && edge.twin( ) != *next )
                  edge.set_next( *next );
               else if( next2 )
                  edge.set_next( *next2 );
            }

            if( ++it == end )
               it = begin;
         }
#ifdef _DEBUG
         for ( FwdIter p = begin; p != end; ++p )
         {
            Assert ( (edges_.at(*p).out( ) == v) || (edges_.at(*p).has_next()) );
         }
#endif
      }

   private:
      edges_type          edges_;
   };
}}
