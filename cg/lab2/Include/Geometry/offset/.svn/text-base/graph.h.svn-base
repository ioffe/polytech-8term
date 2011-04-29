#pragma once

#include <deque>
#include <boost/call_traits.hpp>
#include <boost/bind.hpp>

namespace cg
{
   // Класс для графа
   template< typename vertex_data_type >
      struct Graph
   {
   private:
      typedef     size_t                                                              index_type;

   public:
      typedef     index_type                                                          vertex_index_type;
      typedef     index_type                                                          edge_index_type;
      typedef     typename boost::call_traits< vertex_data_type >::param_type         vertex_data_param_type;
      typedef     std::pair< index_type, index_type >                                 vertex_index_pair_type;

   private:
      // Структура для вершины
      struct vertex_type
      {
         vertex_type( vertex_data_param_type data )
            : data_     ( data )
            , removed_  ( false )
         {}

         bool is_removed ( )  const  { return removed_; }
         void remove( )              { removed_ = true; }
           
         vertex_data_param_type   data( )  const  { return data_; }

      private:
         vertex_data_type  data_;
         bool              removed_;
      };

   private:
      // Структура для ребра
      struct edge_type
      {
         edge_type( vertex_index_type out, vertex_index_type in )
            : out_      ( out )
            , in_       ( in )
            , removed_  ( false )
         {}

         vertex_index_type out(  ) const { return out_;}
         vertex_index_type in (  ) const { return in_; }

         bool is_removed ( )  const  { return removed_;  }
         void remove( )              { removed_ = true;  }
         void restore( )             { removed_ = false; }

      private:
         vertex_index_type out_;
         vertex_index_type in_;
         bool removed_;
      };

   public:
      vertex_index_type add_vertex( vertex_data_param_type data ) 
      {
         const vertex_index_type vi = vertices_.size( );
         vertices_.push_back( vertex_type( data ) );
         return vi;
      }

      void clear( )    
      { 
         vertices_.clear( );
         edges_.clear( );
      }

   private:
      template< template <class> class Comp >
         struct edge_cmp
      {
         bool operator()( edge_type const & e1, edge_type const & e2 ) const
         {
            typedef std::pair< vertex_index_type, vertex_index_type > vpair_type;
            vpair_type const v1 = e1.in() < e1.out()  ? vpair_type( e1.in (), e1.out() ) 
                                                      : vpair_type( e1.out(), e1.in () );

            vpair_type const v2 = e2.in() < e2.out()  ? vpair_type( e2.in (), e2.out() ) 
                                                      : vpair_type( e2.out(), e2.in () );
            return Comp<vpair_type>()( v1, v2 );
         }
      }; 

      typedef edge_cmp< std::equal_to > edge_eq_type;
      typedef edge_cmp< std::less     > edge_lt_type;

   public:
      edge_index_type add_edge( vertex_index_type out, vertex_index_type in )
      {
         Assert( out < vertices_.size( ) );
         Assert( in  < vertices_.size( ) );
         Assert( out != in );

         edges_.push_back( edge_type( out, in ) );
         return ( edge_index_type ) ( edges_.size( ) - 1 );
      }

      vertex_index_type   vertices_count( ) const { return ( index_type ) vertices_.size( ); }
      edge_index_type        edges_count( ) const { return ( index_type )    edges_.size( ); }

      vertex_data_param_type      vertex( vertex_index_type i )     const { return vertices_.at( i ).data( ); }
      vertex_index_pair_type      edge  ( edge_index_type   i )     const 
      { 
         edge_type const & e = edges_.at( i );
         return std::make_pair( e.out( ), e.in( ) );
      }

      void remove_vertex( vertex_index_type i ) { vertices_.at( i ).remove( ); }
      void remove_edge  ( edge_index_type   i ) { edges_   .at( i ).remove( ); }

      bool is_vertex_removed ( index_type i )  const
      {
         return vertices_.at( i ).is_removed( );
      }

      bool is_edge_removed ( index_type  i )  const
      {
         return is_edge_removed_impl( edges_.at( i ) );       
      }

      void kill_dupe_edges( )
      {
         std::sort( edges_.begin( ), edges_.end( ), edge_lt_type( ) );
         edges_.erase( std::unique( edges_.begin( ), edges_.end( ), edge_eq_type( ) ), edges_.end( ) );
      }
       
      bool is_edge_removed_impl ( edge_type const & edge )  const
      {
         return edge.is_removed( ) 
                  || is_vertex_removed( edge.in( ) ) 
                  || is_vertex_removed( edge.out( ) );
      }
         
      void erase_removed_edges( )
      {
         edges_.erase( std::remove_if( edges_.begin( ), edges_.end( ),
            boost::bind( &Graph::is_edge_removed_impl, boost::cref(*this), _1 ) ), edges_.end( ) );
      }

   private:
      typedef     std::vector< vertex_type >   vertices_type;
      typedef     std::vector< edge_type >     edges_type;

   private:
      vertices_type  vertices_;
      edges_type     edges_;
   };   
}
