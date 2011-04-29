#pragma once

#include "graph_base.h"

#pragma pack ( push, 1 )

namespace cg
{

namespace graph_details 
{

//////////////////////////////////////////////////////////////////////////
// structures for vertex 
template< class vertex_data_type >
struct vertex_base_type_mapped
   : public indexed_data< vertex_data_type >
{
   typedef  indexed_data<vertex_data_type>     base_type            ;
   typedef  mapped_vector< size_t >            edges_container_type ;
};

template< class vertex_data_type, bool directed >
   struct vertex_type_mapped 
   : vertex_base_type_mapped< vertex_data_type >
{
   edges_container_type const& out_edges() const { return out ; }

   edges_container_type in  ; // for directed graphs
   edges_container_type out ; // 
};

template< class vertex_data_type >
   struct vertex_type_mapped< vertex_data_type, false > // for indirected graphs 
   : vertex_base_type_mapped< vertex_data_type >
{
   edges_container_type const& out_edges() const { return adjacent ; }

   edges_container_type adjacent ; 
};

template< class edge_data_type >
struct edge_type_mapped 
   : public indexed_data< edge_data_type >
{
   typedef  std::pair< size_t, size_t >    vertices_container_type ;

   vertices_container_type adjacent; 
};
           
} // graph_details

//////////////////////////////////////////////////////////////////////////
//
// graph_mapped declaration 
//

namespace graph_details
{

template< class vertex_data_type, class edge_data_type, bool directed >
struct traits_mapped
{
   typedef vertex_data_type   vertex_data_type ;
   typedef edge_data_type     edge_data_type   ;

   // vertex and edge type 
   typedef graph_details::vertex_type_mapped< vertex_data_type, directed >  vertex_type ;
   typedef graph_details::edge_type_mapped  < edge_data_type             >  edge_type   ;

   typedef mapped_vector< vertex_type >  vertices_type;
   typedef mapped_vector< edge_type   >  edges_type   ;

   static bool is_directed() { return directed; }
};

} // end of namespace graph_details

template< class vertex_data_type, class edge_data_type, bool directed = true >
class graph_mapped : public graph_details::base < graph_mapped<vertex_data_type, edge_data_type, directed>,
                                                  graph_details::traits_mapped<vertex_data_type, edge_data_type, directed> >
{
   typedef graph_details::traits_mapped< vertex_data_type, edge_data_type, directed>  traits_type;
   typedef graph_details::base         < graph_mapped, traits_type >                  base_type;

public:
   typedef typename traits_type::vertex_data_type vertex_data_type ;
   typedef typename traits_type::edge_data_type   edge_data_type   ;

   typedef typename traits_type::vertex_type vertex_type ;
   typedef typename traits_type::edge_type   edge_type   ;

   typedef typename traits_type::vertices_type vertices_type;
   typedef typename traits_type::edges_type    edges_type   ;

   // public interface 
public:
   // vertexes & edges 
   vertices_type const& vertices() const { return vertices_; }
   edges_type    const& edges   () const { return edges_   ; } 

   vertex_type const& vertex( size_t idx ) const { return vertices_.at(idx); }
   edge_type   const& edge( size_t idx ) const   { return edges_.at(idx)   ; }

   template < class Stream >
      friend void read( Stream & s, graph_mapped & g )
   {
      read( s, g.vertices_ );
      read( s, g.edges_ );
   }

private:
   // Storage
   vertices_type vertices_;
   edges_type    edges_   ;
};

} // end of namespace cg

#pragma pack ( pop )