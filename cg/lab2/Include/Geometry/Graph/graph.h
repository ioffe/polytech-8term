#pragma once

// STL
#include <vector>
#include <list>

#include <boost/mpl/bool.hpp>

// Common
#include "common/fixed_id_vector.h"

#include "graph_base.h"

namespace cg
{

namespace graph_details 
{

//////////////////////////////////////////////////////////////////////////
// 
// structures for vertex 

template< class vertex_data_type >
struct vertex_base_type 
   : public indexed_data< vertex_data_type >
{
   typedef  indexed_data<vertex_data_type>   base_type            ;
   typedef  std::vector< size_t >            edges_container_type ;

   vertex_base_type( vertex_data_type const &data, size_t idx ) 
      : base_type( data, idx )
   {
   }

   vertex_base_type() {}
};

template< class vertex_data_type, bool directed >
   struct vertex_type 
   : vertex_base_type< vertex_data_type >
{
   vertex_type( vertex_data_type const &data, size_t idx )
      : vertex_base_type< vertex_data_type > ( data, idx ) 
   {
   }

   vertex_type() {}

   edges_container_type&       out_edges()       { return out ; }
   edges_container_type const& out_edges() const { return out ; }

   edges_container_type in  ; // for directed graphs
   edges_container_type out ; // 

   template< class Stream >
      void serialize( Stream & stream ) const
   {
      write( stream, data()   );
      write( stream, idx()    );
      write( stream, in       );
      write( stream, out      );
   }
   template< class Stream >
      void deserialize( Stream & stream ) 
   {
      read( stream, data()   );
      read( stream, idx()    );
      read( stream, in       );
      read( stream, out      );
   }
};

template< class vertex_data_type >
   struct vertex_type< vertex_data_type, false > // for indirected graphs 
   : vertex_base_type< vertex_data_type >
{
   vertex_type( vertex_data_type const &data, size_t idx )
      : vertex_base_type< vertex_data_type > ( data, idx ) 
   {
   }

   vertex_type() {}

   edges_container_type&       out_edges()       { return adjacent ; }
   edges_container_type const& out_edges() const { return adjacent ; }

   edges_container_type adjacent ; 

   template< class Stream >
      void serialize( Stream & stream ) const
   {
      write( stream, data()   );
      write( stream, idx()    );
      write( stream, adjacent );
   }
   template< class Stream >
      void deserialize( Stream & stream ) 
   {
      read( stream, data()   );
      read( stream, idx()    );
      read( stream, adjacent );
   }
};
             
//////////////////////////////////////////////////////////////////////////
//
// structure for edge 

template< class edge_data_type >
struct edge_type 
   : public indexed_data< edge_data_type >
{
   typedef  indexed_data< edge_data_type >   base_type               ;
   typedef  std::pair< size_t, size_t >      vertices_container_type ;

   edge_type( edge_data_type const &data, size_t idx, size_t begin, size_t end ) 
      : base_type( data , idx )
      , adjacent ( begin, end )
   {
   }

   edge_type() {}

   bool is_begin(size_t v) const
   {
      return adjacent.first == v;
   }

   template< class Stream >
      void serialize( Stream & stream ) const
   {
      write( stream, data()   );
      write( stream, idx()    );
      write( stream, adjacent );
   }

   template <class Stream>
      void deserialize(Stream &stream)
      {
         read( stream, data()   );
         read( stream, idx()    );
         read( stream, adjacent );
      }

public:
   vertices_container_type adjacent; 
};

} // graph_details

//////////////////////////////////////////////////////////////////////////
//
// graph declaration 
//

namespace graph_details
{

template< class vertex_data_type, class edge_data_type, bool directed >
struct traits
{
   typedef vertex_data_type   vertex_data_type ;
   typedef edge_data_type     edge_data_type   ;

   // vertex and edge type 
   typedef graph_details::vertex_type< vertex_data_type, directed >  vertex_type ;
   typedef graph_details::edge_type  < edge_data_type             >  edge_type   ;

   typedef util::fixed_id_vector<vertex_type> vertices_type;
   typedef util::fixed_id_vector<edge_type>   edges_type;

   static bool is_directed() { return directed; }
};

} // end of namespace graph_details

template< class vertex_data_type, class edge_data_type, bool directed = true >
class graph : public graph_details::base < graph<vertex_data_type, edge_data_type, directed>,
                                           graph_details::traits<vertex_data_type, edge_data_type, directed> >
{
   typedef graph_details::traits< vertex_data_type, edge_data_type, directed>  traits_type;
   typedef graph_details::base  < graph, traits_type >                         base_type;

public:
   typedef typename traits_type::vertex_data_type vertex_data_type ;
   typedef typename traits_type::edge_data_type   edge_data_type   ;

   typedef typename traits_type::vertex_type vertex_type ;
   typedef typename traits_type::edge_type   edge_type   ;

   typedef typename traits_type::vertices_type vertices_type;
   typedef typename traits_type::edges_type    edges_type   ;

   typedef typename vertex_type::edges_container_type edges_container_type;
   typedef typename base_type::adj_iterator           adj_iterator        ;

   // public interface 
public:
   size_t add_vertex( vertex_data_type const &data ) ;
   size_t add_edge  ( edge_data_type const &data, size_t beginIdx, size_t endIdx );

   bool   remove_vertex( size_t idx ) ;
   bool   remove_edge  ( size_t idx ) ;

   // const version
   using base_type::for_each_vertex;
   using base_type::for_each_edge;

   template< class Functor > void for_each_vertex( Functor &func );
   template< class Functor > void for_each_edge( Functor &func );

   // vertexes & edges
   vertices_type const& vertices() const;
   edges_type    const& edges   () const; 

   vertex_type const& vertex( size_t idx ) const;
   edge_type   const& edge( size_t idx ) const;

   vertex_type& vertex( size_t idx );
   edge_type  & edge( size_t idx );

   // index of the edge which begins in beginIdx & ends in endIdx 
   size_t edge( size_t beginIdx, size_t endIdx ) const;

   // const version
   using base_type::vertex_data;
   using base_type::edge_data;

   vertex_data_type& vertex_data ( size_t idx ) ;
   edge_data_type  & edge_data   ( size_t idx ) ;

   void inverse_edge( size_t idx );

   // check existence
   bool is_valid_vertex( size_t idx ) const;
   bool is_valid_edge  ( size_t idx ) const;

   template< class Stream >
      void serialize( Stream & stream ) const;

   template <class Stream>
      void deserialize(Stream & stream);

private:
   void add_eadj_impl( size_t edgeIdx, vertex_type &from, vertex_type &to, boost::mpl::true_ )
   {
      Assert(std::find(from.out.begin(), from.out.end(), edgeIdx) == from.out.end());
      Assert(std::find(to.in.begin(), to.in.end(), edgeIdx) == to.in.end());
      from.out.push_back(edgeIdx);
      to.in.push_back(edgeIdx);
   }

   void add_eadj_impl( size_t edgeIdx, vertex_type &from, vertex_type &to, boost::mpl::false_ )
   {
      Assert(std::find(from.adjacent.begin(), from.adjacent.end(), edgeIdx) == from.adjacent.end());
      Assert(std::find(to.adjacent.begin(), to.adjacent.end(), edgeIdx) == to.adjacent.end());
      from.adjacent.push_back(edgeIdx);
      to.adjacent.push_back(edgeIdx);
   }

   template <bool d>
      void add_eadj_impl(size_t edgeIdx, vertex_type &from, vertex_type &to)
   {
      add_eadj_impl(edgeIdx, from, to, boost::mpl::bool_<d>());
   }

   void remove_vadj_impl( vertex_type &v, std::stack<size_t> &toRemove, boost::mpl::true_ )
   {
      for( edges_container_type::iterator it = v.in.begin(), end = v.in.end(); it != end; ++it )
         toRemove.push(*it);
      for( edges_container_type::iterator it = v.out.begin(), end = v.out.end(); it != end; ++it )
         toRemove.push(*it);

      v.in.resize(0);
      v.out.resize(0);
   }

   void remove_vadj_impl( vertex_type &v, std::stack<size_t> &toRemove, boost::mpl::false_ )
   {
      for( edges_container_type::iterator it = v.adjacent.begin(), end = v.adjacent.end(); it != end; ++it )
         toRemove.push(*it);

      v.adjacent.resize(0);
   }

   template< bool d > 
      void remove_vadj_impl( vertex_type &v, std::stack<size_t> &toRemove )
   {
      remove_vadj_impl(v, toRemove, boost::mpl::bool_<d>());
   }

   void remove_eadj_impl( size_t idx, vertex_type &from, vertex_type &to, boost::mpl::true_ )
   {
      edges_container_type::iterator it = std::find(to.in.begin(), to.in.end(), idx);
      if (it != to.in.end())
         to.in.erase(it);

      it = std::find(from.out.begin(), from.out.end(), idx);
      if (it != from.out.end())
         from.out.erase(it);
   }

   void remove_eadj_impl( size_t idx, vertex_type &from, vertex_type &to, boost::mpl::false_ )
   {
      edges_container_type::iterator it = std::find(to.adjacent.begin(), to.adjacent.end(), idx);
      if (it != to.adjacent.end())
         to.adjacent.erase(it);

      it = std::find(from.adjacent.begin(), from.adjacent.end(), idx);
      if (it != from.adjacent.end())
         from.adjacent.erase(it);
   }

   template <bool d>
      void remove_eadj_impl( size_t idx, vertex_type &from, vertex_type &to )
   {
      remove_eadj_impl(idx, from, to, boost::mpl::bool_<d>());
   }
      

private:
   // Storage
   vertices_type vertices_;
   edges_type    edges_   ;
};

//////////////////////////////////////////////////////////////////////////
//
// serialization
//

namespace graph_details
{

template< class Stream, class V, bool D >
   void write( Stream & stream, vertex_type<V, D> const& v )
{
   v.serialize( stream );
}

template< class Stream, class E >
   void write( Stream & stream, edge_type<E> const& e )
{
   e.serialize( stream );
}

template< class Stream, class V, bool D >
   void read( Stream & stream, vertex_type<V, D> & v )
{
   v.deserialize( stream );
}

template< class Stream, class E >
   void read( Stream & stream, edge_type<E> & e )
{
   e.deserialize( stream );
}

} // end of namespace graph_details

template< class Stream, class V, class E, bool D >
   void write( Stream & stream, graph<V, E, D> const& g )
{
   g.serialize( stream );
}

template< class Stream, class V, class E, bool D >
   void read( Stream & stream, graph<V, E, D> & g )
{
   g.deserialize( stream );
}

} // end of namespace cg

#include "graph.inl"