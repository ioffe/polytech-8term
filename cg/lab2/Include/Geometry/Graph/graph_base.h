#pragma once

#include <stack>
#include <queue>

namespace cg
{
namespace graph_details
{

#pragma pack ( push, 1 )

//////////////////////////////////////////////////////////////////////////
//
// structure for indexed data 

template< class data_type >
struct indexed_data
{
   data_type      & data()       { return data_ ; } 
   data_type const& data() const { return data_ ; } 

   size_t     idx () const { return idx_  ; } 
   size_t &   idx ()       { return idx_  ; }

protected:
   indexed_data ( data_type const &data, size_t idx ) 
      : data_( data )
      , idx_ ( idx  )
   {
   }

   indexed_data() {}

protected:
   data_type data_;
   size_t    idx_ ;
};

#pragma pack ( pop )

//////////////////////////////////////////////////////////////////////////
// 
// adjacent edges iterator

template< class graph_type >
struct adj_iterator
{
   adj_iterator( size_t vertex, graph_type const& graph ) ;

   size_t to  () const ;
   size_t from() const ;

   adj_iterator &operator ++ () ;
   
   operator        bool  () const ;
   size_t operator *     () const ;
   
private:
   graph_type  const&   graph_  ; 
   size_t      const    vertex_ ;
   
   typename graph_type::vertex_type::edges_container_type::const_iterator it_ ;
};

//////////////////////////////////////////////////////////////////////////
//
// graph base declaration 

template< class Derived, class Traits >
class base
{
public:
   typedef typename Traits::vertex_data_type  vertex_data_type ;
   typedef typename Traits::edge_data_type    edge_data_type   ;

   // vertex and edge type 
   typedef typename Traits::vertex_type  vertex_type ;
   typedef typename Traits::edge_type    edge_type   ;

   typedef adj_iterator<Derived>     adj_iterator   ;

   typedef typename Traits::vertices_type  vertices_type;
   typedef typename Traits::edges_type     edges_type   ;

   // for invalid index 
   static size_t const INVALID_IDX = static_cast<size_t>(-1);

public:
   size_t count_vertices() const { return self().vertices().size(); }

   // applying functors to each item 
   template< class Functor > void for_each_vertex( Functor &func ) const ;
   template< class Functor > void for_each_edge( Functor &func ) const ;

   edge_data_type   const& edge_data   ( size_t idx ) const;
   vertex_data_type const& vertex_data ( size_t idx ) const;

   adj_iterator adj_edges( size_t vertex ) const ;

   // DFS or BFS
   template< template < class, class > class Storage, class Functor >
      void traversal( size_t beginIdx, Functor &func ) const;

   bool is_directed() const { return Traits::is_directed(); }

private:
   std::stack<size_t>::value_type& storage_top(std::stack<size_t> &st) const
   {
      return st.top();
   }

   std::queue<size_t>::value_type& storage_top(std::queue<size_t> &st) const
   {
      return st.front();
   }

private:
   Derived&       self()       { return static_cast<Derived&>      (*this); }
   Derived const& self() const { return static_cast<Derived const&>(*this); }
};

} // end of namespace graph_details
} // end of namespace cg

#include "graph_base.inl"