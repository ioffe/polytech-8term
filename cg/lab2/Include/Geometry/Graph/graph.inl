namespace cg
{

//////////////////////////////////////////////////////////////////////////
//
// graph implementation 
//

template< class v, class e, bool d > 
   typename graph< v, e, d >::vertices_type const& graph< v, e, d >::vertices() const
{
   return vertices_;
}

template< class v, class e, bool d > 
   typename graph< v, e, d >::edges_type    const& graph< v, e, d >::edges   () const
{
   return edges_;
}

template< class v, class e, bool d > 
   typename graph< v, e, d >::vertex_type& graph< v, e, d >::vertex( size_t idx )
{
   return vertices_.at(idx);
}

template< class v, class e, bool d > 
   typename graph< v, e, d >::edge_type& graph< v, e, d >::edge( size_t idx )
{
   return edges_.at(idx);
}

template< class v, class e, bool d > 
   typename graph< v, e, d >::vertex_type const& graph< v, e, d >::vertex( size_t idx ) const
{
   return vertices_.at(idx);
}

template< class v, class e, bool d > 
  typename graph< v, e, d >::edge_type const& graph< v, e, d >::edge( size_t idx ) const
{
   return edges_.at(idx);
}

// index of the edge which begins in beginIdx & ends in endIdx 
template< class v, class e, bool d >  
   size_t graph< v, e, d >::edge( size_t idx0, size_t idx1 ) const
{
   for (adj_iterator it = adj_edges(idx0); it; ++it)
   {
      if(idx1 == it.to())
         return *it;
   }

   return INVALID_IDX ;
}

template< class v, class e, bool d > 
   template < class Functor >
   void graph< v, e, d >::for_each_vertex( Functor &func )
{
   for( vertices_type::iterator it = vertices_.begin(), end = vertices_.end(); it != end; ++it )
      func( *it );
}

template< class v, class e, bool d > 
   template < class Functor >
   void graph< v, e, d >::for_each_edge( Functor &func ) 
{
   for(edges_type::iterator it = edges_.begin(), end = edges_.end(); it != end; ++it)
      func(*it);
}

template< class v, class e, bool d > 
   typename graph< v, e, d >::vertex_data_type& graph< v, e, d >::vertex_data( size_t idx )
{
   return vertex( idx ).data() ;
}

template< class v, class e, bool d > 
   typename graph< v, e, d >::edge_data_type& graph< v, e, d >::edge_data( size_t idx ) 
{
   return edge( idx ).data() ;
}

template< class v, class e, bool d > 
   void graph< v, e, d >::inverse_edge( size_t idx )
{
   edge_type &e      = edge(idx);
   vertex_type &from = vertex(e.adjacent.first);
   vertex_type &to   = vertex(e.adjacent.second);

   remove_eadj_impl<d>(idx, from, to);
   add_eadj_impl<d>(idx, to, from);

   std::swap(e.adjacent.first, e.adjacent.second);
}

template< class v, class e, bool d > 
   bool graph< v, e, d >::is_valid_vertex( size_t idx ) const
{
   return vertices_.valid(idx);
}

template< class v, class e, bool d > 
   bool graph< v, e, d >::is_valid_edge  ( size_t idx ) const
{
   return edges_.valid(idx);
}

template < class v, class e, bool d >
   size_t graph< v, e, d >::add_vertex( typename graph< v, e, d >::vertex_data_type const &data )
{
   size_t nid = vertices_.next_id();
   vertices_.insert(vertex_type(data, nid));

   return nid;
}

template < class v, class e, bool d >
   size_t graph< v, e, d >::add_edge( typename graph< v, e, d >::edge_data_type const &data, size_t beginIdx, size_t endIdx )
{
   Assert(beginIdx != endIdx);
   Assert(edge(beginIdx, endIdx) == INVALID_IDX);

   size_t nid = edges_.next_id();
   edges_.insert(edge_type(data, nid, beginIdx, endIdx));

   add_eadj_impl<d>(nid, vertex(beginIdx), vertex(endIdx));

   return nid;
}

template < class v, class e, bool d >
   bool graph< v, e, d >::remove_vertex( size_t idx ) 
{
   if(!vertices_.valid(idx)) 
      return false;

   std::stack<size_t> toRemove;
   remove_vadj_impl<d>(vertex(idx), toRemove);

   while(!toRemove.empty())
   {
      remove_edge(toRemove.top());
      toRemove.pop();
   }

   vertices_.erase(idx);

   return true;
}

template < class v, class e, bool d >
   bool graph< v, e, d >::remove_edge( size_t idx )
{
   if(!edges_.valid(idx)) 
      return false;

   remove_eadj_impl<d>(idx, vertex(edge(idx).adjacent.first), vertex(edge(idx).adjacent.second)); 
   edges_.erase(idx);

   return true;
}

template < class v, class e, bool d >
   template< class Stream >
   void graph< v, e, d >::serialize( Stream & stream ) const
{
   write( stream, vertices_ );
   write( stream, edges_ );
}

template <class v, class e, bool d>
   template <class Stream>
      void graph<v,e,d>::deserialize(Stream & stream)
   {
      read(stream, vertices_);
      read(stream, edges_);
   }

} // namespace cg 