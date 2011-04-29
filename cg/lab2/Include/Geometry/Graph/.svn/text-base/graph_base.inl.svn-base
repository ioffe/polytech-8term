namespace cg
{

//////////////////////////////////////////////////////////////////////////
// adj_iterator 

namespace graph_details
{

template< class graph_type >
   adj_iterator< graph_type >::adj_iterator( size_t vertex, graph_type const& graph ) 
   : graph_ ( graph  )
   , vertex_( vertex )   
   , it_    ( graph.vertex( vertex_ ).out_edges().begin())
{
}

template< class graph_type >
   size_t adj_iterator< graph_type >::to() const 
{
   typedef typename graph_type::edge_type::vertices_container_type vertices_container_type;
   vertices_container_type const &adj = graph_.edge( *it_ ).adjacent;

   return (!graph_.is_directed() && (vertex_ == adj.second)) ? adj.first : adj.second;
}

template< class graph_type >
   size_t adj_iterator< graph_type >::from() const 
{
   return vertex_; 
}

template< class graph_type >
   adj_iterator< graph_type >& adj_iterator< graph_type >::operator ++ () 
{ 
   ++it_ ;
   return *this; 
}

template< class graph_type >
   adj_iterator< graph_type >::operator bool () const 
{
   return graph_.vertex( vertex_ ).out_edges().end() != it_ ;
}

template< class graph_type >
   size_t adj_iterator< graph_type >::operator * () const 
{
   Assert( *this ) ;
   return *it_ ;
}

//////////////////////////////////////////////////////////////////////////
//
// graph base implementation 
//

template< class Derived, class Traits >  
   template < class Functor >
   void base< Derived, Traits >::for_each_vertex( Functor &func ) const 
{
   for( vertices_type::const_iterator it = self().vertices().begin(), end = self().vertices().end(); it != end; ++it )
      func( *it );
}

template< class Derived, class Traits >  
   template< class Functor >
   void base< Derived, Traits >::for_each_edge( Functor &func ) const 
{
   for(edges_type::const_iterator it = self().edges().begin(), end = self().edges().end(); it != end; ++it)
      func(*it);
}

template< class Derived, class Traits >  
   typename base< Derived, Traits >::vertex_data_type const& base< Derived, Traits >::vertex_data( size_t idx ) const
{
   return self().vertex( idx ).data() ;
}

template< class Derived, class Traits >  
   typename base< Derived, Traits >::edge_data_type const& base< Derived, Traits >::edge_data( size_t idx ) const
{
   return self().edge( idx ).data() ;
}

template< class Derived, class Traits >  
   typename base< Derived, Traits >::adj_iterator base< Derived, Traits >::adj_edges( size_t vertex ) const 
{
   return adj_iterator( vertex, self() ) ;
}

template< class Derived, class Traits >  
   template< template < class, class > class Storage, class Functor >
   void base< Derived, Traits >::traversal( size_t beginIdx, Functor &func ) const
{
   bool * visited = ( bool* )_alloca( sizeof( bool ) * self().vertices().size()) ;
   memset( visited, 0, sizeof( bool ) * self().vertices().size()) ;

   Storage< size_t > st;
   st.push( beginIdx );

   while( !st.empty())
   {
      size_t idx = storage_top(st) ;
      st.pop() ;

      if(!func(self().vertex(idx))) // Call external functor with vertex
         break;

      visited[ idx ] = true ;

      // Go through adjacent
      for (adj_iterator it = adj_edges(idx); it; ++it)
      {
         size_t nextIdx = it.to();
         if( !visited[ nextIdx ] )
         {
            func(self().vertex(idx), self().edge(*it)); // Call external functor with vertex and outgoing edge
            st.push( nextIdx );
         }
      }
   }
}

} // graph_details 
} // cg