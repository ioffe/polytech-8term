namespace cg            {
namespace triangulation {

   template< class Triangulation >
   struct edge_f
   {
   private:
      typedef typename Triangulation::vertex_handle vertex_handle;
   public:
      typedef std::pair< vertex_handle, vertex_handle > type;
   };

   namespace details
   {
      struct empty_property_t
      {
         template< class Edge >
         bool operator () ( Edge const & edge  ) const
         {
            return false;
         }

         template< class Vertex >
         bool operator () ( Vertex a, Vertex b ) const
         {
            return false;
         }

         template< class Edge >
         void insert( Edge const & edge )
         {}

         template< class Vertex >
         void insert( Vertex a, Vertex b )
         {}

         template< class Edge >
         void remove( Edge const & edge )
         {}

         template< class Vertex >
         void remove( Vertex a, Vertex b )
         {}
      };
   }

   template< class Triangulation, class FwdIter >
   void add_points( Triangulation & trg, FwdIter p, FwdIter q )
   {
      for ( ; p != q; ++p )
         trg.insert( *p );
   }


   template < class Triangulation, class OutIter, class Property >
   OutIter update_property(   Triangulation const & trg, 
                              typename Triangulation::vertex_handle ph,
                              typename Triangulation::vertex_handle qh,
                              OutIter out, Property & pr,
                              std::vector< typename Triangulation::vertex_handle > const & inserted )
   {
      typedef Triangulation                        trg_t;
      typedef typename trg_t::vertex_handle        vertex_handle;
      typedef typename trg_t::vertex_circulator    vertex_circulator;
      if ( !inserted.empty() )
      {
         for ( size_t i = 0, j = 1; j != inserted.size(); ++i, ++j, ++out )
         {
            *out++ = std::make_pair( inserted[i], inserted[j] );
         }
         *out++ = std::make_pair( ph, inserted.front() );
         *out++ = std::make_pair( inserted.back(), qh );
         for ( size_t i = 0; i != inserted.size(); ++i )
         {
            vertex_handle vh = inserted[i];
            vertex_circulator vc = trg.incident_vertices( vh ), done = vc;
            vertex_handle our_begin =   i == 0                     ? ph : inserted[i - 1];
            vertex_handle our_end =     i + 1 == inserted.size()   ? qh : inserted[i + 1];
            vertex_handle v[2];
            size_t j = 0;
            bool flag = true;
            do
            {
               if ( ( vc != our_begin ) && ( vc != our_end ) && trg.is_constrained( vc, vh ) )
               {
                  if ( j == 2 )
                  {
                     flag = false;
                     break;
                  }
                  else
                     v[j++] = vc;
               }
            } while ( ++vc != done );
            if ( flag && ( j == 2 ) )
            {
               if ( pr( v[0], v[1] ) )
               {
                  pr.remove( v[0], v[1] );
                  pr.insert( v[0], vh );
                  pr.insert( v[1], vh );
               }
            }
         }
      }
      else
      {
         *out++ = std::make_pair( ph, qh );
      }
      return out;
   }

   template < class Triangulation, class OutIter, class Property >
   OutIter insert(   Triangulation & trg, 
                     typename Triangulation::vertex_handle ph,
                     typename Triangulation::vertex_handle qh,
                     OutIter out, Property & pr )
   {
      typedef typename Triangulation::vertex_handle   vertex_handle;
      typedef std::vector< vertex_handle >            vertex_handles;
      vertex_handles inserted;
      trg.insert( ph, qh, std::back_inserter( inserted ) );
      return update_property( trg, ph, qh, out, pr, inserted );
   }

   template < class Point, class Triangulation, class OutIter, class Property >
   OutIter insert( Triangulation & trg, Point const & p, Point const & q, OutIter out, Property & pr )
   {
      typedef typename Triangulation::vertex_handle   vertex_handle;
      typedef std::vector< vertex_handle >            vertex_handles;
      vertex_handles inserted;
      typename edge_f< Triangulation >::type edge = trg.insert( p, q, std::back_inserter( inserted ) );
      return update_property( trg, edge.first, edge.second, out, pr, inserted );      
   }

   template < class Triangulation, class FwdIter, class OutIter, class Property >
   OutIter add_polyline( Triangulation & trg, FwdIter p, FwdIter q, OutIter out, Property & pr )
   {
      for ( FwdIter r = p++; p != q; r = p++ )
      {
         out = insert( trg, *r, *p, out, pr );
      }
      return out;
   }

   template < class Triangulation, class FwdIter >
   void add_polyline( Triangulation & trg, FwdIter p, FwdIter q )
   {
      details::empty_property_t _;
      add_polyline( trg, p, q, util::null_iterator(), _ );
   }

   template < class Triangulation, class FwdIter > 
   void add_contour( Triangulation & trg, FwdIter p, FwdIter q )
   { 
      for ( FwdIter r = p; r != q; )
      {
         FwdIter rr = r++;
         if ( r == q )
            trg.insert( *rr, *p );
         else
            trg.insert( *rr, *r );
      }
   }

   template < class Triangulation, class FwdIter > 
   void add_contours( Triangulation & trg, FwdIter p, FwdIter q )
   { 
      for ( FwdIter r = p; r != q; ++r )
         add_contour( trg, r->begin(), r->end() );
   }

   template < class Triangulation, class Property, class OutIter >
   OutIter add_polygon( Triangulation & trg, cg::polygon_2 const & poly, Property & pr, OutIter out )
   {
      for ( size_t i = 0; i != poly.size(); ++i )
      {
         out = add_polyline( trg,  poly[i].begin(), poly[i].end(), out, pr );
         out = insert( trg, poly[i].back(), poly[i].front(), out, pr );
      }
      return out;
   }

   template < class Triangulation, class Property >
   void add_polygon( Triangulation & trg, cg::polygon_2 const & poly, Property & pr )
   {
      add_polygon( trg, poly, pr, util::null_iterator() );
   }

   template < class Triangulation > 
   void add_polygon( Triangulation & trg, cg::polygon_2 const & poly )
   { 
      add_polygon( trg, poly, details::empty_property_t() );
   }


}}