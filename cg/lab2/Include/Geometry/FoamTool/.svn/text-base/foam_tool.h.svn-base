////////////////////////////////////////////////////////////////////////////////////////////////
// @file   foam_tool.h
// @author Kulagin Roman
// @brief  island contours processing for correct foam rendering
//         USE: function foam::processContour()
//

#pragma once

namespace foam
{
   //*** New type definitions ***//
   struct vertex_list_node
   {
      vertex_list_node()
         : prev( -1 )
         , next( -1 )
      {}

      int prev, next;
   };
   
   typedef std::vector< vertex_list_node > vertex_list;
   typedef std::vector< cg::point_2f > polyline;
   typedef std::vector< polyline > polycontours;
   
   struct polyindices : public std::vector< int >
   {
      float offset;
   };

   typedef std::queue< int > vertex_queue;
   typedef std::vector< std::_Bool > vertex_status;

   ////////////////////////////////////////////////////////////////////////////////////////////////
   // @brief  Calculate bisector of angle adjusted by 3 points
   //
   cg::point_2f getBisector( cg::point_2f const & p1, cg::point_2f const & p2, cg::point_2f const & p3 )
   {
      cg::point_2f l = p2 - p1;
      cg::point_2f r = p3 - p2;

      cg::point_2f ln( -l.y, l.x );
      cg::point_2f rn( -r.y, r.x );

      cg::normalize( ln );
      cg::normalize( rn );
      
      cg::point_2f norm = ln + rn;
      if ( cg::eq_zero( norm ) )
      {
         norm.x = -rn.y;
         norm.y = rn.x;
      }
      return norm / ( 1.0 + ln * rn );
   }

   ////////////////////////////////////////////////////////////////////////////////////////////////
   // @brief  Calculate distance form point p2 to segment [p1; p3]
   //
   float getVertexOffset( cg::point_2f const & p1, cg::point_2f const & p2, cg::point_2f const & p3 )
   {
      cg::point_2f edge = p3 - p1;
      cg::point_2f norm( -edge.y, edge.x );
      cg::normalize( norm );

      return norm * ( p2 - p1 );
   }   

   ////////////////////////////////////////////////////////////////////////////////////////////////
   
   bool isBadVertex( polyline const & pts, vertex_list const & list, int i, float offset )
   {
      // Check for correct vertex number
      if ( i == -1 )
         return false;
      
      int pi = list[i].prev;
      int ni = list[i].next;
      int ppi = -1, nni = -1;

      cg::point_2f bis;
      
      // Check if current vertex is not last
      if ( pi == -1 || ni == -1 )
         return false;
      if ( pi == ni )
         return true;
      
      // Vertex i has next and previous vertices
      bis = getBisector( pts[pi], pts[i], pts[ni] );
      ppi = list[pi].prev;
      nni = list[ni].next;
      
      // Calculate bisector in vertex pi
      cg::point_2f lbis;
      if ( ppi != -1 )
         lbis = getBisector( pts[ppi], pts[pi], pts[i] );

      // Calculate bisector in vertex ni
      cg::point_2f rbis;
      if ( nni != -1 )
         rbis = getBisector( pts[i], pts[ni], pts[nni] );

      cg::point_2f u, v;
      
      // Check if current bissector intersecs with left bisector
      if ( ppi != -1 )
         if ( cg::generic_intersection( cg::segment_2f( pts[i], pts[i] + offset * bis ),
                                        cg::segment_2f( pts[pi], pts[pi] + offset * lbis ), &u, &v ) /*||
              cg::generic_intersection( cg::segment_2f( pts[i], pts[i] - offset * bis ),
                                        cg::segment_2f( pts[pi], pts[pi] - offset * lbis ), &u, &v )*/ )
            return true;

      // Check if current bissector intersecs with left bisector
      if ( nni != -1 )
         if ( cg::generic_intersection( cg::segment_2f( pts[i], pts[i] + offset * bis ),
                                        cg::segment_2f( pts[ni], pts[ni] + offset * rbis ), &u, &v ) /*||
              cg::generic_intersection( cg::segment_2f( pts[i], pts[i] - offset * bis ),
                                        cg::segment_2f( pts[ni], pts[ni] - offset * rbis ), &u, &v )*/ )
            return true;
      return false;
   } // end of 'isBadVertex' function

   ////////////////////////////////////////////////////////////////////////////////////////////////
   
   float getInsideOffset( polyline const & pts, vertex_list const & list, int i, float offset )
   {
      // Check for correct vertex number
      if ( i == -1 )
         return true;
      
      int pi = list[i].prev;
      int ni = list[i].next;
      int ppi = -1; 
      //int nni = -1;

      cg::point_2f bis;
      
      // Calculate bisector in vertex i
      if ( pi == -1 || ni == -1 )
         return offset;
      else
      {   
         // Vertex i has next and previous vertices
         bis = getBisector( pts[pi], pts[i], pts[ni] );
         ppi = list[pi].prev;
      }
      
      // Calculate bisector in vertex pi
      cg::point_2f lbis;
      if ( ppi != -1 )
         lbis = getBisector( pts[ppi], pts[pi], pts[i] );

      cg::point_2f u, v;
      float of1 = offset;
      
      // Check if current bissector intersecs with left bisector
      if ( ppi != -1 )
         if ( cg::generic_intersection( cg::segment_2f( pts[i], pts[i] - offset * bis ),
                                        cg::segment_2f( pts[pi], pts[pi] - offset * lbis ), &u, &v ) )
            of1 = cg::norm( u - pts[i] ) / cg::norm( bis );
            
      return of1;
   } // end of 'getInsideOffset' function
   
   ////////////////////////////////////////////////////////////////////////////////////////////////

   void yieldResult( std::vector< polyindices > & results, polyline const & pts, vertex_list & list,
                     int first, int last, float offset )
   {
      polyindices res;
      res.push_back( first );
      
      int cur = list[first].next;
      if ( cur != last )
      {   
         res.push_back( cur );
         cur = list[cur].next;
      }

      float minOffset = offset;
      while ( cur != last )
      {
         float ofi = getInsideOffset( pts, list, cur, offset );
         if ( ofi < minOffset )
            minOffset = ofi;
         res.push_back( cur );
         cur = list[cur].next;
      }
      
      res.offset = minOffset;
      res.push_back( last );
      //res.n1 = n1;
      //res.n2 = n2;
      results.push_back( res );
   } // end of 'yieldResult' funciton
   
   /////////////////////////////////////////////////////////////////////////////////////////

   void processContourInside( std::vector< polyindices > & results, polyline const & pts, vertex_list & list,
                              int first, int last, float offset, float overstep, float r )
   {
      //return yieldResult( results, pts, list, first, last, offset );
         
      int lastres = first;
      int cur = list[first].next;
      int next = list[cur].next;
      while ( next != last && cur != last )
      {
         if ( isBadVertex( pts, list, cur, -offset ) )
         {
            int fwnext = list[next].next;
            list[cur].next = fwnext;
            if ( isBadVertex( pts, list, cur, -offset ) ||
                 isBadVertex( pts, list, cur, offset ) ||
                 cg::abs( getVertexOffset( pts[cur], pts[next], pts[fwnext] )) > offset * overstep )
            {
               list[cur].next = next;
               if ( getInsideOffset( pts, list, next, offset ) < r )
               {                    
                  list[next].next = -1;
                  yieldResult( results, pts, list, lastres, next, offset );
                  list[next].next = fwnext;
                  list[next].prev = -1;
                  lastres = next;
                  next = fwnext;
               }
            }
            else
            {
               list[fwnext].prev = cur;
               next = fwnext;
            }
         }
         cur = next;
         next = list[cur].next;
      }
      yieldResult( results, pts, list, lastres, last, offset );
   }
   
   /////////////////////////////////////////////////////////////////////////////////////////
   // @brief  contour processing function
   // @param  pts     - source island contour
   // @param  results - set of result polyline that approximate island
   // @param  offset  - foam strip width around island
   // @param  edge_length  - average length of foam segment ( distance between neighbour points )
   // @param  line_offset  - maximum linear deflection
   // @param  overstep     - maximum island overstepping
   //
   void processContour( polyline const & pts, std::vector< polyindices > & results, float offset, 
                        float edge_length = 60.f, float line_offset = 10.f, float overstep = 0.5f,
                        float max_chain_length = 1000.f )
   {
      int n = pts.size();
      if ( n < 3 )
         return;
      
      vertex_list list( n );

      // Build obb
      cg::naa_rect_2 rect;
      cg::build_naa_rect( &pts[0], n, rect );
      cg::point_2 size = rect.getrect().size();
      float d = cg::max( size.x, size.y ) / 2.5f;
      float r = cg::min( size.x, size.y ) / 4.0f;

      int first = 0, cur, next;
      
      if ( d < line_offset )
      {
         list[0].next = n / 3;
         list[n / 3].prev = 0;
         list[n / 3].next = (2 * n) / 3;
         list[(2 * n) / 3].prev = n / 3;
         list[(2 * n) / 3].next = 0;
      }
      else
      {
         // Fill vertex list
         for ( int i = 1; i < n; i++ )
         {
            list[i].prev = i - 1;
            list[i - 1].next = i;
         }
         list[0].prev = n - 1;
         list[n - 1].next = 0;
         
         // Make linear approximation using: edeg_length and line_offset
         cur  = first;
         next = list[first].next;
         do
         {
            int fwnext = list[next].next;
            if ( cg::norm( pts[cur] - pts[fwnext] ) < edge_length )
            {
               for ( int i = cur + 1; i != fwnext; i = (i + 1) % n )
                  if ( cg::abs( getVertexOffset( pts[cur], pts[i], pts[fwnext] )) > line_offset )
                     break;
               if ( i == fwnext )
               {
                  list[fwnext].prev = cur;
                  list[cur].next = fwnext;
                  list[next].prev = list[next].next = -1;
               }
               else
                  cur = next;
            }
            else
               cur = next;
            next = fwnext;
         } while ( next != first );
      }
      list[first].prev = -1;

      // Discard bad vertex
      int   lastres = first;
      cur = list[first].next;
      next = list[cur].next;
      float length = cg::norm( pts[first] - pts[cur] );
      
      while ( next != first && cur != first )
      {
         if ( isBadVertex( pts, list, cur, offset ) )
         {
            int fwnext = list[next].next;
            list[cur].next = fwnext;
            while ( isBadVertex( pts, list, cur, offset ) && fwnext != first )
               list[cur].next = fwnext = list[fwnext].next;
            
            int i = fwnext;
            if ( fwnext != first )
               for ( i = cur + 1; i != fwnext; i = (i + 1) % n )
                  if ( cg::abs( getVertexOffset( pts[cur], pts[i], pts[fwnext] )) > offset * overstep )
                     break;

            if ( i != fwnext || isBadVertex( pts, list, cur, offset ) )
            {
               list[cur].next = next;
               fwnext = list[next].next;
               list[next].next = -1;
               processContourInside( results, pts, list, lastres, next, offset, overstep, r );
               list[next].next = fwnext;
               list[next].prev = -1;
               lastres = next;
               length = cg::norm( pts[next] - pts[fwnext] );
               cur = fwnext;
               next = list[cur].next;
               continue;
            }
            else
            {
               list[fwnext].prev = cur;
               next = fwnext;
            }
         }
         length += cg::norm( pts[cur] - pts[next] );
         if ( length > max_chain_length )
         {
            int fwnext = list[next].next;
            list[next].next = -1;
            processContourInside( results, pts, list, lastres, next, offset, overstep, r );
            list[next].next = fwnext;
            list[next].prev = -1;
            lastres = next;
            length = cg::norm( pts[next] - pts[fwnext] );
            next = fwnext;
         }
         cur = next;
         next = list[cur].next;
      }
      
      if ( lastres != first )
      {
         list[lastres].prev = -1;
         list[first].next = -1;
      }
      else
         if ( results.size() > 0 )
            return;
      processContourInside( results, pts, list, lastres, first, offset, overstep, r );
   }

} // end of namespace 'foam'
