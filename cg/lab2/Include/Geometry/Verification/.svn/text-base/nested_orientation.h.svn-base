#pragma once

#include <stack>

#include "contours\readytouse\contours_set.h"

namespace cg
{
namespace verification
{

// Throwed when algorithm applied to contour with self-intersection.
struct nested_orientation_invalid_input_exception
   : virtual std::exception, virtual boost::exception
{
};

namespace detail
{
template < class VertexBuffer, class FwdIter, class OutIter >
   void find_contours_to_reverse( VertexBuffer const &vertices,
                                  FwdIter p, FwdIter q, OutIter out )
{
   find_contours_to_reverse( vertices, p, q, out,
      cg::epsilon< typename VertexBuffer::value_type::scalar_type >() );
}

template < class VertexBuffer, class FwdIter, class OutIter >
   void find_contours_to_reverse( VertexBuffer const &vertices,
                                  FwdIter p, FwdIter q, OutIter out,
                                  typename VertexBuffer::value_type::scalar_type /* eps */ )
{
   // rotated
   std::vector< bool > rotated;

   // create collision
   typedef cg::contours::gen::nested_contours_set<> collision_type;
   collision_type collision;
   for (; p != q; ++p)
   {
      std::vector< VertexBuffer::value_type > tmp;
      tmp.reserve(p->end() - p->begin());
      for (FwdIter::value_type::const_iterator it = p->begin(); it != p->end(); ++it)
         tmp.push_back(vertices.at(*it));

      if (cg::clockwise_ordered(p->begin(), p->end(), vertices))
      {
         rotated.push_back(true);
         std::reverse(tmp.begin(), tmp.end());
      }
      else
         rotated.push_back(false);

      collision.add_contour(tmp.begin(), tmp.end());
   }

   // build nested contours
   collision.process();

   // group contours      
   typedef cg::contours::contour_type contour_id;

   std::map< contour_id, bool > holes;
   std::set< cg::contours::contour_type> used_ids;

   for ( collision_type::contours_iterator it = collision.contours_begin( ); it != collision.contours_end( ); ++it )
   {
      if ( holes.find(*it) == holes.end() )
      {
         std::stack< contour_id > ids;
         ids.push( *it );
         used_ids.insert( *it );

         while (!ids.empty())
         {
            contour_id id  = ids.top();               

            if ( cg::contours::contour_type pid = collision.containing_contour( id ) )
            {
               if (holes.find(pid) == holes.end())
               {
                  if ( used_ids.find(pid) != used_ids.end() )
                     throw nested_orientation_invalid_input_exception();

                  ids.push( pid );
                  used_ids.insert(pid);
                  continue;
               }

               Assert( holes.find(pid) != holes.end() );

               holes[id] = !holes[pid];
            } 
            else
            {
               holes[id] = false;
            }

            used_ids.erase( used_ids.find(ids.top()) );
            ids.pop( );
         }
      }
   }

   // output
   for ( std::map< contour_id, bool >::const_iterator cit = holes.begin( ); cit != holes.end(); ++cit )
      *out++ = ( rotated[cit->first.group()] != cit->second );
}

} // End of 'detail' namespace

template< class ContourType >
   struct Dummy
{
   typedef typename ContourType::value_type value_type;

   template< class C >
      C const &at( C const &c ) const
   {
      return c;
   }
};

template < class FwdIter, class OutIter >
   void find_contours_to_reverse( FwdIter p, FwdIter q, OutIter out )
{
   detail::find_contours_to_reverse(Dummy< typename FwdIter::value_type > (), p, q, out); 
}

template< class VertexBuffer, class ContoursIterator, class OutIter >
   void check_nested_orientation( VertexBuffer const &vertices,
                                  ContoursIterator begin, ContoursIterator end,
                                  OutIter out,
                                  typename VertexBuffer::value_type::scalar_type eps )
{
   std::vector< bool > reverse;
   detail::find_contours_to_reverse(vertices, begin, end, std::back_inserter(reverse), eps);

   if (reverse.empty())
      return;

   for (size_t i = 0; i < reverse.size(); ++i)
   {
      if (reverse[i])
         *out++ = i;
   }
}

template< class VertexBuffer, class ContoursIterator, class OutIter >
   void check_nested_orientation( VertexBuffer const &vertices,
                                  ContoursIterator begin, ContoursIterator end,
                                  OutIter out )
{
   check_nested_orientation( vertices, begin, end, out,
      cg::epsilon< VertexBuffer::value_type::scalar_type >() );
}

template< class VertexBuffer, class ContoursIterator, class OutIter >
   void check_n_correct_nested_orientation( VertexBuffer const &vertices,
                                            ContoursIterator begin, ContoursIterator end,
                                            OutIter out,
                                            typename VertexBuffer::value_type::scalar_type eps )
{
   std::vector< bool > reverse;
   detail::find_contours_to_reverse(vertices, begin, end, std::back_inserter(reverse), eps);

   if (reverse.empty())
      return;

   size_t cIdx = 0;
   for (ContoursIterator cIt = begin; cIt != end; ++cIt, ++cIdx)
   {
      if (reverse[cIdx])
      {
         *out++ = cIdx;
         std::reverse(cIt->begin(), cIt->end());
      }
   }
}

template< class VertexBuffer, class ContoursIterator, class OutIter >
   void check_n_correct_nested_orientation( VertexBuffer const &vertices,
                                            ContoursIterator begin, ContoursIterator end,
                                            OutIter out )
{
   check_n_correct_nested_orientation( vertices, begin, end, out,
      cg::epsilon< VertexBuffer::value_type::scalar_type >() );
}

} // End of 'verification' namespace
} // End of 'cg' namespace
