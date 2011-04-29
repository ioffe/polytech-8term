#pragma once

namespace cg
{
namespace verification
{

namespace impl
{
   template< class VertexBuffer, class ContoursIterator, class OutIterator >
      void check_recurring_points( VertexBuffer const &vertices,
                                   ContoursIterator begin, ContoursIterator end,
                                   OutIterator out,
                                   typename VertexBuffer::value_type::scalar_type eps,
                                   bool closed )
   {
      typedef typename VertexBuffer::value_type::scalar_type scalar_type;
      typedef cg::point_t< scalar_type, 2 > point_type;

      size_t cntIdx = 0;
      for (ContoursIterator cIt = begin; cIt != end; ++cIt, ++cntIdx)
      {
         // Require at least 2 points.
         if (std::distance(cIt->begin(), cIt->end()) < 2)
            continue;

         point_type prevRef (std::numeric_limits< scalar_type >::max(),
                             std::numeric_limits< scalar_type >::max());

         for (ContoursIterator::value_type::const_iterator vIt = cIt->begin(); vIt != cIt->end(); ++vIt)
         {
            point_type ref;
            bool found = false;

            if (closed)
            {
               if (cg::eq(vertices[*vIt],
                          vertices[*(vIt == cIt->begin() ? util::prev(cIt->end()) : util::prev(vIt))], eps))
               {
                  found = true;
                  ref = vertices[*vIt];
               }
            }
            else if (vIt != cIt->begin())
            {
               if (cg::eq(vertices[*vIt], vertices[*util::prev(vIt)], eps))
               {
                  found = true;
                  ref = vertices[*vIt];
               }
            }

            if (found && !cg::eq(ref, prevRef, eps))
            {
               *out++ = std::make_pair(cntIdx, ref);
               prevRef = ref;
            }
         }
      }
   }

   template< class VertexBuffer, class ContoursIterator, class OutIterator >
      void check_n_correct_recurring_points( VertexBuffer const &vertices,
                                             ContoursIterator begin, ContoursIterator end,
                                             OutIterator out,
                                             typename VertexBuffer::value_type::scalar_type eps,
                                             bool closed )
   {
      typedef typename VertexBuffer::value_type::scalar_type scalar_type;
      typedef cg::point_t< scalar_type, 2 > point_type;

      size_t cntIdx = 0;
      for (ContoursIterator cIt = begin; cIt != end; ++cIt, ++cntIdx)
      {
         point_type prevRef (std::numeric_limits< scalar_type >::max(),
                             std::numeric_limits< scalar_type >::max());

         ContoursIterator::value_type::const_iterator vIt = cIt->begin();
         while (vIt != cIt->end() && std::distance(cIt->begin(), cIt->end()) >= 2)
         {
            point_type ref;
            bool removing = false;
            if (closed)
            {
               if (cg::eq(vertices[*vIt], vertices[*(vIt == cIt->begin() ? util::prev(cIt->end()) : util::prev(vIt))], eps))
                  removing = true;
            }
            else if (vIt != cIt->begin())
            {
               if (cg::eq(vertices[*vIt], vertices[*util::prev(vIt)], eps))
                  removing = true;
            }

            if (removing)
            {
               ref = vertices[*vIt];
               vIt = cIt->erase(vIt);

               if (!cg::eq(prevRef, ref, eps))
                  *out++ = std::make_pair(cntIdx, ref);
            }
            else
               ++vIt;
         }
      }
   }
} // End of 'impl' namespace

template< class VertexBuffer, class ContoursIterator, class OutIterator >
   void check_recurring_points( VertexBuffer const &vertices,
                                ContoursIterator begin, ContoursIterator end,
                                OutIterator out )
{
   impl::check_recurring_points(vertices, begin, end, out, 
      cg::epsilon< VertexBuffer::value_type::scalar_type >(), true);
}

template< class VertexBuffer, class ContoursIterator, class OutIterator >
   void check_recurring_points( VertexBuffer const &vertices,
                                ContoursIterator begin, ContoursIterator end,
                                OutIterator out,
                                typename VertexBuffer::value_type::scalar_type eps )
{
   impl::check_recurring_points(vertices, begin, end, out, eps, true);
}

template< class VertexBuffer, class ContoursIterator, class OutIterator >
   void check_n_correct_recurring_points( VertexBuffer const &vertices,
                                          ContoursIterator begin, ContoursIterator end,
                                          OutIterator out )
{
   impl::check_n_correct_recurring_points(vertices, begin, end, out,
      cg::epsilon< VertexBuffer::value_type::scalar_type >(), true);
}

template< class VertexBuffer, class ContoursIterator, class OutIterator >
   void check_n_correct_recurring_points( VertexBuffer const &vertices,
                                          ContoursIterator begin, ContoursIterator end,
                                          OutIterator out,
                                          typename VertexBuffer::value_type::scalar_type eps )
{
   impl::check_n_correct_recurring_points(vertices, begin, end, out, eps, true);
}

template< class VertexBuffer, class PolylinesIterator, class OutIterator >
   void check_recurring_points_in_polyline( VertexBuffer const &vertices,
                                PolylinesIterator begin, PolylinesIterator end,
                                OutIterator out )
{
   impl::check_recurring_points(vertices, begin, end, out, 
      cg::epsilon< VertexBuffer::value_type::scalar_type >(), false);
}

template< class VertexBuffer, class PolylinesIterator, class OutIterator >
   void check_recurring_points_in_polyline( VertexBuffer const &vertices,
                                PolylinesIterator begin, PolylinesIterator end,
                                OutIterator out,
                                typename VertexBuffer::value_type::scalar_type eps )
{
   impl::check_recurring_points(vertices, begin, end, out, eps, false);
}

template< class VertexBuffer, class PolylinesIterator, class OutIterator >
   void check_n_correct_recurring_points_in_polyline( VertexBuffer const &vertices,
                                          PolylinesIterator begin, PolylinesIterator end,
                                          OutIterator out )
{
   impl::check_n_correct_recurring_points(vertices, begin, end, out,
      cg::epsilon< VertexBuffer::value_type::scalar_type >(), false);
}

template< class VertexBuffer, class PolylinesIterator, class OutIterator >
   void check_n_correct_recurring_points_in_polyline( VertexBuffer const &vertices,
                                          PolylinesIterator begin, PolylinesIterator end,
                                          OutIterator out,
                                          typename VertexBuffer::value_type::scalar_type eps )
{
   impl::check_n_correct_recurring_points(vertices, begin, end, out, eps, false);
}

} // End of 'verification' namespace
} // End of 'cg' namespace
