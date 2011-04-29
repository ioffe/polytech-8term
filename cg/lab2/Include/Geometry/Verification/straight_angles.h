#pragma once

namespace cg
{
namespace verification
{

// In this context angle ABC is straight with precision 'eps' if:
// cg::eq(cg::angle(BA, BC), cg::pi, eps)

namespace impl
{
   template< class VertexBuffer, class ContoursIterator, class OutIterator >
      void check_straight_angles( VertexBuffer const &vertices,
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
         // Require at least 3 points.
         if (std::distance(cIt->begin(), cIt->end()) < 3)
            continue;

         for (ContoursIterator::value_type::const_iterator vIt = cIt->begin(); vIt != cIt->end(); ++vIt)
         {
            bool found = false;

            if (closed)
            {
               point_type const pointPrev = vertices[*(vIt != cIt->begin() ? util::prev(vIt) : util::prev(cIt->end()))];
               point_type const pointCur  = vertices[*vIt];
               point_type const pointNext = vertices[*(util::next(vIt) != cIt->end() ? util::next(vIt) : cIt->begin())];

               if (cg::eq(cg::angle(pointPrev - pointCur, pointNext - pointCur), cg::pi, eps))
                  found = true;
            }
            else if (vIt != cIt->begin() && vIt != util::prev(cIt->end()))
            {
               point_type const pointPrev = vertices[*util::prev(vIt)];
               point_type const pointCur  = vertices[*vIt];
               point_type const pointNext = vertices[*util::next(vIt)];

               if (cg::eq(cg::angle(pointPrev - pointCur, pointNext - pointCur), cg::pi, eps))
                  found = true;
            }

            if (found)
               *out++ = std::make_pair(cntIdx, vertices[*vIt]);
         }
      }
   }

   template< class VertexBuffer, class ContoursIterator, class OutIterator >
      void check_n_correct_straight_angles( VertexBuffer const &vertices,
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
         ContoursIterator::value_type::const_iterator vIt = cIt->begin();
         while (vIt != cIt->end() && distance(cIt->begin(), cIt->end()) >= 3)
         {
            bool isRemoving = false;

            if (closed)
            {
               point_type const pointPrev = vertices[*(vIt != cIt->begin() ? util::prev(vIt) : util::prev(cIt->end()))];
               point_type const pointCur  = vertices[*vIt];
               point_type const pointNext = vertices[*(util::next(vIt) != cIt->end() ? util::next(vIt) : cIt->begin())];

               if (cg::eq(cg::angle(pointPrev - pointCur, pointNext - pointCur), cg::pi, eps))
                  isRemoving = true;
            }
            else if (vIt != cIt->begin() && vIt != util::prev(cIt->end()))
            {
               point_type const pointPrev = vertices[*util::prev(vIt)];
               point_type const pointCur  = vertices[*vIt];
               point_type const pointNext = vertices[*util::next(vIt)];

               if (cg::eq(cg::angle(pointPrev - pointCur, pointNext - pointCur), cg::pi, eps))
                  isRemoving = true;
            }

            if (isRemoving)
            {
               point_type ref = vertices[*vIt];
               ContoursIterator::value_type::const_iterator curVIt = vIt;
               vIt = cIt->erase(vIt);

               *out++ = std::make_pair(cntIdx, ref);
            }
            else
               ++vIt;
         }
      }
   }
} // End of 'impl' namespace

template< class VertexBuffer, class ContoursIterator, class OutIterator >
   void check_straight_angles( VertexBuffer const &vertices,
                               ContoursIterator begin, ContoursIterator end,
                               OutIterator out )
{
   impl::check_straight_angles(vertices, begin, end, out, 
      cg::epsilon< VertexBuffer::value_type::scalar_type >(), true);
}

template< class VertexBuffer, class ContoursIterator, class OutIterator >
   void check_straight_angles( VertexBuffer const &vertices,
                               ContoursIterator begin, ContoursIterator end,
                               OutIterator out,
                               typename VertexBuffer::value_type::scalar_type eps )
{
   impl::check_straight_angles(vertices, begin, end, out, eps, true);
}

template< class VertexBuffer, class ContoursIterator, class OutIterator >
   void check_n_correct_straight_angles( VertexBuffer const &vertices,
                                         ContoursIterator begin, ContoursIterator end,
                                         OutIterator out )
{
   impl::check_n_correct_straight_angles(vertices, begin, end, out,
      cg::epsilon< VertexBuffer::value_type::scalar_type >(), true);
}

template< class VertexBuffer, class ContoursIterator, class OutIterator >
   void check_n_correct_straight_angles( VertexBuffer const &vertices,
                                         ContoursIterator begin, ContoursIterator end,
                                         OutIterator out,
                                         typename VertexBuffer::value_type::scalar_type eps )
{
   impl::check_n_correct_straight_angles(vertices, begin, end, out, eps, true);
}

template< class VertexBuffer, class PolylinesIterator, class OutIterator >
   void check_straight_angles_in_polyline( VertexBuffer const &vertices,
                                           PolylinesIterator begin, PolylinesIterator end,
                                           OutIterator out )
{
   impl::check_straight_angles(vertices, begin, end, out, 
      cg::epsilon< VertexBuffer::value_type::scalar_type >(), false);
}

template< class VertexBuffer, class PolylinesIterator, class OutIterator >
   void check_straight_angles_in_polyline( VertexBuffer const &vertices,
                                            PolylinesIterator begin, PolylinesIterator end,
                                            OutIterator out,
                                            typename VertexBuffer::value_type::scalar_type eps )
{
   impl::check_straight_angles(vertices, begin, end, out, eps, false);
}

template< class VertexBuffer, class PolylinesIterator, class OutIterator >
   void check_n_correct_straight_angles_in_polyline( VertexBuffer const &vertices,
                                                     PolylinesIterator begin, PolylinesIterator end,
                                                     OutIterator out )
{
   impl::check_n_correct_straight_angles(vertices, begin, end, out,
      cg::epsilon< VertexBuffer::value_type::scalar_type >(), false);
}

template< class VertexBuffer, class PolylinesIterator, class OutIterator >
   void check_n_correct_straight_angles_in_polyline( VertexBuffer const &vertices,
                                                     PolylinesIterator begin, PolylinesIterator end,
                                                     OutIterator out,
                                                     typename VertexBuffer::value_type::scalar_type eps )
{
   impl::check_n_correct_straight_angles(vertices, begin, end, out, eps, false);
}

} // End of 'verification' namespace
} // End of 'cg' namespace
