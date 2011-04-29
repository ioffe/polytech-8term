#pragma once

#include <algorithm>
#include <iomanip>

#include "Common\util.h"
#include "Common\stl_utils.h"

#include "geometry\point_ops.h"
#include "geometry\polygon_2_fwd.h"
#include "geometry\primitives\segment.h"
#include "geometry\clockwise.h"
#include "geometry\polyline.h"
#include "Geometry\cgal\std_traits.h"
#include "Geometry\cgal\polygon_algorithm.h"

namespace cg
{
   enum ClippingOperation
   {
      Difference,
      Intersection,
      ExclusiveOR,
      Union
   };

#pragma pack(push, 1)
   template< class Traits >
      struct contour_2_t
         : Traits::vector< point_2 >::type
   {
      contour_2_t () : hole_ (false)
      {
      }

      typedef point_2 point_type;
      typedef typename Traits::vector< point_type >::type contour_representation;

      template< class FwdIter >
         contour_2_t ( FwdIter ptBegin, FwdIter ptEnd, bool hole )
            : contour_representation (ptBegin, ptEnd), hole_ (hole)
      {
         if (size() >= 2 && front() == back())
            pop_back();

         if (size() >= 3 && cg::clockwise_ordered(begin(), end()) != hole_)
            std::reverse(begin(), end());
      }

      template< class FwdIter >
         contour_2_t ( FwdIter ptBegin, FwdIter ptEnd )
            : contour_representation (ptBegin, ptEnd)
      {
         if (size() >= 2 && front() == back())
            pop_back();

         if (size() >= 3)
            hole_ = cg::clockwise_ordered(begin(), end());
         else
         {
            // Definitely not correct polygon.
            hole_ = false;
         }
      }

      template< class VertexBuffer, class IndexIterator >
         contour_2_t ( VertexBuffer const & vertices, IndexIterator ibegin, IndexIterator iend )
      {
         for (IndexIterator it = ibegin; it != iend; ++it)
            push_back(vertices[*it]);

         if (size() >= 2 && front() == back())
            pop_back();

         if (size() >= 3)
            hole_ = cg::clockwise_ordered(begin(), end());
         else
         {
            // Definitely not correct polygon.
            hole_ = false;
         }
      }

         struct CmpPt 
         {
            double eps;

            CmpPt(double eps) : eps(eps) {}

            template <class T>
               bool operator () (T const & lhs, T const & rhs) const 
               {
                  return eq(lhs, rhs, eps); 
               }
         };

      friend bool eq(contour_2_t const & lhs, contour_2_t const & rhs, double eps)
      {
         return std::equal(lhs.begin(), lhs.end(), rhs.begin(), CmpPt(eps));
      }

      bool hole() const
      {
         return hole_;
      }

      void updateType()
      {
         hole_ = cg::clockwise_ordered(begin(), end());
      }

      template < class Stream >
         friend void write( Stream & out, contour_2_t const & cnt )
      {
         write( out, static_cast< contour_representation const & >( cnt ) );
         write( out, cnt.hole_ );
      }

      template < class Stream >
         friend void read( Stream & in, contour_2_t & cnt )
      {
         read( in, static_cast< contour_representation & >( cnt ) );
         read( in, cnt.hole_ );
      }

   private:
      bool hole_;
   };

   template < class Traits >
      struct polygon_2_t
         : util::swap_helper< polygon_2_t< Traits > >
   {
      typedef point_2 point_type;

      polygon_2_t ()
      {
      }

      typedef contour_2_t< Traits > value_type;
      typedef typename Traits::vector< value_type >::type polycontour;

      typedef typename polycontour::iterator iterator;
      typedef typename polycontour::const_iterator const_iterator;

      template< class ContoursArray >
         explicit polygon_2_t ( ContoursArray const &contours )
      {
         for (typename ContoursArray::const_iterator cIt = contours.begin(); cIt != contours.end(); ++cIt)
            add_contour(cIt->begin(), cIt->end());
      }

      template < class VertexBuffer, class ContoursIterator >
         polygon_2_t( VertexBuffer const & vertices, ContoursIterator contours_begin, ContoursIterator contours_end )
         {
            for ( ContoursIterator c_it = contours_begin; c_it != contours_end; ++c_it )
            {
               std::vector< point_type > points;               
               typedef typename std::iterator_traits< ContoursIterator >::value_type my_contour_t;
               //typedef typename std::iterator_traits< my_contour_t >::value_type my_contour_elem_t;
               for ( my_contour_t::const_iterator it = c_it->begin(); it != c_it->end(); ++it )
               {
                  points.push_back( vertices[ *it ] );
               }
               add_contour( points.begin(), points.end() );
            }
         }

   public:
      friend void clip_simple_base( ClippingOperation op, polygon_2_t const &sub, polygon_2_t const &clip, polygon_2_t & res );
      friend void clip_simple_base_subd( ClippingOperation op, polygon_2_t &sub, polygon_2_t &clip, polygon_2_t & res );

      template< class PolyFwdIterator > friend
         void clip_base( ClippingOperation op, PolyFwdIterator begin, PolyFwdIterator end, polygon_2_t & res );

      template< class PolyFwdIterator > friend
         void clip_base_subd( ClippingOperation op, PolyFwdIterator begin, PolyFwdIterator end, polygon_2_t & res );

      template< class FwdIter >
         polygon_2_t ( FwdIter p, FwdIter q )
      {   
         assign(p, q); 
      }

      void clear () 
      {
         util::clear(poly);
      }

      template <class FwdIter>
         void assign( FwdIter p, FwdIter q ) 
      {
         clear();
         poly.push_back(value_type (p, q, false));
      };

      friend bool eq(polygon_2_t const & lhs, polygon_2_t const & rhs, double eps)
      {
         return std::equal(lhs.poly.begin(), lhs.poly.end(), rhs.poly.begin(), typename value_type::CmpPt(eps));
      }

      struct CmpContour 
      {
         // todo: effective implementation
         bool operator () (value_type const & lhs, value_type const & rhs) const 
         {
            return std::lexicographical_compare(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
         }
      };

      void normalize()
      {
         std::sort(poly.begin(), poly.end(), CmpContour());
      }

      void push_back( value_type const &val )
      {
         poly.push_back(val);
      }

      template< class FwdIter >
         void add_contour( FwdIter p, FwdIter q, bool hole )
      {
         poly.push_back(value_type (p, q, hole));
      }

      template< class FwdIter >
         void add_contour( FwdIter p, FwdIter q )
      {
         poly.push_back(value_type (p, q));
      }

      const_iterator begin() const { return poly.begin(); }
      const_iterator end  () const { return poly.end(); }

      iterator begin() { return poly.begin(); }
      iterator end  () { return poly.end(); }

      value_type       &back()       { return (*(end() - 1)); }
      value_type const &back() const { return (*(end() - 1)); }

      value_type const &operator[] ( size_t i ) const
      {
         return poly[i];
      }

      value_type &operator[] ( size_t i )
      {
         return poly[i];
      }

      size_t size() const 
      {
         return poly.size();
      }

      size_t hole_size( ) const 
      {
         size_t ret = 0;
         for (size_t i = 0; i < poly.size(); ++i)
            ret += poly[i].hole();
         return ret;
      }

      bool operator ==( polygon_2_t const & other )
      {
         if ( size() != other.size() )
            return false;

         for (size_t c = 0; c < size(); ++c)
            if ( (*this)[c] != other[c]  )
               return false;

         return true;
      }

      void swap( polygon_2 & p )
      {
         poly.swap( p.poly );
      }

   private:
      template < class Stream >
         friend void write( Stream & out, polygon_2_t const & poly )
      {
         write( out, poly.poly );
      }

      template < class Stream >
         friend void read( Stream & in, polygon_2_t & poly )
      {
         read( in, poly.poly );
      }

      polycontour poly;
   };
#pragma pack(pop)

   inline cg::rectangle_2 bounding(polygon_2 const & p)
   {
      rectangle_2 bb;

      for (polygon_2::const_iterator cit = p.begin(); cit != p.end(); ++cit)
      {
         contour_2 c = *cit;

         if (!c.hole())
            bb |= rectangle_2::bounding(c.begin(), c.end());
      }

      return bb;
   }

   inline cg::segment_2 closest_segment(cg::polygon_2 const & p, cg::point_2 const & pt)
   {
      cg::segment_2 the_best;
      double best_distance = 1e10;

      for (cg::polygon_2::const_iterator it = p.begin(); it != p.end(); ++it)
      {
         for (cg::contour_2::const_iterator it_1 = it->begin(); it_1 < it->end() - 1; ++it_1)
         {
            segment_2 s(it_1[0],it_1[1]);

            if (cg::make_min(best_distance, cg::distance(pt, s)))
            {
               the_best = s;
            }
         }

         if (!it->empty() && it->front() != it->back())
         {
            segment_2 s(it->back(),it->front());

            if (cg::make_min(best_distance, cg::distance(pt, s)))
            {
               the_best = s;
            }
         }
      }

      return the_best;
   }

   inline bool contains(cg::polygon_2 const & p, cg::point_2 const & pt)
   {
      cg::segment_2 const closest = closest_segment(p, pt);
      return cg::left_turn_strict(pt, closest.P0(), closest.P1());
   }

   template < class Traits > 
   double area( polygon_2_t< Traits > const & p )
   {
      double res = 0;

      for ( size_t i = 0; i != p.size(); ++i )
         res += signed_area( p[i].begin(), p[i].end() );
      
      return res;
   }

   template< class PointType >
      void indexate_polygon( polygon_2 const &poly, std::vector< PointType > &vertices,
                             std::vector< std::vector< size_t > > &indices )
   {
      for (polygon_2::const_iterator ci = poly.begin(); ci != poly.end(); ++ci)
      {
         indices.push_back(std::vector< size_t > ());
         indices.back().reserve(ci->size());
         for (polygon_2::value_type::const_iterator vIt = ci->begin(); vIt != ci->end(); ++vIt)
         {
            indices.back().push_back(vertices.size());
            vertices.push_back(*vIt);
         }
      }
   }

   inline bool is_finite( polygon_2 const & poly )
   {
      for ( size_t i = 0; i < poly.size(); ++i )
         for ( size_t j = 0; j < poly[i].size(); ++j )
            if ( !is_finite( poly[i][j] ) )
               return false;

      return true;
   }
}

namespace std
{
   template <>
   inline void swap< cg::polygon_2 >( cg::polygon_2 & a, cg::polygon_2 & b )
   {
      swap( a, b );
   }
}
