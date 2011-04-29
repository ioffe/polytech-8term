#pragma once

#include "common\io_utils.h"

#include "primitives\point.h"
#include "primitives\range.h"
#include "primitives\rectangle.h"
#include "primitives\line.h"
#include "primitives\segment.h"

namespace cg
{
   template < class Scalar, class Char >
      std::basic_ostream< Char > & operator << (std::basic_ostream< Char > &out, point_t< Scalar, 2 > const &v)
   {   
      return out << "(" << v.x << "," << v.y << ")";  
   }

   template < class Scalar, class Char >
      std::basic_ostream< Char > & operator << (std::basic_ostream< Char > &out, point_t< Scalar, 3 > const &v)
   {   
      return out << "(" << v.x << "," << v.y << "," << v.z << ")";  
   }

   template < class Char >
      std::basic_istream< Char > & operator >> (std::basic_istream< Char > &in, point_2 &v)
   {   
      util::skip_char( in, '(' );
      in >> v.x;
      util::skip_char( in, ',' );
      in >> v.y;
      util::skip_char( in, ')' );
      return in;
   }

   template < class Scalar, class Char >
      std::basic_istream< Char > & operator >> (std::basic_istream< Char > &in, point_t< Scalar, 3 > &v)
   {   
      util::skip_char( in, '(' );
      in >> v.x;
      util::skip_char( in, ',' );
      in >> v.y;
      util::skip_char( in, ',' );
      in >> v.z;
      util::skip_char( in, ')' );
      return in;
   }

   template < class Char >
      std::basic_ostream< Char > & operator << (std::basic_ostream< Char > &out, line_2 const &L)
   { 
      return out << L.p() << " - " << L.r();  
   }

   template < class Char >
      std::basic_ostream< Char > & operator << (std::basic_ostream< Char > &out, segment_2 const &L)
   { 
      return out << L.P0() << " - " << L.P1();  
   }

   template < class Char, class Scalar >
      std::basic_ostream< Char > & operator << (std::basic_ostream< Char > & out, range_t< Scalar > const & rng)
   {
      return out << "[" << rng.lo() << ", " << rng.hi() << "]";
   }

   template < class Char, class Scalar >
      std::basic_istream< Char > & operator >> (std::basic_istream< Char > & in, range_t< Scalar > & rng)
   {
      Scalar lo, hi;
      util::skip_char(in, '[');
      in >> lo;
      util::skip_char(in, ',');
      in >> hi;
      util::skip_char(in, ']');
      rng = range_2(lo, hi);
      return in;
   }

   template < class Char >
      std::basic_ostream< Char > & operator << (std::basic_ostream< Char > & out, rectangle_2 const & rc)
   {
      return out <<
         "(" << rc.x.lo() << ", " << rc.y.lo() << ", " << rc.x.hi() << ", " << rc.y.hi() << ")";
   }

   template < class Char >
      std::basic_istream< Char > & operator >> (std::basic_istream< Char > & in, rectangle_2 & rc)
   {
      point_2 lo, hi;
      util::skip_char(in, '(');
      in >> lo.x;
      util::skip_char(in, ',');
      in >> lo.y;
      util::skip_char(in, ',');
      in >> hi.x;
      util::skip_char(in, ',');
      in >> hi.y;
      util::skip_char(in, ')');
      rc = rectangle_2(lo, hi);
      return in;
   }

   template< class Char >
      std::basic_istream< Char > & operator >> (std::basic_istream< Char > & in, segment_2 & seg)
   {
      in >> seg[0];
      util::skip_char(in, '-');
      in >> seg[1];
      return in;
   }
}
