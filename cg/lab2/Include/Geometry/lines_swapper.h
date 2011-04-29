// transform (matrix) lines swapper class
// constructs matrix for swapping transform (matrix) columns (rows)
// example:
//    typedef cg::swap_line_t<float, 3> line;
//    cg::lines_swapper_t<float, 3> swapper(cg::LS_swap_columns, line(0, 1), line(2, 1), line(1, -1));
//                                                                    |  |
//                                                       line index - |  |
//                                                                       |
//                                                          multiplier - |
//
//    cg::matrix_3f matrix;
//    cg::matrix_3f swapped_matrix = swapper(matrix);
// The above code generates the following columns swapping equence:
// col[0] ->  col[0]
// col[1] ->  col[2]
// col[2] -> -col[1]

#ifndef BOOST_PP_IS_ITERATING

#ifndef LINES_SWAPPER_H_INCLUDED
#define LINES_SWAPPER_H_INCLUDED

#include "boost\preprocessor.hpp"
#include "Geometry\primitives.h"

#define LS_max_dimensions 4
#define LS_min_dimensions 2

namespace cg
{
   enum line_swap_type
   {
      LS_swap_rows   = 0,
      LS_swap_columns,
   };

   template< typename S, size_t N >
   class swap_line_t
   {
   public:
      swap_line_t( size_t component_idx, S val )
      {
         Assert(cg::eq(cg::abs(val), (S)1));

         line_[component_idx] = val;
      }

      template< typename _S >
      swap_line_t( swap_line_t<_S, N> const & other )
         : line_(other.line_)
      {
      }

      point_t<S, N> const & get() const
      {
         return line_;
      }

   private:
      point_t<S, N> line_;

   template< typename , size_t > friend class swap_line_t;
   };

   // Primary template
   template< typename S, size_t N >
   class lines_swapper_t;
}

// Generate specializations
#define  BOOST_PP_ITERATION_LIMITS (LS_min_dimensions, LS_max_dimensions)
#define  BOOST_PP_FILENAME_1       "lines_swapper.h"
#include BOOST_PP_ITERATE()

#undef BOOST_PP_FILENAME_1
#undef BOOST_PP_ITERATION_LIMITS

#endif // LINES_SWAPPER_H_INCLUDED

#else // BOOST_PP_IS_ITERATING

#define n BOOST_PP_ITERATION()

#define LS_line(S, N) swap_line_t<S, N> const &
#define LS_write_row(z, N, unused) swap_matrix_.put_row(N, BOOST_PP_CAT(axis, N).get());

namespace cg
{
   template< typename S >
   class lines_swapper_t<S, n>
   {
   public:
      lines_swapper_t( line_swap_type swap_type, BOOST_PP_ENUM_PARAMS(n, LS_line(S, n) axis) )
         : swap_type_(swap_type)
      {
         BOOST_PP_REPEAT(n, LS_write_row, ~);
      }

      template< typename _S >
      lines_swapper_t( lines_swapper_t<_S, n> const & other )
         : swap_type_  (other.swap_type_)
         , swap_matrix_(other.swap_matrix_)
      {
      }

      transform_t<S, n> operator()( transform_t<S, n> const & tform ) const
      {
         if (swap_type_ == LS_swap_rows)
            return transform_t<S, n>(swap_matrix_, ss_unscaled) * tform;
         else if (swap_type_ == LS_swap_columns)
            return tform * transform_t<S, n>(swap_matrix_, ss_unscaled);
         else
         {
            Assert(false);
            return tform;
         }
      }

      matrix_t<S, n> operator()( matrix_t<S, n> const & matr ) const
      {
         if (swap_type_ == LS_swap_rows)
            return swap_matrix_ * matr;
         else if (swap_type_ == LS_swap_columns)
            return matr * swap_matrix_;
         else
         {
            Assert(false);
            return matr;
         }
      }

   private:
      line_swap_type swap_type_;
      matrix_t<S, n> swap_matrix_;

      template< typename, size_t > friend class lines_swapper_t;
   };
}

#undef LS_write_row
#undef LS_line
#undef n

#endif // BOOST_PP_IS_ITERATING
