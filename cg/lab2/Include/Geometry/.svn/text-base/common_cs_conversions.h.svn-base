#pragma once

#include "lines_swapper.h"

namespace cg
{
   template< typename S >
   lines_swapper_t<S, 4> const & axes_swapper_modelview_cg_2_gl()
   {
      typedef swap_line_t<S, 4> swap_line_t;

      static lines_swapper_t<S, 4> const swapper(LS_swap_rows,
                                                 swap_line_t(0, (S) 1),
                                                 swap_line_t(2, (S) 1),
                                                 swap_line_t(1, (S)-1),
                                                 swap_line_t(3, (S) 1));

      return swapper;
   }

   template< typename S >
   lines_swapper_t<S, 4> const & axes_swapper_modelview_gl_2_cg()
   {
      typedef swap_line_t<S, 4> swap_line_t;

      static lines_swapper_t<S, 4> const swapper(LS_swap_rows,
                                                 swap_line_t(0, (S) 1),
                                                 swap_line_t(2, (S)-1),
                                                 swap_line_t(1, (S) 1),
                                                 swap_line_t(3, (S) 1));

      return swapper;
   }

   template< typename S >
   lines_swapper_t<S, 4> const & axes_swapper_viewmodel_cg_2_gl()
   {
      typedef swap_line_t<S, 4> swap_line_t;

      static lines_swapper_t<S, 4> const swapper(LS_swap_columns,
                                                 swap_line_t(0, (S) 1),
                                                 swap_line_t(2, (S) 1),
                                                 swap_line_t(1, (S)-1),
                                                 swap_line_t(3, (S) 1));

      return swapper;
   }

   template< typename S >
   lines_swapper_t<S, 4> const & axes_swapper_viewmodel_gl_2_cg()
   {
      typedef swap_line_t<S, 4> swap_line_t;

      static lines_swapper_t<S, 4> const swapper(LS_swap_columns,
                                                 swap_line_t(0, (S) 1),
                                                 swap_line_t(2, (S)-1),
                                                 swap_line_t(1, (S) 1),
                                                 swap_line_t(3, (S) 1));

      return swapper;
   }
}
