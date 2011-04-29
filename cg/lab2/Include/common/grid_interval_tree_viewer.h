#pragma once

#include <common/grid_interval_tree.h>
#include "viewer/ViewerWindowEventListener.h"

namespace cg
{
   template <class Scalar = double, class SegmentId = size_t>
      struct grid_interval_tree_viewer_t
         : viewer::ViewerImpl
   {
      typedef grid_interval_tree_t<Scalar, SegmentId> grid_interval_tree_t;

      grid_interval_tree_viewer_t()
         : viewer::ViewerImpl("Grid interval tree viewer")
      {}

      void InitInstance(grid_interval_tree_t const & git)
      {
         git_ = &git;
      }

   private:

      bool draw_node(size_t segments, double t, double t_parent, size_t h)
      {
         double hd = h * 3;
         dc_.drawLine(point_2(t, -hd - 3), point_2(t_parent, -hd));
         return true;
      }

   public:
      void __stdcall OnDraw()
      {
         dc_.setColor(cg::color_red());
         git_->visit_nodes(boost::bind(&grid_interval_tree_viewer_t::draw_node, this, _1, _2, _3, _4));

         print_info();
      }

      void __stdcall OnMove( BOOL * redraw ) 
      {
         *redraw = TRUE;
         mouse_pos_ = dc_.getMousePos();
      }

   private:
      void print_info()
      {
         dc_.setColor( cg::color_white() );
         viewer::printer prn( dc_ );

         prn << "Mouse pos: (" << mouse_pos_.x << ", " << mouse_pos_.y << ")";
      }

   private:
      grid_interval_tree_t const * git_;
      cg::point_2f         mouse_pos_;
   };
}