#pragma once

#include "cgal_triangulation.h"
#include "viewer/ViewerWindowEventListener.h"

namespace cg            {
namespace triangulation {

   template < class Triangulation >
   void draw_triangulation( viewer::viewer_dc & dc, Triangulation const & trg, bool draw_constraints_only )
   {
      for ( typename Triangulation::edges_iterator i = trg.edges_begin(); i != trg.edges_end(); ++i )
      {
         typename Triangulation::face_handle f = i->first;
         int v = i->second;

         typename Triangulation::vertex_handle v1 = f->vertex( f->cw( v ) ); 
         typename Triangulation::vertex_handle v2 = f->vertex( f->ccw( v ) );

         dc.drawPixel( trg.construct( v1 ), 3 );
         dc.drawPixel( trg.construct( v2 ), 3 );

         if ( f->is_constrained( v ) )
            dc.setColor( cg::color_red() );
         else
            dc.setColor( cg::color_green() );

         if ( f->is_constrained( v ) || !draw_constraints_only )
            dc.drawLine( trg.construct( v1 ), trg.construct( v2 ) );
      }

      for ( typename Triangulation::vertices_iterator i = trg.vertices_begin(); i != trg.vertices_end(); ++i )
      {
         dc.setColor( cg::color_green() );
         dc.drawPixel( trg.construct( i ), 3 );
      }
   }

   template < class Triangulation >
      struct cgal_triangulation_viewer
         : viewer::ViewerImpl
   {
      cgal_triangulation_viewer()
         : viewer::ViewerImpl( "CGAL triangulation viewer" )
      {}

      void InitInstance( Triangulation const * trg )
      {
         trg_ = trg;
         print_coords_ = false;
         draw_constraints_only_ = false;
      }

      void __stdcall OnDraw() 
      {
         draw_triangulation( dc_, *trg_, draw_constraints_only_ );
         draw_vertices();
         print_info();
      }

      void __stdcall OnMove( BOOL * redraw ) 
      {
         *redraw = TRUE;
         mouse_pos_ = dc_.getMousePos();
      }

      BOOL __stdcall OnKey( DWORD key, BOOL down, BOOL * redraw )
      {
         *redraw = true;

         if ( key == VK_RETURN && down )
         {
            print_coords_ = !print_coords_;
            return TRUE;
         }

         if ( key == 'C' && down )
         {
            draw_constraints_only_ = !draw_constraints_only_;
            return TRUE;
         }

         return FALSE;
      }

   private:
      typedef typename Triangulation::vertex_handle   vertex_handle;
      typedef typename Triangulation::face_handle     face_handle;

      void draw_vertices()
      {
         cg::rectangle_2f const & v = dc_.globalViewport();

         if ( !print_coords_ )
            return;

         for ( typename Triangulation::vertices_iterator i = trg_->vertices_begin(); i != trg_->vertices_end(); ++i )
         {
            if ( v.contains( trg_->construct( i ) ) )
            {
               dc_.setColor( cg::color_white() );
               viewer::printer prn( dc_, dc_.global2screen( trg_->construct( i ) ) );
               prn << trg_->construct( i );
            }
         }
      }

      virtual void print_info()
      {
         dc_.setColor( cg::color_white() );
         viewer::printer prn( dc_ );

         prn << "Mouse pos: " << mouse_pos_;
         prn << "Press 'c' to draw constraints only";
         prn << "Press return to print point coordinates";
      }

   protected:
      Triangulation const *   trg_;
      cg::point_2f            mouse_pos_;
      bool                    print_coords_;
      bool                    draw_constraints_only_;
   };

}}