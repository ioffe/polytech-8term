#pragma once

#include "geometry/polygon_2.h"
#include "geometry/polygon_2_io.h"

#include "viewer/ViewerWindowEventListener.h"

#pragma push_macro("min")
#pragma push_macro("max")

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

#include <atlwin.h>
#include <wtl\atlapp.h>
#include <wtl\atldlgs.h>

#pragma pop_macro("min")
#pragma pop_macro("max")

namespace cg
{
   struct polygon_2_viewer
      : viewer::ViewerImpl
   {
      polygon_2_viewer()
         : viewer::ViewerImpl( "Polygon_2 viewer" )
      {}

      void InitInstance( cg::polygon_2 const & poly )
      {
         poly_ = &poly;
      }

      BOOL __stdcall OnKey( DWORD key, BOOL down, BOOL * )
      {
         if ( key == 'S' && !down )
         {
            CFileDialog dlg( FALSE );
            if ( dlg.DoModal() == IDOK )
               std::ofstream( dlg.m_szFileName ) << *poly_;
            return TRUE;
         }

         return FALSE;
      }

      void __stdcall OnDraw() 
      {
         dc_.setColor( cg::color_white() );
         for ( size_t i = 0; i != poly_->size(); ++i )
            draw_contour( dc_, (*poly_)[i], 3 );

         viewer::printer prn( dc_ );
         prn << "Mouse pos: (" << dc_.getMousePos().x << ", " << dc_.getMousePos().y << ")";
         prn << "S - save to file";
      }

   private:
      cg::polygon_2 const * poly_;
   };
}