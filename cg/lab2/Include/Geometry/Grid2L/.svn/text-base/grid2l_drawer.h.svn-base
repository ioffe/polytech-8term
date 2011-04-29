#pragma once

#include "Geometry\aa_transform.h"
#include "Geometry\grid_params.h"
#include "Viewer\ViewerWnd.h"

// ToDo: ѕеренести эту радость в ViewerWindow
namespace cg
{
    inline grid_params getSubGrid( grid_params const & gp, point_2i const &idx, point_2i const &subdivision) 
    {
       Assert( subdivision.x != 0 && subdivision.y != 0 );

       return grid_params ( gp.origin() + (idx & gp.unit()), gp.unit() / subdivision, subdivision );
    }
        
    
    inline void DrawGrid(IViewerWindowContext2D *dc, cg::coloraf color, grid_params const &gp, float line_width)
    {
        dc -> SetColor ( color );
        
        point_2 pts[2];
        
        {
            pts[0] = point_2(gp.origin());
            
            pts[1] = pts[0]; 
            pts[1].y += gp.unit().y * gp.extents().y;
            
            for (int i = 0; i <= gp.extents().x; ++i)
            {
                dc -> Lines( pts, line_width );
                
                pts[0].x += gp.unit().x; 
                pts[1].x += gp.unit().x;
            }
        }
        
        {
            pts[0] = point_2(gp.origin());
            
            pts[1] = pts[0]; 
            pts[1].x += gp.unit().x * gp.extents().x;
            
            for (int i = 0; i <= gp.extents().y; ++i)
            {
                dc -> Lines( pts, line_width );
                
                pts[0].y += gp.unit().y; 
                pts[1].y += gp.unit().y;
            }
        }
    }
    
    struct GridDrawer
    {
        typedef IViewerWindowContext2DPtr dc_type;
        
        GridDrawer(
            dc_type         dc, 
            cg::coloraf     color,
            // положение €чейки в экранных координатах
            grid_params const & gp,
            float           line_width) 
            :   dc_       (dc) 
            ,   color_    (color)  
            ,   gp_       (gp)
            ,   line_width_ (line_width)
        {}
        
        ~GridDrawer()
        {
            DrawGrid(dc_, color_, gp_, line_width_);
        }
        
    protected:
        dc_type     dc_;
        cg::coloraf color_;
        grid_params gp_;
        float       line_width_;
    };
    
    template <class G, class Derived>
        struct grid2l_drawer_base
    {
        typedef IViewerWindowContext2DPtr dc_type;
        
        typedef G                           grid_type;
        typedef typename G::bigcell_type    bigcell_type;
        typedef typename G::smallcell_type  smallcell_type;
        
        grid2l_drawer_base (const grid_type & grid, dc_type dc)
           : dc_(dc)
           , grid_(grid)
        {}
        
        struct SmallCellProcessor : GridDrawer
        {
            SmallCellProcessor(
                dc_type         dc, 
                // положение большой €чейки в экранных координатах
                grid_params const &gp)
                
                :   GridDrawer(dc, cg::coloraf(cg::colorf(.3f, .3f, .3f), 0.5f), gp, 1.f)
            {}
            
            // to be overriden by user
            template <class State>
                bool operator () (State const &, smallcell_type &)
            {
                return false;
            }
        };
        
        struct BigCellProcessor : GridDrawer
        {
            BigCellProcessor(
                dc_type         dc, 
                // положение  растра в экранных координатах
                grid_params const & gp) 
                
//                :   GridDrawer(dc, Color(1.0f,1.0f,.0f,1.f), gp)
                :   GridDrawer(dc, cg::coloraf(cg::colorf(.4f, .7f, .7f), 0.5f), gp, 3.f)
            {}

            typedef typename Derived::SmallCellProcessor  SmallCellProcessor;
            
            template <class State>
                SmallCellProcessor processbigcell(State & state, bigcell_type &bcell)
            {
              if ( bcell )  
                 return SmallCellProcessor(dc_, getSubGrid(gp_, state, bcell.extents()));
              else
                 return SmallCellProcessor(dc_, grid_params());
            }

            template <class State>
                bool postprocessbigcell(State const &state, bigcell_type &bcell)
            {
                return false;
            }
        };
        
        BigCellProcessor processgrid()
        {
           return typename Derived::BigCellProcessor
              (dc_, grid_params(grid_.origin(), grid_.unit(), grid_.extents()));
        }
        
        protected:
            dc_type dc_;
            const grid_type & grid_;
    };

   namespace details
   {
      template < class Grid >
      struct grid2l_drawer
         : cg::grid2l_drawer_base< Grid, grid2l_drawer< Grid > >
      {
         typedef 
            cg::grid2l_drawer_base< Grid, grid2l_drawer< Grid > >
            base;
         
         typedef typename base::BigCellProcessor   BigCellProcessor;
         typedef typename base::SmallCellProcessor SmallCellProcessor;

         grid2l_drawer( Grid const & grid, IViewerWindowContext2DPtr dc)
            : base(grid, dc)
         {}
      };
   }

   template < class Grid, class viewer_dc_type >
   void draw_grid2l( viewer_dc_type & dc, Grid const & grid ) 
   {         
      details::grid2l_drawer< Grid > processor( grid, dc.ptr() );
      cg::visit_every_cell( grid, processor );
   }
}