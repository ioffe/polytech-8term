#pragma once

#include "Geometry\aa_transform.h"
#include "Geometry\grid_params.h"
#include "visit_grid2l_by_beam.h"

namespace cg {
namespace skeleton {
    
    inline void DrawGrid(IViewerWindowContext2D *dc, cg::coloraf color,
                         grid_params const &gp, float line_width, cg::point_2 const &basePoint )
    {
        dc -> SetColor ( color );
        
        point_2 pts[2];
        
        {
            pts[0] = gp.origin() + basePoint;
            
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
            pts[0] = gp.origin() + basePoint;
            
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
            float           line_width,
            cg::point_2 const & basePoint ) 
            :   dc_       (dc) 
            ,   color_    (color)  
            ,   gp_       (gp)
            ,   line_width_ (line_width)
            ,   base_point_ (basePoint)
        {}
        
        ~GridDrawer()
        {
            DrawGrid(dc_, color_, gp_, line_width_, base_point_);
        }
        
    protected:
        dc_type     dc_;
        cg::coloraf color_;
        grid_params gp_;
        float       line_width_;
        cg::point_2 base_point_;
    };

    struct skeleton_grid_drawer_simple
       : cg::grid2l_visitor_base< cg::skeleton::igrid_type, skeleton_grid_drawer_simple >
    {
       typedef IViewerWindowContext2DPtr dc_type;

       typedef cg::skeleton::igrid_type grid_type;
       typedef grid_type::bigcell_type      bigcell_type;
       typedef grid_type::smallcell_type    smallcell_type;

       skeleton_grid_drawer_simple ( cg::point_2 const &basePoint, grid_type const &grid,
                                     dc_type dc, cg::colorf const &smallCellColor )
          : basePoint_ (basePoint), grid_ (grid), dc_(dc)
          , smallCellColor_ (smallCellColor)
       {
       }

       template <class State>
          bool operator () ( State const &st, smallcell_type &cell )
       {
          cg::raster_2 bc_raster = grid_.bigcellraster(st.big);
          point_2 sc_pos = bc_raster(st.small);

          dc_ -> SetColor ( smallCellColor_, 0.25f );
          point_2 pts[2];
          pts[0] = sc_pos + basePoint_;
          pts[1] = sc_pos + basePoint_ + point_2 (bc_raster.unit().x, 0);
          dc_ -> Lines( pts, 4 );
          pts[0] = sc_pos + basePoint_;
          pts[1] = sc_pos + basePoint_ + point_2 (0, bc_raster.unit().y);
          dc_ -> Lines( pts, 4 );
          pts[0] = sc_pos + basePoint_ + point_2 (0, bc_raster.unit().y);
          pts[1] = sc_pos + basePoint_ + bc_raster.unit();
          dc_ -> Lines( pts, 4 );
          pts[0] = sc_pos + basePoint_ + point_2 (bc_raster.unit().x, 0);
          pts[1] = sc_pos + basePoint_ + bc_raster.unit();
          dc_ -> Lines( pts, 4 );

          return false;
       }

       typedef empty_processor SideProcessor;
       SideProcessor & side_processor(int, int) { return dummy_; }

    private:
       cg::point_2 basePoint_;
       dc_type dc_;
       grid_type const &grid_;

       SideProcessor dummy_;
       cg::colorf smallCellColor_;
    };

    struct skeleton_grid_drawer
       //: cg::grid2l_visitor_base< skeleton_grid::grid_type, skeleton_grid_drawer >
    {
        typedef IViewerWindowContext2DPtr dc_type;
        
        typedef cg::skeleton::igrid_type grid_type;
        typedef grid_type::bigcell_type      bigcell_type;
        typedef grid_type::smallcell_type    smallcell_type;
        
        skeleton_grid_drawer ( cg::point_2 const &basePoint, grid_type const &grid, dc_type dc,
                               cg::colorf const &smallCellColor = cg::colorf(.3f, .3f, .3f),
                               cg::colorf const &bigCellColor = cg::colorf(.4f, .7f, .7f))
            : basePoint_ (basePoint), grid_ (grid), dc_(dc)
            , bigCellColor_ (bigCellColor), smallCellColor_ (smallCellColor)
        {
        }
        
        struct SmallCellProcessor : GridDrawer
        {
            SmallCellProcessor(
                dc_type         dc, 
                // положение большой €чейки в экранных координатах
                grid_params const &gp,
                cg::colorf const &color, cg::point_2 const &basePoint )
                
                :   GridDrawer(dc, cg::coloraf(color, 0.5f), gp, 1.f, basePoint)
            {}
            
            // to be overridden by user
            template <class State, class cell_type>
                bool operator () (State const &, cell_type &)
            {
                return false;
            }
        };
        
        struct BigCellProcessor : GridDrawer
        {
            BigCellProcessor(
                dc_type         dc, 
                // положение  растра в экранных координатах
                grid_params const & gp,
                cg::colorf const &smallCellColor,
                cg::colorf const &bigCellColor, cg::point_2 const &basePoint) 
                
//                :   GridDrawer(dc, Color(1.0f,1.0f,.0f,1.f), gp)
                :   GridDrawer(dc, cg::coloraf(bigCellColor, 0.5f), gp, 3.f, basePoint)
                , basePoint_ (basePoint)
                , smallCellColor_ (smallCellColor)
            {}

//            typedef typename Derived::SmallCellProcessor  SmallCellProcessor;
            
            template <class State, class cell_type>
                SmallCellProcessor  processbigcell(State & state, cell_type &bcell)
            {
              if ( bcell )  
                return SmallCellProcessor(dc_, getSubGrid(gp_, state, bcell.extents()), smallCellColor_, basePoint_);
              else
                return SmallCellProcessor(dc_, grid_params(), smallCellColor_, basePoint_);                
            }

            template <class State, class cell_type>
                bool postprocessbigcell(State const &state, cell_type &bcell)
            {
                return false;
            }

        private:
           cg::colorf smallCellColor_;
           cg::point_2 basePoint_;
        };
        
        BigCellProcessor processgrid()
        {
           return BigCellProcessor(dc_,
              cg::grid_params(grid_.origin(), grid_.unit(), grid_.extents()),
              smallCellColor_, bigCellColor_, basePoint_);
        }
        
        private:
            cg::point_2 basePoint_;
            dc_type dc_;
            grid_type const &grid_;

            cg::colorf smallCellColor_;
            cg::colorf bigCellColor_;
    };

    //////////////////////////////////////////////////////////////////////////

    template < class grid_type, class Iterator >
      struct segments_filler
         : cg::grid2l_visitor_base< grid_type, segments_filler< grid_type, Iterator > >
    {
       segments_filler( Iterator it ) : it_( it ) {}

       template < class Smallcell, class State >
          bool operator ( ) ( State const & state, Smallcell const & smallcell )
       {
          std::copy( smallcell.segments().begin( ), smallcell.segments().end( ), it_ );
          return true;
       }

    private:
       Iterator it_;
    };

    template < class grid_type, class Iterator >
       segments_filler< grid_type, Iterator > segments_filler_create( Iterator it )
    {
       return segments_filler< grid_type, Iterator >( it );
    }

    //////////////////////////////////////////////////////////////////////////

    template< class grid_type >
      struct segments_average_calculator
         : cg::grid2l_visitor_base< grid_type, segments_average_calculator< grid_type > >
    {
       segments_average_calculator( double &result ) : result_ (result), n_ (0)
       {
          result_ = 0;
       }

       template < class Smallcell, class State >
          bool operator ( ) ( State const & state, Smallcell const & smallcell )
       {
          if (smallcell.segments().size() != 0)
          {
            result_ = (result_ * n_ + smallcell.segments().size()) / (n_ + 1);
            n_++;
          }
          return false;
       }

    private:
       double &result_;
       size_t n_;
    };

    template< class grid_type >
      segments_average_calculator< grid_type > segments_average_calculator_create( double &result )
    {
       return segments_average_calculator< grid_type > (result);
    }

    //////////////////////////////////////////////////////////////////////////
    template < class grid_type, class Iterator >
      struct contours_filler
         : cg::grid2l_visitor_base< grid_type, contours_filler< grid_type, Iterator > >
    {
       contours_filler( Iterator it ) : it_( it ) {}

       template < class Smallcell, class State >
          bool operator ( ) ( State const & state, Smallcell const & smallcell )
       {
          *it_++ = smallcell.centerBelongsTo();
          return true;
       }

    private:
       Iterator it_;
    };

    template < class grid_type, class Iterator >
       contours_filler< grid_type, Iterator > contours_filler_create( Iterator it )
    {
       return contours_filler< Iterator >( it );
    }

    //////////////////////////////////////////////////////////////////////////

    template< class grid_type >
    struct beams_average_calculator
       : cg::grid2l_visitor_base< grid_type, beams_average_calculator< grid_type > >
    {
       beams_average_calculator( double &result ) : result_ (result), n_ (0)
       {
          result_ = 0;
       }

       template < class Smallcell, class State >
          bool operator ( ) ( State const & state, Smallcell const & smallcell )
       {
          if (smallcell.edges.size() != 0)
          {
             result_ = (result_ * n_ + smallcell.edges.size()) / (n_ + 1);
             n_++;
          }
          return false;
       }

    private:
       double &result_;
       size_t n_;
    };

    template< class grid_type >
       beams_average_calculator< grid_type > beams_average_calculator_create( double &result )
    {
       return beams_average_calculator< grid_type > (result);
    }

    //////////////////////////////////////////////////////////////////////////

    template< class grid_type >
    struct beams_total_calculator
       : cg::grid2l_visitor_base< grid_type, beams_total_calculator< grid_type > >
    {
       beams_total_calculator ( size_t &eResult, size_t &vResult )
          : eResult_ (eResult)
          , vResult_ (vResult)
          , n_ (0)
       {
          eResult_ = vResult_ = 0;
       }

       template < class Smallcell, class State >
          bool operator ( ) ( State const & state, Smallcell const & smallcell )
       {
          eResult_ += smallcell.edges.size();
          vResult_ += smallcell.vertices.size();
          return false;
       }

    private:
       size_t &eResult_;
       size_t &vResult_;
       size_t n_;
    };

    template< class grid_type >
       beams_total_calculator< grid_type > beams_total_calculator_create( size_t &eResult, size_t &vResult )
    {
       return beams_total_calculator< grid_type > (eResult, vResult);
    }

    //////////////////////////////////////////////////////////////////////////

    template < class grid_type, class Iterator >
    struct edges_filler
       : cg::grid2l_visitor_base< grid_type, edges_filler< grid_type, Iterator > >
    {
       edges_filler( Iterator it ) : it_( it ) {}

       template < class Smallcell, class State >
          bool operator ( ) ( State const & state, Smallcell const & smallcell )
       {
          std::copy( smallcell.edges.begin( ), smallcell.edges.end( ), it_ );
          return true;
       }

    private:
       Iterator it_;
    };

    template < class grid_type, class Iterator >
       edges_filler< grid_type, Iterator > edges_filler_create( Iterator it )
    {
       return edges_filler< grid_type, Iterator >( it );
    }

    //////////////////////////////////////////////////////////////////////////

    template < class grid_type >
    struct range_filler
       : cg::grid2l_visitor_base< grid_type, range_filler< grid_type > >
    {
       range_filler( cg::range_2 &low, cg::range_2 &high )
          : low_ (low), high_ (high)
       {
       }

       template < class Smallcell, class State >
          bool operator ( ) ( State const & state, Smallcell const & smallcell )
       {
          low_ = smallcell.low;
          high_ = smallcell.high;
          return true;
       }

    private:
       cg::range_2 &low_;
       cg::range_2 &high_;
    };

    template < class grid_type >
       range_filler< grid_type > range_filler_create( cg::range_2 &low, cg::range_2 &high )
    {
       return range_filler< grid_type >(low, high);
    }

    //////////////////////////////////////////////////////////////////////////

    template < class grid_type, class Iterator >
    struct vertices_filler
       : cg::grid2l_visitor_base< grid_type, vertices_filler< grid_type, Iterator > >
    {
       vertices_filler( Iterator it ) : it_( it ) {}

       template < class Smallcell, class State >
          bool operator ( ) ( State const & state, Smallcell const & smallcell )
       {
          std::copy( smallcell.vertices.begin( ), smallcell.vertices.end( ), it_ );
          return true;
       }

    private:
       Iterator it_;
    };

    template < class grid_type, class Iterator >
       vertices_filler< grid_type, Iterator > vertices_filler_create( Iterator it )
    {
       return vertices_filler< grid_type, Iterator >( it );
    }


} // End of 'skeleton' namespace
} // End of 'cg' namespace
