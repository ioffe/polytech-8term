#pragma once

#include "geometry\Grid1L.h"
#include "common\m_ptr.h"
#include "Index2L.h"
#include "grid2l_bigcell.h"
#include "Geometry\Empty.h"
#include "Grid2L_Mapped.h"

#pragma warning ( disable : 4348 ) // redefinition of default parameter

namespace streams { template<class T,class D = T> struct structured_ostream;}

namespace cg {

    template <class G>  // типа Grid2L
        struct grid_subdivider
    {
        typedef G   grid_type;

        __forceinline point_2i operator () (point_2i const & idx) const 
        {
            return source_.at(idx).extents();
        }

        __forceinline grid_subdivider(grid_type const & source) : source_(source) {}

    private:
        grid_type const & source_;
    };

    
    // тег, указывающий, что из аргумента конструктора можно взять
    // информацию о том, насколько стоит подразбить каждую ячейку
    struct subdividing {};
    
    // T - тип, который по значению хранится в smallcell
    // если мы храним указатели, то std::vector<T> при T = auto_ptr<U> исключается
    //
    // добавить процессор по всем ячейкам
    template <class T, class BigCell = grid2l_bigcell<T>, class GridHeader = Empty >
        struct Grid2L : GridHeader, Grid1L<BigCell>
    {
        typedef BigCell             bigcell_type;
        typedef T                   smallcell_type;
        typedef Index2L             index_type;
        typedef point_2i            bigidx_type;
        typedef point_2i            smallidx_type;

        typedef Grid1L<bigcell_type>    Base;

        Grid2L (cg::point_2 const & org, cg::point_2 const & unit, cg::point_2i const & ext)
            :   Base(cg::aa_transform(org, unit), ext)
        {}

        template <class U> Grid2L (Grid1L<U> const &other)
            :   Base(other)
        {}

        // subdivider знает, насколько можно подразбить каждую ячейку
        template <class S> Grid2L(S const &subdivider, subdividing)
        {

        }

        template < class Grid1LSubdividings >
            explicit Grid2L ( Grid1LSubdividings const & gp )
            :   Base (gp)
        {}

        template <class OtherT, class OtherBigCell> 
            Grid2L(Grid2L<OtherT, OtherBigCell> const & other)
            :   Base(other.grid_params())
        {
            MakeSubdivision(grid_subdivider<Grid2L<OtherT, OtherBigCell> >(other));
        }

        template <class OtherT, class OtherBigCell> 
            Grid2L(MappedGrid2L<OtherT, OtherBigCell> const & other)
            :   Base(other.grid_params())
        {
            MakeSubdivision(grid_subdivider<MappedGrid2L<OtherT, OtherBigCell> >(other));
         }

        Grid2L(Grid2L const & other)
            :   Base(other.grid_params())
        {
            MakeSubdivision(grid_subdivider<Grid2L>(other));
        }

        __forceinline GridHeader       & header()       { return *this; }
        __forceinline GridHeader const & header() const { return *this; }

        __forceinline Base             & grid  ()       { return *this; }
        __forceinline Base const       & grid  () const { return *this; }

        template <class OtherT>
            Grid2L<OtherT> * clone( OtherT * = 0) const
        {
            Grid2L<OtherT> * grid_copy = new Grid2L<OtherT>(*this);

            return grid_copy;
        }

        Grid2L() {}

        Grid2L(aa_transform const &tform, point_2i const &ext)
            :   Base(tform, ext)
        {}

        // Subdivider - это класс, у объекта которого можно спросить 
        // на сколько клеток разбить big cell.
        // для этого он должен иметь point_2i operator () (point_2i) const.
        // если возвращенный point_2i не имеет положительные координаты
        // то подразбиение не происходит
        template <class Subdivider>
            void MakeSubdivision (Subdivider subdivider)
        {
            Assert( extents( ).x != 0 && extents( ).y != 0 );
            for (rectangle_2i::iterator cell(rectangle_by_extents(extents())); cell; ++cell)
            {
                point_2i subdivision = subdivider(*cell);

                at(*cell).subdivide(subdivision);
            }
        }

        void Subdivide(point_2i const & idx_big, point_2i const &subdivision)
        {
            if (0 == at(idx_big))
                at(idx_big).subdivide(subdivision);
        }

        // переименовать в contains
        __forceinline bool is_valid (point_2i const &idx_big) const 
        {
            return Base::contains(idx_big);
        }

        __forceinline raster_2   bigcellraster(point_2i const &bigcell) const
        {
            bigcell_type const & bcell = at(bigcell);

            point_2     origin = aa_transform::local2world(bigcell);
            point_2i    ext    = bcell.extents();
            point_2     unit   = ( ext.x && ext.y ? aa_transform::unit() / ext : aa_transform::unit() ) ;

            return raster_2(origin, unit, ext);
        }

        // Эти операторы живут в предположении, что подразбиение уже произошло
        __forceinline T & operator [] (index_type const &idx)
        {
            Assert(at(idx.big));
            return at(idx.big).at(idx.small); 
        }

        __forceinline T const & operator [] (index_type const &idx) const
        {
            Assert(at(idx.big));
            return at(idx.big).at(idx.small); 
        }

        using Base::at;

        __forceinline T & at (index_type const & idx)
        {
           Assert(at(idx.big));
           return at(idx.big).at(idx.small);
        }

        __forceinline T const & at (index_type const & idx) const
        {
           Assert(at(idx.big));
           return at(idx.big).at(idx.small);
        }

        __forceinline bigcell_type& operator [] (point_2i const &idx_big)
        {
            return at(idx_big);
        }
        
        __forceinline bigcell_type const & operator [] (point_2i const &idx_big) const
        {
            return at(idx_big);
        }

        __forceinline aa_transform const & tform () const { return *this; }

        template <class Derived>
            struct visitor_base
        {
            typedef          Grid2L                    grid_type;
            typedef typename Grid2L<T>::smallcell_type smallcell_type;
            typedef typename Grid2L<T>::bigcell_type   bigcell_type;
            typedef typename Grid2L<T>::index_type     index_type;
            typedef typename Grid2L<T>::bigidx_type    bigidx_type;
            typedef typename Grid2L<T>::smallidx_type  smallidx_type;

            typedef Derived     SmallCellProcessor;

            template <class State>
                __forceinline bool operator () (State & state, smallcell_type &scell)
            {
                return false;
            }

            template <class State>
                __forceinline SmallCellProcessor & processbigcell(State & state, bigcell_type &bcell)
            {
                return static_cast<SmallCellProcessor&>(*this);
            }
        };
    };

    // Считается, что ячейка idx существует
    template <class Grid>
        __forceinline point_2 smallcell_center (Grid const & g, Index2L const & idx)
    {
        return 
            g.local2world(idx.big + (idx.small + point_2(.5,.5)) / g.at(idx.big).extents());
    }

    template <class Grid>
        __forceinline  rectangle_2 smallcell_bound(Grid const & g, Index2L const & idx)
    {
        point_2i ext = g.at(idx.big).extents();
        point_2 lt = g.local2world(idx.big + point_2( idx.small ) / ext);
        point_2  unit = g.unit() / ext;
        return rectangle_2(lt, lt + unit);
    }

    template <class T>
    Index2L index_2l( Grid2L< T > const & grid, point_2 pt )
    {
       Assert( grid.contains( pt ) );
       point_2 local_pt = grid.world2local( pt );
       point_2i big_idx = floor( local_pt );
       point_2i small_idx = floor( ( local_pt - point_2( big_idx ) ) & grid.at(big_idx).extents() );
       return Index2L(big_idx, small_idx);
    }

    template <class T> 
        struct remove_const
    {
        typedef T   result;
    };

    template <class T>
        struct remove_const < T const >
    {
        typedef T   result;
    };

    template <class G, class Derived>
        struct grid2l_visitor_base
    {
        typedef G                            grid_type;
        typedef typename G::smallcell_type   smallcell_type;
        typedef typename G::bigcell_type     bigcell_type;
        typedef typename G::index_type       index_type;
        typedef typename G::bigidx_type      bigidx_type;
        typedef typename G::smallidx_type    smallidx_type;

        typedef Derived     SmallCellProcessor;
        typedef Derived     BigCellProcessor;
        typedef Derived     SideProcessor;

        template <class State>
            __forceinline bool operator () (State &, smallcell_type &)
        {
            return false;
        }

        template <class State>
            __forceinline SmallCellProcessor & processbigcell(State const &, typename remove_const<bigcell_type>::result &)
        {
            return static_cast<SmallCellProcessor&>(*this);
        }

        template <class State>
            __forceinline SmallCellProcessor & processbigcell(State const &, typename remove_const<bigcell_type>::result const &)
        {
            return static_cast<SmallCellProcessor&>(*this);
        }

        template <class State>
            __forceinline const SmallCellProcessor & processbigcell(State const &, bigcell_type &) const
        {
            return static_cast<SmallCellProcessor const &>(*this);
        }

        __forceinline BigCellProcessor & processgrid()
        {
            return static_cast<BigCellProcessor&>(*this);
        }

        template <class State>
            __forceinline bool postprocessbigcell(State const &, bigcell_type &)
        {
            return false ;
        }

        template <class State>
            __forceinline bool postprocessbigcell(State const & state, bigcell_type const & bcell) const
        {
            return false;
        }

        template <class SegmentContructionCellProcessor, class Sides>
        __forceinline void process_triangle_sides(grid_type & grid, Sides & sides, triangle_2 const & t)
        {
            rectangle_2 bb = bounding(grid);
            bb.inflate( - epsilon<double>( ) );

            std::vector<point_2>   clipped;
            
            cull(t, bb, clipped);

            for (size_t i = 0; i != clipped.size(); ++i)
            {
                size_t n = cg::next((int)i, (int)clipped.size());

                SegmentContructionCellProcessor proc (sides, (static_cast<Derived*>(this))->side_processor((int)i,(int)n));

                visit(grid, cg::segment_2(clipped[i], clipped[n]), proc);
            }
        }
    };

   template <class Stream, class T, class B, class H>
      void write(Stream & stream, Grid2L<T,B,H> const & grid)
   {
      write(stream, grid.header());
      write(stream, grid.grid());
   }

   template <class Stream, class T, class B, class H>
      void read(Stream & stream, Grid2L<T,B,H> & grid)
   {
      read(stream, grid.header());
      read(stream, grid.grid());
   }
}
