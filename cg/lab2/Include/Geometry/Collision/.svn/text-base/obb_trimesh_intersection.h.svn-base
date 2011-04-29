#pragma   once

#include "geometry\grid1L.h"
#include "geometry\sat_intersection.h"

namespace cg
{
namespace cdt
{

template <class Grid, class Traits, class Derived>
   struct obb_trimesh_intersection
{
   typedef typename Traits::face_id face_id;

   template<class Point>
   bool obbTrimeshIntersect ( OBB_t<Point> const &obb, OBB_INTERSECT_DETAIL *detail, DWORD flags ) const
   {
      Processor<Point> processor( self().traits(), obb, detail, flags );
      visit ( grid(), OBB2AABB(obb), processor );

      return processor.result();
   }

private:

   Derived const & self () const { return static_cast<Derived const &>(*this); }

   Grid & grid() const { return const_cast<Grid&>(self().grid()); }

   template< typename vertex_type >
      struct Processor : grid2l_visitor_base<Grid, Processor< vertex_type > >
   {
      Processor( Traits const &traits, OBB_t<vertex_type> const &obb, OBB_INTERSECT_DETAIL *detail, DWORD flags )
         : traits_(traits)
         , obb_   (obb)
         , detail_(detail)
         , zrange_(cg::OBB2AABB(obb).z)
         , result_(false)
         , flags_ (flags)
      {}

      template< class State > 
         bool operator() ( State const & state, typename Grid::smallcell_type const & column)
      {
         // если клетка не пустая
         if (!column)
            return false;

         if ( !has_intersection( zrange_, column->zrange() ) )
            return false;

         for (size_t i = 0, size = column->faces().size(); i != size; ++i) 
         {
            size_t const idx = column->faces()[i];
            if ( !( flags_ & CF_INCLUDE_WATER ) && traits_.getOgTriangle( idx ).type == TRT_WATER )
               continue;

            triangle_3_fast<vertex_type> const tri = traits_.getTriangleFast<vertex_type>( idx );

            if( !has_intersection( zrange_, calc_zrange(tri) ))
               continue;

            result_ = detect_intersection(obb_, tri, detail_) || result_;
         }

         return false;
      }

      bool result() const { return result_; }

   private:
      Traits const &traits_;

      OBB_t<vertex_type> const &obb_;
      OBB_INTERSECT_DETAIL     *detail_;

      cg::range_2  zrange_;
      bool         result_;

      DWORD       flags_;
   };
};

} // end of namespace cdt
} // end of namespace cg
