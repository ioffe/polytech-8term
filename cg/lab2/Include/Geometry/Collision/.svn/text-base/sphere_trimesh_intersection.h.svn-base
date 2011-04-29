#pragma   once

#include "geometry\grid1L.h"
#include "geometry\sphere_triangle_intersection.h"

namespace cg
{
namespace cdt
{

template <class Grid, class Traits, class Derived>
   struct sphere_trimesh_intersection
{
   typedef typename Traits::face_id face_id;

   template< class S >
   bool sphereTrimeshIntersect ( cg::sphere_t< S, 3 > const &sph, SPHERE_INTERSECT_DETAIL *detail, DWORD flags ) const
   {
      typedef point_t< S, 2 > point_2_type;

      Processor< S > processor( self().traits(), sph, detail, flags );

      point_2_type    const r ( sph.radius, sph.radius );
      cg::rectangle_2 const rc( point_2_type( sph.center ) - r, point_2_type( sph.center ) + r );

      visit ( grid(), rc, processor );

      return processor.result();
   }

private:

   Derived const & self () const { return static_cast<Derived const &>(*this); }

   Grid & grid() const { return const_cast<Grid&>(self().grid()); }

   template< typename S >
   struct Processor : grid2l_visitor_base< Grid, Processor< S > >
   {
      typedef sphere_t< S, 3 > sphere_type;

      Processor( Traits const &traits, sphere_type const &sph, SPHERE_INTERSECT_DETAIL *detail, DWORD flags )
         : traits_(traits)
         , sph_   (sph)
         , detail_(detail)
         , zrange_(sph.center.z - sph.radius, sph.center.z + sph.radius)
         , result_(false)
         , flags_ (flags)
      {}

      template< class State > 
         bool operator() ( State const & state, typename Grid::smallcell_type const & column)
      {
         // если клетка не пустая
         if (!column)
            return false;

         if (!has_intersection(zrange_, column->zrange()))
            return false;

         for (size_t i = 0, size = column->faces().size(); i != size && !detail_->points.full(); ++i) 
         {
            size_t const idx = column->faces()[i];
            if ( !( flags_ & CF_INCLUDE_WATER ) && traits_.getOgTriangle( idx ).type == TRT_WATER )
               continue;

            typedef point_t< S, 3 > point_type;
            triangle_3_fast< point_type > const tri = traits_.getTriangleFast< point_type >( idx );

            if (!has_intersection(calc_zrange(tri), zrange_))
               continue;

            result_ |= cg::sphere_triangle_intersection(sph_, tri, detail_);
         }

         if (detail_->points.full())
            return true;

         return false;
      }

      bool result() const { return result_; }

   private:
      Traits const &traits_;

      sphere_type const &sph_;
      SPHERE_INTERSECT_DETAIL     *detail_;

      cg::range_2  zrange_;
      bool         result_;

      DWORD        flags_;
   };
};

} // end of namespace cdt
} // end of namespace cg
