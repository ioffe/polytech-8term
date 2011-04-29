#pragma once   

#include <boost\noncopyable.hpp>

#include "triangle_mesh.h"
#include "rectangle_3_intersection.h"
#include "sphere_triangle_intersection.h"
#include "sat_intersection.h"
#include "grid1L.h"
#include "grid2L.h"
#include "Grid2L/subdiv.h"

namespace cg 
{
namespace details
{

// Меш растеризованный по сетке
template< typename vertex_type, typename index_type >
   struct mesh_rasterization_grid 
      : private boost::noncopyable
{
   // Тип ячейки в сетке
   struct cell_type
   {
      cell_type() {}

      typedef  std::vector< index_type > indices_type;
      
      indices_type  indices;
      range_2       zrange;
   };

   // Тип сетки
   typedef Grid1L< cell_type > grid_type;

private:
   // Процессор для растеризации треугольников
   struct rasterization_processor_type
   {
      rasterization_processor_type ( index_type idx, range_2 const & zrange )  
         : idx_( idx )
         , zrange_( zrange )
      {}
         
      template< class State >
            bool operator() ( State const & state, cell_type & cell )
      {
         cell.indices.push_back ( idx_ );
         cell.zrange |= zrange_ ;

         return false;
      }

   private:
      index_type  idx_;
      range_2     zrange_;
   };

   typedef triangle_mesh< vertex_type, index_type > mesh_type;

public:
   mesh_rasterization_grid( mesh_type const & mesh, rectangle_2 bb, point_2i const & ext )
   {
      bb.inflate(cg::epsilon<float>());
      point_2 const unit = bb.size() / ext;

      grid_ = new grid_type( grid_params ( bb.xy(), unit, ext ) );

      for ( size_t i = 0, size = mesh.triangles_count(); i != size; ++i )
      {
         triangle_3 const & tr3 = mesh.triangle< triangle_3 >( i );
         triangle_2 const   tr2( tr3[ 0 ], tr3[ 1 ], tr3[ 2 ] );

         visit ( *grid_, tr2, rasterization_processor_type( i, range_2(tr3[0].z, tr3[1].z) | tr3[2].z ) );
      }

      //pack();
   }

public:
   grid_type const & grid() const { return *grid_; }

private:
   // Сжимает векторы
   //void pack()
   //{
   //   for( grid_type::iterator it = grid_->begin(), end = grid_->end(); it != end; ++it )
   //      cell_type::indices_type( it->indices ).swap( it->indices );
   //}

private:
   m_ptr< grid_type > grid_;
};

} // end of namespace details

// Параметры для растеризации меша
struct rasterized_triangle_mesh_params
{
   rasterized_triangle_mesh_params( int min_triangles, point_2i const & ext )
      : min_triangles_( min_triangles )
      , ext_( ext )
   {}

   int               min_triangles() const { return min_triangles_; } 
   point_2i const &  extents()       const { return ext_; }

private:
   int      min_triangles_;
   point_2i ext_;            
};

// Хранит меш, растеризованный по сетке
template< typename vertex_type, typename index_type >
   struct rasterized_triangle_mesh
{
private:
   typedef typename details::mesh_rasterization_grid< vertex_type, index_type >  raster_type;

public:
   typedef typename triangle_mesh< vertex_type, index_type >            mesh_type;
   typedef typename raster_type :: grid_type                            grid_type;
   typedef std::vector< char >                                          cache_type;

public:
   rasterized_triangle_mesh( const vertex_type * vertices , index_type v_count,
                              const point_3i * indices,      index_type t_count,
                              rasterized_triangle_mesh_params const & params, rectangle_3 const * bb = NULL )
      : mesh_( vertices, v_count, indices, t_count )
   { 
      bb_ = bounding(mesh_);

      if ( bb )
         bb_ &= *bb ;

      if ( !bb_.empty() && mesh_.triangles_count() >= ( index_type ) params.min_triangles() )
      {
         raster_ = new raster_type( mesh_, bb_, params.extents() );
         cache_.resize( mesh_.triangles_count(), 0 );
      }
   }

   void clean_cache() const
   {
      if (!cache_.empty())
         memset(&cache_[0], 0, cache_.size());
   }

   cache_type           &  cache() const { return cache_; }
   mesh_type      const &  mesh()  const { return mesh_; }
   grid_type      const *  grid()  const { return raster_ ? &raster_->grid() : 0; } 
   rectangle_3    const &  bb()    const { return bb_; }

private: 
   mesh_type                mesh_;
   m_ptr< raster_type >     raster_;
   rectangle_3              bb_;                                      
   mutable cache_type       cache_;
};

namespace details
{

enum PROC_CONTINUE_TYPE
{
   PROC_BREAK = 0,
   PROC_CONTINUE,
   PROC_CONTINUE_ONLY_THIS_CELL
};

// Всевозможные процессоры для collision

// OBB vs triangle intersect processor
template< typename vertex_type, typename index_type >
struct obb_intersection_processor
{
   typedef typename rasterized_triangle_mesh< vertex_type, index_type > raster_mesh_type;

   obb_intersection_processor ( OBB_t< vertex_type > const &obb, OBB_INTERSECT_DETAIL *detail, raster_mesh_type const &r_mesh )
      : obb_(obb)
      , detail_(detail)
      , r_mesh_(r_mesh)
      , zrange_(-obb.extents(0).z, obb.extents(0).z)
      , result_(false)
   {
      zrange_.unite(  obb.extents(1).z );
      zrange_.unite( -obb.extents(1).z );
      zrange_.unite(  obb.extents(2).z );
      zrange_.unite( -obb.extents(2).z );

      zrange_.offset( obb.origin.z );

      r_mesh.clean_cache();
   }

private:
   typedef typename raster_mesh_type::grid_type::cell_type cell_type;
   typedef typename raster_mesh_type::cache_type           cache_type;
   typedef typename raster_mesh_type::mesh_type            mesh_type;

public:
   template< class State > 
   bool operator() ( State const & state, cell_type const & cell )
   {
      typedef typename cell_type::indices_type::const_iterator CII;

      if ( !has_intersection( zrange_, cell.zrange ) )
         return false;

      cache_type      &cache = r_mesh_.cache();
      mesh_type const &mesh  = r_mesh_.mesh();

      for ( CII p = cell.indices.begin(), q = cell.indices.end(); p != q; ++p )
      {
         if ( cache[*p] )
            continue;

         cache[*p] = true;
         detect( obb_, mesh.triangle< triangle_3_fast <vertex_type> >( *p ), detail_ );
      }

      return false;
   }

   void detect(OBB_t< vertex_type > const &obb, triangle_3_fast< vertex_type > const &tri, OBB_INTERSECT_DETAIL *detail)
   {
      if( !has_intersection( zrange_, calc_zrange(tri) ))
         return;

      result_ = detect_intersection(obb, tri, detail) || result_;
   }

   bool result() const { return result_; }

private:
   OBB_t<vertex_type> const &obb_;
   OBB_INTERSECT_DETAIL     *detail_;
   raster_mesh_type const   &r_mesh_;
   cg::range_2               zrange_;

   bool result_;
};

// sphere vs triangle intersect processor
template< typename scalar_type, typename index_type >
struct sphere_intersection_processor
{
   typedef point_t< scalar_type, 3 >                                   point_type;
   typedef typename rasterized_triangle_mesh< point_type, index_type > raster_mesh_type;
   typedef sphere_t< scalar_type, 3 >                                  sphere_type;

   sphere_intersection_processor ( sphere_type const &sph, SPHERE_INTERSECT_DETAIL *detail, raster_mesh_type const &r_mesh )
      : sph_   (sph)
      , detail_(detail)
      , r_mesh_(r_mesh)
      , zrange_(sph.center.z - sph.radius, sph.center.z + sph.radius)
      , result_(false)
   {
      r_mesh.clean_cache();
   }

private:
   typedef typename raster_mesh_type::grid_type::cell_type cell_type;
   typedef typename raster_mesh_type::cache_type           cache_type;
   typedef typename raster_mesh_type::mesh_type            mesh_type;

public:
   template< class State > 
   bool operator() ( State const & state, cell_type const & cell )
   {
      typedef typename cell_type::indices_type::const_iterator CII;

      if ( !has_intersection( zrange_, cell.zrange ) )
         return false;

      cache_type      &cache = r_mesh_.cache();
      mesh_type const &mesh  = r_mesh_.mesh();

      for ( CII p = cell.indices.begin(), q = cell.indices.end(); p != q; ++p )
      {
         if ( cache[*p] )
            continue;

         cache[*p] = 1;
         detect( sph_, mesh.triangle< triangle_3_fast < point_type > >( *p ), detail_ );
      }

      return false;
   }

   void detect( sphere_type const &sph, triangle_3_fast< point_type > const &tri, SPHERE_INTERSECT_DETAIL *detail )
   {
      if( !has_intersection( zrange_, calc_zrange(tri) ))
         return;

      result_ |= sphere_triangle_intersection(sph, tri, detail);
   }

   bool result() const { return result_; }

private:
   sphere_type const           &sph_;
   SPHERE_INTERSECT_DETAIL     *detail_;
   raster_mesh_type const      &r_mesh_;
   cg::range_2                 zrange_;

   bool result_;
};


// Универсальный процессор
template< typename vertex_type, typename index_type, typename Proc >
struct mesh_intersection_processor
{
   typedef typename rasterized_triangle_mesh< vertex_type, index_type >   raster_mesh_type;

   // Вектор checked должен быть заполнен false и иметь размер не меньше количества треугольников
   mesh_intersection_processor ( raster_mesh_type const & r_mesh, segment_3 const & seg, Proc & proc, bool nobackface )  
      : r_mesh_      ( r_mesh )
      , ray_         ( seg.P0(), seg.P1() - seg.P0() )
      , ray_z_       ( seg.P0().z, seg.P1().z )
      , proc_        ( proc )
      , nobackface_  ( nobackface )
      , result_      ( false )
   {
      r_mesh.clean_cache();
   }

private:
   typedef typename raster_mesh_type::grid_type::cell_type     cell_type;
   typedef typename raster_mesh_type::cache_type               cache_type;
   typedef typename raster_mesh_type::mesh_type                mesh_type;

public:
   template< class State > 
         bool operator() ( State const & state, cell_type const & cell )
   {
      static const double eps = epsilon< float >();

      range_2 const zrange( blend( ray_z_.second, ray_z_.first, state.in_ratio )  + eps,
                            blend( ray_z_.second, ray_z_.first, state.out_ratio ) + eps );

      if ( !has_intersection( zrange, cell.zrange ) )
            return false;

      typedef  typename cell_type::indices_type::const_iterator    CII;

      bool res = false;
   
      cache_type      & cache = r_mesh_.cache();
      mesh_type const & mesh  = r_mesh_.mesh();

      for ( CII p = cell.indices.begin(), q = cell.indices.end(); p != q; ++p )
      {
         if ( cache[ *p ] )
            continue;

         triangle_3_rti const tr( mesh.triangle< triangle_3_rti >( *p ) );

         if( !has_intersection( zrange, calc_zrange( tr ) ) )
            continue;

         cache[ *p ] = 1;

         double ratio = 1e3;
         if( ray_triangle3_intersection< cartesian<> >( ray_, tr, ratio, nobackface_ ) )
         {
            Assert( ratio + eps > state.in_ratio );
            if( ratio - eps > state.out_ratio )
            {
               cache[ *p ] = 0;
               continue;
            }

            result_ = true; 
            switch( proc_( ratio, *p ) )
            {
            case PROC_BREAK: 
               return true;
            case PROC_CONTINUE:
               break;
            case PROC_CONTINUE_ONLY_THIS_CELL:
               res = true; break;
            default:
               Assert( false );
            }
         }
      }
      return res;
   }

   bool             result() const { return result_; }

private:
   raster_mesh_type           const & r_mesh_; 
   segment_3_ref              const   ray_;
   std::pair< double, double >        ray_z_;
   Proc                             & proc_;
   bool                               result_;
   bool                       const   nobackface_;
};

//////////////////////////////////////////////////////////////////////
template< typename index_type >
struct mesh_ray_intersection_processor
{
   mesh_ray_intersection_processor() 
      : ratio_( 1e3 )
      , idx_  ( -1 ) 
   {}

   PROC_CONTINUE_TYPE operator() ( double ratio, index_type idx )
   {
      make_min( ratio_, ratio, idx_, idx );
      return PROC_CONTINUE_ONLY_THIS_CELL;
   }

   double        ratio() const { return ratio_; }
   index_type    index() const { return idx_;   }

private:
   double      ratio_;
   index_type  idx_;
};

//////////////////////////////////////////////////////////////////////
template< typename index_type >
struct mesh_point_inside_processor               
{
   mesh_point_inside_processor () 
      : count_( 0 ) 
   {}

   PROC_CONTINUE_TYPE operator()( double, index_type )
   {
      ++count_;
      return PROC_CONTINUE;
   }

   size_t count() const { return count_; }

private:
   size_t count_;
};

//////////////////////////////////////////////////////////////////////
template< typename index_type/*, typename C*/ >
struct mesh_point_inside_ext_processor
{
   mesh_point_inside_ext_processor( segment_3 const & s/*, C * c*/ ) 
      : count_( 0 )
      , idx_  ( 0 )
      , ratio_( 1e3 )
      , s_    ( s )
      //, c_    ( c )
   {}

   PROC_CONTINUE_TYPE operator()( double ratio, index_type idx )
   {
      make_min( ratio_, ratio, idx_, idx );
      ++count_;

      //c_->push_back( s_(ratio) );

      return PROC_CONTINUE;
   }

   size_t      count() const { return count_; }
   index_type  index() const { return idx_;   }
   double      ratio() const { return ratio_; }

private:
   size_t      count_;
   index_type  idx_;
   double      ratio_;
   segment_3 const & s_;
   //C * c_;
};

} // end of namespace details

// ===================================================== RAY INTERSECT ==================================================

// Пересечение меша с лучом
template< typename vertex_type, typename index_type >
bool ray_intersection( point_3 const & p1, point_3 const & p2, rasterized_triangle_mesh< vertex_type, index_type > const & r_mesh, 
                        double * p_ratio, point_3 * normal, bool nobackface, bool aabb_test)
{
   //Profiler::CProfiler prof( "ray_intersection" );

   if( aabb_test && !ray_intersection( p1, p2, r_mesh.bb(), 0, 0 ) )
      return false;

   boost::optional< index_type > idx;
   double ratio = 1e3;

   segment_3 const seg ( p1, p2 );
   if ( r_mesh.grid() )
   {
      typedef  details::mesh_ray_intersection_processor< index_type >                        triangle_proc_type;
      typedef  details::mesh_intersection_processor< vertex_type, index_type, triangle_proc_type >   intersect_proc_type;

      triangle_proc_type  proc;
      intersect_proc_type i_proc( r_mesh, seg, proc, nobackface );

      if ( visit( *r_mesh.grid(), segment_2(seg), i_proc ) )
      {
         idx = proc.index();
         ratio = proc.ratio();
      }
   }
   else
   {
      range_2 ray_zrange( seg.P0().z, seg.P1().z );
      segment_3_ref const ray( seg.P0(), seg.P1() - seg.P0() );

      typedef  rasterized_triangle_mesh< vertex_type, index_type >::mesh_type::triangle_iterator_type< triangle_3 > TI;

      // Перебираем все треугольники
      for ( TI it = r_mesh.mesh().t_begin< triangle_3 >(), q = r_mesh.mesh().t_end< triangle_3 >(); it != q; ++it )
      {
         triangle_3 const & t = *it;

         if( !has_intersection( ray_zrange, range_2(t[0].z, t[1].z) | t[2].z ) )
            continue;

         triangle_3_rti const tr( t[ 0 ], t[ 2 ], t[ 1 ], cg::normal( t ) );

         double r = 1e3;
         if ( ray_triangle3_intersection< cartesian<> >( ray, tr, r, nobackface ) && r < ratio )
         {
            idx = it.index();
            ratio = r;
            ray_zrange = range_2( seg.P0().z, blend( seg.P1().z, seg.P0().z, ratio ) );
         }
      }
   }

   if ( idx )
   {
      if ( p_ratio  ) 
         *p_ratio = ratio;

      if ( normal )
      {
         *normal = point_3( r_mesh.mesh().normal( *idx ) );
         
         if ( normal->z < 0. ) 
            *normal = -*normal;
      }
      return true;
   }

   return false;
}

// ================================================= END RAY INTERSECT ==================================================

// ===================================================== POINT INSIDE ===================================================
template< typename vertex_type, typename index_type >
bool is_point_inside( point_3 const & pt, rasterized_triangle_mesh< vertex_type, index_type > const & r_mesh,
                      point_3 * nearest = NULL, double * depth = NULL )
{
   int isect_count = 0;
   
   point_3 const pbegin = eq_zero(pt) 
                              ? r_mesh.bb().xyz() 
                              : normalized( pt ) * norm( r_mesh.bb().XYZ() - r_mesh.bb().xyz() ) ;

   Assert( !r_mesh.bb().contains( pbegin ) );

   segment_3 const seg( pbegin, pt );

   if ( r_mesh.grid() )
   {
      typedef  details::mesh_point_inside_processor< index_type >  triangle_proc_type;
      typedef  details::mesh_intersection_processor< vertex_type, index_type, triangle_proc_type > intersect_proc_type;

      triangle_proc_type  proc;
      intersect_proc_type i_proc( r_mesh, seg, proc, false );
      
      visit( *r_mesh.grid(), segment_2(seg), i_proc ) ;

      isect_count = proc.count();
   }
   else
   {
      range_2 const ray_zrange( seg.P0().z, seg.P1().z );
      segment_3_ref const ray( seg.P0(), seg.P1() - seg.P0() );

      typedef  rasterized_triangle_mesh< vertex_type, index_type >::mesh_type::triangle_iterator_type< triangle_3 > TI;

      // Перебираем все треугольники
      for ( TI it = r_mesh.mesh().t_begin< triangle_3 >(), q = r_mesh.mesh().t_end< triangle_3 >(); it != q; ++it )
      {
         triangle_3 const & t = *it;

         if( !has_intersection( ray_zrange, range_2(t[0].z, t[1].z) | t[2].z ) )
            continue;

         // todo :: избавиться от этого бреда
         //const triangle_3_rti tr( t[ 0 ], t[ 1 ], t[ 2 ], point_3f( -CalcNormal( t ) ) );
         const triangle_3_rti tr( t[ 0 ], t[ 2 ], t[ 1 ], normal( t ) );

         double r;
         if ( ray_triangle3_intersection< cartesian<> >( ray, tr, r, false ) )
            ++isect_count;
      }
   }

   return ( isect_count % 2 == 0 ) ;
}
// ================================================ END POINT INSIDE ===================================================

// ===================================================== POINT INSIDE EXT ==============================================
// dir - direction to center of pt owner
template< typename vertex_type, typename index_type/*, typename C*/ >
bool is_point_inside( point_3 const & pt, point_3 const & dir, rasterized_triangle_mesh< vertex_type, index_type > const & r_mesh,
                      point_3 * pnormal = NULL, double * pdepth = NULL/*, C * c = NULL*/ )
{
   if ( !r_mesh.bb().contains( pt ) )
      return false;

   size_t      isect_count = 0;
   index_type  first_isect = 0;
   double      first_ratio = 1e6;

   point_3 const pend = pt + dir * norm( r_mesh.bb().XYZ() - r_mesh.bb().xyz() ) ;
   Assert( !r_mesh.bb().contains( pend ) );

   segment_3 const seg( pt, pend );

   if ( r_mesh.grid() )
   {
      typedef  details::mesh_point_inside_ext_processor< index_type/*, C*/ >  triangle_proc_type;
      typedef  details::mesh_intersection_processor< vertex_type, index_type, triangle_proc_type > intersect_proc_type;

      triangle_proc_type  proc(seg/*, c*/);
      intersect_proc_type i_proc( r_mesh, seg, proc, false );
      
      visit( *r_mesh.grid(), seg, i_proc ) ;

      isect_count = proc.count();
      first_isect = proc.index();
      first_ratio = proc.ratio();
   }
   else
   {
      range_2 const ray_zrange( seg.P0().z, seg.P1().z );
      segment_3_ref const ray( seg.P0(), seg.P1() - seg.P0() );

      typedef  rasterized_triangle_mesh< vertex_type, index_type >::mesh_type::triangle_iterator_type< triangle_3 > TI;

      // Перебираем все треугольники
      for ( TI it = r_mesh.mesh().t_begin< triangle_3 >(), q = r_mesh.mesh().t_end< triangle_3 >(); it != q; ++it )
      {
         triangle_3 const & t = *it;

         if( !has_intersection( ray_zrange, calc_zrange( t ) ) )
            continue;

         // todo :: избавиться от этого бреда
         //const triangle_3_rti tr( t[ 0 ], t[ 1 ], t[ 2 ], point_3f( -CalcNormal( t ) ) );
         const triangle_3_rti tr( t[ 0 ], t[ 2 ], t[ 1 ], normal( t ) );

         double r;
         if ( ray_triangle3_intersection< cartesian<> >( ray, tr, r, false ) )
         {
            make_min(first_ratio, r, first_isect, it.index());
            ++isect_count;
         }
      }
   }

   if ( isect_count % 2 )
   {
      if ( pnormal ) 
         *pnormal = r_mesh.mesh().normal( first_isect );
      
      if ( pdepth )  
         *pdepth = norm( pt - seg(first_ratio) );

      return true;
   }

   return false;
}
// ================================================ END POINT INSIDE EXT ===============================================

// ================================================ OBB vs triangle mesh intersection ==================================
template< typename vertex_type, typename index_type >
bool obb_trimesh_intersect(OBB_t< vertex_type > const &obb, OBB_INTERSECT_DETAIL *detail, rasterized_triangle_mesh< vertex_type, index_type > const &r_mesh)
{
   if((NULL != detail) && detail->points.full())
      return false;

   typedef details::obb_intersection_processor< vertex_type, index_type > intersect_proc_type;
   intersect_proc_type i_proc( obb, detail, r_mesh );

   if ( r_mesh.grid() )
   {
      visit( *r_mesh.grid(), OBB2AABB(obb), i_proc );
   }
   else
   {
      typedef rasterized_triangle_mesh< vertex_type, index_type >::mesh_type::triangle_iterator_type< triangle_3_fast < vertex_type > > TI;

      // Process all triangles
      for ( TI it = r_mesh.mesh().t_begin< triangle_3_fast < vertex_type > >(), q = r_mesh.mesh().t_end< triangle_3_fast < vertex_type > >(); it != q; ++it )
      {
         i_proc.detect(obb, *it, detail);  
      }
   }

   return i_proc.result();
}
// ================================================ End OBB vs triangle mesh intersection ==============================

// ================================================ sphere vs triangle mesh intersection ==================================
template< typename scalar_type, typename index_type >
   bool sphere_trimesh_intersect( sphere_t< scalar_type, 3 > const &sph, 
      SPHERE_INTERSECT_DETAIL *detail, rasterized_triangle_mesh< point_t< scalar_type, 3 >, index_type > const &r_mesh )
{
   if((NULL != detail) && detail->points.full())
      return false;

   typedef point_t< scalar_type, 3 >         point_type;
   typedef cg::rectangle_t< scalar_type, 3 > rectangle_type;
   typedef details::sphere_intersection_processor< scalar_type, index_type > intersect_proc_type;

   bool noDetail = (NULL == detail);
   SPHERE_INTERSECT_DETAIL tmpDetail;
   SPHERE_INTERSECT_DETAIL *tmpDetailPtr = noDetail ? NULL : &tmpDetail;

   intersect_proc_type i_proc( sph, tmpDetailPtr, r_mesh );

   if ( r_mesh.grid() )
   {
      point_type     r  ( sph.radius, sph.radius, sph.radius ) ;
      rectangle_type rc ( sph.center - r, sph.center + r ) ;

      visit( *r_mesh.grid(), rc, i_proc );
   }
   else
   {
      typedef triangle_3_fast < point_type > triangle_3_fast_type;
      typedef rasterized_triangle_mesh< point_type, index_type >::mesh_type::triangle_iterator_type< triangle_3_fast_type > TI;

      // Process all triangles
      for ( TI it = r_mesh.mesh().t_begin< triangle_3_fast_type >(), q = r_mesh.mesh().t_end< triangle_3_fast_type >(); it != q; ++it )
      {
         i_proc.detect(sph, *it, tmpDetailPtr);  
      }
   }

   if(!noDetail)
      cg::filter_dup_collision_points(tmpDetail.points, detail->points);

   return i_proc.result();
}
// ================================================ End sphere vs triangle mesh intersection ==============================

// ================================================= DISTANCE ==========================================================

template< typename vertex_type, typename index_type >
   double scaled_distance( point_3 const & pt, rasterized_triangle_mesh< vertex_type, index_type > const & r_mesh, 
                           point_3 const & scale, point_3 * point = 0, point_3 * normal = 0 )
{
   double maxdist = 1e100;

   typedef  rasterized_triangle_mesh< vertex_type, index_type > :: mesh_type :: triangle_iterator_type< triangle_3 > TI;

   cartesian<> const traits;
   // Перебираем все треугольники
   for ( TI it = r_mesh.mesh().t_begin< triangle_3 >(), q = r_mesh.mesh().t_end< triangle_3 >(); it != q; ++it )
   {
      point_3 resp ;
      double dist;

      triangle_3 const & t = *it;
      const triangle_3_rti tr( t[ 0 ], t[ 2 ], t[ 1 ], normal( t ) );

      if ( scaled_distance_ex ( maxdist, tr, pt, dist, resp, traits, scale ) )
      {
         maxdist = dist ;

         if( point )
            *point = resp;

         if( normal )
            *normal = normal ( t ) ;
      }
   }

   return maxdist;
}

// ============================================= END DISTANCE ==========================================================

// ================================================ CYLINDER INTERSECT =================================================

template< typename vertex_type, typename index_type >
   bool cylinder_intersection( point_3 const & p1, point_3 const & p2, double radius, 
                                 rasterized_triangle_mesh< vertex_type, index_type > const & r_mesh, 
                                 point_3 * normal, point_3 * pos, double * p_ratio )
{
   point_3 p( p2 );

   bool res = false;
   double ratio = 1 ;

   typedef  triangle_3_fast< vertex_type >     TF;
   typedef  rasterized_triangle_mesh< vertex_type, index_type >::mesh_type::triangle_iterator_type< TF >   TI;

   cylinder_intersection_details::std_traits< triangle_3_fast< vertex_type > > traits ; 

   // Перебираем все треугольники
   for ( TI it = r_mesh.mesh().t_begin< TF >(), q = r_mesh.mesh().t_end< TF >(); it != q; ++it )
   {
      TF const & tr = *it;

      if(ratio <= 1e-5)
         break;

      point_3 resp;
      double r = 0;
      segment_3 const length ( p1, p ) ;
      if ( cylinder_triangle_intersection ( length, radius, tr, r, resp, traits ) )
      {
         res = true;

         if( !normal && !pos && !p_ratio )
            return true;

         ratio *= r;
         p = length( r );

         if ( pos ) 
            *pos  = resp;

         if ( normal )
            *normal = tr.normal();
      }
   }

   if ( res && p_ratio ) 
      *p_ratio = ratio ;

   return res ;
}

// ============================================ END CYLINDER INTERSECT =================================================

} // end of namespace cg