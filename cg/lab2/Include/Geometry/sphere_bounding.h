#pragma once

#include <CGAL/Cartesian_d.h>
#include <CGAL/Min_sphere_of_spheres_d.h>

namespace cg
{
   template < class sphere_type, class FwdIter >
      sphere_type bounding_sphere ( FwdIter p, FwdIter q )
   {
      typedef sphere_type::scalar_type S ;
      size_t const D = sphere_type::dimension ;
      typedef sphere_type::point_type  point_type ;

      typedef CGAL::Cartesian_d<S>                                K ;
      typedef CGAL::Min_sphere_of_spheres_d_traits_d< K, S, D >   traits_type ;
      typedef CGAL::Min_sphere_of_spheres_d< traits_type >        min_sphere_type ;

      std::vector< traits_type::Sphere > spheres ;
      spheres.reserve(std::distance(p, q)) ;

      for (FwdIter it = p; it != q; ++it)
      {
         S p[D] ;
         for (size_t i = 0; i < D; i++)
            p[i] = (*it)[i] ;
         spheres.push_back(traits_type::Sphere(K::Point_d(D, p, p + D), S(std::numeric_limits< double >::min()))) ;
      }

      min_sphere_type ms(spheres.begin(), spheres.end()) ;

      sphere_type sph ;
      sph.radius = ms.radius() ;
      traits_type::Cartesian_const_iterator cit = ms.center_cartesian_begin() ;
      for (size_t i = 0; i < D; i++, ++cit)
         sph.center[i] = *cit ;


      return sph ;
   }

   // описанная окружность минимального радиуса для точек p
   //       template < class S, size_t D >
   //          __forceinline static sphere_t< S, D > min_sphere ( point_t< S, D > * p, size_t size )
   //       {
   //          typedef sphere_t< S, D > sphere_type ;
   //          
   //          Assert(size > 1) ;
   // 
   //          perturb(p, size) ;
   // 
   //          sphere_type sph((p[0] + p[1]) * 0.5, distance(p[0], p[1]) * 0.5 + epsilon<float>()) ;
   //    
   //          for (size_t i = 2; i < size; i++)
   //             if (!sph.contains(p[i]))
   //             {
   //                sph = min_sphere_1(p, i, p[i]) ;
   // 
   //                Assert(sph.contains(p[i])) ;
   //                for (size_t j = 0; j <= i; j++)
   //                   Assert(sph.contains(p[j])) ;
   //             }
   // 
   //          return sph ;
   //       }
   // 
   //       // описанная окружность минимального радиуса для точек p, такая, что  точка q лежит на её границе
   //       template < class S, size_t D >
   //          __forceinline static sphere_t< S, D > min_sphere_1 ( point_t< S, D > * p, size_t size, point_t< S, D > const& q )
   //       {
   //          typedef sphere_t< S, D > sphere_type ;
   // 
   //          perturb(p, size) ;
   // 
   //          sphere_type sph((p[0] + q) * 0.5, distance(p[0], q) * 0.5 + epsilon<float>()) ;
   // 
   //          for (size_t i = 1; i < size; i++)
   //             if (!sph.contains(p[i]))
   //             {
   //                sph = min_sphere_2(p, i, q, p[i]) ;
   // 
   //                Assert(sph.contains(p[i])) ;
   //                Assert(sph.contains(q)) ;
   //                for (size_t j = 0; j <= i; j++)
   //                   Assert(sph.contains(p[j])) ;
   //             }
   // 
   //          return sph ;
   //       }
   // 
   //       // описанная окружность минимального радиуса для точек p, такая, что  точки q, r лежали на её границе
   //       template < class S, size_t D >
   //          __forceinline static sphere_t< S, D > min_sphere_2 ( point_t< S, D > * p, size_t size, point_t< S, D > const& q, point_t< S, D > const& r )
   //       {
   //          typedef sphere_t< S, D > sphere_type ;
   // 
   //          perturb(p, size) ;
   // 
   //          sphere_type sph((r + q) * 0.5, distance(r, q) * 0.5 + epsilon<float>()) ;
   // 
   //          for (size_t i = 0; i < size; i++)
   //             if (!sph.contains(p[i]))
   //             {
   //                sph = min_sphere_3(p, i, q, r, p[i]) ;
   // 
   //                Assert(sph.contains(p[i])) ;
   //                Assert(sph.contains(q)) ;
   //                Assert(sph.contains(r)) ;
   // 
   //                for (size_t j = 0; j <= i; j++)
   //                   Assert(sph.contains(p[j])) ;
   //             }
   // 
   //          return sph ;
   //       }
   // 
   //       // описанная окружность минимального радиуса для точек p, такая, что  точки q, r, s лежали на её границе
   //       template < class S, size_t D >
   //          __forceinline static sphere_t< S, D > min_sphere_3 ( point_t< S, D > * p, size_t size, point_t< S, D > const& q, point_t< S, D > const& r, point_t< S, D > const& s )
   //       {
   //          typedef sphere_t< S, D > sphere_type ;
   // 
   //          perturb(p, size) ;
   // 
   //          sphere_type sph = min_sphere_in_plane(q, r, s) ;
   // 
   //          for (size_t i = 0; i < size; i++)
   //             if (!sph.contains(p[i]))
   //             {
   //                sph = min_sphere_4(q, r, s, p[i]) ;
   // 
   //                Assert(sph.contains(p[i])) ;
   //                Assert(sph.contains(q)) ;
   //                Assert(sph.contains(r)) ;
   //                Assert(sph.contains(s)) ;
   // 
   //                for (size_t j = 0; j <= i; j++)
   //                   Assert(sph.contains(p[j])) ;
   //             }
   // 
   //             return sph ;
   //       }
   // 
   //       // описанная окружность вокруг для точек p, q, r in plane (p, q, r)
   //       template < class S, size_t D >
   //          __forceinline static sphere_t< S, D > min_sphere_in_plane ( point_t< S, D > const& p, point_t< S, D > const& q, point_t< S, D > const& r )
   //       {
   //          typedef sphere_t< S, D > sphere_type ;
   //          typedef point_t< S, D >  point_type ;
   //          
   //          point_type a = q - p ;
   //          point_type b = r - p ;
   // 
   //          point_type c = (a ^ b) ;
   //          point_type d = c ^ a ;
   // 
   //          if (-b * (q - r) < 0)
   //             d *= -1 ;
   // 
   //          S aNorm = norm(a) ;
   //          S bNorm = norm(b) ;
   //          S cNorm = norm(c) ;
   // 
   //          Assert(!cg::eq_zero(cNorm)) ;
   // 
   //          S radius = distance(q, r) / (2 * cNorm) * aNorm * bNorm ;
   // 
   //          Assert(cg::ge(radius * radius - aNorm * aNorm / 4, 0)) ;
   // 
   //          point_type center = (p + q) / 2 + sqrt(cg::max(S(0), radius * radius - aNorm * aNorm / 4)) * cg::normalized_safe(d)  ;
   // 
   //          return sphere_type(center, radius + epsilon<float>()) ;
   //       }
   // 
   //       // описанная окружность вокруг для точек p, q, r, s
   //       template < class S, size_t D >
   //          __forceinline static sphere_t< S, D > min_sphere_4 ( point_t< S, D > const& p, point_t< S, D > const& q, point_t< S, D > const& r, point_t< S, D > const& s )
   //       {
   //          typedef sphere_t< S, D > sphere_type ;
   //          typedef point_t< S, D >  point_type ;
   // 
   //          cg::matrix_4 m ;
   //          m(0, 0) = p.x, m(0, 1) = p.y, m(0, 2) = p.z, m(0, 3) = 1 ;
   //          m(1, 0) = q.x, m(1, 1) = q.y, m(1, 2) = q.z, m(1, 3) = 1 ;
   //          m(2, 0) = r.x, m(2, 1) = r.y, m(2, 2) = r.z, m(2, 3) = 1 ;
   //          m(3, 0) = s.x, m(3, 1) = s.y, m(3, 2) = s.z, m(3, 3) = 1 ;
   //          S M11 = determinant(m) ;
   // 
   //          m(0, 0) = norm_sqr(p) ;
   //          m(1, 0) = norm_sqr(q) ;
   //          m(2, 0) = norm_sqr(r) ;
   //          m(3, 0) = norm_sqr(s) ;
   //          S M12 = determinant(m) ;
   // 
   //          m(0, 1) = p.x ;
   //          m(1, 1) = q.x ;
   //          m(2, 1) = r.x ;
   //          m(3, 1) = s.x ;
   //          S M13 = determinant(m) ;
   // 
   //          m(0, 2) = p.y ;
   //          m(1, 2) = q.y ;
   //          m(2, 2) = r.y ;
   //          m(3, 2) = s.y ;
   //          S M14 = determinant(m) ;
   // 
   //          m(0, 3) = p.z ;
   //          m(1, 3) = q.z ;
   //          m(2, 3) = r.z ;
   //          m(3, 3) = s.z ;
   //          S M15 = determinant(m) ;
   // 
   //          Assert(!cg::eq_zero(M11)) ;
   //          S M11Inv = 1 / M11 ;
   //          point_type center(0.5 * M12 * M11Inv, -0.5 * M13 * M11Inv, 0.5 * M14 * M11Inv) ;
   // 
   //          Assert(cg::ge(norm_sqr(center) - M15 * M11Inv, 0)) ;
   // 
   //          return sphere_type(center, sqrt(cg::max(S(0), norm_sqr(center) - M15 * M11Inv)) + epsilon<float>()) ;
   //       }
   // 
   //       template < class point_t >
   //          __forceinline void perturb( point_t * p, size_t size )
   //       {
   //          for (size_t i = 0; i < size; i++)
   //          {
   //             size_t idx = ::rand() % size ;
   //             point_t pt = p[idx] ;
   //             p[idx] = p[i] ;
   //             p[i] = pt ;
   //          }
   //       }
}