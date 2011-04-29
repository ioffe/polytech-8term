#pragma once

#include "primitives/triangle_fwd.h"
#include "primitives/sphere_fwd.h"
#include "sphere_intersect.h"

namespace cg
{
   namespace details
   {
      template< class S, class Triangle >
      S fast_distance_sqr( point_t< S, 3 > const &point, Triangle const &tri, point_t< S, 3 > &closest )
      {
         typedef point_t< S, 3 > point_type; 
         typedef S               scalar_type;

         scalar_type rS, rT;

         point_type vOrigin(tri[0]);
         point_type vEdge0 (tri[1] - vOrigin);
         point_type vEdge1 (tri[2] - vOrigin);

         point_type vDiff(vOrigin - point);
         scalar_type fA00 = cg::norm_sqr(vEdge0);
         scalar_type fA01 = vEdge0 * vEdge1;
         scalar_type fA11 = cg::norm_sqr(vEdge1);
         scalar_type fB0  = vDiff * vEdge0;
         scalar_type fB1  = vDiff * vEdge1;
         scalar_type fC   = cg::norm_sqr(vDiff);
         scalar_type fDet = cg::abs(fA00 * fA11-fA01 * fA01);
         scalar_type fS = fA01 * fB1 - fA11 * fB0;
         scalar_type fT = fA01 * fB0 - fA00 * fB1;
         scalar_type fSqrDist;

         if (cg::eq_zero(fDet)) //Triangle is degenerate!
         {
            rS = 1;
            rT = 0;
            return fC;
         }

         if (fS + fT <= fDet)
         {
            if (fS < 0)
            {
               if (fT < 0)  // region 4
               {
                  if (fB0 < 0)
                  {
                     fT = 0;
                     if (-fB0 >= fA00)
                     {
                        fS = 1;
                        fSqrDist = fA00 + 2 * fB0 + fC;
                     }
                     else
                     {
                        fS = -fB0 / fA00;
                        fSqrDist = fB0 * fS + fC;
                     }
                  }
                  else
                  {
                     fS = 0;
                     if (fB1 >= 0)
                     {
                        fT = 0;
                        fSqrDist = fC;
                     }
                     else if (-fB1 >= fA11)
                     {
                        fT = 1;
                        fSqrDist = fA11 + 2 * fB1 + fC;
                     }
                     else
                     {
                        fT = -fB1 / fA11;
                        fSqrDist = fB1 * fT + fC;
                     }
                  }
               }
               else  // region 3
               {
                  fS = 0;
                  if (fB1 >= 0)
                  {
                     fT = 0;
                     fSqrDist = fC;
                  }
                  else if (-fB1 >= fA11)
                  {
                     fT = 1;
                     fSqrDist = fA11 + 2 * fB1 + fC;
                  }
                  else
                  {
                     fT = -fB1 / fA11;
                     fSqrDist = fB1 * fT + fC;
                  }
               }
            }
            else if (fT < 0)  // region 5
            {
               fT = 0;
               if (fB0 >= 0)
               {
                  fS = 0;
                  fSqrDist = fC;
               }
               else if (-fB0 >= fA00)
               {
                  fS = 1;
                  fSqrDist = fA00 + 2 * fB0 + fC;
               }
               else
               {
                  fS = -fB0 / fA00;
                  fSqrDist = fB0 * fS + fC;
               }
            }
            else  // region 0
            {
               // minimum at interior point
               scalar_type fInvDet = scalar_type(1) / fDet;
               fS *= fInvDet;
               fT *= fInvDet;
               fSqrDist = fS * (fA00 * fS+fA01 * fT + 2 * fB0) +
                  fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
            }
         }
         else
         {
            scalar_type fTmp0, fTmp1, fNumer, fDenom;

            if (fS < 0)  // region 2
            {
               fTmp0 = fA01 + fB0;
               fTmp1 = fA11 + fB1;
               if (fTmp1 > fTmp0)
               {
                  fNumer = fTmp1 - fTmp0;
                  fDenom = fA00 - 2 * fA01 + fA11;
                  if (fNumer >= fDenom)
                  {
                     fS = 1;
                     fT = 0;
                     fSqrDist = fA00 + 2 * fB0 + fC;
                  }
                  else
                  {
                     fS = fNumer / fDenom;
                     fT = 1 - fS;
                     fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) +
                        fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
                  }
               }
               else
               {
                  fS = 0;
                  if (fTmp1 <= 0)
                  {
                     fT = 1;
                     fSqrDist = fA11 + 2 * fB1 + fC;
                  }
                  else if (fB1 >= 0)
                  {
                     fT = 0;
                     fSqrDist = fC;
                  }
                  else
                  {
                     fT = -fB1 / fA11;
                     fSqrDist = fB1 * fT + fC;
                  }
               }
            }
            else if (fT < 0)  // region 6
            {
               fTmp0 = fA01 + fB1;
               fTmp1 = fA00 + fB0;
               if (fTmp1 > fTmp0)
               {
                  fNumer = fTmp1 - fTmp0;
                  fDenom = fA00 - 2 * fA01 + fA11;
                  if (fNumer >= fDenom)
                  {
                     fT = 1;
                     fS = 0;
                     fSqrDist = fA11 + 2 * fB1 + fC;
                  }
                  else
                  {
                     fT = fNumer / fDenom;
                     fS = 1 - fT;
                     fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) +
                        fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
                  }
               }
               else
               {
                  fT = 0;
                  if (fTmp1 <= 0)
                  {
                     fS = 1;
                     fSqrDist = fA00 + 2 * fB0 + fC;
                  }
                  else if (fB0 >= 0)
                  {
                     fS = 0;
                     fSqrDist = fC;
                  }
                  else
                  {
                     fS = -fB0 / fA00;
                     fSqrDist = fB0 * fS + fC;
                  }
               }
            }
            else  // region 1
            {
               fNumer = fA11 + fB1 - fA01 - fB0;
               if (fNumer <= 0)
               {
                  fS = 0;
                  fT = 1;
                  fSqrDist = fA11 + 2 * fB1 + fC;
               }
               else
               {
                  fDenom = fA00 - 2 * fA01 + fA11;
                  if (fNumer >= fDenom)
                  {
                     fS = 1;
                     fT = 0;
                     fSqrDist = fA00 + 2 * fB0 + fC;
                  }
                  else
                  {
                     fS = fNumer / fDenom;
                     fT = 1 - fS;
                     fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) +
                        fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
                  }
               }
            }
         }

         rS = 1 - fS - fT;
         rT = fS;  

         // Avoid little problem with loss of accuracy    
         rS = cg::bound<scalar_type>(rS, 0, 1);
         rT = cg::bound<scalar_type>(rT, 0, 1);

         closest = vOrigin + fS * vEdge0 + fT * vEdge1;

         return cg::abs(fSqrDist);
      }

   }

   template< class S, class Triangle >
      bool sphere_triangle_intersection( sphere_t< S, 3 > const& sph, Triangle const& tri, SPHERE_INTERSECT_DETAIL *detail )
   {
      // Check if detail is already filled
      if(detail && detail->points.full())
         return false;

      typedef Triangle        triangle_type;
      typedef S               scalar_type;
      typedef point_t< S, 3 > point_type; 

      point_type tp;
      scalar_type distSqr = details::fast_distance_sqr(sph.center, tri, tp);

      if (distSqr > sph.radius * sph.radius)
         return false;

      point_type tn = normal(tri);

      point_type projDir (tp - sph.center);
      projDir = cg::normalized_safe(projDir);

      point_type sn;

      scalar_type dot = projDir * tn;
      if (dot < 0)
      {  
         // Triangle lies lower than sphere center
         Assert(!cg::eq_zero(cg::norm(projDir)));
         sn = projDir;    
      }
      else if (!cg::ge(dot, 1))
      {
         // Triangle lies upper than sphere center, and intersection point is not exactly under sphere center
         Assert(!cg::eq_zero(cg::norm(projDir)));
         sn = projDir - (2 * dot) * tn;
      }  
      else
      {
         // Triangle lies upper than sphere center, and intersection point is exactly under sphere center
         sn = -tn;
      }

      point_type sp = sph.center + sn * sph.radius;  

      // Old method calculates depth, on which sphere intersects plane containing triangle.
      //scalar_type depth = sph.radius - ((sph.center - tp) * tn);

      // This method calculates depth, on which sphere intersects triangle.
      scalar_type depth;
      if (cg::ge(abs(dot), 1))
      {
         // Intersection point is on same vertical with sphere center
         depth = sph.radius - ((sph.center - tp) * tn);
      }
      else if (cg::le(abs(dot), 0))
      {
         // Intersection point is on same horizontal with sphere center
         depth = 0;
      }
      else
      {
         scalar_type h = (sph.center - tp) * tn;
         scalar_type x2 = distSqr - cg::sqr(h);
         if (x2 < 0)
            return false;
         scalar_type y2 = cg::sqr(sph.radius) - x2;
         if (y2 < 0)
            return false;

         depth = cg::sqrt(y2) - h;
      }

      if (depth < 0.) // Hm... looks like hack :(
         return false;

      if(detail)
         detail->points.push_back(SPHERE_INTERSECT_DETAIL::POINT_DETAIL(sp - sph.center, tn, depth));

      return true;
   }

} // cg
