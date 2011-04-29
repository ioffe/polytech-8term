#pragma once

#include <geometry\primitives\glb_point.h>
#include <geometry\primitives\geo_point.h>

// interface 
namespace cg 
{
   namespace gk_ck42
   {
      int get_zone  ( const glb_point_2 & pos ) ; 
      int get_zone  ( const geo_point_2 & pos ) ; 

      geo_point_2 toGeo ( const glb_point_2 & pos, int zone ); 
      glb_point_2 toGlb ( const geo_point_2 & pos, int zone ); 

      geo_point_2 toGeo ( const glb_point_2 & pos ); 
      glb_point_2 toGlb ( const geo_point_2 & pos ); 
   }
}

///////////////////////////////////////////////////////////////////////////////////////
// implementation 
namespace cg 
{
   namespace gk_ck42
   {
      namespace details
      {
         const double r   = 6367558.497;
         const double bb2 =  8376121;
         const double bb4 =  590.42;
         const double bb6 =  1.68;

         const double R   = 6367558.497;
         const double AA2 = 5333.5419;
         const double AA4 = 4.84339;
         const double AA6 = 0.007622;

         inline void Linear2Gp(const double xx, const double yy, double &lat, double &lon, int FZone) 
         {
            double FEast = FZone*1000000. + 500000.;

            double Lgrad = ( FZone - 1 )*6 + 3;
            double FL0 = Lgrad * cg::pi / 180;

            double a3,b3,a1,b1,a2,b2,sfi,u;
            double c1,d1,c2,d2,c3,d3,tgl,v;
            double fi,psi,p;

            double x = yy;
            double y = xx - FEast;

            u     = x / r ;
            v     = y / r ;
            a1    = sin (2.0 * u);
            b1    = cos (2.0 * u);
            a2    = 2.0 * a1 * b1;
            b2    = 1.0 - 2.0*a1*a1;
            a3    = a1 * b2 + a2 * b1;
            b3    = b1 * b2 - a1 * a2;
            c1    = ( exp ( 2.0 * v) - exp (-2.0 * v ) ) / 2.0 ;
            d1    = ::sqrt ( 1.0 + c1*c1 ) ;
            c2    = 2.0 * c1 * d1 ;
            d2    = 1.0 + 2.0 * c1*c1 ;
            c3    = c1 * d2 + c2 * d1 ;
            d3    = c1 * c2 + d1 * d2 ;
            psi   = u - ( bb2*a1*d1 + bb4*a2*d2 + bb6*a3*d3 ) * 1.0E-10 ;
            p     = v - ( bb2*b1*c1 + bb4*b2*c2 + bb6*b3*c3 ) * 1.0E-10 ;
            sfi   =  sin ( psi ) / ( ( exp(p) + exp(-p) ) / 2.0 ) ;
            fi    =  atan ( sfi / (::sqrt (1.0 - sfi*sfi) +1.E-20 )  ) ;
            tgl   =( ( exp(p) - exp(-p) ) / 2.0 ) / ( cos(psi) + 1.E-20 ) ;
            lon = (atan ( tgl ) + FL0)*180/cg::pi;
            lat = ((fi + (( 5645.0 * sfi*sfi - 531245.0 ) * sfi * sfi + 67385254.0)
               * sfi * cos(fi) * 1.0E-10)*180)/cg::pi;
         }

         static   void Gp2Linear(const double lat, const double lon, double &x, double &y, int FZone) 
         {
            double a3,b3,a1,b1,a2,b2;
            double c1,c2,c3,d1,d2,d3;
            double sin2b,thp;
            double fi,psi,p;

            double L0 = double((FZone-1)*6 + 3)*cg::pi/180; // central meridium
            double East = FZone*1000000.+500000.; // East offset

            double b = lat*cg::pi/180;
            double l = lon*cg::pi/180;

            if ( b > 2*cg::pi )
               b = ( b/(2*cg::pi) - long ( b/(2*cg::pi) )) * 2*cg::pi;
            if ( l > 2*cg::pi )
               l = ( l/(2*cg::pi) - long ( l/(2*cg::pi) )) * 2*cg::pi;

            l = l - L0;

            sin2b = sin(b) * sin(b);
            fi = b-(( 2624.0*sin2b + 372834.0) * sin2b + 66934216.0)*sin(b)*cos(b)*1.0E-10;
            thp = cos(fi)*sin(l);
            psi = atan( sin(fi)/(cos(fi) + 1.E-20)/(cos(l) + 1.E-20 ));
            p = 0.5*log ((1.0+thp) / (1.0-thp));

            a1 = sin( 2.0*psi );
            b1 = cos( 2.0*psi );
            c1 = 2.0*thp           / ( 1.0 - thp*thp );
            d1 = ( 1.0 + thp*thp ) / ( 1.0 - thp*thp );
            a2 = 2.0*a1*b1;
            b2 = 1.0 - 2.0*a1*a1;
            c2 = 2.0*c1*d1;
            d2 = 1.0 + 2.0*c1*c1;
            a3 = a1*b2 + a2*b1;
            b3 = b1*b2 - a1*a2;
            c3 = c1*d2 + c2*d1;
            d3 = d1*d2 + c1*c2;

            y =  R*psi + AA2*a1*d1 + AA4*a2*d2 + AA6*a3*d3;
            x =  R*p   + AA2*b1*c1 + AA4*b2*c2 + AA6*b3*c3 + East;
         }
      }

      inline int get_zone ( const glb_point_2 & pos ) 
      {
         return (int)floor(pos.X / 1000000) ; 
      }

      inline int get_zone ( const geo_point_2 & pos ) 
      {
         return (int)floor(pos.lon / 6) + 1 ; 
      }

      inline geo_point_2 toGeo ( const glb_point_2 & pos, int zone )
      {
         geo_point_2 ret ;
         details::Linear2Gp( pos.X, pos.Y, ret.lat, ret.lon, zone ) ;

         return   ret ;
      }

      inline glb_point_2 toGlb ( const geo_point_2 & pos, int zone )
      {
         glb_point_2 ret ;
         details::Gp2Linear( pos.lat, pos.lon, ret.X, ret.Y, zone ) ;

         return   ret ;
      }

      inline geo_point_2 toGeo ( const glb_point_2 & pos )
      {
         return toGeo ( pos, get_zone(pos) );
      }

      inline glb_point_2 toGlb ( const geo_point_2 & pos )
      {
         return toGlb ( pos, get_zone(pos) );
      }
   }
}