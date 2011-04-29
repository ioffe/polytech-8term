#pragma once

// interface 
namespace cg 
{
   namespace merkator_wgs84
   {
      geo_point_2 toGeo ( const glb_point_2 & pos, double base_lat = 0, double eps = 0.001 ); 
      glb_point_2 toGlb ( const geo_point_2 & pos, double base_lat = 0 ); 

      geo_point_2 toGeoSphere ( const glb_point_2 & pos, double base_lat = 0 ); 
      glb_point_2 toGlbSphere ( const geo_point_2 & pos, double base_lat = 0 ); 

      // note. "toGeo::eps" in meters, 10 cycles max. eps=0.001 may take 7 cycles  
   }
}

///////////////////////////////////////////////////////////////////////////////////////
// implementation 
namespace cg 
{
   namespace merkator
   {
      namespace details
      {
         inline double next_sinQ ( double KY, double E, double sinQ )
         {
            return 1 - (1 + sinQ)*pow((1 - E*sinQ)/(1 + E*sinQ), E) / KY ; 
         }
      }

      inline double melat ( double lat )
      {
         return log(tan(lat/2 + pi/4));
      }

      inline double latit ( double mlt )
      {
         return (atan(exp(mlt)) - pi/4) * 2;
      }

      inline double melat ( double lat, double E ) 
      {
         return log(tan(pi/4 + lat/2) * pow ((1 - E*sin(lat))/(1 + E*sin(lat)), E/2)) ;  
      }

      inline double latit ( double mlt, double E, double eps )
      {
         double KY = exp(2*mlt) ; 

         double sinQ = sin(latit(mlt)) ; 

         double eps_sq = fabs(sinQ - sin(latit(mlt+eps))); // todo 

         for ( size_t i = 0 ; i < 10 ; i ++ ) 
         {
            double next = details::next_sinQ(KY, E, sinQ) ; 

            if ( eq(sinQ, next, eps_sq) ) 
               return asin(next) ; 

            sinQ = next ; 
         }
         return asin(sinQ); 
      }
   }

   namespace wgs84
   {
      const double Rbig   = 6378137 ; 
      const double Rsmall = 6356752.314245; 
      const double E      = sqrt(1 - sqr(Rsmall/Rbig)); 
   }

   namespace merkator_wgs84
   {
      inline geo_point_2 toGeo ( const glb_point_2 & pos, double base_lat, double eps )
      {
         double factor = wgs84::Rbig*cos(grad2rad(base_lat)); 

         return geo_point_2( rad2grad(merkator::latit(pos.Y/factor, wgs84::E, eps/factor)), rad2grad(pos.X/factor) ) ;  
      }

      inline glb_point_2 toGlb ( const geo_point_2 & pos, double base_lat )
      {
         double factor = wgs84::Rbig*cos(grad2rad(base_lat)); 

         return glb_point_2(grad2rad(pos.lon)*factor, merkator::melat(grad2rad(pos.lat), wgs84::E)*factor ) ; 
      }

      inline geo_point_2 toGeoSphere ( const glb_point_2 & pos, double base_lat )
      {
         double factor = wgs84::Rbig*cos(grad2rad(base_lat)); 

         return geo_point_2(rad2grad(merkator::latit(pos.Y/factor)), rad2grad(pos.X/factor) ) ;  
      }

      inline glb_point_2 toGlbSphere ( const geo_point_2 & pos, double base_lat )
      {
         double factor = wgs84::Rbig*cos(grad2rad(base_lat)); 

         return glb_point_2(grad2rad(pos.lon)*factor, merkator::melat(grad2rad(pos.lat))*factor ) ; 
      }
   }
}
