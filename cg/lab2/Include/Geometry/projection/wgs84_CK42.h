#pragma once

#include <geometry\primitives\geo_point.h>


namespace cg 
{
   geo_point_2 WGS84ToCK42 ( const geo_point_2 & pos ); 
   geo_point_2 CK42ToWGS84 ( const geo_point_2 & pos ); 
}

///////////////////////////////////////////////////////////////////////////////////////
// implementation 
namespace cg 
{
   namespace details
   {
      // source:   http://gis-lab.info/qa/wgs84-sk42-wgs84-formula.html

      // see also: http://www.gisinfo.ru/forum/phpBB2/viewtopic.php?t=4607

      double const Pi = 3.14159265358979 ;// Число Пи
      double const ro = 206264.8062 ;// Число угловых секунд в радиане

      // Эллипсоид Красовского
      double const aP  = 6378245 ;// Большая полуось
      double const alP = 1 / 298.3 ;// Сжатие
      double const e2P = 2 * alP - sqr(alP) ;// Квадрат эксцентриситета

      // Эллипсоид WGS84 (GRS80, эти два эллипсоида сходны по большинству параметров)
      double const aW  = 6378137 ;// Большая полуось
      double const alW = 1 / 298.257223563 ;// Сжатие
      double const e2W = 2 * alW - sqr(alW) ;// Квадрат эксцентриситета

      // Вспомогательные значения для преобразования эллипсоидов
      double const a   = (aP + aW) / 2 ;
      double const e2  = (e2P + e2W) / 2 ;
      double const da  = aW - aP ;
      double const de2 = e2W - e2P ;

      // Линейные элементы трансформирования, в метрах
      //double const dx_42_84 = 23.92 ;
      //double const dy_42_84 = -141.27;
      //double const dz_42_84 = -80.9;

      double const dx_42_84 = 28 ;
      double const dy_42_84 = -130;
      double const dz_42_84 = -95;

      double const dx_42_90 = 25 ;
      double const dy_42_90 = -141;
      double const dz_42_90 = -80;

      double const dx_90_84 = -1.08 ;
      double const dy_90_84 = -0.27 ;
      double const dz_90_84 = -0.9;

      // Угловые элементы трансформирования, в секундах
      double const wx = 0;
      double const wy = 0;
      double const wz = 0;
      // Дифференциальное различие масштабов
      double const ms = 0;

      inline double dB( double Bd, double Ld, double H, double dx, double dy, double dz)
      {
         double B, L, M, N;
         B = Bd * Pi / 180 ;
         L = Ld * Pi / 180 ;
         M = a * (1 - e2) / pow(1 - e2 * sqr(sin(B)), 1.5);
         N = a * pow(1 - e2 * sqr(sin(B)), -0.5);
         
         return ro / (M + H) * (N / a * e2 * sin(B) * cos(B) * da + 
                   (sqr(N) / sqr(a) + 1) * N * sin(B) * cos(B) * de2 / 2 
                   - (dx * cos(L) + dy * sin(L)) * sin(B) + dz * cos(B)) 
                   - wx * sin(L) * (1 + e2 * cos(2 * B)) + wy * cos(L) * (1 + e2 * cos(2 * B)) 
                   - ro * ms * e2 * sin(B) * cos(B);
      }

      inline double dL(double Bd, double Ld, double H, double dx, double dy, double dz)
      {
         double B, L, N ;
         B = Bd * Pi / 180;
         L = Ld * Pi / 180;
         N = a * pow(1 - e2 * sqr(sin(B)), -0.5);
         return ro / ((N + H) * cos(B)) * (-dx * sin(L) + dy * cos(L)) 
            + tan(B) * (1 - e2) * (wx * cos(L) + wy * sin(L)) - wz;
      }

      inline double WGS84Alt(double Bd, double Ld, double H, double dx, double dy, double dz)
      {
         double B, L, N, dH ;
         B = Bd * Pi / 180;
         L = Ld * Pi / 180;
         N = a * pow(1 - e2 * sqr(sin(B)), -0.5);
         dH = -a / N * da + N * sqr(sin(B)) * de2 / 2 
            + (dx * cos(L) + dy * sin(L)) * cos(B) + dz * sin(B) 
            - N * e2 * sin(B) * cos(B) * (wx / ro * sin(L) - wy / ro * cos(L)) 
            + (sqr(a) / N + H) * ms;
         return H + dH ; 
      }

      inline double PZ90_WGS84_Lat(double Bd, double Ld, double H)
      {
         return Bd + dB(Bd, Ld, H, dx_90_84, dy_90_84, dz_90_84) / 3600 ;
      }

      inline double PZ90_WGS84_Long(double Bd, double Ld, double H)
      {
         return Ld + dL(Bd, Ld, H, dx_90_84, dy_90_84, dz_90_84) / 3600 ; 
      }

      inline double WGS84_PZ90_Lat(double Bd, double Ld, double H)
      {
         return Bd - dB(Bd, Ld, H, dx_90_84, dy_90_84, dz_90_84) / 3600 ; 
      }

      inline double WGS84_PZ90_Long(double Bd, double Ld, double H)
      {
         return Ld - dL(Bd, Ld, H, dx_90_84, dy_90_84, dz_90_84) / 3600 ; 
      }

      //
      inline double CK42_PZ90_Lat(double Bd, double Ld, double H)
      {
         return Bd + dB(Bd, Ld, H, dx_42_90, dy_42_90, dz_42_90) / 3600 ;
      }

      inline double CK42_PZ90_Long(double Bd, double Ld, double H)
      {
         return Ld + dL(Bd, Ld, H, dx_42_90, dy_42_90, dz_42_90) / 3600 ; 
      }

      inline double PZ90_CK42_Lat(double Bd, double Ld, double H)
      {
         return Bd - dB(Bd, Ld, H, dx_42_90, dy_42_90, dz_42_90) / 3600 ; 
      }

      inline double PZ90_CK42_Long(double Bd, double Ld, double H)
      {
         return Ld - dL(Bd, Ld, H, dx_42_90, dy_42_90, dz_42_90) / 3600 ; 
      }

      //
      inline double CK42_WGS84_Lat(double Bd, double Ld, double H)
      {
         return Bd + dB(Bd, Ld, H, dx_42_84, dy_42_84, dz_42_84) / 3600 ;
      }

      inline double CK42_WGS84_Long(double Bd, double Ld, double H)
      {
         return Ld + dL(Bd, Ld, H, dx_42_84, dy_42_84, dz_42_84) / 3600 ; 
      }

      inline double WGS84_CK42_Lat(double Bd, double Ld, double H)
      {
         return Bd - dB(Bd, Ld, H, dx_42_84, dy_42_84, dz_42_84) / 3600 ; 
      }

      inline double WGS84_CK42_Long(double Bd, double Ld, double H)
      {
         return Ld - dL(Bd, Ld, H, dx_42_84, dy_42_84, dz_42_84) / 3600 ; 
      }



      // reference point from gis viewer 2008
      // geo_point_2 ref_wgs  (58.15972634, 38.20420185) ; 
      // geo_point_2 ref_ck42 (58.15964081, 38.20620722) ; 
   }

   inline geo_point_2 CK42ToPZ90 ( const geo_point_2 & pos )
   {
      return geo_point_2 ( details::CK42_PZ90_Lat(pos.lat, pos.lon, 0), details::CK42_PZ90_Long(pos.lat, pos.lon, 0)); 
   }

   inline geo_point_2 PZ90ToCK42 ( const geo_point_2 & pos ) 
   {
      return geo_point_2 ( details::PZ90_CK42_Lat(pos.lat, pos.lon, 0), details::PZ90_CK42_Long(pos.lat, pos.lon, 0)); 
   }

   inline geo_point_2 PZ90ToWGS84 ( const geo_point_2 & pos )
   {
      return geo_point_2 ( details::PZ90_WGS84_Lat(pos.lat, pos.lon, 0), details::PZ90_WGS84_Long(pos.lat, pos.lon, 0)); 
   }

   inline geo_point_2 WGS84ToPZ90 ( const geo_point_2 & pos ) 
   {
      return geo_point_2 ( details::WGS84_PZ90_Lat(pos.lat, pos.lon, 0), details::WGS84_PZ90_Long(pos.lat, pos.lon, 0)); 
   }

   inline geo_point_2 CK42ToWGS84 ( const geo_point_2 & pos )
   {
      return geo_point_2 ( details::CK42_WGS84_Lat(pos.lat, pos.lon, 0), details::CK42_WGS84_Long(pos.lat, pos.lon, 0)); 
   }

   inline geo_point_2 WGS84ToCK42 ( const geo_point_2 & pos ) 
   {
      return geo_point_2 ( details::WGS84_CK42_Lat(pos.lat, pos.lon, 0), details::WGS84_CK42_Long(pos.lat, pos.lon, 0)); 
   }

   // 
   //inline geo_point_2 CK42ToWGS84 ( const geo_point_2 & pos )
   //{
   //   return PZ90ToWGS84(CK42ToPZ90(pos)); 
   //}

   //inline geo_point_2 WGS84ToCK42 ( const geo_point_2 & pos ) 
   //{
   //   return PZ90ToCK42(WGS84ToPZ90(pos)); 
   //}
}



