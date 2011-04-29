#pragma once

namespace cg
{
   // ----------------------------------------------------------------------------
   //
   //       0 1  
   //          /
   // y1    | |
   // y0    |/|
   //       /
   // ----------------------------------------------------------------------------
   template<class T, class D> 
   D lerp01( T x, D y0, D y1 )
   {
      return y0 + x * (y1 - y0);
   }

   // ----------------------------------------------------------------------------
   // Hermit curve lerp
   // t = 3x^2 - 2x^3
   // ----------------------------------------------------------------------------
   template<class T, class D> 
   D slerp01( T x, D y0, D y1 )
   {
      T const t = (3 - 2 * x) * x * x;
      return y0 + t * (y1 - y0);
   }

   // ----------------------------------------------------------------------------
   //
   //      x0 x1  
   //          /
   // y1    | |
   // y0    |/|
   //      /
   // ----------------------------------------------------------------------------
   /*
   template<class T, class D = T>
   struct Lerp
   {
      Lerp( T x0, T x1, D y0, D y1 ) :
         k((y1 - y0) / (x1 - x0)), b(y0 - k * x0)
      {
      }

      D operator () ( T x ) const
      {
         return k * x + b;
      }

   private:
      D k, b;
   };
   */

   template<class T, class D> 
   D lerp( T x, T x0, T x1, D y0, D y1 )
   {
      return (y1 - y0) * ((x - x0) / (x1 - x0)) + y0;
   }

   template<class T, class D> 
   D lerp_points( T x, T x0, T x1, D y0, D y1 )
   {
      return ((y1 - y0) & ((x - x0) / (x1 - x0)) )+ y0;
   }

   // ----------------------------------------------------------------------------
   //
   //      x0 x1  
   // y1    | |____
   // y0____|/|
   //
   // ----------------------------------------------------------------------------
   template<class T, class D = T>
   struct LerpClamp
   {
      LerpClamp( T x0, T x1, D y0, D y1 ) :
         x0(x0), x1(x1), y0(y0), y1(y1), 
         k((y1 - y0) / (x1 - x0)), b(y0 - k * x0)
      {
      }

      D operator () ( T x ) const
      {
         if (x <= x0)
            return y0;
         if (x >= x1)
            return y1;
         return k * x + b;
      }

      T x0, x1;
      D y0, y1, k, b;
   };

   template<class T, class D> 
   D lerp_clamp( T x, T x0, T x1, D y0, D y1 )
   {
      if (x <= x0)
         return y0;
      if (x >= x1)
         return y1;
      return (y1 - y0) * ((x - x0) / (x1 - x0)) + y0;
   }


   // ----------------------------------------------------------------------------
   //
   //     x0 x1 x2 
   // y1    | | |
   // y0____|/|\|____
   //
   // ----------------------------------------------------------------------------
   template<class T, class D>
   D key_lerp_clamp( T x, T x0, T x1, T x2, D y0, D y1 )
   {
      if (x <= x0 || x >= x2)
         return y0;
      if (x < x1)
         return (y1 - y0) * ((x - x0) / (x1 - x0)) + y0;
      return (y1 - y0) * ((x - x2) / (x1 - x2)) + y0;
   }


   // ----------------------------------------------------------------------------
   //
   //     x0 x1 x2 
   // y2    | | |____
   // y1    | |/|
   // y0____|/| |
   //
   // ----------------------------------------------------------------------------
   template<class T, class D = T>
   struct KeyLerpClamp
   {
//       KeyLerpClamp( T x0, T x1, T x2, D y0, D y1 ) :
//          x0(x0), x1(x1), x2(x2), y0(y0), y1(y1), y2(y0), 
//          k0((y1 - y0) / (x1 - x0)), b0(y0 - k0 * x0),
//          k1((y1 - y0) / (x1 - x2)), b1(y0 - k1 * x2)
//       {
//       }

      KeyLerpClamp( T x0, T x1, T x2, D y0, D y1, D y2 )
         : x0(x0), x1(x1), x2(x2), y0(y0), y1(y1), y2(y2)
         , k0((y1 - y0) / (x1 - x0)), b0(y0 - k0 * x0)
         , k1((y2 - y1) / (x2 - x1)), b1(y2 - k1 * x2)
      {
      }

      D operator () ( T x ) const
      {
         if (x <= x0)
            return y0;
         if (x < x1)
            return k0 * x + b0;
         if (x < x2)
            return k1 * x + b1;
         return y2;
      }

   public:
      T x0, x1, x2;
      D y0, y1, y2, k0, b0, k1, b1;
   };


   template<class T, class D>
   D key_lerp_clamp( T x, T x0, T x1, T x2, D y0, D y1, D y2 )
   {
      if (x <= x0)
         return y0;
      if (x >= x2)
         return y2;
      if (x < x1)
         return (y1 - y0) * ((x - x0) / (x1 - x0)) + y0;
      return (y2 - y1) * ((x - x1) / (x2 - x1)) + y1;
   }


   // ----------------------------------------------------------------------------
   //      x0 x1  
   // y1    |  |____
   // y0____|_/|
   //
   // ----------------------------------------------------------------------------
   template<class T, class D>
      D square_clamp( T x, T x0, T x1, D y0, D y1 )
   {
      if (x <= x0)
         return y0;
      if (x >= x1)
         return y1;
      T const k = (x - x0) / (x1 - x0);
      return y0 + (k * k) * (y1 - y0);
   }


   // ----------------------------------------------------------------------------
   //      x0 x1  
   // y1    |  |____
   // y0____|_/|
   //
   // ----------------------------------------------------------------------------
   template<class T, class D>
      D pow_clamp( T x, T x0, T x1, D y0, D y1, T power )
   {
      if (x <= x0)
         return y0;
      if (x >= x1)
         return y1;
      T const k = (x - x0) / (x1 - x0);
      return y0 + pow(k, power) * (y1 - y0);
   }


   // ----------------------------------------------------------------------------
   //
   //      x0 x1 x2 x3
   // y1    | |___| |
   // y0____|/|   |\|____
   //
   // ----------------------------------------------------------------------------
   template<class T, class D = T>
   struct TrapezoidLinearClamp
   {
      TrapezoidLinearClamp( T x0, T x1, T x2, T x3, D y0, D y1 ) :
         x0(x0), x1(x1), x2(x2), x3(x3), y0(y0), y1(y1), 
         k0((y1 - y0) / (x1 - x0)), b0(y0 - k0 * x0),
         k1((y0 - y1) / (x3 - x2)), b1(y1 - k1 * x2)
      {
      }

      D operator () ( T x ) const
      {
         if (x <= x0 || x >= x3)
            return y0;
         if (x >= x1 && x <= x2)
            return y1;
         if (x < x1)
            return k0 * x + b0;
         return k1 * x + b1;
      }

      T x0, x1, x2, x3;
      D y0, y1, k0, b0, k1, b1;
   };

   template<class T, class D>
   D trapezoid_linear_clamp( T x, T x0, T x1, T x2, T x3, D y0, D y1 )
   {
      if (x < x0 || x > x3)
         return y0;
      if (x >= x1 && x <= x2)
         return y1;
      if (x < x1)
         return (y1 - y0) * ((x - x0) / (x1 - x0)) + y0;
      return (y0 - y1) * ((x - x2) / (x3 - x2)) + y1;
   }


   // ----------------------------------------------------------------------------
   //      x0 x1 x2 x3
   // y2    | |   | |____
   // y1    | |___|/|
   // y0____|/|   | |
   //
   // ----------------------------------------------------------------------------

   template<class T, class D>
   D trapezoid_linear_clamp( T x, T x0, T x1, T x2, T x3, D y0, D y1, D y2 )
   {
      if (x <= x0)
         return y0;
      if (x >= x3)
         return y2;
      if (x >= x1 && x <= x2)
         return y1;
      if (x < x1)
         return (y1 - y0) * ((x - x0) / (x1 - x0)) + y0;
      return (y2 - y1) * ((x - x2) / (x3 - x2)) + y1;
   }


   // ----------------------------------------------------------------------------
   // exponential interpolate two arguments
   // ----------------------------------------------------------------------------
   template<class T, class D> 
   D interpolate_exp( T x, T x0, T x1, D y0, D y1 )
   {
      return y0 * exp((log(y1) - log(y0)) * ((x - x0) / (x1 - x0)));
   }


   // ----------------------------------------------------------------------------
   // exponential interpolate three arguments
   // ----------------------------------------------------------------------------
   template<class T, class D> 
   D interpolate_exp( T x, T x0, T x1, T x2, D y0, D y1, D y2 )
   {
      D const d10 = log(y1 / y0), d20 = log(y2 / y0);
      D const t1 = (d20 * x1 - d10 * x2) / (x1 * x2 * (x2 - x1));
      D const t2 = d10 / x1 - t1 * x1;
      return y0 * exp(t1 * x * x + t2 * x);
   }


   // ----------------------------------------------------------------------------
   // Clamps to 0..1
   // ----------------------------------------------------------------------------
   template<class T> 
   inline T clamp01( T x )
   {
      return (x < 0) ? 0 : (x > 1) ? 1 : x;
   }


   // ----------------------------------------------------------------------------
   // Clamps to x0..x1
   // ----------------------------------------------------------------------------
   template<class T> 
   inline T clamp( T x, T x0, T x1 )
   {
      return (x < x0) ? x0 : (x > x1) ? x1 : x;
   }

   // ----------------------------------------------------------------------------
   // Clamps color components to 0..1
   // ----------------------------------------------------------------------------
   template<class T> 
   inline color_t<T> clamp01( color_t<T> const& c )
   {
      return color_t<T>(clamp01(c.r), clamp01(c.g), clamp01(c.b));
   }

   // ----------------------------------------------------------------------------
   // Clamps color components to x0..x1
   // ----------------------------------------------------------------------------
   template<class T> 
   inline color_t<T> clamp( color_t<T> const& c, T x0, T x1 )
   {
      return color_t<T>(clamp(c.r, x0, x1), clamp(c.g, x0, x1), clamp(c.b, x0, x1));
   }
}
