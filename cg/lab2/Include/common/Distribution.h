#ifndef _DISTRIBUTION_CLASS
#define _DISTRIBUTION_CLASS

#include "randgen.h"


class RANDOM_NUMBER
{
public:
   RANDOM_NUMBER() : m_rg() {}
   RANDOM_NUMBER(unsigned int seed) 
      : m_rg(seed) {}

   void Init(unsigned int seed) { m_rg.Init(seed); }
   void Init(int seed) { m_rg.Init(seed); }

   void Init(float time) 
   { 
      time = time - floor(time);
      Init( unsigned int( time * float(0xFFFF) ) );
   }
   void Init(double time) { Init(float(time)); }

   //  returns int in range [0...max)
   int Int(int max) { return m_rg.RandInt() % max; }
   int Int() { return m_rg.RandInt(); }

   // range [0;1]
   float Float() {	return m_rg.RandFloat(); }
   // [-0.5, 0.5]
   float FloatSgn() { return Float() - 0.5f; }
   float FloatSgn(float x) { return FloatSgn() * x; }

   void Floats(float v[], int len) { m_rg.Generate(v,len); }

   //	Returns random value in range [min-deviation; min + deviation]
   float FloatFromDelta( float min, float deviation )
   {
      float x = Float();
      return min + (x + x - 1) * deviation;	// x + x - 1 := scale from [0;1] to [-1; 1]
   }

   //	Returns random value in range [min, max]
   float FloatFromRange ( float min, float max )
   {
      return min + Float() * (max-min);
   }

   //	Returns normal-distributed random float (in range [0;1])
   float GaussFloat()
   {
      static int available = false;
      static float ynext;

      if (!available)
      {
         float x1, x2, w, y1, y2;

         do {
            float r1 = Float();
            x1 = r1 + r1 - 1.0f;
            r1 = Float();
            x2 = r1 + r1 - 1.0f;
            w = x1 * x1 + x2 * x2;
         } while ( w >= 1.0f );

         w = sqrtf( (-2.0f * log( w ) ) / w );
         y1 = x1 * w;
         y2 = x2 * w;

         available = true;
         ynext = y2;
         return y1;
      }
      else
      {
         available = false;
         return ynext;
      }
   }

   //	Random float with normal distrib
   __declspec(deprecated) float NormFloat ( float fSigma = 1.0 )
   {
      static const float ONE_OVER_SIGMA_EXP = 1.0f / 0.7975f;

      if ( fSigma == 0 ) return 0;

      float Unit;

      do
      {
         Unit = -logf( Float());
      }
      while ( Float() > expf( -sqrtf( Unit - 1.0f ) * 0.5f ) );

      if ( m_rg.RandInt() & 0x1 )
      {
         return Unit * fSigma * ONE_OVER_SIGMA_EXP;
      }
      else
      {
         return -Unit * fSigma * ONE_OVER_SIGMA_EXP;
      }

   }

   //	С вероятностью 'prob' ([0;1]) случайное значение будет "сжато" к нулю (x^4)
   float NonUniformFloat(float prob)
   {
      float x = Float();
      if ( Float() < prob )
      {
         x *= x;
         return x*x;
      }
      return x;
   }

   float NonUniformFloatFromRange(
      float min, float max, // range of random values
      float prob)
   {
      return min + NonUniformFloat(prob) * (max-min);
   }

private:

   CRandGen	m_rg;
};





// Returns random normalized plane direction
// (point on identity circle)
// ------------------------------------------
inline void RandomDirection2D(float d[2], RANDOM_NUMBER& rnd)
{
   //	random normalized direction
   d[0] = rnd.Float() * 2 - 1;
   d[1] = rnd.Float() * 2 - 1;
   float l = 1.0f/sqrtf(d[0]*d[0] + d[1]*d[1]);
   d[0] *= l;	
   d[1] *= l;
}



// Returns random normalized space direction
// (point on identity sphere)
// ------------------------------------------

inline void RandomDirection3D(float d[3], RANDOM_NUMBER& rnd)
{
   //	random normalized direction
   d[0] = rnd.Float() * 2 - 1;
   d[1] = rnd.Float() * 2 - 1;
   d[2] = rnd.Float() * 2 - 1;
   float l = 1.0f/sqrtf(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
   d[0] *= l;	
   d[1] *= l;
   d[2] *= l;
}




#endif 
