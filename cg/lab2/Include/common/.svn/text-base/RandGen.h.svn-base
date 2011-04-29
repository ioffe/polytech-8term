#pragma once
// needs windows.h

#define INV_RAND_MAX (1.0f / float(0x7fff))

class CRandGen
{
public:
   //   create uninitialized generator
   CRandGen() : m_seed(1L) {}    

   //   create & init generator by the given seed
   CRandGen( unsigned int seed ) : m_seed(seed) {}   

   //   re-init generator with a new seed
   void Init( unsigned int seed )  { m_seed = seed; }

   //   generate sequence of floats - each float in range [0;1]
   void Generate( float rvec[], int len );

   //   generate sequence of ints - each int in range [0;32767] 
   void Generate( int rvec[], int len );

   //   generate one random float value ( in range [0;1] )
   float RandFloat() { return float(RandInt()) * INV_RAND_MAX; }

   //   generate one random int value (in range [0;32767])
   int RandInt();

   //   generate one random int value (in range [0; max - 1])
   int RandInt( unsigned short max );

   //   generate one random short int value (in range [lo; hi - 1])
   int RandInt( int lo, int hi )
   {
      Assert(lo < hi);
      int const range = (hi - lo);
      // Note: This is not uniform distribution.
      return lo + (RandInt() % range) ;
   }

   //   generate one random float value (in range [lo; hi])
   double operator()( double lo, double hi )
   {
      return lo + RandFloat()*(hi - lo) ;
   }

private:
   long m_seed;
};



// ----------------------------------------------------------------------------
inline    int CRandGen::RandInt()
{
   return(((m_seed = m_seed * 214013L + 2531011L) >> 16) & 0x7fff);
}

// ----------------------------------------------------------------------------
inline    int CRandGen::RandInt( unsigned short max )
{
   // Note: This is not uniform distribution.
   if (max != 0)
      return RandInt() % max;
   else
      return 0;
}

// ----------------------------------------------------------------------------
inline void CRandGen::Generate(float rvec[], int len)
{
   for ( int i = 0; i < len; i++ )
      rvec[i] = RandFloat();
}

// ----------------------------------------------------------------------------
inline void CRandGen::Generate(int rvec[], int len)
{
   for ( int i = 0; i < len; i++ )
      rvec[i] = RandInt();
}

// provides 32-bits pseudo-random generator ( method by D. Knuth & H.W. Lewis )
namespace LongRand
{
   const unsigned mul = 1664525L    ;
   const unsigned inc = 1013904223L ;
   const unsigned max = 0xffffffff  ;

   //--------------------------------------- CLongRandGen
   // generates 32-bits random value
   // warning: no multithread protection
   class CLongRandGen 
   {
   public:
      CLongRandGen      ( unsigned seed = 1 )
         : m_next ( seed )
      {}

      void     srand    ( unsigned seed )    { m_next = seed ; }

      void     randomize()                   
      { 
         LARGE_INTEGER li ;
         QueryPerformanceCounter( &li ) ; 
         m_next = li.LowPart ;
      } 

      unsigned rand     ()                   { return  m_next = mul * m_next + inc ; }

   private:
      unsigned m_next ;
   };

   //--------------------------------------- random functions for 32-bits usage
   // warning: no multithread protection

   __declspec ( selectany ) unsigned holdrand = 1 ;

   inline void     srand      ( unsigned seed ) { holdrand = seed ; }
   inline void     randomize  ()                   
   { 
      LARGE_INTEGER li ;
      QueryPerformanceCounter( &li ) ; 
      holdrand = li.LowPart ;
   } 
   inline unsigned rand  ()                { return holdrand = mul * holdrand + inc ; }
}

