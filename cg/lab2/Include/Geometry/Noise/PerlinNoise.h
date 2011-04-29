// perlinNoise.h
// Class to implement coherent noise over 1, 2, or 3 dimensions.
// Implementation based on the Perlin noise function. Thanks to 
// Ken Perlin of NYU for publishing his algorithm online.
/////////////////////////////////////////////////////////////////////
// Copyright (c) 2001, Matt Zucker
// You may use this source code as you wish, but it is provided
// with no warranty. Please email me at mazucker@vassar.edu if 
// you find it useful.
/////////////////////////////////////////////////////////////////////

#ifndef _PERLIN_NOISE_CLASS_H_
#define _PERLIN_NOISE_CLASS_H_

#if !defined(_INC_STDLIB)
#error "You must include <stdlib.h> before this file"
#endif
#if !defined(_INC_TIME)
#error "You must include <time.h> before this file"
#endif
#if !defined(_INC_MATH)
#error "You must include <math.h> before this file"
#endif


// It must be true that (x % NOISE_WRAP_INDEX) == (x & NOISE_MOD_MASK)
// so NOISE_WRAP_INDEX must be a power of two, and NOISE_MOD_MASK must be
// that power of 2 - 1.  as indices are implemented, as unsigned chars,
// NOISE_WRAP_INDEX shoud be less than or equal to 256.
// There's no good reason to change it from 256, really.

#define NOISE_WRAP_INDEX    256
//65536
#define NOISE_MOD_MASK      255
//65535

// A large power of 2, we'll go for 4096, to add to negative numbers
// in order to make them positive

const int NOISE_LARGE_PWR2    = 4096;//65536 * 2;

// TODO: change these preprocessor macros into inline functions

// S curve is (3x^2 - 2x^3) because it's quick to calculate
// though -cos(x * PI) * 0.5 + 0.5 would work too

#define easeCurve(t)            ( t * t * (3.0 - 2.0 * t) )
#define linearInterp(t, a, b)   ( a + t * (b - a) )
#define dot2(rx, ry, q)         ( rx * q[0] + ry * q[1] )
#define dot3(rx, ry, rz, q)     ( rx * q[0] + ry * q[1] + rz * q[2] )

#define setupValues(t, axis, g0, g1, d0, d1, pos) \
                    t = pos[axis] + NOISE_LARGE_PWR2; \
                    g0 = ((int)t) & NOISE_MOD_MASK; \
                    g1 = (g0 + 1) & NOISE_MOD_MASK; \
                    d0 = t - (int)t; \
                    d1 = d0 - 1.0; \

namespace cg
{

  class PerlinNoise 
  {
    public:
        PerlinNoise()
            : initialized ( 0 )
        {}

    private:
      mutable unsigned initialized;
  
      mutable unsigned  permutationTable[NOISE_WRAP_INDEX*2 + 2];    // permutation table
      mutable double    gradientTable1d[NOISE_WRAP_INDEX*2 + 2];     // 1d gradient lookup table.
      mutable double    gradientTable2d[NOISE_WRAP_INDEX*2 + 2][2];  // 2d gradient lookup table.
      mutable double    gradientTable3d[NOISE_WRAP_INDEX*2 + 2][3];  // 3d gradient lookup table.
    
      double randNoisedouble() const             // generate a random double in [-1,1]
      {
        return (double)((::rand() % (NOISE_WRAP_INDEX + NOISE_WRAP_INDEX)) - 
                        NOISE_WRAP_INDEX) / NOISE_WRAP_INDEX;
      }
    
      void normalize2d(double vector[2]) const  // normalize a 2d vector
      {
        double length = cg::sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]));
        vector[0] /= length;
        vector[1] /= length;
      }
    
      void normalize3d(double vector[3]) const  // normalize a 3d vector
      {
        double length = cg::sqrt((vector[0] * vector[0]) + 
                             (vector[1] * vector[1]) +
                             (vector[2] * vector[2]));
        vector[0] /= length;
        vector[1] /= length;
        vector[2] /= length;
      }

      void generateLookupTables() const         // fill in table entries
      {
        unsigned i, j, temp;

        for (i=0; i<NOISE_WRAP_INDEX; i++) {
          // put index into permutationTable[index], we will shuffle later
          permutationTable[i] = i;

          gradientTable1d[i] = randNoisedouble();  

		  double a = pi * randNoisedouble () ;
		  gradientTable2d[i][0] = sin ( a );
		  gradientTable2d[i][1] = cos ( a );/**/
          /*for (j=0; j<2; j++) { gradientTable2d[i][j] = randNoisedouble(); }
          normalize2d(gradientTable2d[i]);/**/

          for (j=0; j<3; j++) { gradientTable3d[i][j] = randNoisedouble(); }
          normalize3d(gradientTable3d[i]);
        }

        // Shuffle permutation table up to NOISE_WRAP_INDEX
        for (i=0; i<NOISE_WRAP_INDEX; i++) {
          j = ::rand() & NOISE_MOD_MASK;
          temp = permutationTable[i];
          permutationTable[i] = permutationTable[j];
          permutationTable[j] = temp;
        }

        // Add the rest of the table entries in, duplicating 
        // indices and entries so that they can effectively be indexed
        // by unsigned chars.  I think.  Ask Perlin what this is really doing.
        //
        // This is the only part of the algorithm that I don't understand 100%.
  
        for (i=0; i<NOISE_WRAP_INDEX+2; i++) {
          permutationTable[NOISE_WRAP_INDEX + i] = permutationTable[i];

          gradientTable1d[NOISE_WRAP_INDEX + i] = gradientTable1d[i];

          for (j=0; j<2; j++) {
            gradientTable2d[NOISE_WRAP_INDEX + i][j] = gradientTable2d[i][j]; 
          }

          for (j=0; j<3; j++) {
            gradientTable3d[NOISE_WRAP_INDEX + i][j] = gradientTable3d[i][j]; 
          }
        }
    
        // And we're done. Set initialized to true
        initialized = 1;
      }
    
      void generateLookupTables_adv(double density_param) const
      {
        unsigned i, j, temp;

        for (i=0; i<NOISE_WRAP_INDEX; i++) {
          // put index into permutationTable[index], we will shuffle later
          permutationTable[i] = i;

          gradientTable1d[i] = randNoisedouble();  

          double a = pi * randNoisedouble () ;
          gradientTable2d[i][0] = sin ( a );
          gradientTable2d[i][1] = cos ( a );/**/
          /*for (j=0; j<2; j++) { gradientTable2d[i][j] = randNoisedouble(); }
          normalize2d(gradientTable2d[i]);/**/

          if (cg::rand(1.) < density_param)
          {
            for (j=0; j<3; j++) { gradientTable3d[i][j] = 0; }
          }
          else
          {
            for (j=0; j<3; j++) { gradientTable3d[i][j] = randNoisedouble(); }
            normalize3d(gradientTable3d[i]);
          }
        }

        // Shuffle permutation table up to NOISE_WRAP_INDEX
        for (i=0; i<NOISE_WRAP_INDEX; i++) {
          j = ::rand() & NOISE_MOD_MASK;
          temp = permutationTable[i];
          permutationTable[i] = permutationTable[j];
          permutationTable[j] = temp;
        }

        // Add the rest of the table entries in, duplicating 
        // indices and entries so that they can effectively be indexed
        // by unsigned chars.  I think.  Ask Perlin what this is really doing.
        //
        // This is the only part of the algorithm that I don't understand 100%.

        for (i=0; i<NOISE_WRAP_INDEX+2; i++) {
          permutationTable[NOISE_WRAP_INDEX + i] = permutationTable[i];

          gradientTable1d[NOISE_WRAP_INDEX + i] = gradientTable1d[i];

          for (j=0; j<2; j++) {
            gradientTable2d[NOISE_WRAP_INDEX + i][j] = gradientTable2d[i][j]; 
          }

          for (j=0; j<3; j++) {
            gradientTable3d[NOISE_WRAP_INDEX + i][j] = gradientTable3d[i][j]; 
          }
        }

        // And we're done. Set initialized to true
        initialized = 1;
      }

    public:
      void reseed() const          // reseed random generator & regenerate tables
      {
        ::srand((unsigned int) (::time(NULL) + ::rand()));
        generateLookupTables();
      }

      void reseed(unsigned int rSeed) const // same, but with specified seed
      {
        ::srand(rSeed);
        generateLookupTables();
      }

      void reseed_adv(unsigned int rSeed, double density_param) const
      {
        ::srand(rSeed);
        generateLookupTables_adv(density_param);
      }

      /////////////////////////////////////////////////////////////////////
      //
      // Mnemonics used in the following 3 functions:
      //   L = left       (-X direction)
      //   R = right      (+X direction)
      //   D = down       (-Y direction)
      //   U = up         (+Y direction)
      //   B = backwards  (-Z direction)
      //   F = forwards       (+Z direction)
      //
      // Not that it matters to the math, but a reader might want to know.
      //
      // noise1d - create 1-dimensional coherent noise
      //   if you want to learn about how noise works, look at this
      //   and then look at noise2d.
      double   noise1d(double pos[1]) const
      {
        int   gridPointL, gridPointR;
        double distFromL, distFromR, sX, t, u, v;

        if (! initialized) { reseed(); }  
  
        // find out neighboring grid points to pos and signed distances from pos to them.
        setupValues(t, 0, gridPointL, gridPointR, distFromL, distFromR, pos);

        sX = easeCurve(distFromL);

        // u, v, are the vectors from the grid pts. times the random gradients for the grid points
        // they are actually dot products, but this looks like scalar multiplication
        u = distFromL * gradientTable1d[ permutationTable[ gridPointL ] ];
        v = distFromR * gradientTable1d[ permutationTable[ gridPointR ] ];

        // return the linear interpretation between u and v (0 = u, 1 = v) at sX.
        return linearInterp(sX, u, v);
      }
    
      double   noise2d(double pos[2]) const    // 2D call using an array for passing pos
      {
        int gridPointL, gridPointR, gridPointD, gridPointU;
        int indexLD, indexRD, indexLU, indexRU;
        double distFromL, distFromR, distFromD, distFromU;
        double *q, sX, sY, a, b, t, u, v;
        register int indexL, indexR;

        if (! initialized) { reseed(); }  

        // find out neighboring grid points to pos and signed distances from pos to them.
        setupValues(t, 0, gridPointL, gridPointR, distFromL, distFromR, pos);
        setupValues(t, 1, gridPointD, gridPointU, distFromD, distFromU, pos);

        // Generate some temporary indexes associated with the left and right grid values
        indexL = permutationTable[ gridPointL ];
        indexR = permutationTable[ gridPointR ];

        // Generate indexes in the permutation table for all 4 corners
        indexLD = permutationTable[ indexL + gridPointD ];
        indexRD = permutationTable[ indexR + gridPointD ];
        indexLU = permutationTable[ indexL + gridPointU ];
        indexRU = permutationTable[ indexR + gridPointU ];

        // Get the s curves at the proper values
        sX = easeCurve(distFromL);
        sY = easeCurve(distFromD);

        // Do the dot products for the lower left corner and lower right corners.
        // Interpolate between those dot products value sX to get a.
        q = gradientTable2d[indexLD]; u = dot2(distFromL, distFromD, q);
        q = gradientTable2d[indexRD]; v = dot2(distFromR, distFromD, q);
        a = linearInterp(sX, u, v);

        // Do the dot products for the upper left corner and upper right corners.
        // Interpolate between those dot products at value sX to get b.
        q = gradientTable2d[indexLU]; u = dot2(distFromL, distFromU, q);
        q = gradientTable2d[indexRU]; v = dot2(distFromR, distFromU, q);
        b = linearInterp(sX, u, v);

        // Interpolate between a and b at value sY to get the noise return value.
        return linearInterp(sY, a, b);
      }

      //
      // noise2d_blend added by Peter Azmanov
      //
      /*

      // Noise2d with blending of node gradient vectors
      double noise2d_blend(double pos[2], double blend_p[4][2], double alpha) const
      {
          int gridPointL, gridPointR, gridPointD, gridPointU;
          int indexLD, indexRD, indexLU, indexRU;
          double distFromL, distFromR, distFromD, distFromU;
          double *q, sX, sY, a, b, t, u, v;
          register int indexL, indexR;

          if (! initialized) { reseed(); }  

          // find out neighboring grid points to pos and signed distances from pos to them.
          setupValues(t, 0, gridPointL, gridPointR, distFromL, distFromR, pos);
          setupValues(t, 1, gridPointD, gridPointU, distFromD, distFromU, pos);

          // Generate some temporary indexes associated with the left and right grid values
          indexL = permutationTable[ gridPointL ];
          indexR = permutationTable[ gridPointR ];

          // Generate indexes in the permutation table for all 4 corners
          indexLD = permutationTable[ indexL + gridPointD ];
          indexRD = permutationTable[ indexR + gridPointD ];
          indexLU = permutationTable[ indexL + gridPointU ];
          indexRU = permutationTable[ indexR + gridPointU ];

          // Get the s curves at the proper values
          sX = easeCurve(distFromL);
          sY = easeCurve(distFromD);

          // Do the dot products for the lower left corner and lower right corners.
          // Interpolate between those dot products value sX to get a.
          q = gradientTable2d[indexLD];
          double vb[2] = { q[0], q[1] };
          blendVectors(vb, blend_p[0], alpha);
          u = dot2(distFromL, distFromD, vb);
          q = gradientTable2d[indexRD];
          vb[0] = q[0], vb[1] = q[1];
          blendVectors(vb, blend_p[1], alpha);
          v = dot2(distFromR, distFromD, vb);
          a = linearInterp(sX, u, v);

          // Do the dot products for the upper left corner and upper right corners.
          // Interpolate between those dot products at value sX to get b.
          q = gradientTable2d[indexLU];
          vb[0] = q[0], vb[1] = q[1];
          blendVectors(vb, blend_p[3], alpha);
          u = dot2(distFromL, distFromU, vb);
          q = gradientTable2d[indexRU];
          vb[0] = q[0], vb[1] = q[1];
          blendVectors(vb, blend_p[2], alpha);
          v = dot2(distFromR, distFromU, vb);
          b = linearInterp(sX, u, v);

          // Interpolate between a and b at value sY to get the noise return value.
          return linearInterp(sY, a, b);
      }
      */

      double noise3d(double pos[3]) const    // 3D call using an array for passing pos
      {
        int gridPointL, gridPointR, gridPointD, gridPointU, gridPointB, gridPointF;
        int indexLD, indexLU, indexRD, indexRU;
        double distFromL, distFromR, distFromD, distFromU, distFromB, distFromF;
        double *q, sX, sY, sZ, a, b, c, d, t, u, v;
        register int indexL, indexR;

        if (! initialized) { reseed(); }  

        // find out neighboring grid points to pos and signed distances from pos to them.
        setupValues(t, 0, gridPointL, gridPointR, distFromL, distFromR, pos);
        setupValues(t, 1, gridPointD, gridPointU, distFromD, distFromU, pos);
        setupValues(t, 2, gridPointB, gridPointF, distFromB, distFromF, pos);
  
        indexL = permutationTable[ gridPointL ];
        indexR = permutationTable[ gridPointR ];

        indexLD = permutationTable[ indexL + gridPointD ];
        indexRD = permutationTable[ indexR + gridPointD ];
        indexLU = permutationTable[ indexL + gridPointU ];
        indexRU = permutationTable[ indexR + gridPointU ];

        sX = easeCurve(distFromL);
        sY = easeCurve(distFromD);
        sZ = easeCurve(distFromB);

        q = gradientTable3d[indexLD+gridPointB]; u = dot3(distFromL, distFromD, distFromB, q);
        q = gradientTable3d[indexRD+gridPointB]; v = dot3(distFromR, distFromD, distFromB, q);
        a = linearInterp(sX, u, v);

        q = gradientTable3d[indexLU+gridPointB]; u = dot3(distFromL, distFromU, distFromB, q);
        q = gradientTable3d[indexRU+gridPointB]; v = dot3(distFromR, distFromU, distFromB, q);
        b = linearInterp(sX, u, v);

        c = linearInterp(sY, a, b);

        q = gradientTable3d[indexLD+gridPointF]; u = dot3(distFromL, distFromD, distFromF, q);
        q = gradientTable3d[indexRD+gridPointF]; v = dot3(distFromR, distFromD, distFromF, q);
        a = linearInterp(sX, u, v);

        q = gradientTable3d[indexLU+gridPointF]; u = dot3(distFromL, distFromU, distFromF, q);
        q = gradientTable3d[indexRU+gridPointF]; v = dot3(distFromR, distFromU, distFromF, q);
        b = linearInterp(sX, u, v);

        d = linearInterp(sY, a, b);

        return .75 * linearInterp(sZ, c, d);
      }

      double noise(double x) const
      {
        return noise1d(&x);
      }

      double noise(double x, double y) const
      {
        double p[2] = { x, y };
        return noise2d(p); 
      }

      double noise(double x, double y, double z) const
      {
        double p[3] = { x, y, z };
        return noise3d(p);
      }
  };
}

#endif //_PERLIN_NOISE_CLASS_H_