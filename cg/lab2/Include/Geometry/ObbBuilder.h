#pragma once

#include "common\points.h"

namespace cg
{
   template < class Scalar, int N >
      __forceinline bool eq_zero( cg::matrix_t< Scalar, N > const & m, NO_DEDUCE(Scalar) eps = epsilon< Scalar >( ) )
   {
      for (size_t i = 0; i < N; i++)
         if (!cg::eq_zero(m.get_row(i), eps))
            return false;

      return true;
   }

   // Get characteristics polynomial function
   static void GetCharacteristicPolynom( const cg::matrix_3 &m, double coeffs[4] )
   {
      coeffs[3] = -1;
      coeffs[2] = m(0, 0) + m(1, 1) + m(2, 2);
      coeffs[1] = -m(0, 0) * (m(1, 1) + m(2, 2)) - m(1, 1) * m(2, 2) + m(2, 1) * m(1, 2) + m(0, 1) * m(1, 0) +
         m(0, 2) * m(2, 0);
      coeffs[0] = m(0, 0) * m(1, 1) * m(2, 2) - m(0, 0) * m(2, 1) * m(1, 2) -
         m(1, 0) * m(0, 1) * m(2, 2) + m(0, 1) * m(2, 0) * m(1, 2) +
         m(0, 2) * m(1, 0) * m(2, 1) - m(0, 2) * m(2, 0) * m(1, 1);
   }

   // Solve 3 degree equation
   static bool Solve3degPolynom( const double coeffs[4], double roots[3] )
   {
      // Polynomial coeffs
      double a = coeffs[2] / coeffs[3], b = coeffs[1] / coeffs[3], c = coeffs[0] / coeffs[3];

      double p = -a * a / 3.0 + b;
      double q = 2.0 * a * a * a / 27.0 - a * b / 3.0 + c;

      // Discriminant
      double D = -4.0 * p * p * p - 27.0 * q * q;

      // Applied substitutions
      double p1 = p / 3.0;
      double q1 = q * 0.5;
      double D1 = D / 108.0;

      // Make sure that all solutions are real
      if (D1 <= 0.0)
         D1 = 0.0;

      // Calculate this values

      // :(
      if (cg::eq_zero(D * 1e3/*, 0.001*/))
      {
         double r; 

         if (q1 > 0)
            r = -pow(q1, 1.0 / 3.0);
         else
            r = pow(-q1, 1.0 / 3.0);

         roots[0] = 2.0 * r - a / 3.0;
         roots[1] = roots[2] = -r - a / 3.0; 
         return true;
      }
      else
      {
         double m = sqrt(-p / 3.0);
         double th = acos(-q1 / sqrt(-p1 * p1 * p1)) / 3.0;

         roots[0] = 2.0 * m * cos(th) - a / 3.0;
         roots[1] = 2.0 * sqrt(-p1) * cos(th + 2 * cg::pi / 3.0) - a / 3.0;
         roots[2] = 2.0 * sqrt(-p1) * cos(th - 2 * cg::pi / 3.0) - a / 3.0;
         return false;
      }
   }

   // Swap matrix rows function
   template <int N> inline void SwapRows( cg::matrix_t<double, N> &m, int i1, int i2 )
   {
      point_t<double, N> p1, p2, pTemp;

      pTemp = p1 = m.get_row(i1);
      p2 = m.get_row(i2);
      p1 = p2;
      p2 = pTemp;

      m.put_row(i1, p1);
      m.put_row(i2, p2);
   }

   // Advanced uniform 3 degree linear systems solver
   static void SolveUniformLinearSystem2( const cg::matrix_3 &m, cg::point_3 &res )
   {
      cg::matrix_3 mc = m;

      // Search non zero row
      int nzr = -1;

      for (int i = 0; i < 3; i++)
         if (!cg::eq_zero(mc.get_row(i), 0.001))
         {
            nzr = i;
            break;
         }

         if (nzr == -1)
         {
            // Random eigenvector was found
            res = point_3f(30.0f, 30.0f, 30.0f);
            return;
         }

         // Place non zero to be first
         SwapRows(mc, 0, nzr);
         // Place non zero row element to be at first column
         int replaceVec[3] = {0, 1, 2};
         for (int i = 0; i < 3; i++)
            if (!cg::eq_zero(mc(0, i), 0.001))
            {
               std::swap(mc(0, 0), mc(0, i));
               std::swap(mc(1, 0), mc(1, i));
               std::swap(mc(2, 0), mc(2, i));
               replaceVec[0] = i;
               replaceVec[i] = 0;
               break;
            }

            // Perform Gaussian step
            for (int i = 1; i < 3; i++)
            {
               double mul = mc(i, 0) / mc(0, 0);
               for (int j = 0; j < 3; j++)
                  mc(i, j) -= mc(0, j) * mul;
            }

            // Analyze 2x2 lower right minor
            cg::matrix_2 ms;
            ms(0, 0) = mc(1, 1);
            ms(0, 1) = mc(1, 2);
            ms(1, 0) = mc(2, 1);
            ms(1, 1) = mc(2, 2);

            cg::point_3 result;

            if (cg::eq_zero(ms.get_row(0), 0.001) && cg::eq_zero(ms.get_row(1), 0.001))
            {
               result[1] = 30.0;
               result[2] = 30.0;
               result[0] = -(result[1] * mc(0, 1) + result[2] * mc(0, 2)) / mc(0, 0);

               for (int i = 0; i < 3; i++)
                  res[i] = result[replaceVec[i]];
               return;
            }

            if (cg::eq_zero(ms.get_row(0), 0.001))
            {
               SwapRows(ms, 0, 1);
               SwapRows(mc, 1, 2);
            }

            if (!cg::eq_zero(ms(0, 0), 0.001))
            {
               double mul = ms(1, 0) / ms(0, 0);
               ms(1, 0) = 0.0;
               ms(1, 1) -= ms(0, 1) * mul;
               if (cg::eq_zero(ms(1, 1), 0.001))
                  result[2] = 30.0;
               else
                  result[2] = 0.0;
               result[1] = - ms(0, 1) * result[2] / ms(0, 0);
               result[0] = -(result[1] * mc(0, 1) + result[2] * mc(0, 2)) / mc(0, 0);
            }
            else
            {
               double mul = ms(1, 1) / ms(0, 1);
               ms(1, 1) = 0.0f;
               ms(1, 0) -= ms(1, 0) * mul;
               if (cg::eq_zero(ms(1, 0), 0.001))
                  result[1] = 30.0;
               else
                  result[1] = 0.0;
               result[2] = - ms(0, 0) * result[1] / ms(0, 1);
               result[0] = -(result[1] * mc(0, 1) + result[2] * mc(0, 2)) / mc(0, 0);
            }

            for (int i = 0; i < 3; i++)
               res[i] = result[replaceVec[i]];
   }

   // Solve eigenvalues and eigenvectors system
   static void CalculateEigenSystem( const cg::matrix_3 &m, double vals[3], cg::point_3 vecs[3] )
   {
      // Eigenvectors matrix
      cg::matrix_3 r;
      static const double EPS = 1e-5;
      static const int NUM_ITER = 32;
      double m11 = m(0, 0);
      double m12 = m(0, 1);
      double m13 = m(0, 2);
      double m22 = m(1, 1);
      double m23 = m(1, 2);
      double m33 = m(2, 2);

      r = cg::matrix_3(1.0);
      for (size_t a = 0; a < NUM_ITER; a++)
      {
         // Check for criteria match
         if (cg::eq_zero(abs(m11) + abs(m22) + abs(m33), EPS))
            break;

         // (1, 2)
         if (!cg::eq_zero(m12, EPS))
         {
            double u = (m22 - m11) * 0.5 / m12;
            double u2 = u * u;
            double u2p1 = u2 + 1.0;
            double t = (u2p1 != u2) ? 
               sign(u) * (sqrt(u2p1) - abs(u))
               : 0.5 / u;

            double c = 1.0 / sqrt(t * t + 1.0);
            double s = c * t;

            m11 -= t * m12;
            m22 += t * m12;
            m12 = 0.0;

            double temp = c * m13 - s * m23;
            m23 = s * m13 + c * m23;
            m13 = temp;

            for (size_t i = 0; i < 3; i++)
            {
               double temp = c * r(i, 0) - s * r(i, 1);
               r(i, 1) = s * r(i, 0) + c * r(i, 1);
               r(i, 0) = temp;
            }
         }

         // (1, 3)
         if (!cg::eq_zero(m13, EPS))
         {
            double u = (m33 - m11) * 0.5 / m13;
            double u2 = u * u;
            double u2p1 = u2 + 1.0;
            double t = (u2p1 != u2) ? 
               sign(u) * (sqrt(u2p1) - abs(u))
               : 0.5 / u;

            double c = 1.0 / sqrt(t * t + 1.0);
            double s = c * t;

            m11 -= t * m13;
            m33 += t * m13;
            m13 = 0.0;

            double temp = c * m12 - s * m23;
            m23 = s * m12 + c * m23;
            m12 = temp;

            for (size_t i = 0; i < 3; i++)
            {
               double temp = c * r(i, 0) - s * r(i, 2);
               r(i, 2) = s * r(i, 0) + c * r(i, 2);
               r(i, 0) = temp;
            }
         }

         // (2, 3)
         if (!cg::eq_zero(m23, EPS))
         {
            double u = (m33 - m22) * 0.5 / m23;
            double u2 = u * u;
            double u2p1 = u2 + 1.0;
            double t = (u2p1 != u2) ? 
               sign(u) * (sqrt(u2p1) - abs(u))
               : 0.5 / u;

            double c = 1.0 / sqrt(t * t + 1.0);
            double s = c * t;

            m22 -= t * m23;
            m33 += t * m23;
            m23 = 0.0;

            double temp = c * m12 - s * m23;
            m13 = s * m12 + c * m13;
            m12 = temp;

            for (size_t i = 0; i < 3; i++)
            {
               double temp = c * r(i, 1) - s * r(i, 2);
               r(i, 2) = s * r(i, 1) + c * r(i, 2);
               r(i, 1) = temp;
            }
         }
      }

      // Write eigenvalues and eigenvectors
      for (size_t i = 0; i < 3; i++)
         vecs[i] = r.get_col(i);

      vals[0] = m11;
      vals[1] = m22;
      vals[2] = m33;
   }

   // Get matrix eigenvalues and eigenvectors
   static void GetEigenValuesAndVectors( const cg::matrix_3 &m, double vals[3], cg::point_3 vecs[3] )
   {
      // Getting eigenvalue
      double coeffs[4];

      GetCharacteristicPolynom(m, coeffs);
      bool equalRoots = Solve3degPolynom(coeffs, vals);

      // Identity matrix
      cg::matrix_3 I;
      I(0, 0) = 1.0;
      I(1, 1) = 1.0;
      I(2, 2) = 1.0;
      // Linear system matrix 
      cg::matrix_3 sysM;

      sysM = m;
      sysM(0, 0) =  m(0, 0) - vals[0];
      sysM(1, 1) =  m(1, 1) - vals[0];
      sysM(2, 2) =  m(2, 2) - vals[0];
      if (cg::eq_zero(sysM))
      {
         vecs[0] = point_3f(1.0, 0.0, 0.0);
         vecs[1] = point_3f(0.0, 1.0, 0.0);
         vecs[2] = point_3f(0.0, 0.0, 1.0);
      }
      else
      {
         if (!equalRoots)
         {
            // Calculating eigenvectors and normalizing them
            for (int i = 0; i < 3; i++)
            {
               sysM = m;
               sysM(0, 0) =  m(0, 0) - vals[i];
               sysM(1, 1) =  m(1, 1) - vals[i];
               sysM(2, 2) =  m(2, 2) - vals[i];
               SolveUniformLinearSystem2(sysM, vecs[i]);
               vecs[i] = normalized_safe(vecs[i]);
            }
         }
         else
         {
            // Equal roots found

            // Calculating eigenvectors and normalizing them
            for (int i = 0; i < 2; i++)
            {
               sysM = m;
               sysM(0, 0) =  m(0, 0) - vals[i];
               sysM(1, 1) =  m(1, 1) - vals[i];
               sysM(2, 2) =  m(2, 2) - vals[i];
               SolveUniformLinearSystem2(sysM, vecs[i]);
               vecs[i] = normalized_safe(vecs[i]);
            }
            vecs[2] = normalized_safe(vecs[0] ^ vecs[1]);
         }
      }
   }

   // Transform points to AABB
   static void TransformPointsToAABB( std::vector<cg::point_3> &pts, cg::transform_4 &invTrans )
   {
      // Minimal and maximal coords of points
      cg::rectangle_3 aabbRect;

      // Evaluating extreme values
      for (size_t i = 0; i < pts.size(); i++)
         aabbRect |= pts[i];

      const double dx = aabbRect.hi().x - aabbRect.lo().x,
         dy = aabbRect.hi().y - aabbRect.lo().y,
         dz = aabbRect.hi().z - aabbRect.lo().z;

      const double midExt = cg::min(cg::max(dx, dy), cg::max(dy, dz), cg::max(dx, dz));
      const double midExtInv = 1.0 / midExt;

      cg::matrix_4 mT;
      mT(0, 0) = midExtInv;
      mT(0, 1) = 0.0;
      mT(0, 2) = 0.0;
      mT(0, 3) = -aabbRect.lo().x * midExtInv;
      mT(1, 0) = 0.0;
      mT(1, 1) = midExtInv;
      mT(1, 2) = 0.0;
      mT(1, 3) = -aabbRect.lo().y * midExtInv;
      mT(2, 0) = 0.0;
      mT(2, 1) = 0.0;
      mT(2, 2) = midExtInv;
      mT(2, 3) = -aabbRect.lo().z * midExtInv;
      mT(3, 0) = 0.0;
      mT(3, 1) = 0.0;
      mT(3, 2) = 0.0;
      mT(3, 3) = 1.0;

      // Transformation to AABB coordinate system
      cg::transform_4 curT(mT, cg::ss_equal_scaled);

      // Performing transformation
      for (size_t i = 0; i < pts.size(); i++)
         pts[i] = curT.treat_point(pts[i]);

      // Calculating inverse transformation matrix
      cg::matrix_4 invM;

      invM(0, 0) = midExt;
      invM(0, 1) = 0.0;
      invM(0, 2) = 0.0;
      invM(0, 3) = aabbRect.lo().x;
      invM(1, 0) = 0.0;
      invM(1, 1) = midExt;
      invM(1, 2) = 0.0;
      invM(1, 3) = aabbRect.lo().y;
      invM(2, 0) = 0.0;
      invM(2, 1) = 0.0;
      invM(2, 2) = midExt;
      invM(2, 3) = aabbRect.lo().z;
      invM(3, 0) = 0.0;
      invM(3, 1) = 0.0;
      invM(3, 2) = 0.0;
      invM(3, 3) = 1.0;

      cg::transform_4 inverseTransfromation(invM, cg::ss_equal_scaled);

      invTrans = inverseTransfromation;
   }

   // Build OBB function
   inline cg::obb_clip_data BuildObb( const std::vector<point_3f> &inPoints )
   {
      // Check for correct point number
      if (inPoints.size() <= 3)
         return cg::obb_clip_data();

      std::vector<point_3> copyPoints(inPoints.begin(), inPoints.end());

      // Special CS oriented transform for precision match calculations
      cg::transform_4 invT;
      TransformPointsToAABB(copyPoints, invT);

      // Number of points
      size_t pointsNum = copyPoints.size();
      const float ptsNumInv = 1.0f / pointsNum;

      // Calculate middle point 
      point_3 middleP;
      for (size_t i = 0; i < pointsNum; i++ )
         middleP += copyPoints[i] * ptsNumInv;

      // Calculate covariance matrix
      cg::matrix_3 c;

      // Fill diagonal elements
      for (size_t j = 0; j < 3; j++)
         for (size_t i = 0; i < pointsNum; i++)
            c(j, j) += (copyPoints[i][j] - middleP[j]) * (copyPoints[i][j] - middleP[j]) * ptsNumInv;

      // Fill remaining elements
      for (size_t i = 0; i < pointsNum; i++)
      {
         c(0, 1) = c(1, 0) += (copyPoints[i].x - middleP.x) * (copyPoints[i].y - middleP.y) * ptsNumInv;
         c(0, 2) = c(2, 0) += (copyPoints[i].x - middleP.x) * (copyPoints[i].z - middleP.z) * ptsNumInv;
         c(1, 2) = c(2, 1) += (copyPoints[i].y - middleP.y) * (copyPoints[i].z - middleP.z) * ptsNumInv;
      }

      // Getting eigenvalues and eigenvectors
      double eVals[3];
      point_3 eVecs[3];

      CalculateEigenSystem(c, eVals, eVecs);
      //GetEigenValuesAndVectors(c, eVals, eVecs);

      cg::rectangle_3 dotRect;

      for (size_t j = 0; j < pointsNum; j++)
      {
         point_3 dotVec;
         for (size_t i = 0; i < 3; i++)
            dotVec[i] = copyPoints[j] * eVecs[i];

         dotRect |= dotVec;
      }

      // Half sizes
      double aSize, bSize, cSize;

      aSize = (dotRect.lo()[0] + dotRect.hi()[0]) * 0.5;
      bSize = (dotRect.lo()[1] + dotRect.hi()[1]) * 0.5;
      cSize = (dotRect.lo()[2] + dotRect.hi()[2]) * 0.5;

      cg::point_3f center = aSize * eVecs[0] + bSize * eVecs[1] + cSize * eVecs[2];
      cg::point_3f dir1 = (dotRect.hi()[0] - dotRect.lo()[0]) * 0.5 * eVecs[0]; 
      cg::point_3f dir2 = (dotRect.hi()[1] - dotRect.lo()[1]) * 0.5 * eVecs[1]; 
      cg::point_3f dir3 = (dotRect.hi()[2] - dotRect.lo()[2]) * 0.5 * eVecs[2]; 

      // Performing translation from AABB coordinate system to previous one
      center = invT.treat_point(center);
      dir1 = invT.treat_vector(dir1);
      dir2 = invT.treat_vector(dir2);
      dir3 = invT.treat_vector(dir3);

      return cg::obb_clip_data(center, dir1, dir2, dir3);
   } // End of 'BuildObb' function

}

// END OF 'OBBBUILDER.H' FILE
