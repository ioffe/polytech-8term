#pragma once

#include <geometry\lerp.h>

namespace ProjectiveTransform
{
   using namespace cg;

   /* Create projection trapezoid function
   * NOTE:
   * - frustum points order:
   *    0 - right top near     1 - right top far
   *    2 - right bottom near  3 - right bottom far
   *    4 - left bottom near   5 - left bottom far
   *    6 - left top near      7 - left top far
   * - trapezoid points:
   *    3   2
   *    0   1
   * RETURNS: 
   *    visibility flag
   */
   static inline bool CreateProjectionTrapezoid( const point_3f frustumPoints[8], 
      const point_3f &camPos, const point_3f &camDir, const planef &midPlane, 
      float bottomOffset, float topOffset, point_2f trapezoid[4] );   

   static const float g_TranslateScale2ExtendedQuad[9] = 
   {  
      0.5f, 0.0f, 0.5f, 
      0.0f, 0.5f, 0.5f,
      0.0f, 0.0f, 1.0f
   };
   static const cg::matrix_3f g_MatrixTranslateScale2ExtendedQuad(g_TranslateScale2ExtendedQuad);

   /* Build perspective projection transform by 4 points
   * NOTE: Projective mapping (perspective transformation) - 
   *       projection from one plane through a point to another plane
   * - translateScaleMatrix - scale and translate matrix for unit quad
   *   for transform to -1 1 quad following matrix is needed:
   *   0.5 0.0 0.5                 (-1,  1) (1,  1)       p4   p3
   *   0.0 0.5 0.5   results int                    <--->
   *   0.0 0.0 1.0                 (-1, -1) (1, -1)       p1   p2
   * - viewportSize is needed, when this projection strongly obliges to enclose
   *   whole trapezoid by its boundaries with half-pixel correction
   * - the result is projection transform like this:
   *   (0, 1)  (1, 1)       p4   p3
   *                  <--->
   *   (0, 0), (1, 0)       p1   p2
   * COMMONE USAGE EXAMPLE:
   *    BuildProjectionMatrixBy4Points(trapezoid[0], trapezoid[1], trapezoid[2], trapezoid[3], trapT,
   *       g_MatrixTranslateScale2ExtendedQuad);
   */
   static inline void BuildProjectionMatrixBy4Points( const point_2f &p1, const point_2f &p2, const point_2f &p3, 
      const point_2f &p4, cg::transform_4f &trapT, 
      const cg::matrix_3f &translateScaleMatrix = g_MatrixTranslateScale2ExtendedQuad, 
      const cg::point_2i &viewportSize = cg::point_2i());
}

//
// Cycled iterator class
//

template <typename Base>
class c_iter
{
   typedef typename Base::iterator iterator;
   typedef typename Base::value_type value_type;
   typedef value_type& value_type_ref;
   typedef typename Base::_Tptr value_type_ptr;
   typedef typename Base::_Ctptr value_type_const_ptr;

   Base & base_obj; // owner object         
   iterator d_iterator; // during iterator
   int d_position; // during position

public:
   c_iter( const Base &_base_obj, int start_pos = 0 ) : base_obj(const_cast< Base & >(_base_obj))
   {
      d_iterator = base_obj.begin();      
      range_pos(start_pos);
   }

   c_iter( const c_iter &_c_iter ) : base_obj(_c_iter.base_obj)
   {
      copy(_c_iter);
   }

private:
   void copy( const c_iter &_c_iter )
   {
      range_pos(_c_iter.d_position);
   }

public:      

   // operators
   value_type_ref operator*()
   {
      return *d_iterator;
   }

   value_type_ptr operator->()
   {
      return (&**this);
   }
   value_type_const_ptr operator->() const
   {
      return (&**this);
   }

   c_iter & operator++()
   {
      range_pos(d_position + 1);
      return *this;
   }
   c_iter operator++(int)
   {
      c_iter _it(*this);
      //_it.range_pos(d_position + 1);
      ++(*this);
      return _it;
   }

   c_iter & operator--()
   {
      range_pos(d_position - 1);
      return *this;
   }
   c_iter operator--(int)
   {
      c_iter _it(*this);
      //_it.range_pos(d_position - 1);
      --(*this);
      return _it;
   }

   c_iter operator+(int off)
   {
      c_iter _it(*this);
      _it.range_pos(_it.d_position + off);
      return _it;
   }
   c_iter operator-(int off)
   {
      c_iter _it(*this);
      _it.range_pos(_it.d_position - off);
      return _it;
   }

   c_iter & operator+=(int off)
   {
      range_pos(d_position + off);
      return *this;
   }
   c_iter & operator-=(int off)
   {
      range_pos(d_position - off);
      return *this;
   }

   value_type_ref operator[](int Pos)
   {
      range_pos(Pos);
      return *d_iterator;
   }

   operator iterator&()
   {
      return d_iterator;
   }

   operator const iterator&() const
   {
      return d_iterator;
   }

   bool operator!=( const c_iter &_it) const
   {
      return (iterator)(*this) != (iterator)(_it);
   }

   bool operator!=( const iterator &_it) const
   {
      return (iterator)(*this) != _it;
   }

   void erase( void )
   {
      base_obj.erase(d_iterator);
      range_pos(d_position - 1);
   }

   // indexing
   void set_pos( int pos )
   {
      range_pos(pos);
   }
   int get_pos( void ) const
   {
      return d_position;
   }

private:
   void range_pos( int Pos )
   {
      int size = (int)base_obj.size();
      if (size == 0)      
         d_position = 0;
      else
      {
         while (Pos < 0)
            Pos += size;
         d_position = Pos % size;      
      }
      d_iterator = (base_obj.begin() + d_position);
   }
};

//
// Realization
//

namespace ProjectiveTransform
{
   using namespace cg;

   // "Projected on plane" point representation structure
   struct ProjPoint
   {
      ProjPoint( const point_2f &newPos ) : pos(newPos), angle(0.0f)
      {
      }

      ProjPoint() : pos(0.0f, 0.0f), angle(0.0f)
      {
      }

      point_2f pos; // Position on plane
      double angle;  // Polar angle relative to middle point
   };

   static inline int ClampCycle( int i, int size )
   {
      if (i >= size)
         i -= size;
      if (i < 0)
         i += size;

      return i;
   }

   // Comparison function
   static inline bool ProjPointGreater( const ProjPoint &p1, const ProjPoint &p2 )
   {
      return p1.angle < p2.angle;
   }

   // Check for point belong to convex polygon and return vertices indices forming covering angle
   static inline bool PointInsideConvexPolygon( const std::vector<ProjPoint> &poly, const point_2f &polyCenterPoint, const point_2f &pos, 
      int *leftIdx, int *rightIdx )
   {
      point_2 D = normalized_safe(point_2(pos) - point_2(polyCenterPoint));
      double posRAngle = acos(clamp(D.x, -1.0, 1.0));
      if ((point_3(1.0, 0.0, 0.0) ^ point_3(D.x, D.y, 0.0)).z < 0.0)
         posRAngle = 2.0f * cg::pi - posRAngle;
      int minIdx = poly.size() - 1;
      bool found = false;
      for (size_t i = 0; i < poly.size() - 1; i++)
      {
         if (poly[i].angle < posRAngle && poly[i + 1].angle > posRAngle)
         {
            minIdx = i;
            found = true;
            break;
         }
      }
      if (!found)
         minIdx = poly.size() - 1;
      int maxIdx = (minIdx == poly.size() - 1 ? 0 : minIdx + 1);
      cg::line_2 testLine(poly[minIdx].pos, poly[maxIdx].pos, cg::line::by_points);

      *leftIdx = minIdx;
      *rightIdx = maxIdx;
      return on_left_side(testLine, pos);
   }

   // Build optimal trapezoid for polygon on plane function
   static inline void BuildOptimalTrapezoid( const point_2f &camPosProj, point_2f &dirProj, 
      const std::vector<ProjPoint> &polygon, const point_2f &polyCenterPoint, point_2f *trapezoid )
   {
      typedef std::vector< ProjPoint > polygone_type;
      typedef std::vector< ProjPoint >::iterator iterator;

      // search near and far points on bases of trapezoid 
      point_2 baseLine(cg::normalized_safe(dirProj));
      point_2 normalLine(dirProj.y, -dirProj.x);
      int minIndex = 0, maxIndex = 0;
      double minDist = baseLine * polygon[0].pos;
      double maxDist = baseLine * polygon[0].pos;
      for (unsigned int i = 1; i < polygon.size(); i++)
      {
         double dist = baseLine * polygon[i].pos;
         if (dist > maxDist)
         {
            maxDist = dist;
            maxIndex = i;
         }
         else if (dist < minDist)
         {
            minDist = dist;
            minIndex = i;
         }
      }
      if ((polygon[maxIndex].pos - polygon[minIndex].pos) * dirProj < 0.0f)
      {
         std::swap(minDist, maxDist);
         std::swap(minIndex, maxIndex);
      }
      cg::line_2 minBase(polygon[minIndex].pos, polygon[minIndex].pos + normalLine, cg::line::by_points);
      cg::line_2 maxBase(polygon[maxIndex].pos, polygon[maxIndex].pos + normalLine, cg::line::by_points);
      cg::line_2 dirLine(camPosProj, camPosProj + dirProj, cg::line::by_points);
      cg::point_2 nearBaseCenter, farBaseCenter;
      cg::has_intersection(minBase, dirLine, nearBaseCenter);
      cg::has_intersection(maxBase, dirLine, farBaseCenter);

      int rightEdgeIndex = -1;
      int leftEdgeIndex = -1;

      int idxLeft, idxRight;
      point_2 dirToCenter;
      bool pointOutside = !PointInsideConvexPolygon(polygon, polyCenterPoint, camPosProj, &idxLeft, &idxRight);
      dirToCenter = normalized_safe(0.5f * (polygon[idxLeft].pos + polygon[idxRight].pos) - camPosProj);

      if (pointOutside)
      {
         // Get left apex forming vertex index
         point_2f leftDir, rightDir;

         // Search left value
         double leftAngle = -1.0;
         float leftDistToV = 1e6f;
         for (size_t i = 0; i < polygon.size(); i++)
         {
            float distToV = norm(polygon[i].pos - camPosProj);
            point_2 V = normalized_safe(point_2(polygon[i].pos) - point_2(camPosProj));
            double curAngle = acos(clamp(V * dirToCenter, -1.0, 1.0));

            bool angleEqFlag = cg::eq(curAngle, leftAngle, 0.01);
            if ((point_3f(point_3f(V) ^ point_3f(dirToCenter)).z < 0.0f || cg::eq(V, dirToCenter)) && 
               (curAngle > leftAngle || angleEqFlag) )
            {
               if (angleEqFlag)
               {
                  if (distToV < leftDistToV)
                  {
                     idxLeft = i;
                     leftDir = V;
                     leftAngle = curAngle;
                     leftDistToV = distToV;
                  }
               }
               else
               {
                  idxLeft = i;
                  leftDir = V;
                  leftAngle = curAngle;
                  leftDistToV = distToV;
               }
            }
         }
         double rightAngle = -1.0f;
         float rightDistToV = 1e6f;;
         for (size_t i = 0; i < polygon.size(); i++)
         {
            float distToV = norm(polygon[i].pos - camPosProj);
            point_2 V = normalized_safe(point_2(polygon[i].pos) - point_2(camPosProj));
            double curAngle = acos(clamp(V * dirToCenter, -1.0, 1.0));

            bool angleEqFlag = cg::eq(curAngle, rightAngle, 0.01);
            if ((point_3f(point_3f(V) ^ point_3f(dirToCenter)).z > 0.0f || cg::eq(V, dirToCenter)) && 
               (curAngle > rightAngle || angleEqFlag))
            {
               if (angleEqFlag)
               {
                  if (distToV < rightDistToV)
                  {
                     idxRight = i;
                     rightDir = V;
                     rightAngle = curAngle;
                     rightDistToV = distToV;
                  }
               }
               else
               {
                  idxRight = i;
                  rightDir = V;
                  rightAngle = curAngle;
                  rightDistToV = distToV;
               }
            }
         }

#if 0
         //
         // Collapse excess points for easier computations
         //
         int clampSize = polygon.size();
         int idxRightDecr = ClampCycle(idxRight - 1, clampSize);
         bool passTest = (idxRightDecr == idxLeft);

         if (!passTest)
         {
            passTest = true;
            int i = idxLeft;
            point_2f initVec = normalized_safe(polygon[i].pos -  polygon[ClampCycle(i + 1, clampSize)].pos);
            i = ClampCycle(i + 1, clampSize);
            while (i != idxRight)
            {
               point_2f nextVec = normalized_safe(polygon[i].pos -  polygon[ClampCycle(i + 1, clampSize)].pos);
               if (!cg::eq(nextVec, initVec, 0.1f))
               {
                  passTest = false;
                  break;
               }
               initVec = nextVec;
               i = ClampCycle(i + 1, clampSize);
            }
         }

         if (passTest)
         {
            normalLine = normalized_safe(polygon[idxRight].pos - polygon[idxLeft].pos);
            dirProj = point_2(-normalLine.y, normalLine.x);
            baseLine = point_2(cg::normalized_safe(dirProj));

            nearBaseCenter = camPosProj;
            minBase = cg::line_2(polygon[minIndex].pos, polygon[minIndex].pos + normalLine, cg::line::by_points);
            maxBase = cg::line_2(polygon[maxIndex].pos, polygon[maxIndex].pos + normalLine, cg::line::by_points);
         }
#endif

         // Intersection points of 2d frustum with near line
         point_2 leftBaseInter;
         point_2 rightBaseInter;
         cg::line_2 leftLine(camPosProj, camPosProj + leftDir, cg::line::by_points);
         cg::line_2 rightLine(camPosProj, camPosProj + rightDir, cg::line::by_points);
         cg::has_intersection(minBase, leftLine, leftBaseInter);
         cg::has_intersection(minBase, rightLine, rightBaseInter);

         // New heuristic check
         double minCriterion = -1.0;
         double minSymmNearC = -1.0;
         for (c_iter< polygone_type > itLeft(polygon, minIndex); (iterator)itLeft != (polygon.begin() + maxIndex); itLeft--)
         {
            cg::line_2 leftDir((*(itLeft - 1)).pos, itLeft->pos, cg::line::by_points);
            point_2 leftNearBaseInter, leftFarBaseInter;

            if (!cg::has_intersection(minBase, leftDir, leftNearBaseInter) ||
               !cg::has_intersection(maxBase, leftDir, leftFarBaseInter))
               continue;

            for (c_iter< polygone_type > itRight(polygon, minIndex); (iterator)itRight != (polygon.begin() + maxIndex); itRight++)
            {
               cg::line_2 rightDir((*(itRight + 1)).pos, itRight->pos, cg::line::by_points);
               point_2 rightNearBaseInter, rightFarBaseInter;

               if (!cg::has_intersection(minBase, rightDir, rightNearBaseInter) ||
                  !cg::has_intersection(maxBase, rightDir, rightFarBaseInter))
                  continue;

               // Calculate criterion
               double pureLeft = norm(leftNearBaseInter) + norm(leftFarBaseInter);
               double pureRight = norm(rightNearBaseInter) + norm(rightFarBaseInter);

               double symmNear = abs(norm(rightNearBaseInter - nearBaseCenter) - norm(leftNearBaseInter - nearBaseCenter));
               double symmFar = abs(norm(rightFarBaseInter - farBaseCenter) - norm(leftFarBaseInter - farBaseCenter));

               double resC = pureLeft + pureRight + symmNear + symmFar;
               double prec = norm(farBaseCenter - nearBaseCenter) * 1e-3;
               if (minSymmNearC < 0.0 || minSymmNearC > symmNear + symmFar || cg::eq(minSymmNearC, symmNear + symmFar, prec))
               {
                  minSymmNearC = symmNear + symmFar;
                  if (minCriterion < 0.0 || minCriterion > resC)
                  {
                     minCriterion = resC;
                     leftEdgeIndex = itLeft.get_pos();
                     rightEdgeIndex = itRight.get_pos();
                  }
               }
               //if (minCriterion < 0.0 || minCriterion > resC)
               //{
               //   minCriterion = resC;
               //   leftEdgeIndex = itLeft.get_pos();
               //   rightEdgeIndex = itRight.get_pos();
               //}
            }
         }

#if 0
         // searching right trapezoid edge
         double minRightSquare = -1.0f;
         const float NEAR_LINE_WEIGHT = 0.0f;
         for (c_iter< polygone_type > it(polygon, minIndex); (iterator)it != (polygon.begin() + maxIndex); it++)
         {
            cg::line_2 dir((*(it + 1)).pos, it->pos, cg::line::by_points);
            point_2 nearBaseInter, farBaseInter;
            if (!cg::has_intersection(minBase, dir, nearBaseInter) ||
               !cg::has_intersection(maxBase, dir, farBaseInter))
               continue;

            double nearRightDist = cg::norm(nearBaseInter - rightBaseInter);

            nearBaseInter -= nearBaseCenter;
            farBaseInter -= farBaseCenter;

            double interSquare = (double)cg::norm(farBaseInter) + (double)cg::norm(nearBaseInter) + 
               nearRightDist * NEAR_LINE_WEIGHT;
            if (minRightSquare < 0.0f || minRightSquare > interSquare)
            {
               minRightSquare = interSquare;
               rightEdgeIndex = it.get_pos();
            }
         }
         // searching left trapezoid edge
         double minLeftSquare = -1.0f;
         for (c_iter< polygone_type > it(polygon, minIndex); (iterator)it != (polygon.begin() + maxIndex); it--)
         {
            cg::line_2 dir((*(it - 1)).pos, it->pos, cg::line::by_points);
            point_2 nearBaseInter, farBaseInter;
            if (!cg::has_intersection(minBase, dir, nearBaseInter) ||
               !cg::has_intersection(maxBase, dir, farBaseInter))
               continue;

            double nearLeftDist = cg::norm(nearBaseInter - leftBaseInter);

            nearBaseInter -= nearBaseCenter;
            farBaseInter -= farBaseCenter;
            double interSquare = (double)cg::norm(farBaseInter) + (double)cg::norm(nearBaseInter) + 
               nearLeftDist * NEAR_LINE_WEIGHT;
            if (minLeftSquare < 0.0f || minLeftSquare > interSquare)
            {
               minLeftSquare = interSquare;
               leftEdgeIndex = it.get_pos();
            }
         }
#endif
      }
      else
      {
         // searching right trapezoid edge
         double minRightSquare = -1.0f;
         for (c_iter< polygone_type > it(polygon, minIndex); (iterator)it != (polygon.begin() + maxIndex); it++)
         {
            cg::line_2 dir((*(it + 1)).pos, it->pos, cg::line::by_points);
            point_2 nearBaseInter, farBaseInter;
            if (!cg::has_intersection(minBase, dir, nearBaseInter) ||
               !cg::has_intersection(maxBase, dir, farBaseInter))
               continue;

            nearBaseInter -= nearBaseCenter;
            farBaseInter -= farBaseCenter;
            double interSquare = (double)cg::norm(farBaseInter) + (double)cg::norm(nearBaseInter);
            if (minRightSquare < 0.0f || minRightSquare > interSquare)
            {
               minRightSquare = interSquare;
               rightEdgeIndex = it.get_pos();
            }
         }
         // searching left trapezoid edge
         double minLeftSquare = -1.0f;
         for (c_iter< polygone_type > it(polygon, minIndex); (iterator)it != (polygon.begin() + maxIndex); it--)
         {
            cg::line_2 dir((*(it - 1)).pos, it->pos, cg::line::by_points);
            point_2 nearBaseInter, farBaseInter;
            if (!cg::has_intersection(minBase, dir, nearBaseInter) ||
               !cg::has_intersection(maxBase, dir, farBaseInter))
               continue;

            nearBaseInter -= nearBaseCenter;
            farBaseInter -= farBaseCenter;
            double interSquare = (double)cg::norm(farBaseInter) + (double)cg::norm(nearBaseInter);
            if (minLeftSquare < 0.0f || minLeftSquare > interSquare)
            {
               minLeftSquare = interSquare;
               leftEdgeIndex = it.get_pos();
            }
         }
      }
      c_iter< polygone_type > iter(polygon);
      cg::line_2 left_edge(iter[leftEdgeIndex].pos, iter[leftEdgeIndex - 1].pos, cg::line::by_points);
      cg::line_2 right_edge(iter[rightEdgeIndex].pos, iter[rightEdgeIndex + 1].pos, cg::line::by_points);

      // constructing quadrilateral
      point_2 res;
      cg::has_intersection(minBase, left_edge, res);
      trapezoid[0] = res;
      cg::has_intersection(minBase, right_edge, res);
      trapezoid[1] = res;
      cg::has_intersection(maxBase, right_edge, res);
      trapezoid[2] = res;
      cg::has_intersection(maxBase, left_edge, res);
      trapezoid[3] = res;

      // Check for degenerated case (trapezoid[0] = trapezoid[1])
      if (cg::eq(trapezoid[0], trapezoid[1]))
      {
         if (!cg::eq(point_2(trapezoid[0]), nearBaseCenter))
         {
            // Trapezoid points and center point are not equal
            point_2f dirNearLine = trapezoid[0] - nearBaseCenter;
            if (dirNearLine * normalLine > 0.0f)
               trapezoid[0] = nearBaseCenter - dirNearLine;
            else
               trapezoid[1] = nearBaseCenter - dirNearLine;
         }
         else
         {
            // All points are equal
            const float TRAPEZOID_OFFSET_DIST = 3.0f;

            trapezoid[1] = nearBaseCenter - normalLine * TRAPEZOID_OFFSET_DIST;
            trapezoid[0] = nearBaseCenter + normalLine * TRAPEZOID_OFFSET_DIST;
         }
      }

      // Check for degenerated case (trapezoid[2] = trapezoid[3])
      if (cg::eq(trapezoid[2], trapezoid[3]))
      {
         if (!cg::eq(point_2(trapezoid[2]), farBaseCenter))
         {
            // Trapezoid points and center point are not equal
            point_2f dirFarLine = trapezoid[2] - farBaseCenter;
            if (dirFarLine * normalLine > 0.0f)
               trapezoid[3] = farBaseCenter - dirFarLine;
            else
               trapezoid[2] = farBaseCenter - dirFarLine;
         }
         else
         {
            // All points are equal
            const float TRAPEZOID_OFFSET_DIST = 3.0f;

            trapezoid[2] = farBaseCenter - normalLine * TRAPEZOID_OFFSET_DIST;
            trapezoid[3] = farBaseCenter + normalLine * TRAPEZOID_OFFSET_DIST;
         }
      }
      float normNear = norm(trapezoid[1] - trapezoid[0]);
      float normFar = norm(trapezoid[2] - trapezoid[3]);
      float dist = (float)norm(farBaseCenter - nearBaseCenter);//0.5f * (norm(trapezoid[1] - trapezoid[3]) + norm(trapezoid[0] - trapezoid[2]));

      dist = abs(max(dist, normNear) / max(1.0f, norm(camPosProj - cg::point_2f(nearBaseCenter)) / 20.0f));

      if (dist > normFar)
      {
         point_2f trapezoidMid = 0.5f * (trapezoid[2] + trapezoid[3]);
         trapezoid[2] = trapezoidMid + normalLine * dist * 0.5f;
         trapezoid[3] = trapezoidMid - normalLine * dist * 0.5f;
      }
   }

   static bool CreateProjPointsList( const point_3f frustumPoints[8], const point_3f &camPos, 
                                     const point_3f &camDir, const planef &bottomPlane, 
                                     const planef &topPlane, const planef &mainPlane,
                                     point_2f &middlePoint, std::vector<ProjPoint> &projPointsList )
   {
      // Frustum edges special order for processing
      static const int edges[] = 
      {
         0, 1,   2, 3,   4, 5,   6, 7,
         0, 2,   2, 4,   4, 6,   6, 0,
         1, 3,   3, 5,   5, 7,   7, 1
      };

      // Handling near and near-far edges
      for (size_t i = 0; i < 8; i++)
      {
         const point_3f &P1 = frustumPoints[edges[2 * i]];
         const point_3f &P2 = frustumPoints[edges[2 * i + 1]];
         point_3f projPoint;
         if (has_intersection(topPlane, cg::segment_3f(P1, P2), &projPoint))
            projPointsList.push_back(ProjPoint(point_2f(projPoint.x, projPoint.y)));
         if (has_intersection(bottomPlane, cg::segment_3f(P1, P2), &projPoint))
            projPointsList.push_back(point_2f(projPoint.x, projPoint.y));
      }

      // Handling far edges specially
      for (int i = 8; i < 12; i++)
      {
         const point_3f &P1 = frustumPoints[edges[2 * i]];
         const point_3f &P2 = frustumPoints[edges[2 * i + 1]];
         point_3f projPoint;
         if (has_intersection(mainPlane, segment_3f(P1, P2), &projPoint))
            projPointsList.push_back(point_2f(projPoint.x, projPoint.y));
      }

      // Handling frustum points
      for (int i = 0; i < 8; i++)
         if (topPlane(frustumPoints[i]) * bottomPlane(frustumPoints[i]) < 0.0f)
            projPointsList.push_back(point_2f(frustumPoints[i].x, frustumPoints[i].y));

      // Visibility check
      if (projPointsList.size() <= 3)
         return false;

      // Building convex hull
      middlePoint = point_2f(0.0f, 0.0f);
      {
         // Rearranging points
         for (size_t i = 0; i < projPointsList.size(); i++)
            middlePoint += projPointsList[i].pos;
         middlePoint /= (float)projPointsList.size();

         // Count angles
         for (size_t i = 0; i < projPointsList.size(); i++)
         {
            point_2 D = normalized(point_2(projPointsList[i].pos) - point_2(middlePoint));
            projPointsList[i].angle = acos(clamp(D.x, -1.0, 1.0));
            if ((point_3f(1.0f, 0.0f, 0.0f) ^ point_3f(D, 0.0f)).z < 0.0f)
               projPointsList[i].angle = 2.0f * (float)cg::pi - projPointsList[i].angle;
         }

         // Sorting by polar angle relative to middle point
         std::sort(projPointsList.begin(), projPointsList.end(), ProjPointGreater);

         // Building convex hull
         size_t iter1, iter2, iter3;
         bool flag3 = true, flag2 = true;
         iter1 = 0;
         iter2 = 1;
         iter3 = 2;
         do 
         {
            point_3f P1, P2;

            // test for ear
            P1 = point_3f(projPointsList[iter2].pos - projPointsList[iter1].pos, 0.0f);
            P2 = point_3f(projPointsList[iter3].pos - projPointsList[iter2].pos, 0.0f);
            if ((P1 ^ P2).z > 0.0f)
            {
               // convex angle, continue
               iter1++;
               iter2++;
               iter3++;
            }
            else
            {
               // remove ear-vertex
               projPointsList.erase(projPointsList.begin() + iter2);
               if (flag3 != flag2)
                  iter3++;
            }

            if (iter3 == projPointsList.size())
            {
               iter3 = 0;
               flag3 = false;
            }
            if (iter2 == projPointsList.size())
            {
               iter2 = 0;
               flag2 = false;
            }
         } while (iter1 != projPointsList.size());
      }

      return true;
   }

   static bool CreateTrapezoid( const point_3f frustumPoints[8], const point_3f &camPos, 
      const point_3f &camDir, const planef &bottomPlane, const planef &topPlane, const planef &mainPlane,
      point_2f *trapezoid )
   {
      // Projected points list
      std::vector<ProjPoint> projPointsList;
      // middle point
      cg::point_2f middlePoint;

      if (!CreateProjPointsList(frustumPoints, camPos, camDir, bottomPlane, 
                                topPlane, mainPlane, middlePoint, projPointsList))
         return false;

      if (projPointsList.size() == 3)
      {
         // Split closest point 
         point_2f camPosXY(camPos.x, camPos.y);

         float minDist = 1e6f;
         int minIdx = -1;
         for (int i = 0; i < 3; i++)
         {
            float dist = norm(camPosXY - projPointsList[i].pos);
            if (dist < minDist)
            {
               dist = minDist;
               minIdx = i;
            }
         }

         Assert(minIdx != -1);

         point_2f splitP = projPointsList[minIdx].pos;
         point_2 normalLine(normalized_safe(point_2f(camDir.y, -camDir.x)));
         point_2f left = splitP - normalLine, right = splitP + normalLine;

         projPointsList[minIdx] = right ; 
         projPointsList.insert(projPointsList.begin() + minIdx, left);
         //std::vector<ProjPoint>::iterator erasedIter = projPointsList.begin() + minIdx;
         //erasedIter = projPointsList.erase(erasedIter);
         //projPointsList.insert(erasedIter, right);
         //projPointsList.insert(erasedIter, left);
      }
      if (projPointsList.size() < 3)
      {
         //Assert(false);
         return false;
      }
#if 0
      middlePoint = point_2f();
      for (size_t i = 0; i < projPointsList.size(); i++)
         middlePoint += projPointsList[i].pos;
      middlePoint /= projPointsList.size();

      // Count angles
      for (size_t i = 0; i < projPointsList.size(); i++)
      {
         point_2 D = normalized(point_2(projPointsList[i].pos) - point_2(middlePoint));
         projPointsList[i].angle = acos(clamp(D.x, -1.0, 1.0));
         if ((point_3f(1.0f, 0.0f, 0.0f) ^ point_3f(D, 0.0f)).z < 0.0f)
            projPointsList[i].angle = 2.0f * (float)cg::pi - projPointsList[i].angle;
      }
#endif      

      // Make optimal quadrilateral(trapezoid) from convex hull
      BuildOptimalTrapezoid(point_2f(camPos.x, camPos.y), normalized(point_2f(camDir.x, camDir.y)), 
         projPointsList, middlePoint, trapezoid);

      //BuildOptimalQuadrilateral(point_2f(camPos.x, camPos.y), normalized(point_2f(camDir.x, camDir.y)), 
      //   projPointsList, trapezoid);
      return true;
   }

   // Build perspective projection matrix by 4 points
   // NOTE: Projective mapping (perspective transformation) - 
   //       projection from one plane through a point to another plane
   static inline void BuildProjectionMatrixBy4Points( const point_2f &p1, const point_2f &p2, const point_2f &p3, 
      const point_2f &p4, cg::transform_4f &trapT, const cg::matrix_3f &translateScaleMatrix, 
      const cg::point_2i &viewportSize )
   {
      point_2 center;
      double x0 = p1.x - center.x, y0 = p1.y - center.y;
      double x1 = p2.x - center.x, y1 = p2.y - center.y;
      double x2 = p3.x - center.x, y2 = p3.y - center.y;
      double x3 = p4.x - center.x, y3 = p4.y - center.y;

      const double DELTA_X1 = x1 - x2;
      const double DELTA_X2 = x3 - x2;
      const double SUM_X    = x0 - x1 + x2 - x3;
      const double DELTA_Y1 = y1 - y2;
      const double DELTA_Y2 = y3 - y2;
      const double SUM_Y    = y0 - y1 + y2 - y3;

      double a, b, c, d, e, f, g, h;   
      double DET = DELTA_X1 * DELTA_Y2 - DELTA_Y1 * DELTA_X2;
      if (cg::eq_zero(DET))
         DET = 1e-10;
      g = (SUM_X * DELTA_Y2 - SUM_Y * DELTA_X2) / DET;
      h = (DELTA_X1 * SUM_Y - DELTA_Y1 * SUM_X) / DET;
      a = x1 - x0 + g * x1;
      b = x3 - x0 + h * x3;
      c = x0;
      d = y1 - y0 + g * y1;
      e = y3 - y0 + h * y3;
      f = y0;

      // half-pixel correction
      cg::point_2 halfPix;
      if (viewportSize.x > 1)
         halfPix.x = 0.5 / (viewportSize.x - 1.0);
      if (viewportSize.y > 1)
         halfPix.y = 0.5 / (viewportSize.y - 1.0);

      // Translate / Scale matrix
      cg::matrix_3 tRS = translateScaleMatrix;
      tRS(0, 0) += halfPix.x;
      tRS(1, 1) += halfPix.y;
      // Unit -> trap and inversed 3x3 matrices
      cg::matrix_3 U2T, U2TInv;
      U2T(0, 0) = a, U2T(0, 1) = b, U2T(0, 2) = c;
      U2T(1, 0) = d, U2T(1, 1) = e, U2T(1, 2) = f;
      U2T(2, 0) = g, U2T(2, 1) = h, U2T(2, 2) = 1.0;
      U2T = U2T * tRS;
      cg::inverse(U2T, U2TInv);

      //
      // Special transposed order
      //

      double proj_matr[16] = 
      {
         U2T(0, 0), U2T(0, 1), 0.0, U2T(0, 2),
         U2T(1, 0), U2T(1, 1), 0.0, U2T(1, 2),
               0.0,       0.0, 1.0,       0.0,
         U2T(2, 0), U2T(2, 1), 0.0, U2T(2, 2),
      };
      double proj_matr_inv[16] = 
      {
         U2TInv(0, 0), U2TInv(0, 1), 0.0, U2TInv(0, 2),
         U2TInv(1, 0), U2TInv(1, 1), 0.0, U2TInv(1, 2),
                  0.0,          0.0, 1.0,          0.0,
         U2TInv(2, 0), U2TInv(2, 1), 0.0, U2TInv(2, 2),
      };

      // and finally, make desired transformation   
      trapT = cg::transform_4f(cg::matrix_4(proj_matr), cg::matrix_4(proj_matr_inv));
   }

   // Create projection trapezoid function
   // RETURNS: visibility flag
   static inline bool CreateProjectionTrapezoid( const point_3f frustumPoints[8],
      const point_3f &camPos, const point_3f &camDir, const planef &mainPlane,
      float bottomOffset, float topOffset, point_2f trapezoid[4] )
   {
      planef bottomPlane(mainPlane.n(), mainPlane.d() - bottomOffset),
         topPlane(mainPlane.n(), mainPlane.d() - topOffset);

      // Create optimal trapezoid
      if (!CreateTrapezoid(frustumPoints, camPos, camDir, bottomPlane, topPlane, mainPlane, trapezoid))
         return false;

      return true;
   }
};

// END OF 'PROJECTIVETRANSFORM.H' FILE
