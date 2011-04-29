#pragma once

#include <vector>

#include "Geometry/lerp.h"
#include "Geometry/primitives/point.h"
#include "Geometry/segment_clip_by_rect.h"
#include "Geometry/segment_2_intersection.h"

namespace cg
{
// Simple polyline route for special purposes.
struct polyline_route_type
{
public:
   typedef std::vector<point_2> points_vec_type;

private:
   typedef std::vector<point_2> dirs_vec_type;
   typedef std::vector<double>  dists_vec_type;

public:
   polyline_route_type()
      : valid_(false)
   {}

   polyline_route_type & operator+=( polyline_route_type const& route )
   {
      std::unique_copy(route.points().begin(), route.points().end(), std::back_inserter(points_), point_2_eq_predicate());
      validate() ;
      return *this ;
   }

   template< class PointsIterator >
      polyline_route_type( PointsIterator first, PointsIterator beyond )
   {
      std::unique_copy(first, beyond, std::back_inserter(points_), point_2_eq_predicate());
      validate() ;
   }

   polyline_route_type( point_2 const& begin, point_2 const& end )
   {
      points_.push_back(begin) ;
      points_.push_back(end) ;
      validate() ;
   }

   // Route valid if it contains at least two not equal points.
   bool valid() const { return valid_; }

   point_2 interpolate ( double len ) const
   {
      return interpolate(len, NULL, NULL) ;
   }
   point_2 direction ( double len ) const
   {
      Assert(valid_);

      size_t p0, p1;
      interpolate(len, &p0, &p1);

      return dirs_[cg::range_2i(0, dirs_.size() - 1).closest_point(p0)];
   }
   // TODO: Do it faster.
   point_2 smoothed_direction( double len ) const
   {
      Assert(valid_);

      size_t p0, p1;
      interpolate(len, &p0, &p1);

      if (p0 == p1)
      {
         if (p0 == 0)
            return dirs_[0];
         else 
            return dirs_[dirs_.size() - 1];
      }

      if (p0 == 0 && len < dists_[1] / 2.0)
         return dirs_[0];
      if (p1 == points_.size() - 1 && len > (dists_[p1] + dists_[p0]) / 2.0)
         return dirs_[dirs_.size() - 1];

      point_2 dir0, dir1, dir2;
      double  c0, c1, c2;

      if (len < (dists_[p0] + dists_[p1]) / 2.0)
      {
         dir0 = dirs_ [p0 - 1];
         c0   = dists_[p0 - 1] + segmentLength(p0 - 1) / 2.0;
         dir2 = dirs_ [p0];
         c2   = dists_[p0]     + segmentLength(p0)     / 2.0;
         c1   = dists_[p0];

         point_2 dirsum = dir0 + dir2;
         if (eq_zero(dirsum))
            dir1 = normalized_safe(normal(dir0));
         else
            dir1 = normalized_safe(dirsum);
      }
      else
      {
         dir0 = dirs_ [p0];
         c0   = dists_[p0] + segmentLength(p0) / 2.0;
         dir2 = dirs_ [p1];
         c2   = dists_[p1] + segmentLength(p1) / 2.0;
         c1   = dists_[p1];

         point_2 const dirsum = dir0 + dir2;
         if (eq_zero(dirsum))
            dir1 = normalized_safe(normal(dir0));
         else
            dir1 = normalized_safe(dirsum);
      }

      Assert(cg::le(c0, len) && cg::le(len, c2));
      point_2 const dir = cg::key_lerp_clamp(len, c0, c1, c2, dir0, dir1, dir2);

      return dir;
   }
   double  length         () const 
   {
      if (points_.size() < 2)
         return 0.0;
      else
         return dists_[dists_.size() - 1]; 
   }

   // Finds closest segment on a route to provided point.
   // TODO: Add 'startLen' parameter.
   segment_2 getClosestSegment( point_2 const &pnt, double *len = 0 ) const
   {
      Assert(valid_);

      size_t    closestSegmentIdx = 0;
      segment_2 closestSegment    = segment(closestSegmentIdx);
      double    closestDist       = distance(pnt, closestSegment);

      for (size_t i = 1; i < points_.size() - 1; ++i)
      {
         segment_2 const seg  = segment(i);
         double    const dist = distance(pnt, seg);

         if (dist < closestDist)
         {
            closestSegmentIdx = i;
            closestSegment    = seg;
            closestDist       = dist;
         }
      }

      if (len)
         *len = dists_[closestSegmentIdx];

      return closestSegment;
   }

   // Finds closest point on a route to provided point.
   // TODO: Add 'startLen' parameter.
   point_2 getClosestPoint( point_2 const &pnt, double *len = 0, size_t * idx = 0 ) const
   {
      // TODO: Implement through getClosestSegment().
      Assert(valid_);


      point_2 closestPoint = points_[0];
      double  closestDist  = distance(pnt, closestPoint);
      double  closestLen   = 0;
      size_t  closestIdx = 0 ;

      for (size_t i = 0; i < points_.size() - 1; ++i)
      {
         segment_2 const seg   = segment(i);
         double    const coef  = seg(pnt);

         point_2   const point = seg.closest_point(pnt);
         double    const dist  = distance(pnt, point);

         if (dist < closestDist - cg::epsilon<double>())
         {
            closestPoint = point;
            closestDist  = dist;
            closestLen   = dists_[i] + cg::range_2(0.0, 1.0).closest_point(coef) * segmentLength(i);
            closestIdx   = i ;
         }
      }

      if (idx)
         *idx = closestIdx ;

      if (len)
         *len = closestLen;

      return closestPoint;
   }

   bool getOutPoint( rectangle_2 const &rect, double *outLen, bool reverse = false, double startLen = 0 ) const
   {
      Assert(valid_);

      size_t i;
      interpolate(startLen, &i, 0);

      for (; i < points_.size() - 1; ++i)
      {
         segment_2 const seg = segment(i);

         if (cg::has_intersection(seg, rect) && !rect.contains(reverse ? seg.P0() : seg.P1()))
         {
            point_2 p = cg::correct((reverse ? opposite(seg) : seg), rect);
            double  l = dists_[i] + seg(p) * segmentLength(i);

            if (startLen <= l)
            {
               if (outLen)
                  *outLen = l;
               return true;
            }
         }
      }

      return false;
   }

   // Note: usually will contain equal points, can be fixed if needed.
   template< class OutPointIterator >  
   void subRoute( double startLen, double endLen, OutPointIterator out )
   {
      Assert(valid_);
      Assert(startLen <= endLen);

      *out++ = interpolate(startLen);

      size_t startP0, startP1;
      interpolate(startLen, &startP0, &startP1);
      size_t endP0,   endP1;
      interpolate(endLen, &endP0,   &endP1);

      if (startP1 <= endP0)
         out = std::copy(boost::next(points_.begin(), startP1), boost::next(points_.begin(), endP0 + 1), out);

      *out++ = interpolate(endLen);
   }

   points_vec_type const & points() const
   {
      return points_;
   }

   void swap( polyline_route_type & route )
   {
      std::swap(this->valid_, route.valid_) ;
      this->dirs_.swap(route.dirs_) ;
      this->dists_.swap(route.dists_) ;
      this->points_.swap(route.points_) ;
   }

   point_2 interpolate ( double len, size_t * pp0, size_t * pp1 ) const
   {
      Assert(valid_);

      size_t p0, p1;
      interpolateImpl(len, &p0, &p1);

      if (pp0)
         *pp0 = p0 ;
      if (pp1)
         *pp1 = p1 ;

      if (p0 == p1)
         return points_[p0];
      else
      {
         segment_2 const seg = segment(p0);
         Assert(!cg::eq_zero(segmentLength(p0)));
         return seg((len - dists_[p0]) / segmentLength(p0));
      }
   }

   double pointLen( size_t idx ) const
   {
      Assert(idx < dists_.size());
      return dists_[idx];
   }

   void setPoint( size_t idx, point_2 const& p )
   {
      Assert(idx < points_.size()) ;
      points_[idx] = p ;
      validate() ;
   }

private:
   void validate()
   {
      if (points_.size() < 2)
      {
         valid_ = false;
      }
      else
      {
         valid_ = true;

         dists_.resize(points_.size()) ;
         dirs_.resize(points_.size() - 1) ;
         dists_[0] = 0 ;
         for (size_t i = 1; i < points_.size(); ++i)
         {
            point_2 const &p0 = points_[i - 1];
            point_2 const &p1 = points_[i];

            double  const segmentLen = distance(p0, p1);

            dists_[i] = (dists_[i - 1] + segmentLen);
            if (!cg::eq_zero(segmentLen))
               dirs_[i - 1] = (p1 - p0) / segmentLen;
            else
               dirs_[i - 1] = point_2() ;
         }
      }
   }

   void interpolateImpl( double len, size_t *p0, size_t *p1 ) const
   {
      Assert(valid_);

      if (cg::le(len, 0))
      {
         if (p0)
            *p0 = 0;
         if (p1)
            *p1 = 0;
      }
      else if (cg::ge(len, length()))
      {
         if (p0)
            *p0 = points_.size() - 1;
         if (p1)
            *p1 = points_.size() - 1;
      }
      else
      {
         size_t p = &(*std::lower_bound(dists_.begin(), dists_.end(), len)) - &(dists_[0]);
         Assert(p > 0 && p < points_.size());

         if (p0)
            *p0 = p - 1;
         if (p1)
            *p1 = p;
      }
   }

   segment_2 segment( size_t idx ) const
   {
      Assert(idx + 1 < dists_.size());
      return segment_2(points_[idx], points_[idx + 1]);
   }

   double segmentLength( size_t idx ) const
   {
      Assert(idx + 1 < dists_.size());
      return dists_[idx + 1] - dists_[idx];
   }

private:
   inline boost::function<bool (point_2 const &, point_2 const &)> point_2_eq_predicate()
   {
      boost::function<bool (point_2 const &, point_2 const &)> f;
      f = boost::bind<bool>((bool (*)(cg::point_t<double, 2> const &, cg::point_t<double, 2> const &, double ))&cg::eq<double>, _1, _2, cg::epsilon<double>());
      return f;
   }


private:
   bool            valid_;
   points_vec_type points_;
   dirs_vec_type   dirs_;
   dists_vec_type  dists_;


public:
   template<class Stream>
   friend void write( Stream& stream, const polyline_route_type & route ) ;

   template<class Stream>
   friend void read( Stream& stream, polyline_route_type & route ) ;
};

template<class Stream>
void write( Stream& stream, const polyline_route_type & route )
{
   write( stream, route.valid_ ) ;
   write( stream, route.points_ ) ;
   write( stream, route.dirs_ ) ;
   write( stream, route.dists_ ) ;
}

template<class Stream>
void read( Stream& stream, polyline_route_type & route )
{
   read( stream, route.valid_ ) ;
   read( stream, route.points_ ) ;
   read( stream, route.dirs_ ) ;
   read( stream, route.dists_ ) ;
}

inline cg::polyline_route_type get_polyline_part( cg::polyline_route_type const& polyline, double len0, double len1 )
{
   size_t idx0, idx1 ;
   point_2 pnt0 = polyline.interpolate(len0, &idx0, NULL) ;
   point_2 pnt1 = polyline.interpolate(len1, &idx1, NULL) ;
   std::vector<point_2> points ;
   points.push_back(pnt0) ;
   for (size_t j = idx0 + 1; j <= idx1; ++j)
      points.push_back(polyline.points()[j]) ;
   if (!cg::eq(pnt1, points.back()))
      points.push_back(pnt1) ;

   return cg::polyline_route_type(points.begin(), points.end()) ;
}

inline void lerp_polylines( cg::polyline_route_type const& l, cg::polyline_route_type const& r, double t, cg::polyline_route_type &res )
{
   std::vector<double> lens ;
   lens.reserve(r.points().size() + l.points().size()) ;

   for (size_t i = 1; i < l.points().size() - 1; i++)
      lens.push_back(l.pointLen(i) / l.length()) ;

   for (size_t i = 1; i < r.points().size() - 1; i++)
      lens.push_back(r.pointLen(i) / r.length()) ;

   std::sort(lens.begin(), lens.end()) ;

   std::vector<point_2> points ;
   points.push_back(cg::segment_2(l.points().front(), r.points().front())(t)) ;

   for (size_t i = 0; i < lens.size(); i++)
      points.push_back(cg::segment_2(l.interpolate(lens[i] * l.length()), r.interpolate(lens[i] * r.length()))(t)) ;

   points.push_back(cg::segment_2(l.points().back(), r.points().back())(t)) ;

   cg::polyline_route_type(points.begin(), points.end()).swap(res) ;
}

inline bool has_intersection( cg::polyline_route_type const& l, cg::segment_2 const& seg, double *len )
{
   point_2 pt ;
   for(size_t i = 0; i < l.points().size() - 1; ++i)
   {
      if (cg::generic_intersection(cg::segment_2(l.points()[i], l.points()[i + 1]), seg, &pt, (point_2 *)NULL) != cg::disjoint)
      {
         if (len)
            l.getClosestPoint(pt, len) ;

         return true ;
      }
   }

   return false ;
}


}
