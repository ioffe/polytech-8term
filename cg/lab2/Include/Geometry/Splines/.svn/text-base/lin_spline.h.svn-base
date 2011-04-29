#pragma once

namespace cg
{
template <class point>
struct lin_spline_t
{
   typedef point                                            point_t;
   typedef typename point::scalar_type                      scalar_type;
   typedef cg::segment_t<scalar_type, point_t::dimension>   segment_t;
   typedef std::vector<point_t>                             points_t;

   template< class Iterator >
   lin_spline_t( Iterator p, Iterator q )
      : points_    (p, q)
      , points_len_(std::distance(p, q))
   {
      Assert(points_.size() > 1);

      scalar_type len = 0;
      points_len_[0] = 0;

      for (size_t i = 1; i < points_.size(); ++i)
      {
         len += norm(points_[i] - points_[i - 1]);
         points_len_[i] = len;
      }
   }


   lin_spline_t(points_t const& points)
      : points_    (points)
      , points_len_(points.size())
   {
      Assert(points.size() > 1);

      scalar_type len = 0;
      points_len_[0] = 0;

      for (size_t i = 1; i < points.size(); ++i)
      {
         len += norm(points[i] - points[i - 1]);
         points_len_[i] = len;
      }
   }

   scalar_type length() const
   {
      return points_len_.back();
   }

   point_t interpolate(scalar_type len) const
   {
      scalar_type param;
      segment_t seg = len2segment(len, &param);

      return seg(param);
   }

   point_t direction(scalar_type len) const
   {
      return normalized(cg::direction(len2segment(len)));
   }

   scalar_type distance(point_t const& pos, scalar_type* len = NULL) const
   {
      scalar_type min_dist    = scalar_type(1e10);
      scalar_type closest_len = scalar_type();
      point_t     closest_point;

      for(size_t i = 0; i + 1 < points_.size(); ++i)
      {
         segment_2 seg(points_[i], points_[i+1]);

         double  cur_param = cg::bound(seg(pos), 0., 1.);
         point_2 cur_point = seg(cur_param);
         double  cur_dist  = norm(pos - cur_point);

         if (cur_dist < min_dist)
         {
            min_dist      = cur_dist;
            closest_point = cur_point;
            closest_len   = points_len_[i] + cur_param * cg::length(seg);
         }
      }

      if (len)
         *len = closest_len;

      return min_dist;
   }

   point_t closest_point(point_t const& pos, scalar_type* len) const
   {
      double closest_len;
      distance(pos, &closest_len);

      if (len)
         *len = closest_len;

      return interpolate(closest_len);
   }

   points_t const& points() const
   {
      return points_;
   }

   scalar_type len2param(scalar_type len) const
   {
      if (cg::le(len, 0))
         return 0;

      if (cg::ge(len, length()))
         return points_.size() - 1;

      lens_t::const_iterator it = upper_bound(points_len_.begin(), points_len_.end(), len);
      Assert(it != points_len_.end());

      size_t index1 = std::distance(points_len_.begin(), it);
      size_t index0 = index1 - 1;

      Assert(index1 > 0);

      scalar_type param = index0 + cg::lerp(points_len_[index0], points_len_[index1], 
         scalar_type(0), scalar_type(1))(len);

      Assert(!cg::eq(param, scalar_type(points_.size())));
      return param;
   }

   scalar_type param2len(scalar_type param) const
   {
      if (cg::le(param, 0))
         return 0;

      if (cg::ge(param, points_.size() - 1))
         return points_len_.back();

      size_t index = size_t(param);

      if (param == points_.size())
         return points_len_.back();

      return cg::lerp(scalar_type(0), scalar_type(1), points_len_[index], points_len_[index + 1])(param - index);
   }

private:
   segment_t len2segment(scalar_type len, scalar_type* param = NULL) const
   {
      scalar_type p = len2param(len);

      if (cg::ge(p, points_.size() - 1))
      {
         if (param)
            *param = 1;

         return segment_t(points_[points_.size() - 2], points_[points_.size() - 1]);
      }

      size_t index0 = size_t(p);
      size_t index1 = index0 + 1;

      point_t const& p0 = points_[index0];
      point_t const& p1 = points_[index1];

      if (param)
         *param = p - index0;

      return segment_t(p0, p1);
   }

private:
   typedef std::vector<scalar_type> lens_t;

   lens_t   points_len_;
   points_t points_;
};
} // namespace cg
