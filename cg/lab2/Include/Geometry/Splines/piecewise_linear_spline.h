#pragma once

#include "Geometry/primitives.h"
#include "Geometry/Smoothing/AngleSmoothing.h"
#include "Geometry/Splines/1DSplines.h"

namespace cg {
namespace Splines {

namespace details 
{
   template<class T>
   struct iterator_2d : std::iterator<std::bidirectional_iterator_tag, T>
   {
      iterator_2d(typename std::vector<std::vector<T>> const & vec,
                  typename std::vector<std::vector<T>>::const_iterator iter_1,
                  typename std::vector<T>::const_iterator iter_2)
         : iter_1_(iter_1)
         , iter_2_(iter_2)
         , begin_(vec.begin())
         , end_(vec.end())
      {}

   private:
      void inc()
      {
         ++iter_2_;
         if (iter_2_ == iter_1_->end())
         {
            ++iter_1_;
            if (iter_1_ != end_)
               iter_2_ = iter_1_->begin();
         }
      }

      void dec()
      {
         if (iter_2_ == iter_1_->begin())
         {
            if (iter_1_ == begin_)
               return;
            
            --iter_1_;
            iter_2_ = iter_1_->end();
         }
         --iter_2_;
      }

   public:
      iterator_2d & operator ++ ()
      {
         typename std::vector<T>::const_iterator last = iter_2_;
         while ((iter_1_ != end_) && (*last == *iter_2_))
            inc();

         return *this;
      }

      iterator_2d operator++ (int)
      {
         iterator_2d temp = *this;
         ++*this;
         return temp;
      }

      iterator_2d & operator -- ()
      {
         typename std::vector<T>::const_iterator last = iter_2_;
         while (!((iter_1_ == begin_) && (iter_2_ == iter_1_->begin())) && (*last == *iter_2_))
            dec();

         return *this;
      }

      iterator_2d operator-- (int)
      {
         iterator_2d temp = *this;
         --*this;
         return temp;
      }

      T const & operator * () const
      {
         return *iter_2_;
      }

      T const * operator -> () const
      {
         return &*iter_2_;
      }

      bool operator != (iterator_2d<T> const & other)
      {
         return (iter_1_ != other.iter_1_) || (iter_2_ != other.iter_2_);
      }

   private:
      typename std::vector<std::vector<T>>::const_iterator iter_1_, begin_, end_;
      typename std::vector<T>::const_iterator iter_2_;
   };
}

struct piecewise_linear_spline
{
   struct knot_t
   {
      cg::point_2 pos;
      double angle;
      double radius;

      bool fixed;

      knot_t(cg::point_2 const & pos = cg::point_2(), double angle = 19.0, double radius = 1e10, bool fixed = false)
         : pos( pos )
         , angle( angle )
         , radius( radius )
         , fixed( fixed )
      {}
   };

   piecewise_linear_spline(double first_y, double last_y, double period, bool fixed_period)
      : multiplier_(1.0)
      , fixed_period_(fixed_period)
   {
      knots_.push_back(knot_t(cg::point_2(0, first_y)));
      knots_.push_back(knot_t(cg::point_2(period, last_y)));
      for (size_t i = 0; i != 2; ++i)
         points_.push_back(std::vector<cg::point_2>(1, knot(i).pos));
   }

   bool is_period_fixed() const { return fixed_period_; }

   double get_period() const { return knots_.back().pos.x - knots_.front().pos.x; }

   void set_period(double new_period)
   {
      Assert(!fixed_period_);
      Assert(new_period > 0);

      double old_period = get_period();
      
      points_.back()[0].x = new_period;
      knot(knots_num() - 1).pos.x = new_period;

      for (size_t i = 1; i + 1 != knots_num(); ++i)
      {
         knot(i).pos.x *= new_period / old_period;
      }

      for (size_t i = 1; i + 1 != knots_num(); ++i)
         rebuild_points(i);
   }

   size_t knots_num() const { return knots_.size(); }
   knot_t const   & knot(size_t idx) const   { return knots_[idx]; }
   knot_t         & knot(size_t idx)         { return knots_[idx]; }

   optional<size_t> index(cg::point_2 const & pos, double eps) const
   {
      for (size_t i = 1; i != knots_num() - 1; ++i)
         if (cg::distance(knot(i).pos, pos) < eps)
            return i;

      return boost::none;
   }

   optional<size_t> insert_knot(cg::point_2 const & pos)
   {
      knot_t knot(pos);
      const size_t idx = std::distance(knots_.begin(), std::upper_bound(knots_.begin(), knots_.end(), knot));
      if (idx > 0 && idx <= knots_num() - 1)
      {
         knots_.insert(knots_.begin() + idx, knot);
         points_.insert(points_.begin() + idx, std::vector<cg::point_2>());
         rebuild_points(idx - 1);
         rebuild_points(idx);
         rebuild_points(idx + 1);
         return idx;
      }
      else
         return boost::none;
   }

   void remove_knot(size_t idx)
   {
      knots_.erase(knots_.begin() + idx);
      points_.erase(points_.begin() + idx);
      rebuild_points(idx - 1);
      rebuild_points(idx);
   }

   void move_knot(size_t idx, cg::point_2 const & pos)
   {
      if ((pos.x <= knot(idx - 1).pos.x) || (pos.x >= knot(idx + 1).pos.x))
         return;

      knot(idx).pos = pos;
      rebuild_points(idx - 1);
      rebuild_points(idx);
      rebuild_points(idx + 1);
   }

   void set_multiplier(double multiplier)
   {
      multiplier_ = multiplier;
      for (size_t i = 1; i != knots_num() - 1; ++i)
         rebuild_points(i);
   }

   double get_multiplier() const { return multiplier_; }

   typedef details::iterator_2d<cg::point_2> points_iterator;

   points_iterator points_begin() const 
   {
      return points_iterator(points_, points_.begin(), points_.front().begin());
   }

   points_iterator points_end() const
   {
      return points_iterator(points_, points_.end(), points_.back().end());
   }

   void rebuild_points(size_t idx)
   {
      if ((idx == 0) || (idx == knots_num() - 1))
         return;

      points_[idx].clear();
      if (knot(idx).fixed)
         points_[idx].push_back(knot(idx).pos);
      else
         cg::AngleSmoothing::smooth_angle(knot(idx - 1).pos, knot(idx).pos, knot(idx + 1).pos, 
                                          knot(idx).radius, multiplier_ * cg::grad2rad(knot(idx).angle),
                                          std::back_inserter(points_[idx]));
   }

   double Interpolate(double x) const
   {
      x -= get_period() * cg::floor(x / get_period());
      points_iterator left = std::lower_bound(points_begin(), points_end(), point_2(x, 0)), right = left--;
      return cg::Clamp<double>(left->x, right->x, left->y, right->y)(x);
   }

private:
   std::vector<knot_t> knots_;
   std::vector<std::vector<cg::point_2>> points_;
   double multiplier_;
   bool fixed_period_;
};

inline bool operator < (piecewise_linear_spline::knot_t const & a, piecewise_linear_spline::knot_t const & b)
{
   return a.pos.x < b.pos.x;
}

inline bool operator == (piecewise_linear_spline::knot_t const & a, piecewise_linear_spline::knot_t const & b)
{
   return (a.pos == b.pos) && (a.angle == b.angle) && (a.fixed == b.fixed) && (a.radius == b.radius);
}

inline bool operator != (piecewise_linear_spline::knot_t const & a, piecewise_linear_spline::knot_t const & b)
{
   return !(a == b);
}

inline bool operator == (piecewise_linear_spline const & a, piecewise_linear_spline const & b)
{
   if ((a.get_multiplier() != b.get_multiplier()) || (a.is_period_fixed() != b.is_period_fixed())
         || (a.knots_num() != b.knots_num()))
      return false;
   for (size_t i = 0; i != a.knots_num(); ++i)
   {
      if (a.knot(i) != b.knot(i))
         return false;
   }
   
   return true;
}

inline bool operator != (piecewise_linear_spline const & a, piecewise_linear_spline const & b)
{
   return !(a == b);
}

struct subdivided_spline : Spline1D<double, cg::Splines::Constructors::BicubicHermite>
{
   typedef Spline1D<double, cg::Splines::Constructors::BicubicHermite> base_t;
   
   typedef base_t::KnotPoint           knot_t;
   typedef base_t::KnotPoints          knots_t;
   typedef base_t::PointDerivatives    derivatives_t;
   
   explicit subdivided_spline(bool is_period_fixed)
      : is_period_fixed_(is_period_fixed)
   {}

   double subdivision_angle() const { return subdivision_angle_; }

   void subdivide(double angle)
   {
      subdivision_angle_ = angle;

      pts_.clear();
      pts_.push_back(from_knot(knots().front()));

      for (size_t i = 1; i + 1 != knots().size(); ++i)
      {
         point_2 knot = from_knot(knots()[i]);

         if (is_acute(pts_.back(), knot, from_knot(knots()[i + 1])))
         {
            for (size_t num_generated_points = 1; ; num_generated_points *= 2)
            {
               std::vector<point_2> gen_points(2 * num_generated_points + 1);
               
               cg::Clamp<double> left( 0.0, num_generated_points * 1.0, 
                                       cg::blend(knots()[i - 1].param, knot.x, 1.0 / 3), knot.x);
               cg::Clamp<double> right(0.0, num_generated_points * 1.0,
                                       cg::blend(knots()[i + 1].param, knot.x, 1.0 / 3), knot.x);
               
               for (size_t j = 0; j != num_generated_points; ++j)
               {
                  gen_points[j].x                             = left(j * 1.0);
                  gen_points[2 * num_generated_points - j].x  = right(j * 1.0);
               }
               gen_points[num_generated_points].x = knot.x;
               for (std::vector<point_2>::iterator it = gen_points.begin(); it != gen_points.end(); ++it)
                  it->y = Interpolate(it->x);
               
               bool to_continue = false;
               for (size_t j = 1; j != 2 * num_generated_points; ++j)
               {
                  to_continue = is_acute(gen_points[j - 1], gen_points[j], gen_points[j + 1]);
                  if (!to_continue)
                     break;
               }
               if (!to_continue)
                  to_continue = is_acute(pts_.back(), gen_points[0], gen_points[1]);
               if (!to_continue)
                  to_continue = is_acute(gen_points[gen_points.size() - 2], gen_points.back(), from_knot(knots()[i + 1]));

               if (!to_continue)
               {
                  pts_.insert(pts_.end(), gen_points.begin(), gen_points.end());
                  break;
               }
            }
         }
         else
         {
            pts_.push_back(knot);
         }
      }
      
      pts_.push_back(from_knot(knots().back()));
   }

   typedef std::vector<point_2>::const_iterator points_iterator;

   points_iterator points_begin()   const { return pts_.begin();  }
   points_iterator points_end()     const { return pts_.end();    }

   double get_period() const 
   { 
      Assert(knots().size() >= 2);
      return knots().back().param - knots().front().param;
   }

   bool is_period_fixed() const { return is_period_fixed_; }

private:

   point_2 from_knot(knot_t const & knot) const { return point_2(knot.param, knot.value); }

   bool is_acute(point_2 const & p1, point_2 const & p2, point_2 const & p3) const
   {
      double a = cg::angle(p2 - p1, p3 - p2);
      if (a > cg::pi)
         a = 2 * cg::pi - a;

      return  a > subdivision_angle();
   }

private:
   std::vector<point_2> pts_;
   double subdivision_angle_;
   bool is_period_fixed_;
};

inline bool operator == (subdivided_spline const & a, subdivided_spline const & b)
{
   return (a.subdivision_angle() == b.subdivision_angle()) && (a.is_period_fixed() == b.is_period_fixed()) &&
      (static_cast<subdivided_spline::base_t const &>(a) == static_cast<subdivided_spline::base_t const &>(b));
}

inline bool operator != (subdivided_spline const & a, subdivided_spline const & b)
{
   return !(a == b);
}

}}
