#pragma once

#include <boost/intrusive_ptr.hpp>
#include <hash_map>

namespace cg
{

namespace details
{
   template <class Scalar, class SegmentId>
      struct interval_tree_t
   {
      typedef Scalar scalar_t;
      typedef SegmentId segment_id;

      interval_tree_t(scalar_t const & pos)
         : pos_(pos)
      {}

      void add_segment(scalar_t const & a, scalar_t const & b, segment_id id)
      {
         Assert(a <= b);

         if (a <= pos_ && pos_ <= b)
         {
            starts_.insert(std::make_pair(a, id));
            ends_.insert(std::make_pair(b, id));
         }
         else if (b < pos_) // to the left
         {
            if (!left_)
               left_.reset(new interval_tree_t((a + b) / 2));

            left_->add_segment(a, b, id);
         }
         else if (a > pos_) // to the right
         {
            if (!right_)
               right_.reset(new interval_tree_t((a + b) / 2));

            right_->add_segment(a, b, id);
         }
      }

      bool visit_segments(scalar_t const & t, boost::function<bool (segment_id)> visitor) const
      {
         typedef std::multimap<scalar_t, segment_id> tails_t;

         if (t == pos_)
         {
            for each (std::pair<scalar_t, segment_id> const & p in starts_)
               if (!visitor(p.second))
                  return false;
         }
         else if (t < pos_)
         {
            for (tails_t::const_iterator it = starts_.begin(), end = starts_.upper_bound(t); it != end; ++it)
               if (!visitor(it->second))
                  return false;

            if (left_)
               return left_->visit_segments(t, visitor);
         }
         else // t > pos_
         {
            for (tails_t::const_iterator it = ends_.lower_bound(t), end = ends_.end(); it != end; ++it)
               if (!visitor(it->second))
                  return false;

            if (right_)
               return right_->visit_segments(t, visitor);
         }

         return false;
      }

      bool visit_nodes(boost::function<bool (size_t, scalar_t, scalar_t, size_t)> visitor) const
      {
         if (left_)
            if (!left_->visit_nodes(visitor, pos_, 1))
               return false;

         if (!visitor(starts_.size(), pos_, pos_, 0))
            return false;

         if (right_)
            if (!right_->visit_nodes(visitor, pos_, 1))
               return false;

         return true;
      }

   private:
      bool visit_nodes(boost::function<bool (size_t, scalar_t, scalar_t, size_t)> visitor, scalar_t parent_pos, size_t h) const
      {
         if (left_)
            if (!left_->visit_nodes(visitor, pos_, h + 1))
               return false;

         if (!visitor(starts_.size(), pos_, parent_pos, h))
            return false;

         if (right_)
            if (!right_->visit_nodes(visitor, pos_, h + 1))
               return false;

         return true;
      }

   private:
      DECLARE_INTRUSIVE_COUNTER(interval_tree_t);

      typedef boost::intrusive_ptr<interval_tree_t> interval_tree_ptr;
   private:
      interval_tree_ptr left_;
      interval_tree_ptr right_;

      scalar_t pos_;

      std::multimap<scalar_t, segment_id> starts_;
      std::multimap<scalar_t, segment_id> ends_;
   };
}

// TODO :: add dimension
template <typename Scalar = double, typename SegmentId = size_t>
   struct grid_interval_tree_t
{
   typedef Scalar scalar_t;
   typedef SegmentId segment_id;

   grid_interval_tree_t(scalar_t bucket_width)
      : bucket_width_(bucket_width)
   {}

   bool visit_segments(scalar_t const & time, boost::function<bool (segment_id)> visitor) const
   {
      int bucket_id = (int)(time / bucket_width_);
      if (time < 0) bucket_id--;

      buckets_t::const_iterator it = buckets_.find(bucket_id);
      if (it == buckets_.end())
         return false;

      return it->second->visit_segments(time, visitor);
   }

   void add_segment(scalar_t const & a, scalar_t const & b, segment_id id)
   {
      Assert(a <= b);

      int a_id = (int)(a / bucket_width_);
      int b_id = (int)(b / bucket_width_);

      if (a < 0) --a_id;
      if (b < 0) --b_id;

      Assert(a_id <= b_id);

      for (; a_id <= b_id; ++a_id)
      {
         buckets_t::const_iterator it = buckets_.find(a_id);
         if (it == buckets_.end())
         {
            buckets_[a_id].reset(new details::interval_tree_t<scalar_t, segment_id>((a_id + 0.5) * bucket_width_));
            it = buckets_.find(a_id);
         }

         it->second->add_segment(cg::max(a, a_id * bucket_width_), cg::min(b, (a_id + 1) * bucket_width_), id);
      }
   }

   bool visit_nodes(boost::function<bool (size_t, scalar_t, scalar_t, size_t)> visitor) const
   {
      for each (std::pair<const int, interval_tree_ptr> const & p in buckets_)
         if (!p.second->visit_nodes(visitor))
            return false;

      return !buckets_.empty();
   }

private:
   scalar_t bucket_width_;

   typedef boost::intrusive_ptr<details::interval_tree_t<scalar_t, segment_id>> interval_tree_ptr;
   typedef stdext::hash_map<int, interval_tree_ptr> buckets_t;
   buckets_t buckets_;
};

}

