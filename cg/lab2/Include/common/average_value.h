#pragma once

#include <boost/circular_buffer.hpp>

template<typename T>
struct average_value
{
private:
   boost::circular_buffer<T> values_;
   T sum_, average_;

public:
   average_value( size_t steps = 1 )
      : values_(steps)
      , sum_   (T())
   {
   }

   void resize( size_t steps )
   {
      values_.initialize_buffer(steps);
   }

   void add( T const & value )
   {
      if (values_.full())
      {
         sum_ -= values_.back();
         values_.pop_back();
      }
      values_.push_front(value);
      sum_ += value;
   }

   T get() const
   {
      return values_.empty() ? T() : sum_ / values_.size();
   }

   T last() const
   {
      return values_.empty() ? T() : values_.front();
   }
};
