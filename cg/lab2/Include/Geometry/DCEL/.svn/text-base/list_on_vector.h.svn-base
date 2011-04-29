#pragma once

#include <vector>

namespace cg
{

template< class value_type >
   struct list_on_vector
{
   typedef list_on_vector< value_type > self_type;

   typedef std::vector< value_type >            container_type;
   typedef std::vector< int >                   Connectivity;

   typedef typename container_type::size_type   size_type;
   typedef typename container_type::value_type  value_type;

// Iterators
public:
   class const_iterator
   {
   public:
      typedef value_type value_type;

      const_iterator ()
         : next_ (NULL), prev_ (NULL), val_ (NULL), idx_ (static_cast< size_t >( -1 ))
      {
      }

      const_iterator ( container_type const *val, Connectivity const *next, Connectivity const *prev, size_t idx = -1 )
         : val_ ((container_type *)val), next_ ((Connectivity *)next), prev_ ((Connectivity *)prev), idx_ (idx)
      {
      }

      value_type const &operator * () const
      {
         return (*val_)[idx_];
      }

      value_type const *operator -> () const
      {
         return (&**this);
      }

      const_iterator &operator ++ ()
      {
         idx_ = (*next_)[idx_];
         return (*this);
      }

      const_iterator operator ++ ( int )
      {
         const_iterator temp = *this;
         ++*this;
         return temp;
      }

      const_iterator &operator -- ()
      {
         idx_ = (*prev_)[idx_];
         return (*this);
      }

      const_iterator operator -- ( int )
      {
         const_iterator temp = *this;
         --*this;
         return temp;
      }

      bool operator == ( const_iterator const &right ) const
      {
         return val_ == right.val_ && next_ == right.next_ &&
            prev_ == right.prev_ && idx_ == right.idx_;
      }

      bool operator != ( const_iterator const &right ) const
      {
         return !(*this == right);
      }

      size_t index() const
      {
         return idx_;
      }

   protected:
      size_t idx_;
      Connectivity *next_;
      Connectivity *prev_;
      container_type *val_;
   };

   class iterator : public const_iterator
   {
   public:
      typedef value_type value_type;

      iterator ()
      {
      }

      iterator ( container_type *val, Connectivity *next, Connectivity *prev, size_t idx = -1 )
         : const_iterator (val, next, prev, idx)
      {
      }

      value_type &operator * () const
      {
         return ((value_type &)**(const_iterator *)this);
      }

      value_type *operator -> () const
      {
         return (&**this);
      }

      iterator &operator ++ ()
      {
         idx_ = (*next_)[idx_];
         return (*this);
      }

      iterator operator ++ ( int )
      {
         iterator temp = *this;
         ++*this;
         return temp;
      }

      iterator &operator -- ()
      {
         idx_ = (*prev_)[idx_];
         return (*this);
      }

      iterator operator -- ( int )
      {
         iterator temp = *this;
         --*this;
         return temp;
      }
   };

public:
   list_on_vector () : firstValid_ (-1), firstEmpty_ (-1), numValid_ (0)
   {
   }

   void clear()
   {
      v_.clear();
      next_.clear();
      prev_.clear();
      firstValid_ = -1;
      firstEmpty_ = -1;
      numValid_ = 0;
   }

   //void resize( size_type newSize )
   //{
   //   container_type::size_type curSize = size();

   //   container_type::resize(newSize);
   //   next_.resize(newSize);
   //   prev_.resize(newSize);

   //   // All new elements are supposed to be empty
   //   for (size_t i = curSize; i < newSize; ++i)
   //   {
   //      next_[i] = firstEmpty_;
   //      if (firstEmpty_ != -1)
   //         prev_[firstEmpty_] = i;
   //      prev_[i] = -1;
   //      firstEmpty_ = i;
   //   }
   //}

   iterator begin()
   {
      return iterator (&v_, &next_, &prev_, firstValid_);
   }

   const_iterator begin() const
   {
      return const_iterator (&v_, &next_, &prev_, firstValid_);
   }

   iterator end()
   {
      return iterator (&v_, &next_, &prev_, static_cast< size_t >( -1 ));
   }

   const_iterator end() const
   {
      return const_iterator (&v_, &next_, &prev_, static_cast< size_t >( -1 ));
   }

// Specific list methods
public:
   size_t size() const
   {
      return numValid_;
   }

   size_t containerSize() const
   {
      return v_.size();
   }

   int head() const
   {
      return firstValid_;
   }

   int next( size_type pos ) const
   {
      return next_.at(pos);
   }

   int prev( size_type pos ) const
   {
      return prev_.at(pos);
   }

   int push( value_type const &val )
   {
      if (firstEmpty_ == -1)      
      {
         v_.push_back(val);
         next_.push_back(-1);
         prev_.push_back(-1);
         firstEmpty_ = (int)v_.size() - 1;
      }

      int idx = firstEmpty_;
      firstEmpty_ = next_[idx];
      if (firstEmpty_ != -1)
         prev_[firstEmpty_] = -1;

      next_[idx] = firstValid_;
      if (firstValid_ != -1)
         prev_[firstValid_] = idx;
      firstValid_ = idx;

      operator[](idx) = val;
      numValid_++;
      return idx;
   }

   void remove( size_type pos )
   {
      int prev = prev_[pos];
      int next = next_[pos];

      if (prev != -1)
         next_[prev] = next;
      if (next != -1)
         prev_[next] = prev;

      if (static_cast< int >( pos ) == firstValid_)
         firstValid_ = next;

      if (firstEmpty_ != -1)
         prev_[firstEmpty_] = pos;

      next_[pos] = firstEmpty_;
      firstEmpty_ = pos;

      numValid_--;
   }

   value_type const & operator []( size_t pos ) const
   {
      return v_[pos];
   }

   value_type & operator []( size_t pos )
   {
      return v_[pos];
   }

   void reserve( size_t size )
   {
      v_.reserve(size);
      next_.reserve(size);
      prev_.reserve(size);
   }

   iterator push( self_type const &list )
   {
      if (list.numValid_ <= 0)
         return end();

      size_t initialSize = v_.size();

      v_.insert(v_.end(), list.v_.begin(), list.v_.end());
      next_.insert(next_.end(), list.next_.begin(), list.next_.end());
      prev_.insert(prev_.end(), list.prev_.begin(), list.prev_.end());

      for (size_t v = initialSize; v < v_.size(); ++v)
      {
         if (next_[v] != -1)
            next_[v] += initialSize;
         if (prev_[v] != -1)
            prev_[v] += initialSize;
      }

      if (numValid_ == 0)
         firstValid_ = (list.firstValid_ == -1) ? (-1) : (list.firstValid_ + initialSize);
      else
      {
         int lastElem = firstValid_;
         while (next_[lastElem] != -1)
            lastElem = next_[lastElem];

         next_[lastElem] = list.firstValid_ + initialSize;
         prev_[next_[lastElem]] = lastElem;
      }

      if (numValid_ == initialSize)
         firstEmpty_ = (list.firstEmpty_ == -1) ? (-1) : (list.firstEmpty_ + initialSize);
      else
      {
         int lastElem = firstEmpty_;
         while (next_[lastElem] != -1)
            lastElem = next_[lastElem];
         
         next_[lastElem] = list.firstEmpty_ + initialSize;
         prev_[next_[lastElem]] = lastElem;
      }

      numValid_ += list.numValid_;
      return iterator (&v_, &next_, &prev_, list.firstValid_ + initialSize);
   }

   template< class Stream >
      void dump( Stream &stream )
   {
      stream << firstValid_ << ' ';
      stream << firstEmpty_ << ' ';
      stream << numValid_ << ' ';

      stream << next_.size() << ' ';
      for (size_t i = 0; i < next_.size(); ++i)
         stream << next_[i] << ' ';

      stream << prev_.size() << ' ';
      for (size_t i = 0; i < prev_.size(); ++i)
         stream << prev_[i] << ' ';

      stream << v_.size() << ' ';
      for (size_t i = 0; i < v_.size(); ++i)
      {
         v_[i].dump(stream);
         stream << ' ';
      }
   }

   template< class Stream >
      void restore( Stream &stream )
   {
      stream >> firstValid_;
      stream >> firstEmpty_;
      stream >> numValid_;

      size_t nextSize;
      stream >> nextSize;
      next_.reserve(nextSize);
      for (size_t i = 0; i < nextSize; ++i)
      {
         int curVal;
         stream >> curVal;
         next_.push_back(curVal);
      }

      size_t prevSize;
      stream >> prevSize;
      prev_.reserve(prevSize);
      for (size_t i = 0; i < prevSize; ++i)
      {
         int curVal;
         stream >> curVal;
         prev_.push_back(curVal);
      }

      size_t vSize;
      stream >> vSize;
      v_.reserve(vSize);
      for (size_t i = 0; i < vSize; ++i)
      {
         value_type val;
         val.restore(stream);
         v_.push_back(val);
      }
   }

// Private data
private:
   Connectivity next_;
   Connectivity prev_;

   int firstValid_;
   int firstEmpty_;

   size_t numValid_;

   std::vector< value_type > v_;
};

} // End of 'cg' namespace
