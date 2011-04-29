////////////////////////////////////////////////////////////////////////////////////////////////
// linked_ptr.h

#pragma once

#include "common\assert.h"
#include "common\safe_bool.h"
#include <utility>
#include <functional>

////////////////////////////////////////////////////////////////////////////////////////////////

namespace util 
{

////////////////////////////////////////////////////////////////////////////////////////////////
class linked
{
public:
   linked()
      : right_(0)
      , left_(0)
   {
   }

   linked(linked const& other)
      : right_(other.right_)
      , left_(&other)
   {
      this->insert_me();
   }


   linked const& operator=(linked const& other)
   {
      linked(other).swap(*this);
      return *this;
   }

   ~linked()
   {
      if(left_)
         left_->right_ = right_;
      if(right_)
         right_->left_ = left_;

      left_ = right_ = 0;
   }

public:
   bool is_unique() const
   {
      return !left_ && !right_;
   }

   void swap(linked& other)
   {
      bool l_neighbor = left_  == &other ;
      bool r_neighbor = right_ == &other ;

      std::swap( left_ , other.left_  ) ;
      std::swap( right_, other.right_ ) ;
      
      if (l_neighbor) 
      {
         right_      = &other;
         other.left_ = this;
      }
      else if (r_neighbor) 
      {
         left_        = &other;
         other.right_ = this;
      }

      this->insert_me();
      other.insert_me();
   }

private:
   void insert_me()
   {
      if(left_)
         left_->right_ = this;
      if(right_)
         right_->left_ = this;
   }

private:
   mutable linked const* left_;
   mutable linked const* right_;
};

////////////////////////////////////////////////////////////////////////////////////////////////

namespace detail
{
   struct static_cast_tag {};
   struct dynamic_cast_tag {};
   struct const_cast_tag {};

   template<class T, class U>
   inline
   T* do_cast(U* u, static_cast_tag)
   {
      return static_cast<T*>(u);
   }

   template<class T, class U>
   inline
   T* do_cast(U* u, dynamic_cast_tag)
   {
      return dynamic_cast<T*>(u);
   }

   template<class T, class U>
   inline
   T* do_cast(U* u, const_cast_tag)
   {
      return const_cast<T*>(u);
   }

} // namespace detail {
   
////////////////////////////////////////////////////////////////////////////////////////////////

#   define STATIC_CAST_TO_LINKED_CONST(p) static_cast<linked const&>(p)

////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
class linked_ptr : private util::linked
{
private:
   typedef linked_ptr self_t;

   template<class> friend class linked_ptr;

public:
   typedef T element_type;
   typedef T value_type;

public:
   linked_ptr()
      : p_(0)
   {
   }

   explicit linked_ptr(T* p)
      : p_(p)
   {
   }
   
   template<class U, class cast_tag>
   linked_ptr(U* p, cast_tag t)
      : p_(detail::do_cast<T>(p, t))
   {
   }

   template<class U>
   linked_ptr(linked_ptr<U> const& other)
      : linked(STATIC_CAST_TO_LINKED_CONST(other))
      , p_(other.get())
   {
   }

   linked_ptr(self_t const& other)
      : linked(STATIC_CAST_TO_LINKED_CONST(other))
      , p_(other.get())
   {
   }

   template<class U, class cast_tag>
   linked_ptr(linked_ptr<U> const& other, cast_tag t)
      : linked(STATIC_CAST_TO_LINKED_CONST(other))
      , p_(detail::do_cast<T>(other.get(), t))
   {
   }

   void reset(T* p = 0)
   {
      self_t(p).swap(*this);
   }

   template<class U>
   self_t& operator=(linked_ptr<U> const& other)
   {
      self_t(other).swap(*this);
      return *this;
   }

   self_t& operator=(self_t const& other)
   {
      if (this != &other)
         self_t(other).swap(*this);
      return *this;
   }

   ~linked_ptr()
   {
      enum { T_must_be_a_comlete_type = sizeof(T) };

      if(this->linked::is_unique())
         delete p_;

      p_ = 0;
   }

public:
   T* get() const
   {
      return p_;
   }

   T* operator->() const
   {
      Assert( p_ );
      return p_;
   }

   T& operator*() const
   {
      Assert( p_ );
      return *p_;
   }

   bool unique() const
   {
      return is_unique();
   }

   SAFE_BOOL_OPERATOR(p_ != NULL)

   void swap(self_t& other)
   {
      this->linked::swap(static_cast<linked&>(other));
      std::swap(p_, other.p_);
   }

private:
   T* p_;
};

template <class Stream, class T>
   void write(Stream & stream, linked_ptr<T> const & p)
   {
      T const * t = p.get();
      write(stream, bool(t != 0));
      if (t) 
         write(stream, *t);
   }

template <class Stream, class T>
   void read(Stream &s,  linked_ptr<T> & p)
   {
      bool b;
      read(s, b);

      if (b)
      {
         p.reset(new T());
         T &t = *p;
         read(s, t);
      }
   }




////////////////////////////////////////////////////////////////////////////////////////////////

#undef STATIC_CAST_TO_LINKED_CONST

////////////////////////////////////////////////////////////////////////////////////////////////

template<class T, class U>
inline linked_ptr<T> static_pointer_cast(linked_ptr<U> const& u)
{
   return linked_ptr<T>(u, detail::static_cast_tag());
}

template<class T, class U>
inline linked_ptr<T> dynamic_pointer_cast(linked_ptr<U> const& u)
{
   return linked_ptr<T>(u, detail::dynamic_cast_tag());
}

template<class T, class U>
inline linked_ptr<T> const_pointer_cast(linked_ptr<U> const& u)
{
   return linked_ptr<T>(u, detail::const_cast_tag());
}

// < ///////////////////////////////////////////////////////////////////////////////////////////
template<class T> inline bool operator<(linked_ptr<T> const & a, linked_ptr<T> const & b)
{
    return std::less<T *>()(a.get(), b.get());
}

// == ///////////////////////////////////////////////////////////////////////////////////////////
template<class T, class U> inline bool operator==(linked_ptr<T> const & a, linked_ptr<U> const & b)
{
   return a.get() == b.get();
}

template<class T, class U> inline bool operator==(U * a, linked_ptr<T> const & b)
{
   return a == b.get();
}

template<class T, class U> inline bool operator==(linked_ptr<T> const & a, U * b)
{
   return a.get() == b;
}

// != ///////////////////////////////////////////////////////////////////////////////////////////
template<class T, class U> inline bool operator!=(linked_ptr<T> const & a, linked_ptr<U> const & b)
{
   return !(a == b);
}

template<class T, class U> inline bool operator!=(U const * a, linked_ptr<T> const & b)
{
   return !(a == b);
}

template<class T, class U> inline bool operator!=(linked_ptr<T> const & a, U * b)
{
   return !(a == b);
}

////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
inline T* get_pointer(linked_ptr<T> const& p)
{
   return p.get();
}

////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
class linked_array : private util::linked
{
private:
   typedef linked_array self_t;

public:
   typedef T element_type;
   typedef T value_type;

public:
   linked_array()
      : p_(0)
   {
   }

   explicit linked_array(T* p)
      : p_(p)
   {
   }
   
   void reset(T* p)
   {
      self_t(p).swap(*this);
   }

   self_t& operator=(T* p)
   {
      self_t(p).swap(*this);
      return *this;
   }

   self_t& operator=(self_t const& other)
   {
      self_t(other).swap(*this);
      return *this;
   }

   ~linked_array()
   {
      enum { T_must_be_a_comlete_type = sizeof(T) };

      if(this->linked::is_unique())
         delete [] p_;

      p_ = 0;
   }

public:
   T* get() const
   {
      return p_;
   }

   T* operator->() const
   {
      Assert( p_ );
      return p_;
   }

   T & operator[](size_t idx) const
   {
      Assert( p_ );
      return p_[idx];
   }

   T& operator*() const
   {
      Assert( p_ );
      return *p_;
   }

   SAFE_BOOL_OPERATOR(p_ != NULL)

   void swap(self_t& other)
   {
      this->linked::swap(static_cast<linked&>(other));
      std::swap(p_, other.p_);
   }

private:
   T* p_;
};

// < ///////////////////////////////////////////////////////////////////////////////////////////
template<class T> inline bool operator<(linked_array<T> const & a, linked_array<T> const & b)
{
   return std::less<T *>()(a.get(), b.get());
}

// == ///////////////////////////////////////////////////////////////////////////////////////////
template<class T> inline bool operator==(linked_array<T> const & a, linked_array<T> const & b)
{
   return std::equal_to<T *>()(a.get(), b.get());
}

template<class T> inline bool operator==(T * a, linked_array<T> const & b)
{
   return std::equal_to<T *>()(a, b.get());
}

template<class T> inline bool operator==(linked_array<T> const & a, T * b)
{
   return b == a;
}

// != ///////////////////////////////////////////////////////////////////////////////////////////
template<class T> inline bool operator!=(linked_array<T> const & a, linked_array<T> const & b)
{
   return !(a == b);
}

template<class T> inline bool operator!=(T * a, linked_array<T> const & b)
{
   return !(a == b);
}

template<class T> inline bool operator!=(linked_array<T> const & a, T * b)
{
   return !(a == b);
}

////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace util 

////////////////////////////////////////////////////////////////////////////////////////////////
