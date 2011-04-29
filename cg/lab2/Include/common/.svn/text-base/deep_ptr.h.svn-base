#pragma once
#include "Assert.h"
#include "safe_bool.h"
#include <utility>


struct SimpleTypePolicy
{
  template< class T > static T* Clone( const T* p )
  {
    return new T( *p );
  }

  template< class T > static bool IsEqual( const T& p1, const T& p2 )
  {
    return p1 == p2;
  }
};


template <class T, class Policy = SimpleTypePolicy >
struct deep_ptr
{
  typedef T   value_type;

  deep_ptr( ) : p_ (0) {}
  explicit deep_ptr(T *p) : p_(p) {}

  void swap( deep_ptr & rhs )
  {
    std::swap( p_, rhs.p_ );
  }

  const deep_ptr& operator = (const deep_ptr &rhs) 
  {
    deep_ptr tmp( rhs );
    this->swap( tmp );
    return *this;
  }

  const deep_ptr& operator = (T * p ) 
  {
    deep_ptr tmp( p );
    this->swap( tmp );
    return *this;
  }

  deep_ptr(const deep_ptr & rhs)
  {
    if ( rhs.p_ == 0 )
      p_ = 0;
    else
      p_ = Policy::Clone( rhs.p_ );
  }

  ~deep_ptr() {
    delete p_;
  }


  friend bool operator == ( const deep_ptr & lhs, const deep_ptr & rhs ) 
  {
    if ( lhs.empty( ) != rhs.empty( ) )
      return false;
    return lhs.empty( ) || Policy::IsEqual( *lhs.p_, *rhs.p_ );
  }

  void release() {
    delete p_;
    p_ = 0;
  }

  T * get() const { return p_; }

  bool empty() const { return p_ == 0; }

  SAFE_BOOL_OPERATOR(p_ != 0)

  T& operator * () { return *p_; }

  T const & operator * () const { return *p_; }

  T* operator -> () { return p_; }

  T const * operator -> () const { return p_; }


private:
  T	*p_;
};
