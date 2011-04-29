#pragma once

#include "common/assert.h"

//
template < class Interface, class Ptr >
   void ptrcpy( Interface ** pp, Ptr p ) ; 

//
template < class T > 
   T * CreateInstance ();
template<class T, typename P1>
   T * CreateInstance ( P1 const& p1 ); 
template<class T, typename P1, typename P2>
   T * CreateInstance ( P1 const& p1, P2 const& p2 ); 
template<class T, typename P1, typename P2, typename P3>
   T * CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3 ); 
template<class T, typename P1, typename P2, typename P3, typename P4>
   T * CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4 ); 
template<class T, typename P1, typename P2, typename P3, typename P4, typename P5>
   T * CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5 ); 
template<class T, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
   T * CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6 ) ;
template<class T, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
   T * CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6, P7 const& p7 ) ;

//
template<class T> struct com_impl_ptr
{
   com_impl_ptr();
   com_impl_ptr(T * p, bool add_ref = true);
   com_impl_ptr(com_impl_ptr const & rhs);
   template<class U> com_impl_ptr(U * p, bool add_ref = true);
   template<class U> com_impl_ptr(com_impl_ptr<U> const & rhs);
   template<class I> com_impl_ptr(_com_ptr_t< _com_IIID<I, &__uuidof(I)> > ptr);

   ~com_impl_ptr();

   com_impl_ptr & operator=(T * p);
   com_impl_ptr & operator=(com_impl_ptr const & rhs);
   template<class U> com_impl_ptr & operator=(U * rhs);
   template<class U> com_impl_ptr & operator=(com_impl_ptr<U> const & rhs);
   template<class I> com_impl_ptr & operator=(_com_ptr_t< _com_IIID<I, &__uuidof(I)> > ptr);

   void reset(T * rhs = 0);

   T * get() const;
   operator T * () const; 

   T * operator->() const;
   T & operator*() const;

   T ** operator &();

   IUnknown * get_unknown() const; 
   void swap(com_impl_ptr & rhs);

   void attach ( T * ) ; 
   T *  detach () ; 

//
   bool CreateInstance () ;
   template<typename P1>
      bool CreateInstance ( P1 const& p1 ); 
   template<typename P1, typename P2>
      bool CreateInstance ( P1 const& p1, P2 const& p2 ); 
   template<typename P1, typename P2, typename P3>
      bool CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3 ); 
   template<typename P1, typename P2, typename P3, typename P4>
      bool CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4 ); 
   template<typename P1, typename P2, typename P3, typename P4, typename P5>
      bool CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5 ); 
   template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
      bool CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6 ); 
   template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
      bool CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6, P7 const& p7 ); 

private:
   T * p_;

private:
   IUnknown * unknown_cast    () const ;
   void       internal_addref () ;
   void       internal_release() ;
};

template<class T> struct com_impl_obj : com_impl_ptr<T>
{
   com_impl_obj (); 
   template<typename P1>
      com_impl_obj ( P1 const& p1 ); 
   template<typename P1, typename P2>
      com_impl_obj ( P1 const& p1, P2 const& p2 ); 
   template<typename P1, typename P2, typename P3>
      com_impl_obj ( P1 const& p1, P2 const& p2, P3 const& p3 ); 
   template<typename P1, typename P2, typename P3, typename P4>
      com_impl_obj ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4 ); 
   template<typename P1, typename P2, typename P3, typename P4, typename P5>
      com_impl_obj ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5 ); 
   template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
      com_impl_obj ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6 ); 
   template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
      com_impl_obj ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6, P7 const& p7 ); 
} ; 


//////////////////////////////////////////////////////////////////////////////////////
// implementation 

#include "com_impl_ptr.inl"

template < class Interface, class Ptr >
   void ptrcpy( Interface ** pp, Ptr p )
   {
      *pp = static_cast<Interface *>(p) ; 
      if ( *pp ) 
         (*pp) -> AddRef () ; 
   } 


template < class T >
   T * CreateInstance ()
   {
      CComObject<T> * t ; 
      if ( CComObject<T> :: CreateInstance ( &t ) == S_OK ) 
      {
         __if_exists ( T::InitInstance )
         {
            if ( com_impl::instancer<T>::init(t, &T::InitInstance) ) 
               return t ; 
            IUnknownPtr destroyer(t) ; 
         }
         __if_not_exists ( T::InitInstance )
         {
            return t ; 
         }
      }
      return NULL ; 
   }  

template<class T, typename P1>
   T * CreateInstance ( P1 const& p1 ) 
   {
      CComObject<T> * t ; 
      if ( CComObject<T> :: CreateInstance ( &t ) == S_OK ) 
      {
         if ( com_impl::instancer<T>::init(t, &T::InitInstance, p1) ) 
            return t ; 
         IUnknownPtr destroyer(t) ; 
      }
      return NULL ; 
   }

template<class T, typename P1, typename P2>
   T * CreateInstance ( P1 const& p1, P2 const& p2 ) 
   {
      CComObject<T> * t ; 
      if ( CComObject<T> :: CreateInstance ( &t ) == S_OK ) 
      {
         if ( com_impl::instancer<T>::init(t, &T::InitInstance, p1, p2) ) 
            return t ; 
         IUnknownPtr destroyer(t) ; 
      }
      return NULL ; 
   }

template<class T, typename P1, typename P2, typename P3>
   T * CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3 ) 
   {
      CComObject<T> * t ; 
      if ( CComObject<T> :: CreateInstance ( &t ) == S_OK ) 
      {
         if ( com_impl::instancer<T>::init(t, &T::InitInstance, p1, p2, p3 ) )  
            return t ; 
         IUnknownPtr destroyer(t) ; 
      }
      return NULL ; 
   }

template<class T, typename P1, typename P2, typename P3, typename P4>
   T * CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4 ) 
   {
      CComObject<T> * t ; 
      if ( CComObject<T> :: CreateInstance ( &t ) == S_OK ) 
      {
         if ( com_impl::instancer<T>::init(t, &T::InitInstance, p1, p2, p3, p4 ) )  
            return t ; 
         IUnknownPtr destroyer(t) ; 
      }
      return NULL ; 
   }

template<class T, typename P1, typename P2, typename P3, typename P4, typename P5>
   T * CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5 ) 
   {
      CComObject<T> * t ; 
      if ( CComObject<T> :: CreateInstance ( &t ) == S_OK ) 
      {
         if ( com_impl::instancer<T>::init(t, &T::InitInstance, p1, p2, p3, p4, p5 ) )  
            return t ; 
         IUnknownPtr destroyer(t) ; 
      }
      return NULL ; 
   }

template<class T, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
   T * CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6 ) 
   {
      CComObject<T> * t ; 
      if ( CComObject<T> :: CreateInstance ( &t ) == S_OK ) 
      {
         if ( com_impl::instancer<T>::init(t, &T::InitInstance, p1, p2, p3, p4, p5, p6 ) )  
            return t ; 
         IUnknownPtr destroyer(t) ; 
      }
      return NULL ; 
   }

template<class T, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
   T * CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6, P7 const& p7 ) 
   {
      CComObject<T> * t ; 
      if ( CComObject<T> :: CreateInstance ( &t ) == S_OK ) 
      {
         if ( com_impl::instancer<T>::init(t, &T::InitInstance, p1, p2, p3, p4, p5, p6, p7 ) )  
            return t ; 
         IUnknownPtr destroyer(t) ; 
      }
      return NULL ; 
   }

//
template<class T>
   com_impl_ptr<T>::com_impl_ptr(): p_(0)
   {
   }

template<class T>
   com_impl_ptr<T>::com_impl_ptr(T * p, bool add_ref = true): p_(p)
   {                                                        
      if(p_ != 0 && add_ref) internal_addref();             
   }

template<class T>
   com_impl_ptr<T>::com_impl_ptr(com_impl_ptr const & rhs): p_(rhs.p_)
   {
      if(p_ != 0) internal_addref();
   }

template<class T> template<class U> 
   com_impl_ptr<T>::com_impl_ptr(com_impl_ptr<U> const & rhs): p_(rhs.get())
   {
      if(p_ != 0) internal_addref();
   }

template<class T> template<class U> 
   com_impl_ptr<T>::com_impl_ptr(U * p, bool add_ref = true)
      : p_ ( static_cast<T *>(p) )
   {                                                        
      if(p_ != 0 && add_ref) internal_addref();             
   }

template<class T> template<class I> 
   com_impl_ptr<T>::com_impl_ptr( _com_ptr_t< _com_IIID<I, &__uuidof(I)> > ptr)
      : p_ ( static_cast<T *>(ptr.GetInterfacePtr()) )
   {
      if(p_ != 0) internal_addref();             
   }

//
template<class T>
   com_impl_ptr<T>::~com_impl_ptr()
   {
      if(p_ != 0) internal_release();
   }

//
template<class T>
   com_impl_ptr<T> & com_impl_ptr<T>::operator=(T * rhs)
   {
      com_impl_ptr<T>(rhs).swap(*this);
      return *this;
   }

template<class T>
   com_impl_ptr<T> & com_impl_ptr<T>::operator=(com_impl_ptr<T> const & rhs)
   {
      com_impl_ptr<T>(rhs).swap(*this);
      return *this;
   }

template<class T> template<class U> 
   com_impl_ptr<T> & com_impl_ptr<T>::operator=(com_impl_ptr<U> const & rhs)
   {
      com_impl_ptr<T>(rhs).swap(*this);
      return *this;
   }

template<class T>  template<class U> 
   com_impl_ptr<T> & com_impl_ptr<T>::operator=(U * rhs)
   {
      com_impl_ptr<T>(rhs).swap(*this);
      return *this;
   }

template<class T> template<class I> 
   com_impl_ptr<T> & com_impl_ptr<T>::operator=(_com_ptr_t< _com_IIID<I, &__uuidof(I)> > rhs)
   {
      com_impl_ptr<T>(rhs).swap(*this);
      return *this;
   }

//
template<class T>
   void com_impl_ptr<T>::reset(T * rhs = 0)
   {
      com_impl_ptr<T>(rhs).swap(*this);
   }

template<class T>
   T * com_impl_ptr<T>::get() const
   {
      return static_cast<T *>(p_);
   }

template<class T>
   com_impl_ptr<T>::operator T * () const 
   {
      return p_; 
   }

template<class T>
   T & com_impl_ptr<T>::operator*() const
   {
      Assert( p_ );
      return *p_;
   }

template<class T>
   T ** com_impl_ptr<T>::operator &()
   {
      if(p_ != 0) internal_release();
      p_ = 0 ; 

      return &p_ ; 
   }

template<class T>
   T * com_impl_ptr<T>::operator->() const
   {
      Assert( p_ );
      return p_;
   }

template<class T>
   IUnknown * com_impl_ptr<T>::get_unknown() const 
   {
      return unknown_cast() ; 
   }

template<class T>
   void com_impl_ptr<T>::swap(com_impl_ptr & rhs)
   {
      T * tmp = p_;
      p_ = rhs.p_;
      rhs.p_ = tmp;
   }

template<class T>
   void com_impl_ptr<T>::attach ( T * rhs) 
   {
      com_impl_ptr<T>(rhs, false).swap(*this);
   }

template<class T>
   T * com_impl_ptr<T>::detach () 
   {
      T * p = p_ ; 
      p_ = NULL ; 
      return p ; 
   }

//
template<class T>
   bool com_impl_ptr<T>::CreateInstance () 
   {
      reset ( ::CreateInstance<T>() ) ; 
      return p_ != NULL ; 
   }

template<class T> template<typename P1>
   bool com_impl_ptr<T>::CreateInstance ( P1 const& p1 ) 
   {
      reset ( ::CreateInstance<T>(p1) ) ; 
      return p_ != NULL ; 
   }

template<class T> template<typename P1, typename P2>
   bool com_impl_ptr<T>::CreateInstance ( P1 const& p1, P2 const& p2 ) 
   {
      reset ( ::CreateInstance<T>(p1, p2) ) ; 
      return p_ != NULL ; 
   }

template<class T> template<typename P1, typename P2, typename P3>
   bool com_impl_ptr<T>::CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3 ) 
   {
      reset ( ::CreateInstance<T>(p1, p2, p3) ) ; 
      return p_ != NULL ; 
   }

template<class T> template<typename P1, typename P2, typename P3, typename P4>
   bool com_impl_ptr<T>::CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4 ) 
   {
      reset ( ::CreateInstance<T>(p1, p2, p3, p4) ) ; 
      return p_ != NULL ; 
   }

template<class T> template<typename P1, typename P2, typename P3, typename P4, typename P5>
   bool com_impl_ptr<T>::CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5 ) 
   {
      reset ( ::CreateInstance<T>(p1, p2, p3, p4, p5) ) ; 
      return p_ != NULL ; 
   }

template<class T> template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
   bool com_impl_ptr<T>::CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6 ) 
   {
      reset ( ::CreateInstance<T>(p1, p2, p3, p4, p5, p6) ) ; 
      return p_ != NULL ; 
   }

template<class T> template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
   bool com_impl_ptr<T>::CreateInstance ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6, P7 const& p7 ) 
   {
      reset ( ::CreateInstance<T>(p1, p2, p3, p4, p5, p6, p7) ) ; 
      return p_ != NULL ; 
   }
//
template<class T>
   IUnknown * com_impl_ptr<T>::unknown_cast () const 
   {
      __if_exists ( T::GetUnknown )
      {
         return p_ -> GetUnknown() ; 
      }
      __if_not_exists ( T::GetUnknown )
      {
         return p_ ; 
      }
   }

template<class T>
   void com_impl_ptr<T>::internal_addref()
   {
      unknown_cast () -> AddRef () ; 
   }


template<class T>
   void com_impl_ptr<T>::internal_release()
   {
      unknown_cast () -> Release () ; 
   }

//
template<class T>
   com_impl_obj<T>::com_impl_obj () 
   {
      CreateInstance() ; 
   }

template<class T> template<typename P1>
   com_impl_obj<T>::com_impl_obj ( P1 const& p1 ) 
   {
      CreateInstance(p1) ; 
   }

template<class T> template<typename P1, typename P2>
   com_impl_obj<T>::com_impl_obj ( P1 const& p1, P2 const& p2 ) 
   {
      CreateInstance(p1, p2) ; 
   }

template<class T> template<typename P1, typename P2, typename P3>
   com_impl_obj<T>::com_impl_obj ( P1 const& p1, P2 const& p2, P3 const& p3 ) 
   {
      CreateInstance(p1, p2, p3) ; 
   }

template<class T> template<typename P1, typename P2, typename P3, typename P4>
   com_impl_obj<T>::com_impl_obj ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4 ) 
   {
      CreateInstance(p1, p2, p3, p4) ; 
   }

template<class T> template<typename P1, typename P2, typename P3, typename P4, typename P5>
   com_impl_obj<T>::com_impl_obj ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5 ) 
   {
      CreateInstance(p1, p2, p3, p4, p5) ; 
   }

template<class T> template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
   com_impl_obj<T>::com_impl_obj ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6 ) 
   {
      CreateInstance(p1, p2, p3, p4, p5, p6) ; 
   }

template<class T> template<typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
   com_impl_obj<T>::com_impl_obj ( P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6, P7 const& p7 ) 
   {
      CreateInstance(p1, p2, p3, p4, p5, p6, p7) ; 
   }

