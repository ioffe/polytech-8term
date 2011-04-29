#pragma once
#include "safe_bool.h"

// to be substituted by boost::scoped_ptr
template <class T>
struct m_ptr
{
   typedef T element_type ;
   typedef T value_type   ;

     m_ptr(T *p = 0) : p_(p) {}

     void operator = (T *p) {
          if (p != p_)
               release();
          p_ = p;
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

     void swap( m_ptr & rhs )
     {
          T* tmp = rhs.p_;
          rhs.p_ = p_;
          p_     = tmp;
     }

     T* operator -> () { return p_; }

     T const * operator -> () const { return p_; }

     ~m_ptr() {
          delete p_;
     }

private:
     T    *p_;

     m_ptr(const m_ptr&);
     void operator = (m_ptr const &);
};


namespace streams 
{ 
   template<class,class> struct structured_ostream;   
   template<class> struct structured_istream; 

   template <class Stream, class TS, class T>
      void write(structured_ostream<Stream, TS> & stream, m_ptr<T> const &ptr)
   {
      T const * t = ptr.get();
      write(stream, t);
   }


   template <class Stream, class T>
      void read(structured_istream<Stream> & stream, m_ptr<T> &ptr)
   {
      STATIC_ASSERT( false, incorrect_deserialization );
   }

template <class Stream, class T>
   void write(Stream & stream, m_ptr<T> const &ptr)
{
   T const * t = ptr.get();
   write(stream, bool(t != 0));
   if (t) 
      write(stream, *t);
}

template <class Stream, class T>
   void read(Stream & stream, m_ptr<T> &ptr)
{
   bool b;
   read(stream, b);

   if (b) 
   {
      ptr = new T();
      read(stream, *ptr.get());
   }
}
}

//
template<class T> struct smart_ptr
{
   typedef T element_type ;
   typedef T value_type   ;

     smart_ptr () : pCounter_ ( NULL ), pElement_ ( NULL )
     {
     } 
    ~smart_ptr () 
     {
         if ( pCounter_ ) 
         {
              if ( -- (*pCounter_) == 0 ) 
              {
                   delete pCounter_ ; 
                   delete pElement_ ; 
              }
         }          
     }         

     smart_ptr ( T * element ) : pCounter_ ( NULL ), pElement_ ( NULL )
     {
         if ( element ) 
         {
              pCounter_ = new int ( 1 ); 
              pElement_ = element ; 
         }     
     }  

     smart_ptr ( const smart_ptr<T>& ptr ) : pCounter_ ( NULL ), pElement_ ( NULL )
     {
         if ( ptr.pCounter_ ) 
         {
              pCounter_ = ptr.pCounter_ ; 
              pElement_ = ptr.pElement_ ; 

              (*pCounter_) ++ ; 
         }
     }         

     smart_ptr<T>& operator = ( const smart_ptr<T>& ptr ) 
     {
         smart_ptr<T> temp ( ptr ) ; 
         temp.swap ( *this ) ; 

         return *this ; 
     }     

     smart_ptr<T>& operator = ( T * ptr ) 
     {
         smart_ptr<T> temp ( ptr ) ; 
         temp.swap ( *this ) ; 

         return *this ; 
     }

     void swap ( smart_ptr<T>& ptr ) 
     {
         std::swap( pCounter_, ptr.pCounter_ ) ; 
         std::swap( pElement_, ptr.pElement_ ) ; 
     }    

     SAFE_BOOL_OPERATOR(pCounter_ != 0)

     T       *         get ()       { return pElement_ ; }
     T const *         get () const { return pElement_ ; }

     T&        operator *  ()       { return *pElement_; }
     T const & operator *  () const { return *pElement_; }

     T*        operator -> ()       { return pElement_; }
     T const * operator -> () const { return pElement_; }

private :
     int * pCounter_ ; 
     T   * pElement_ ; 
} ;
