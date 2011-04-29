#pragma once 

struct _hglobal_t
{
   _hglobal_t ( HGLOBAL hglb = NULL ) ; 
   _hglobal_t ( size_t size ) ; 
   _hglobal_t ( LPCVOID pdata, size_t size ) ; 
  ~_hglobal_t () ; 

   LPCVOID data () const ; 
   LPVOID  data () ;   
   size_t  size () const ;

   void    clear () ;

   HGLOBAL detach () ; 

   operator HGLOBAL () const ;
   HGLOBAL * operator & () ; 

private:
   _hglobal_t ( const _hglobal_t& ) ;
   _hglobal_t& operator = ( const _hglobal_t& ) ;

   HGLOBAL hglb_ ; 
} ; 

template<typename T>
   struct _hglobal_vector_t
   {
      _hglobal_vector_t ( HGLOBAL hglb = NULL ) ;   
      _hglobal_vector_t ( size_t size ) ; 

      template <class Iter>
         _hglobal_vector_t ( Iter first, Iter last ) ; 

      T&        operator[] ( size_t i ) ; 
      T const&  operator[] ( size_t i ) const; 

      size_t  size () const ;

      HGLOBAL detach () ; 

      operator HGLOBAL () const ;
      HGLOBAL * operator & () ; 

   private:
      _hglobal_t data_ ; 
   } ; 



////////////////////////////////////////////////////////////////////////////////////////////
// implementation 

inline _hglobal_t::_hglobal_t ( HGLOBAL hglb ) 
   : hglb_ ( hglb ) 
{
}
inline _hglobal_t::_hglobal_t ( size_t size ) 
   : hglb_ ( size ? GlobalAlloc(GMEM_FIXED, size) : NULL )
{
}
inline _hglobal_t::_hglobal_t ( LPCVOID pdata, size_t size )
   : hglb_ ( size ? GlobalAlloc(GMEM_FIXED, size) : NULL )
{
   memcpy ( data(), pdata, size ) ; 
}

inline _hglobal_t::~_hglobal_t () 
{
   clear () ; 
}    
inline LPCVOID _hglobal_t::data () const 
{
   return hglb_ ? GlobalLock ( hglb_ ) : NULL;    
}
inline LPVOID  _hglobal_t::data ()   
{
   return hglb_ ? GlobalLock ( hglb_ ) : NULL;    
}
inline size_t  _hglobal_t::size () const 
{
   return hglb_ ? GlobalSize ( hglb_ ) : 0 ;    
}
inline _hglobal_t::operator HGLOBAL () const 
{
   return hglb_ ; 
}
inline HGLOBAL _hglobal_t::detach () 
{
   HGLOBAL ret = hglb_ ; 
   hglb_ = NULL ;
   return ret ; 
}
inline HGLOBAL * _hglobal_t::operator & () 
{
   clear () ; 
   return &hglb_ ; 
}
inline void _hglobal_t::clear () 
{
   if ( hglb_ )
   {
      GlobalFree ( hglb_ ) ; 
      hglb_ = NULL ;
   }
}

//
template<typename T>
   _hglobal_vector_t<T>::_hglobal_vector_t ( HGLOBAL hglb ) 
      : data_(hglb)
   {
   }
template<typename T>
   _hglobal_vector_t<T>::_hglobal_vector_t ( size_t size ) 
      : data_(size*sizeof(T))
   {
   }
template<typename T> template <class Iter>
   _hglobal_vector_t<T>::_hglobal_vector_t ( Iter first, Iter last ) 
      : data_(std::distance(first, last)*sizeof(T))
   {
      std::copy(first, last, &(*this)[0] ) ; 
   }
template<typename T>
   T& _hglobal_vector_t<T>::operator[] ( size_t i ) 
   {
      return reinterpret_cast<T *>(data_.data())[i] ; 
   }
template<typename T>
   T const&  _hglobal_vector_t<T>::operator[] ( size_t i ) const 
   {
      return reinterpret_cast<T const *>(data_.data())[i] ; 
   }
template<typename T>
   size_t  _hglobal_vector_t<T>::size () const 
   {
      return data_.size() / sizeof(T) ; 
   }
template<typename T>
   HGLOBAL  _hglobal_vector_t<T>::detach ()
   {
      return data_.detach();
   }
template<typename T>
   _hglobal_vector_t<T>::operator HGLOBAL () const 
   {
      return data_ ; 
   }
template<typename T>
   HGLOBAL * _hglobal_vector_t<T>::operator & ()  
   {
      return &data_ ; 
   }

