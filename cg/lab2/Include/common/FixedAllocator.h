#pragma once 

template < const size_t N, const size_t CHUNK = 1000> 
struct fixed_allocator
{
   fixed_allocator () 
      : chunks_ ( NULL ) 
      , head_   ( NULL ) 
   {
   }
   ~fixed_allocator () 
   {
#ifdef _DEBUG
      size_t total, free ; 
      get_info( total, free ) ; 
      Assert ( total == free ) ; 
#endif

      delete chunks_ ; 
   }
   void * allocate () 
   {
      if ( head_ == NULL ) 
      {
         chunks_ = new Chunk ( chunks_ ) ; 
         head_ = chunks_ -> head() ; 
      }

      void * ret = head_ ; 
      head_ = next(head_) ; 
      return ret ;  
   }
   void deallocate ( void * ptr ) 
   {
      next(ptr) = head_ ; 
      head_ = ptr ; 
   }

   void get_info ( size_t &total, size_t &free )
   {
      total = 0 ; 
      for ( Chunk * chunk = chunks_ ; chunk != NULL ; chunk = chunk -> next() ) 
          total += CHUNK ; 

      free  = 0 ; 
      for ( void * ptr = head_ ; ptr != NULL ; ptr = next(ptr) ) 
         free ++ ; 
   }

private:
   static void *& next( void * ptr ) 
   {
      return *(void **)ptr ; 
   }

private:
   #pragma pack(push, 1) 

   struct Chunk 
   {
      Chunk ( Chunk * next ) : next_ ( next ) 
      {
         BYTE * last  = data_ + N * (CHUNK-1); 

         for ( BYTE * ptr = data_ ; ptr != last ; ptr += N ) 
            fixed_allocator::next(ptr) = ptr + N ; 

         fixed_allocator::next(last) = NULL ; 
      }
      ~Chunk () 
      {
         delete next_ ; 
      }
      void * head () 
      {
         return data_ ; 
      }
      Chunk * next() 
      {
         return next_ ; 
      }
   private:
      Chunk * next_ ; 
      BYTE    data_[CHUNK * N] ; 
   } ; 

   #pragma pack(pop) 

   Chunk * chunks_ ; 
   void  * head_; 

} ; 

template <class T>
   fixed_allocator<sizeof(T)>& fixed_allocator_instance()
   {
      static fixed_allocator<sizeof(T)> instance ; 
      return instance ; 
   }

#define FIXED_ALLOCATED(type)                                 \
   static void *operator new( size_t size )                   \
   {                                                          \
      Assert(size == sizeof(type) ) ;                         \
      return fixed_allocator_instance<type>().allocate() ;    \
   }                                                          \
                                                              \
   static void operator delete ( void * ptr )                 \
   {                                                          \
      fixed_allocator_instance<type>().deallocate ( ptr ) ;   \
   }                                                          



