#pragma once

inline void HeapWalk ()
{
   struct heap_counters
   {
      size_t small_blocks[0x10000] ;

      size_t big_blocks[0x10000] ;
      size_t big_blocks_count; 

      size_t  extra_size ; 
      size_t  extra_count ; 

      size_t  full_size ; 
      size_t  full_count ; 
   } ; 

   static heap_counters counters[2] ; 
   memset ( &counters[0], 0, sizeof(counters[0])) ; 
   memset ( &counters[1], 0, sizeof(counters[1])) ; 

   _HEAPINFO hinfo = { NULL, 0, 0 } ;
   while( _heapwalk(&hinfo) == _HEAPOK ) 
   {
      heap_counters& cnt = hinfo._useflag == _USEDENTRY ? counters[0] : counters[1] ; 

      if ( hinfo._size < 0x10000 ) 
         cnt.small_blocks[hinfo._size] ++ ; 
      else if ( cnt.big_blocks_count != 0x10000 ) 
         cnt.big_blocks[cnt.big_blocks_count++] = hinfo._size ; 
      else
      {
         cnt.extra_size  += hinfo._size ; 
         cnt.extra_count ++ ; 
      }
      cnt.full_size  += hinfo._size ; 
      cnt.full_count ++ ; 
   }


   for ( size_t i = 0 ; i < 2 ; i ++ ) 
   {
      heap_counters const& cnt = counters[i] ; 

      OutputDebugString( i == 0 ? "Used\n" : "Free\n" ) ; 

      char buf[256] ; 
      sprintf(buf, "Total %d bytes in %d blocks\n", cnt.full_size, cnt.full_count ) ; 
      OutputDebugString(buf) ; 

      for ( size_t j = 0 ; j < 0x10000 ; j ++ ) 
         if ( cnt.small_blocks[j] != 0 ) 
         {
            sprintf(buf, "%d bytes in %d blocks of size %d \n", 
                    cnt.small_blocks[j]*j, cnt.small_blocks[j], j ) ; 
            OutputDebugString(buf) ; 
         }
      for ( size_t j = 0 ; j < cnt.big_blocks_count ; j ++ ) 
      {
         sprintf(buf, "block of size %d\n", cnt.big_blocks[j] ) ; 
         OutputDebugString(buf) ; 
      }

      sprintf(buf, "More %d bytes in %d blocks\n", cnt.extra_size, cnt.extra_count ) ; 
      OutputDebugString(buf) ; 
   }
}