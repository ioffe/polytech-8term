#pragma once

namespace util
{
   //
   // max raised bit
   //

   template <unsigned int val>
   struct max_raised_bit_index_t
   {
      enum { value = max_raised_bit_index_t<(val >> 1)>::value + 1 };
   };

   template <> 
   struct max_raised_bit_index_t<0> 
   {
      enum { value = -1 };
   };

   static __forceinline unsigned int max_raised_bit_index( unsigned int val )
   {
      __asm bsr eax, val
   }


   //
   // Max-min value
   //

   template <int x, int y>
   struct max_value_t
   {
      enum { value = (x > y) ? x : y };
   };

   template <int x, int y>
   struct min_value_t
   {
      enum { value = (x < y) ? x : y };
   };


   //
   // Nearest power of two
   //

   // nearest power of two, which is equal-or-more, then value
   static __forceinline unsigned int nearest_power_of_two_ge( unsigned int val )
   {
      __asm 
      {
         dec val
         bsr ecx, val
         inc ecx
         xor eax, eax
         inc eax
         shl eax, cl
      };
   }

   // nearest power of two, which is equal-or-less, then value
   static __forceinline unsigned int nearest_power_of_two_le( unsigned int val )
   {
      __asm 
      {
         xor eax, eax
         bsr ecx, val
         inc eax
         shl eax, cl
      };
   }
}
