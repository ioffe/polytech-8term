#pragma once

#include "boost\static_assert.hpp"

#include "safe_bool.h"

#pragma pack(push, 1)

template< typename tag, typename scalar >
struct tagged_bit_mask
{
public:
   BOOST_STATIC_ASSERT(boost::is_integral<scalar>::value);

   __forceinline explicit tagged_bit_mask( scalar mask = 0);

   __forceinline tagged_bit_mask & operator &=( tagged_bit_mask other );
   __forceinline tagged_bit_mask & operator |=( tagged_bit_mask other );
   __forceinline tagged_bit_mask & operator ^=( tagged_bit_mask other );

   SAFE_BOOL_OPERATOR(mask != 0)
public:
   scalar mask;
};
#pragma pack(pop)

template< typename tag, typename scalar >
__forceinline tagged_bit_mask<tag, scalar>::tagged_bit_mask( scalar mask )
   : mask(mask)
{
}

template< typename tag, typename scalar >
__forceinline tagged_bit_mask<tag, scalar> & tagged_bit_mask<tag, scalar>::operator &=( tagged_bit_mask other )
{
   mask &= other.mask;
   return *this;
}

template< typename tag, typename scalar >
__forceinline tagged_bit_mask<tag, scalar> & tagged_bit_mask<tag, scalar>::operator |=( tagged_bit_mask other )
{
   mask |= other.mask;
   return *this;
}

template< typename tag, typename scalar >
__forceinline tagged_bit_mask<tag, scalar> & tagged_bit_mask<tag, scalar>::operator ^=( tagged_bit_mask other )
{
   mask ^= other.mask;
   return *this;
}

template< typename tag, typename scalar >
__forceinline tagged_bit_mask<tag, scalar> operator|( tagged_bit_mask<tag, scalar> t1, tagged_bit_mask<tag, scalar> t2 )
{
   return tagged_bit_mask<tag, scalar>(t1.mask | t2.mask);
}

template< typename tag, typename scalar >
__forceinline tagged_bit_mask<tag, scalar> operator&( tagged_bit_mask<tag, scalar> t1, tagged_bit_mask<tag, scalar> t2 )
{
   return tagged_bit_mask<tag, scalar>(t1.mask & t2.mask);
}

template< typename tag, typename scalar >
__forceinline tagged_bit_mask<tag, scalar> operator^( tagged_bit_mask<tag, scalar> t1, tagged_bit_mask<tag, scalar> t2 )
{
   return tagged_bit_mask<tag, scalar>(t1.mask ^ t2.mask);
}

template< typename tag, typename scalar >
__forceinline tagged_bit_mask<tag, scalar> operator~( tagged_bit_mask<tag, scalar> t1 )
{
   return tagged_bit_mask<tag, scalar>(~t1.mask);
}

template< typename tag, typename scalar >
__forceinline bool operator==( tagged_bit_mask<tag, scalar> t1, tagged_bit_mask<tag, scalar> t2 )
{
   return t1.mask == t2.mask;
}

template< typename tag, typename scalar >
__forceinline bool operator!=( tagged_bit_mask<tag, scalar> t1, tagged_bit_mask<tag, scalar> t2 )
{
   return t1.mask != t2.mask;
}
