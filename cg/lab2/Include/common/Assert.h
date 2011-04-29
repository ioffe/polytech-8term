#pragma once 

#include <stdexcept>
#include <iostream>

#ifndef Assert // ����� ���������� ������
#  ifdef _DEBUG 
#     define Assert(x) void((x) || (__debugbreak(), 0))
#  else
#     define Assert(x) void(sizeof(x))
#  endif
#endif

#ifndef TracedAssert  // � release ����������� � TRACE
#  ifdef _DEBUG 
#     define TracedAssert(x) Assert(x)
#  elif defined (__TRACE_H)
#     define TracedAssert(x) void((x) || (__TRACE(#x), 0))
#  else
#     define TracedAssert(x) void(sizeof(x))
#  endif
#endif

struct VerifyException : std::exception
{
   VerifyException(const char * msg) : std::exception(msg) {}
};

#ifndef Verify // ��� � ������� ����������� � ����� ������������; � release ������ VerifyException ��� ������������ �������
#  ifdef _DEBUG 
#     define Verify(x) Assert(x)
#  else
#     define Verify(x) do { if (!(x)) { std::cerr << "Assertion " << #x << " failed at function " << __FUNCTION__ << " at file " << __FILE__ << ":" << __LINE__ << std::endl << std::flush; throw VerifyException(#x); } } while ( 0 )
#  endif
#endif

#ifndef AssertRelease // ����� ������� ����� - ���� � release ��������� ���������� ��������� ��� ������������ �������
#  define AssertRelease(x) void((x) || (__debugbreak(), 0))
#endif

#ifndef STATIC_ASSERT

namespace meta_details
{
   template< bool pred >
      struct StaticAssert
   {
      StaticAssert(...);
   };

   template<> struct StaticAssert<false> {};
}

#define STATIC_ASSERT(expr,msg) \
{   \
    class ERR_##msg{} x;   \
    (void)sizeof(meta_details::StaticAssert<(expr)>( x ) ); \
}

#endif
