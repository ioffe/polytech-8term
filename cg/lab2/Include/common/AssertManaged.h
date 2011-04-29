#pragma once 

#ifdef Assert 
#undef Assert
#endif

namespace Utils 
{
   class DebugHelper
   {
   public:
      static void QAssertRelease(bool test)
      {
         System::Diagnostics::Debug::Assert(test);
      }
   };
}

#define AssertManagedRelease(x)  do { if (!(x)) System::Diagnostics::Debug::Fail(#x); } while( 0 )

#ifndef Assert // ����� ���������� ������
#  ifdef _DEBUG 
#     define Assert(x) AssertManagedRelease(x)
#  else
#     define Assert(x)
#  endif
#endif


#ifndef TracedAssert  // � release ����������� � TRACE
#  ifdef _DEBUG 
#     define TracedAssert(x) AssertManagedRelease(x)
#  elif defined (__TRACE_H) // ���� �� ����, ��� ������ � ������ ������
#     define TracedAssert(x) AssertManagedRelease(x)
#  else
#     define TracedAssert(x)
#  endif
#endif

#ifndef Verify // ��� � ������� ����������� � ����� ������������
#  ifdef _DEBUG 
#     define Verify(x) AssertManagedRelease(x)
#  elif defined (__TRACE_H) // ���� �� ����, ��� ������ � ������ ������
#     define Verify(x) AssertManagedRelease(x)
#  else
#     define Verify(x) do { (x); } while ( 0 )
#  endif
#endif

#ifndef AssertRelease // ����� ������� ����� - ���� � release ��������� ���������� ��������� ��� ������������ �������
#  define AssertRelease(x) do { Utils::DebugHelper::QAssertRelease(x); } while( 0 )
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