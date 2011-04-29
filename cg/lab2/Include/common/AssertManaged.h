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

#ifndef Assert // чисто отладочный ассерт
#  ifdef _DEBUG 
#     define Assert(x) AssertManagedRelease(x)
#  else
#     define Assert(x)
#  endif
#endif


#ifndef TracedAssert  // в release вырождается в TRACE
#  ifdef _DEBUG 
#     define TracedAssert(x) AssertManagedRelease(x)
#  elif defined (__TRACE_H) // пока не знаю, что делать в случае трассы
#     define TracedAssert(x) AssertManagedRelease(x)
#  else
#     define TracedAssert(x)
#  endif
#endif

#ifndef Verify // код в скобках выполняется в ЛЮБОЙ конфигурации
#  ifdef _DEBUG 
#     define Verify(x) AssertManagedRelease(x)
#  elif defined (__TRACE_H) // пока не знаю, что делать в случае трассы
#     define Verify(x) AssertManagedRelease(x)
#  else
#     define Verify(x) do { (x); } while ( 0 )
#  endif
#endif

#ifndef AssertRelease // самая жесткая штука - даже в release прерывает выполнение программы при невыполнении условия
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