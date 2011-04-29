#pragma once

inline __int64 GetPerformanceCounter () 
{
   __int64 counter ; 
   QueryPerformanceCounter ( (LARGE_INTEGER *)&counter ) ; 
   return counter ; 
}

inline __int64 GetPerformanceFrequency () 
{
   static __int64 frequency = -1; 

   if ( frequency == -1 ) 
      QueryPerformanceFrequency ( (LARGE_INTEGER *)&frequency ) ; 

   return frequency ; 
}

inline double GetPerformanceInterval ( __int64 delta ) 
{
   return (double)delta / GetPerformanceFrequency () ; 
}

inline long GetPerformanceIntervalMS ( __int64 delta ) 
{
   return long(delta * 1000 / GetPerformanceFrequency ()) ; 
}

inline __int64 GetCPUCounter () 
{
   return __rdtsc();
}

inline __int64 GetCPUFrequency () 
{
   static __int64 frequency = -1; 

   if ( frequency == -1 ) 
   {
      __int64 perf_start = GetPerformanceCounter () ; 
      __int64 cpu_start  = GetCPUCounter () ; 

      Sleep ( 100 ) ; 

      __int64 cpu_finish  = GetCPUCounter         () ; 
      __int64 perf_finish = GetPerformanceCounter () ; 

      double ratio = double(cpu_finish - cpu_start) / (perf_finish - perf_start);

      bool equal = ratio > 0.999 && ratio < 1.001 ; 

      frequency = equal ? GetPerformanceFrequency() : __int64(ratio*GetPerformanceFrequency()) ; 
   }
   return frequency ; 
}

inline double GetCPUInterval ( __int64 delta ) 
{
   return (double)delta / GetCPUFrequency () ; 
}

inline long GetCPUIntervalMS ( __int64 delta ) 
{
   return long(delta * 1000 / GetCPUFrequency ()) ; 
}

struct PerfCounter
{
    PerfCounter()
    {
        ::QueryPerformanceFrequency(&freq_);
        ::QueryPerformanceCounter(&start_);
    }

    void   restart() 
    {
      ::QueryPerformanceCounter(&start_);
    }
    
    double time() const {
        LARGE_INTEGER  now;
        ::QueryPerformanceCounter(&now);
        return double(now.QuadPart - start_.QuadPart) / freq_.QuadPart;
    }

    operator double () const {
       return time();
    }

private:
    ::LARGE_INTEGER  freq_, start_;
};

struct PerfCounterEx
{
  PerfCounterEx( ) 
  {
    FILETIME ftDummy;
    GetThreadTimes( 
       GetCurrentThread( ), &ftDummy, &ftDummy, 
       reinterpret_cast<FILETIME*>(&ftKernelStart), 
       reinterpret_cast<FILETIME*>(&ftUserStart) 
    );
  }

  double time( ) const
  {
    FILETIME ftDummy;
    unsigned __int64 ftKernelFinish;
    unsigned __int64 ftUserFinish;

    GetThreadTimes( 
       GetCurrentThread( ), &ftDummy, &ftDummy, 
       reinterpret_cast<FILETIME*>(&ftKernelFinish), 
       reinterpret_cast<FILETIME*>(&ftUserFinish) 
    );

    __int64 qwUserTime    = ftUserFinish - ftUserStart;
    __int64 qwKernelTime  = ftKernelFinish - ftKernelStart;
    __int64 qwTotalTime   = qwKernelTime + qwUserTime;

    return qwTotalTime / 1e7;
  }

  operator double( ) const 
  {
    return time( );
  }

private:
  unsigned __int64 ftUserStart;
  unsigned __int64 ftKernelStart;
};

inline int GetTickCountEx()
{   
    static __int64 frequency = -1;
    static __int64 starttick = -1;

    if ( frequency == -1 ) 
                ::QueryPerformanceFrequency((LARGE_INTEGER *)&frequency);

    if ( starttick == -1 ) 
                ::QueryPerformanceCounter ( (LARGE_INTEGER *)&starttick ) ;

    __int64 tick ;
        ::QueryPerformanceCounter ( (LARGE_INTEGER *)&tick ) ;

    return static_cast<int>(((tick - starttick) * 1000.) / frequency); 
}

inline long GetTickCountExMicro()
{   
    static __int64 frequency = -1;
    static __int64 starttick = -1;

    if ( frequency == -1 ) 
                ::QueryPerformanceFrequency((LARGE_INTEGER *)&frequency);

    if ( starttick == -1 ) 
                ::QueryPerformanceCounter ( (LARGE_INTEGER *)&starttick ) ;

    __int64 tick ;
        ::QueryPerformanceCounter ( (LARGE_INTEGER *)&tick ) ;

    return static_cast<long>(((tick - starttick) * 1000000.) / (double)frequency); 
}

struct PerfCounterFast
{
   typedef void(*auto_start_tag)( void );
   static inline void auto_start( void ) {}

   // do nothing
   inline PerfCounterFast( void )
   {
   }

   // set start time
   inline PerfCounterFast( auto_start_tag )
   {
      m_qwTimer = __rdtsc();
   }

   // reset start time
   inline void restart( void )
   {
      m_qwTimer = __rdtsc();
   }

   // returns delta between auto start constructor or last restart() call and time() call
   inline __int64 time( void ) const
   {
      return __rdtsc() - m_qwTimer;
   }

   // returns CPU frequency
   static inline __int64 freq( void )
   {
      return GetCPUFrequency();
   }

   // returns interval in seconds
   static inline double interval( __int64 delta ) 
   {
      return (double)delta / freq();
   }

   // returns interval in milliseconds
   static inline double interval_ms( __int64 delta ) 
   {
      return 1000.0 * delta / freq();
   }

private:

   // internal time holder
   __int64 m_qwTimer;
};

