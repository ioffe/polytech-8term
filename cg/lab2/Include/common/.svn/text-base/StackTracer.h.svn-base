#pragma once 

#include <ostream>

#ifdef USE_ZLIB 
  #include "zlib/zlib.h"
  #include "zlib/zconf.h"
#endif   

#define filter_function(data) tracer::filter_function_impl(data, exception_info() )  

#pragma warning (disable:4509)

namespace tracer
{
   typedef std::vector<DWORD> adress_stack ; 
   
   struct exception_data
   {
      DWORD        code ;
      DWORD        address ;
      adress_stack stack ;
      exception_data () : code ( 0 ), address ( 0 ) {} 
   } ;
   
   void store_exception_data ( exception_data& data, const EXCEPTION_POINTERS * ptrs ) ; 
   void store_stack          ( adress_stack& stack, DWORD ebp ) ;
   void write_exception_data ( std::ostream& stream, const exception_data& data ) ; 
   void write_stack          ( std::ostream& stream, const adress_stack& stack ) ; 
   void write_exception_text ( std::ostream& stream, DWORD code ) ; 
   void write_address_text   ( std::ostream& stream, DWORD ptr ) ; 

   ///////////////////////////////////////////////////////////////////////////////////////
   // implementation 
   inline static bool virtual_query ( DWORD ptr, DWORD& base, DWORD& reg, DWORD& size )
   {
      MEMORY_BASIC_INFORMATION mbi ;
      if ( VirtualQuery ( (void*)ptr, & mbi, sizeof ( mbi ) ) != sizeof ( mbi ) ) 
         return false ;
      base = (DWORD)mbi.AllocationBase ;
      reg  = (DWORD)mbi.BaseAddress ;
      size = mbi.RegionSize ;
   
      if ( (base % 0x1000) == 0xFFF ) base ++ ;
      if ( (size % 0x1000) == 0xFFF ) size ++ ;
      return true ;
   }

   inline bool get_visual_address ( char * buf, const char * data, DWORD size, DWORD offset ) 
   {
      if ( size > 4 ) 
      {
         DWORD i = 0 ; 
         DWORD start = *(DWORD*)(data + i)  ; 
         for ( ; ; ) 
         {
            DWORD j = i + 4 ; 
            while ( data[j++] ) ;
            if ( j + 4 > size ) 
               return false ; 
            DWORD end = *(DWORD*)(data + j) ;
            if ( offset < end && offset >= start ) 
            {
               if ( ! data[i + 4] ) 
                  return false ; 
               // found
               sprintf ( buf, "%s + %03x", data + i + 4, offset - start ) ;
               return true ; 
            }
            start = end ;
            i = j ; 
         }
      }
      return false ;
   }

   inline void store_exception_data ( exception_data& data, const EXCEPTION_POINTERS * ptrs )  
   {
      if ( ptrs != NULL )
      {
         data.code      =         ptrs -> ExceptionRecord -> ExceptionCode ;
         data.address   = (DWORD)(ptrs -> ExceptionRecord -> ExceptionAddress) ;

         store_stack ( data.stack, ptrs -> ContextRecord -> Ebp ) ; 
      }
      else 
      {
         data.code    = 0 ; 
         data.address = 0 ; 

         DWORD currebp ;
         __asm mov currebp, ebp ;
         store_stack ( data.stack, currebp ) ; 
      }
   }   

   inline void store_stack ( adress_stack& stack, DWORD ebp ) 
   {
      DWORD eip = 0 ; 

      // get stack limits
      DWORD base, size, stackHigh, stackLow ; 
      virtual_query ( (DWORD)&base, base, stackHigh, size ) ; 
      stackHigh += size ; 
      virtual_query ( base, base, stackLow,  size ) ;
      stackLow += size ; 

      __try
      {
         for ( ; ; ) 
         {
            if ( eip ) 
               stack.push_back( eip ) ;

            DWORD newebp = ((DWORD *)ebp)[0];
            DWORD neweip = ((DWORD *)ebp)[1];

            if ( newebp <= ebp || newebp <= stackLow || newebp >= stackHigh )
               break ; 
            ebp = newebp;
            eip = neweip;
         }
      }
      __except ( EXCEPTION_EXECUTE_HANDLER ) 
      {
         // this means that we fall out of the stack
      }
   }   
   
   inline void write_exception_data ( std::ostream& stream, const exception_data& data ) 
   {
      if ( data.code != 0 ) 
      {
         write_exception_text ( stream, data.code    ) ; 
         stream << " at " ; 
         write_address_text   ( stream, data.address ) ;
         stream << "\r\n" ; 
      }

      write_stack ( stream, data.stack ) ; 
   }
   
   inline void write_stack ( std::ostream& stream, const adress_stack& stack ) 
   {
      stream << "Stack walk\r\naddress  ==> module               : [function +] offset \r\n" ; 
      for ( size_t i = 0 ; i < stack.size() ; ++ i ) 
         if ( stack.size() > 50 && i == 10 ) 
         {
            stream << "... Skipping " << stack.size() - 50 << " stack levels\r\n" ; 
            i += stack.size() - 50 ; 
         }
         else 
         {
            write_address_text ( stream, stack[i] ) ;
            stream << "\r\n" ; 
         }   
   }
   
   inline void write_exception_text ( std::ostream& stream, DWORD code )
   {
      struct exc_text 
      {
         DWORD  code ; 
         LPCSTR text ; 
      } ; 
      
      static exc_text excText[] =
      {
         { EXCEPTION_ACCESS_VIOLATION         , "EXCEPTION_ACCESS_VIOLATION"            },
         { EXCEPTION_ARRAY_BOUNDS_EXCEEDED    , "EXCEPTION_ARRAY_BOUNDS_EXCEEDED"       },
         { EXCEPTION_BREAKPOINT               , "EXCEPTION_BREAKPOINT"                  },
         { EXCEPTION_DATATYPE_MISALIGNMENT    , "EXCEPTION_DATATYPE_MISALIGNMENT"       },
         { EXCEPTION_FLT_DENORMAL_OPERAND     , "EXCEPTION_FLT_DENORMAL_OPERAND"        },
         { EXCEPTION_FLT_DIVIDE_BY_ZERO       , "EXCEPTION_FLT_DIVIDE_BY_ZERO"          },
         { EXCEPTION_FLT_INEXACT_RESULT       , "EXCEPTION_FLT_INEXACT_RESULT"          },
         { EXCEPTION_FLT_INVALID_OPERATION    , "EXCEPTION_FLT_INVALID_OPERATION"       },
         { EXCEPTION_FLT_OVERFLOW             , "EXCEPTION_FLT_OVERFLOW"                },
         { EXCEPTION_FLT_STACK_CHECK          , "EXCEPTION_FLT_STACK_CHECK"             },
         { EXCEPTION_FLT_UNDERFLOW            , "EXCEPTION_FLT_UNDERFLOW"               },
         { EXCEPTION_ILLEGAL_INSTRUCTION      , "EXCEPTION_ILLEGAL_INSTRUCTION"         },
         { EXCEPTION_IN_PAGE_ERROR            , "EXCEPTION_IN_PAGE_ERROR"               },
         { EXCEPTION_INT_DIVIDE_BY_ZERO       , "EXCEPTION_INT_DIVIDE_BY_ZERO"          },
         { EXCEPTION_INT_OVERFLOW             , "EXCEPTION_INT_OVERFLOW"                },
         { EXCEPTION_INVALID_DISPOSITION      , "EXCEPTION_INVALID_DISPOSITION"         },
         { EXCEPTION_NONCONTINUABLE_EXCEPTION , "EXCEPTION_NONCONTINUABLE_EXCEPTION"    },
         { EXCEPTION_PRIV_INSTRUCTION         , "EXCEPTION_PRIV_INSTRUCTION"            },
         { EXCEPTION_SINGLE_STEP              , "EXCEPTION_SINGLE_STEP"                 },
         { EXCEPTION_STACK_OVERFLOW           , "EXCEPTION_STACK_OVERFLOW"              },
      } ;
      const size_t excCount = sizeof(excText) / sizeof(exc_text) ; 

      for ( size_t i = 0 ; i < excCount ; i ++ ) 
         if ( excText[i].code == code )
         {
             stream << excText[i].text ;
             return ; 
         }    

      char scode [10] ;
      sprintf ( scode, "%08X", code ) ; 
      stream << "Unknown exception " << scode ;
   }
   
   inline void write_address_text ( std::ostream& stream, DWORD ptr )
   {
      char sptr [10] ;
      sprintf ( sptr, "%08X", ptr ) ; 
      stream << sptr << " ==> " ; 

      char buf [1024] ;
      char module [MAX_PATH] ; 
      DWORD base, reg, size ;
      if ( virtual_query ( ptr, base, reg, size ) && GetModuleFileNameA ( (HINSTANCE)base, module, sizeof ( module ) ) != 0 )
      {
         // module found, try to get tracing resource 
         char * p = strrchr ( module, '\\' ) ;
         sprintf ( buf, "%-20s : ", p ? p + 1 : module ) ;
         p = strchr(buf, 0) ;

         BOOL found = FALSE ; 
         HRSRC hrsrc = FindResourceA ( (HINSTANCE)base, "PUBLICS", "TRACING" ) ; 
         if ( hrsrc != NULL ) 
         {
            LPCSTR data = (LPCSTR) LockResource ( LoadResource ( (HINSTANCE)base, hrsrc ) ) ;
            long dataSize = SizeofResource ( (HINSTANCE)base, hrsrc) ;
#ifdef USE_ZLIB 
            if ( 0 == _strnicmp ( data, "VS5+C", 5 ) )
            {
               unsigned long size = *( ( unsigned long * ) ( data + 10 ) ) ;
               char * rawData = new char [ size ] ;

               int decomp = uncompress ( (Bytef *) rawData, &size, (Bytef *)( data + 10 + sizeof ( long ) ), dataSize - 10 - sizeof ( long )) ;
               if ( Z_OK == decomp )
                  found = get_visual_address ( p, rawData + 10,  size - 10, ptr - base ) ;
               else 
               {
                  sprintf ( buf, "Unable to decompress symbol data.. " ) ;
                  found = FALSE ;
               }

               delete [] rawData ;
            }
            else 
#endif
            if ( _strnicmp ( data, "VISUAL5&6", 9 ) == 0 )
            { 
               found = get_visual_address ( p, data + 10,  dataSize - 10, ptr - base ) ;
            }
         }
         if ( ! found ) 
            sprintf ( p, "%08X", ptr - base ) ;
      }
      else 
         sprintf ( buf, "unknown module" ) ;
         
      stream << buf ;    
   }
   
   inline int filter_function_impl (exception_data& data, const EXCEPTION_POINTERS * ptrs )                        
   {                                                                 
      store_exception_data ( data, ptrs ) ; 
      return EXCEPTION_EXECUTE_HANDLER ;                            
   }   
} ; 