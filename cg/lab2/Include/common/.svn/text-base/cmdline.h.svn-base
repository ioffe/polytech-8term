#pragma once 

#include <vector>

#include "less_nocase.h"

template<typename _char_t>
class CCommandLineT
{                                                                                        
public : 
   CCommandLineT () {}

   CCommandLineT ( const _char_t* ptr )
   {
      SetCommandLine ( ptr ) ;
   }

   void Clear ()
   {
      m_Params.clear();
      m_Unnamed.clear();
   }

   void SetCommandLine ( const _char_t* ptr )
   { 
      if ( ptr == NULL )
      {
         Clear ();
         return;
      }

      _char_t name [1024] ; 
      _char_t value[1024] ; 

      while ( EnumParameter ( ptr, name, value ) ) 
      {
         if ( name[0] != 0 )
            m_Params[name] = value ;
         else
         {
            m_Params[value] = get_one();

            if ( value[0] != get_slash() ) 
               m_Unnamed.push_back(value);
         }
      }
         
      m_Iterator = m_Params.begin () ;      
   }

   const _char_t* GetParam   ( const _char_t* name, const _char_t* defValue ) const 
   {
      const _char_t* val = NULL ; 
      if ( FindParam ( name, &val ) )
         return val ; 

      return defValue ; 
   }

   double GetParam   ( const _char_t* name, double defValue ) const 
   {
      const _char_t* val = NULL ; 
      if ( FindParam ( name, &val ) )
         GetDouble ( val, &defValue ) ; 

      return defValue ; 
   }

   int GetParam   ( const _char_t* name, int defValue ) const 
   {
      const _char_t* val = NULL ; 
      if ( FindParam ( name, &val ) )
         GetInt ( val, &defValue ) ; 

      return defValue ; 
   }

   BOOL   CheckParam ( const _char_t* name ) const 
   {
      return m_Params.find ( name ) != m_Params.end () ; 
   }

   BOOL   CheckParam ( const _char_t* name, const _char_t** value  ) const 
   { 
      const _char_t* val = NULL ; 
      if ( ! FindParam ( name, &val ) )
         return FALSE ; 

      *value = val ; 

      return TRUE ; 
   }

   BOOL   CheckParam ( const _char_t* name, std::string & value  ) const 
   { 
      const _char_t* val = NULL ; 
      if ( ! FindParam ( name, &val ) )
         return FALSE ; 

      value = val ; 

      return TRUE ; 
   }

   BOOL   CheckParam ( const _char_t* name, double * value  ) const 
   { 
      const _char_t* val = NULL ; 
      if ( ! FindParam ( name, &val ) )
         return FALSE ; 

      return GetDouble ( val, value ) ; 
   }

   BOOL   CheckParam ( const _char_t* name, int * value  ) const 
   { 
      const _char_t* val = NULL ; 
      if ( ! FindParam ( name, &val ) )
         return FALSE ; 

      return GetInt ( val, value ) ; 
   }

   BOOL GetUnnamed ( size_t index, const _char_t** value ) const
   {
      if ( index < m_Unnamed.size() )
      {
          *value = m_Unnamed[index].c_str() ; 
          return TRUE ; 
      }
      return FALSE ; 
   }

   DWORD GetUnnamedCount() 
   {
      return m_Unnamed.size();
   }

public : 
   void Reset () 
   {
      m_Iterator = m_Params.begin () ; 
   }
   
   BOOL Next ( const _char_t** name, const _char_t** value ) 
   {
      if ( m_Iterator != m_Params.end () ) 
      {
         if ( m_Iterator -> second.empty() && m_Iterator -> first[0] != '/' )
         {
            *name  = get_empty_string() ; 
            *value = m_Iterator -> first.c_str () ;
         }
         else
         {
            *name  = m_Iterator -> first.c_str () ;      
            *value = m_Iterator -> second.c_str () ;      
         }
      
         m_Iterator ++ ; 
                  
         return TRUE ; 
      }
      return FALSE ;      
   } 

   BOOL Next ( const _char_t* * name, double * value ) 
   {
      const _char_t* str = NULL ; 
      while ( Next ( name, &str ) )
         if ( GetDouble ( str, value ) )
            return TRUE ; 

      return FALSE ; 
   } 
   
   BOOL Next ( const _char_t* * name, int * value ) 
   {
      const _char_t* str = NULL ; 
      while ( Next ( name, &str ) )
         if ( GetInt ( str, value ) )
            return TRUE ; 

      return FALSE ; 
   } 

private : 
   BOOL FindParam ( const _char_t* name, const _char_t** value ) const
   {
      CStringMap :: const_iterator i = m_Params.find ( name ) ;
      if ( i != m_Params.end () ) 
      {
         *value = i -> second.c_str() ; 
         return TRUE ; 
      }

      return FALSE ; 
   }

   typedef std::map<std::basic_string<_char_t>, std::basic_string<_char_t>, less_nocase > CStringMap ; 

   CStringMap                   m_Params ; 
   typename CStringMap :: const_iterator m_Iterator ; 
   std::vector<std::basic_string<_char_t> >     m_Unnamed ; 
   
   static BOOL GetDouble ( const _char_t* str, double * pValue ) 
   {
      if ( str[0] == '0' && str[1] == 'x' ) 
      {
         int iValue ; 
         if ( ! GetInt ( str, &iValue ) )  
            return FALSE ; 

         *pValue = iValue ;      
         return TRUE ; 
      }     
            
      _char_t * stop ; 
      double dValue = strtod_t ( str, &stop ) ; 
      if ( *stop != 0 )
         return FALSE ; 
            
      *pValue = dValue ; 
      return TRUE ;       
   }

   static BOOL GetInt ( const _char_t* str, int * pValue ) 
   {
      int radix = 10 ; 
      if ( str[0] == '0' && str[1] == 'x' ) 
      {
         str += 2 ; 
         radix = 16 ; 
      }     
      _char_t * stop ; 
      int iValue = strtol_t ( str, &stop, radix ) ; 
      if ( *stop != 0 )
         return FALSE ; 
      
      *pValue = iValue ; 
      return TRUE ; 
   }         

   static BOOL EnumParameter ( const _char_t *& ptr, _char_t * name, _char_t * value ) 
   {
      // skip all spaces
      const _char_t * end = ptr ; 
      while ( *end == ' ' ) end ++ ; 
      ptr = end ; 

      if ( *ptr == 0 ) 
         return FALSE ; 

      _char_t * p = name ; 
      while ( *end != '=' && *end != ' ' && *end != 0 ) 
      {
         if ( *end == '\"' ) 
         {
            end ++ ; 
            while ( *end != '\"' && *end != 0 ) 
                  *p ++ = *end ++ ; 
            if ( *end == 0 ) 
                  return FALSE ; 
            end ++ ; 
         }
         else *p ++ = *end ++ ; 
      }
      *p = 0 ; 

      if ( *end == 0 || *end == ' ' ) 
      {
         ptr = end ; 
         strcpy_t ( value, name ) ;
         *name = 0 ; 
         return TRUE ; 
      }

      end ++ ; 

      p = value ; 
      while ( *end != ' ' && *end != 0 ) 
      {
         if ( *end == '\"' ) 
         {
            end ++ ; 
            while ( *end != '\"' && *end != 0 ) 
                  *p ++ = *end ++ ; 
            if ( *end == 0 ) 
                  return FALSE ; 
            end ++ ; 
         }
         else *p ++ = *end ++ ; 
      }
      *p = 0 ; 

      ptr = end ; 

      return TRUE ; 
   }

   static long strtol_t( const _char_t *nptr, _char_t **endptr, int base );
   static _char_t* strcpy_t( _char_t* strDestination, const _char_t* strSource );
   static double strtod_t( const _char_t* nptr, _char_t** endptr );

   static _char_t* get_one();
   static _char_t get_slash();
   static _char_t const * get_empty_string();
} ; 

inline long CCommandLineT<char>::strtol_t( const char *nptr, char **endptr, int base )
{
   return strtol(nptr, endptr, base);
}

inline long CCommandLineT<wchar_t>::strtol_t( const wchar_t *nptr, wchar_t **endptr, int base )
{
   return wcstol(nptr, endptr, base);
}

inline char* CCommandLineT<char>::strcpy_t( char* strDestination, const char* strSource )
{
   return strcpy( strDestination, strSource );
}

inline wchar_t* CCommandLineT<wchar_t>::strcpy_t( wchar_t* strDestination, const wchar_t* strSource )
{
   return wcscpy( strDestination, strSource );
}

inline double CCommandLineT<char>::strtod_t( const char *nptr, char **endptr )
{
   return strtod(nptr, endptr);
}

inline double CCommandLineT<wchar_t>::strtod_t( const wchar_t *nptr, wchar_t **endptr )
{
   return wcstod(nptr, endptr);
}

inline char* CCommandLineT<char>::get_one()
{
   return "1";
}

inline wchar_t* CCommandLineT<wchar_t>::get_one()
{
   return L"1";
}

inline char CCommandLineT<char>::get_slash()
{
   return '/';
}

inline wchar_t CCommandLineT<wchar_t>::get_slash()
{
   return L'/';
}

inline char const * CCommandLineT<char>::get_empty_string()
{
   return "";
}

inline wchar_t const * CCommandLineT<wchar_t>::get_empty_string()
{
   return L"";
}

typedef CCommandLineT<char> CCommandLine;
typedef CCommandLineT<wchar_t> CCommandLineW;