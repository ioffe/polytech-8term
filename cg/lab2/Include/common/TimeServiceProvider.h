#pragma once

// CTimeServiceProvider
//////////////////////////////////////////////////////////////////////////
class ATL_NO_VTABLE CTimeServiceProvider
   :  public CComObjectRootEx< CComMultiThreadModel >
   ,  public ITimeService 
   ,  private lcheck::resource< CTimeServiceProvider >
{
public      :
   CTimeServiceProvider ()
   {
   }

public      :
   //////////////////////////////////////////////////////////////////////////
   HRESULT  FinalConstruct () 
   {
      HRESULT hr = m_EnvironmentUnk.CoCreateInstance( __uuidof( ExerciseEnvironment ), GetUnknown() ) ; 

      if ( FAILED( hr ) ) 
      {
         __TRACE ( "Failed to create ExerciseEnvironment, hr : %08X\n", hr ) ; 

         return   S_FALSE ; 
      }

      return   S_OK ; 
   }

   //////////////////////////////////////////////////////////////////////////
   void  FinalRelease ()
   {
      m_EnvironmentUnk = NULL ;
   }

public      :
   DECLARE_PROTECT_FINAL_CONSTRUCT()

   BEGIN_COM_MAP( CTimeServiceProvider )
      COM_INTERFACE_ENTRY( ITimeService )

      COM_INTERFACE_ENTRY_AGGREGATE_BLIND( m_EnvironmentUnk.p )
   END_COM_MAP()

// ITimeService  
public      :
   //////////////////////////////////////////////////////////////////////////
   double   __stdcall   GetUpdateTime () 
   {
      return IExerciseEnvironmentPtr(m_EnvironmentUnk.p)->GetUpdateTime ();
   }

   //////////////////////////////////////////////////////////////////////////
   double   __stdcall   GetCurrentTime () 
   {
      return   m_SessionServer->GetSessionTime() ; 
   }
public      :
   void InitInstance ( ISessionServer * sessionServer )
   {
      Assert( sessionServer != NULL ) ;
      m_SessionServer = sessionServer ;
   }

private     :
   DECLARE_DEBUG( "CTimeServiceImpl" )

private     :
   // ExerciseEnvironment
   CComPtr< IUnknown >  m_EnvironmentUnk ; 

   ISessionServerPtr    m_SessionServer ;

   double               m_UpdateTime ; 
} ;
