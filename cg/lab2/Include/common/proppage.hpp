
#include "common\stringex.h"
#include "common\assert.h"
#include "WinControls\MultiLangResources.h"
#include "trnsystems\chartsystemevents.h"

struct CPropPageCore :: EventLocker
{
   EventLocker ( CPropPageCore * page ) : page_(page) 
   { 
      page_ -> LockEvent() ; 
   }
   ~EventLocker () 
   { 
      page_ -> UnlockEvent() ; 
   }

   CPropPageCore * page_ ;       
} ;    

inline CPropPageCore :: CPropPageCore ( UINT dlgId, UINT titleId ) 
   : m_dwDialogID   ( dlgId )
   , m_Name         ( LoadStringEx ( _Module.GetResourceInstance(), titleId ? titleId : dlgId ) )
   , m_dwLockDialog ( 0 )  
   , m_dwLockEvent  ( 0 )  
{
} 

// INtiPanel
inline LPCWSTR  __stdcall CPropPageCore :: GetName ()   
{
   return m_Name.c_str() ;
}

inline HWND __stdcall CPropPageCore :: Create ( IUnknown * obj, HWND parent, LPCRECT rect ) 
{
   _Module.AddCreateWndData ( &m_thunk.cd, (CDialogImplBase*)this ) ;

   HINSTANCE hInst = _Module.GetResourceInstance() ;

   HRSRC hrRes = FindResourceEx( hInst, RT_DIALOG, MAKEINTRESOURCE(m_dwDialogID), LANGIDFROMLCID( GetThreadLocale())) ;

   if ( !hrRes )
      hrRes = FindResource( hInst, MAKEINTRESOURCE(m_dwDialogID), RT_DIALOG ) ;

   if ( CreateDialogIndirect( hInst, LPDLGTEMPLATE( LoadResource( hInst, hrRes )), 
      parent, (DLGPROC)StartDialogProc ) == NULL )
   {
      Assert(0); 
      return NULL;
   }

   m_Object = obj ; 

   GetUnknown() -> AddRef() ; 

   m_Listener.reset( new listener::Subscriber<ntiapp::IObjectChangedEvent>( m_Object, this ) ) ;

   Lock   () ;  
   OnCreate () ; 
   Unlock () ;  

   MoveWindow ( rect );

   return m_hWnd; 
}

inline BOOL __stdcall CPropPageCore :: Pretranslate ( MSG * msg )
{
   return OnPretranslate (msg) ; 
}

//
inline BOOL CPropPageCore :: IsLocked  () 
{ 
   return m_dwLockDialog != 0 ; 
}

inline BOOL CPropPageCore :: IsEventLocked () 
{ 
   return m_dwLockEvent != 0 ; 
}

inline void CPropPageCore :: Lock      () 
{ 
   m_dwLockDialog ++ ; 
}

inline void CPropPageCore :: Unlock    () 
{ 
   m_dwLockDialog -- ; 
}

inline void CPropPageCore :: LockEvent      () 
{ 
   m_dwLockEvent ++ ; 
}

inline void CPropPageCore :: UnlockEvent    () 
{ 
   m_dwLockEvent-- ; 
}

// 
inline void CPropPageCore :: OnFinalMessage( HWND ) 
{
   GetUnknown() -> Release () ; 
}

inline void CPropPageCore :: OnChartObjectChanged ( IUnknown * obj, DWORD flags, LPARAM lParam ) 
{
   if ( m_dwLockEvent == 0 ) 
   {
      Lock () ;  
      OnObjectChanged ();
      Unlock () ;  
   }
}  

// -----------------------------------------------------------------
inline LRESULT CPropPageCore :: OnInitDialog ( UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL & bHandled )
{
   ModifyStyleEx ( 0, WS_EX_CONTROLPARENT ) ; 
   return 0;
}

inline LRESULT CPropPageCore :: OnDestroy ( UINT, WPARAM wParam, LPARAM, BOOL & )
{
   OnDestroy() ; 
   m_Object = NULL ; 
   m_Listener.reset() ;
   return 0 ;
}



