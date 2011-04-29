#pragma once

#include <common\pointers.h>
#include <common\listener.h>

#include <trnsystems\chartsystem.h>

class ATL_NO_VTABLE CPropPageCore 
   : public CComObjectRootEx<CComMultiThreadModel>
   , public CDialogImplBase
   , public ntiapp::INtiPanel
   , private lcheck::resource<CPropPageCore>
{
public:
   struct EventLocker; 

// ntiapp::INtiPanel
public : 
   LPCWSTR  __stdcall GetName       () ;   
   HWND     __stdcall Create        ( IUnknown * obj, HWND parent, LPCRECT rect );  
   BOOL     __stdcall Pretranslate  ( MSG * msg );

// events
public :
  void OnChartObjectChanged ( IUnknown * obj, DWORD flags, LPARAM lParam ) ; 
   
// overridable
protected : 
   virtual void OnCreate        () {}
   virtual void OnDestroy       () {}
   virtual void OnObjectChanged () {} 
   virtual BOOL OnPretranslate  ( LPMSG pMsg ) { return FALSE ; } 

protected : 


   IUnknownPtr                   m_Object ; 
   scoped_ptr<listener::Holder>  m_Listener ;
   std::wstring                  m_Name ; 

   CPropPageCore ( UINT dlgId, UINT titleId = 0 ) ; 
  
   BOOL IsLocked      () ;
   void Lock          () ;
   void Unlock        () ;
  
   BOOL IsEventLocked () ;
   void LockEvent     () ;
   void UnlockEvent   () ;

   BEGIN_COM_MAP( CPropPageCore )
      COM_INTERFACE_ENTRY( ntiapp::INtiPanel )
   END_COM_MAP()

   BEGIN_MSG_MAP( CPropPageCore )
      MESSAGE_HANDLER( WM_INITDIALOG   , OnInitDialog   )
      MESSAGE_HANDLER( WM_DESTROY      , OnDestroy      )
   END_MSG_MAP()

   LRESULT OnInitDialog   ( UINT, WPARAM, LPARAM, BOOL & ) ;
   LRESULT OnDestroy      ( UINT, WPARAM, LPARAM, BOOL & ) ;

//
private :
   void OnFinalMessage( HWND ) ; 
  
   DWORD m_dwLockDialog ; 
   DWORD m_dwLockEvent ; 
   DWORD m_dwDialogID ;

} ;

#include "proppage.hpp"
