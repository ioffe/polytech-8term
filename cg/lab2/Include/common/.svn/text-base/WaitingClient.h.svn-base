#ifndef __WAITINGCLIENT_H
#define __WAITINGCLIENT_H

class ATL_NO_VTABLE CWaitingSessionClient : 
	public CComObjectRootEx<CComMultiThreadModel>,
	public ISessionClient,
     private lcheck::resource<CWaitingSessionClient>
{
public:
     CWaitingSessionClient () 
     {
     } 
    ~CWaitingSessionClient () 
     {
     } 

     void InitInstance ( IUnknown * switcher ) 
     {
          m_Switcher = switcher ; 
     }

DECLARE_GET_CONTROLLING_UNKNOWN()
DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CWaitingSessionClient)
	COM_INTERFACE_ENTRY(ISessionClient)
END_COM_MAP()

DECLARE_DEBUG ( "NTR_WaitingClient")

// ISessionClient
public :
     void         __stdcall OnSessionBegin    ( ISessionServer * server ) 
     {
         m_Server = server ; 
         server -> SendAll ( NT_ASSIGN_TASK_READY, NULL, 0, FALSE ) ; 
     } 
     void         __stdcall OnSessionStop     () 
     {
         m_Server   = NULL ; 
         m_Switcher = NULL ; 
     }
     void         __stdcall OnReceive         ( short type, const void * data, DWORD size, short sender ) 
     {
         if ( type == NT_MISC_SWITCH_TO_SESSION ) 
         {
              __TRACE ( "Switching ... \r\n" ) ; 
              ISessionServerPtr old ; 
              ISessionClientPtr lock ( GetUnknown () ) ; 

              m_Server -> Unadvise () ; 

              m_Switcher -> SwitchToSession ( m_Server, &old ) ; 
              m_Server -> SendLogControl ( LC_REQUEST_STATE, NULL, 0 ) ; 

              if ( old )
                   old -> Advise ( this, FALSE ) ; 
              else m_Server = NULL ; 
         }
     }
     void         __stdcall OnReceiveExternal ( short type, const void * data, DWORD size, short sender, short trnNum ) {}
     void         __stdcall OnStart           () {} 
     void         __stdcall OnPause           () {} 
     void         __stdcall OnContinue        () {} 
     void         __stdcall OnStop            () {} 
     void         __stdcall OnAccelFactor     ( double ) {}
     void         __stdcall OnSetTime         ( double ) {}
     void         __stdcall OnRingIncluded    ( short  ) {}  
     void         __stdcall OnRingExcluded    ( short  ) {}  

private : 
     ISystemSessionSwitchPtr m_Switcher ; 
     ISessionServerPtr       m_Server ; 
} ; 


#endif
