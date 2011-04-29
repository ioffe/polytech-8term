#pragma once

#include "com_impl_ptr.h"

template <class T1>
   class CComSingleInstanceCreator : protected CComCreator<T1>
   {
   public:
      static HRESULT WINAPI CreateInstance(void* pv, REFIID riid, LPVOID* ppv)
      {
         _ASSERTE(*ppv == NULL);
         if ( T1::Instance () )
            return T1::Instance () -> _InternalQueryInterface(riid, ppv) ;
         else return CComCreator<T1> :: CreateInstance( pv, riid, ppv) ;
      }
   } ;

template <class T1>
   class CSingleWrapper : public T1
   {
   public :
      static T1 *& Instance () 
      { 
         static T1 * instance = NULL ; 
         return instance ; 
      }
      CSingleWrapper () : T1 ()
      {
         Instance () = this ;
      }
      ~CSingleWrapper ()
      {
         Instance () = NULL ;
      }
   } ;

#define DECLARE_SINGLE_INSTANCE(x) public:\
   typedef CComCreator2< CComSingleInstanceCreator< CComObject< CSingleWrapper < x > > >, CComFailCreator<CLASS_E_NOAGGREGATION> > _CreatorClass; 

#define DECLARE_REGISTRY_PROGID_DESCR(progid,descr)\
   static HRESULT WINAPI UpdateRegistry(BOOL bRegister) throw()\
   {\
      return ::UpdateRegistry(bRegister, GetObjectCLSID(), _T(progid), _T(descr));\
   }

#define DECLARE_REGISTRY_PROGID(progid)\
   static HRESULT WINAPI UpdateRegistry(BOOL bRegister) throw()\
   {\
      return ::UpdateRegistry(bRegister, GetObjectCLSID(), _T(progid) );\
   }\
   static LPCSTR GetProgId()\
   {\
      return progid;\
   }

#define DECLARE_REGISTRY_NOPROGID()\
   static HRESULT WINAPI UpdateRegistry(BOOL bRegister) throw()\
   {\
      return ::UpdateRegistry(bRegister, GetObjectCLSID() );\
   }

#define BEGIN_BASE_COM_MAP(x) public: \
	typedef x _ComMapClass; \
	IUnknown* _GetRawUnknown() throw() \
	{ ATLASSERT(_GetEntries()[0].pFunc == _ATL_SIMPLEMAPENTRY); return (IUnknown*)((INT_PTR)this+_GetEntries()->dw); } \
	_ATL_DECLARE_GET_UNKNOWN(x)\
	const static ATL::_ATL_INTMAP_ENTRY* WINAPI _GetEntries() throw() { \
	static const ATL::_ATL_INTMAP_ENTRY _entries[] = { DEBUG_QI_ENTRY(x)


//
#define COM_INTERFACE_ENTRY_AGGREGATE_BLIND_EX(ppunk)\
   {NULL,\
   (DWORD)offsetof(_ComMapClass, ppunk),\
   _DelegateEx},

static HRESULT WINAPI _DelegateEx(void* pv, REFIID iid, void** ppvObject, DWORD dw)
{
   HRESULT hRes = E_NOINTERFACE;
   CComPtr<IUnknown> * pvec = (CComPtr<IUnknown> *)((BYTE *)pv + dw);

   CComPtr<IUnknown> p;

   int i = 0 ;
   while ( (p = pvec[i++]) != NULL )
   {
      hRes = p -> QueryInterface ( iid, ppvObject );
      if ( hRes == S_OK ) break;
   }

   return hRes;
}

//
inline HRESULT UpdateRegistry( BOOL bRegister, REFCLSID clsid, LPCTSTR progid = _T(""), LPCTSTR descr = _T(""), LPCWSTR extra = NULL ) 
{
   static LPCWSTR script_noprogid = 
      L"HKCR "
      L"{ "
      L"  NoRemove CLSID "
      L"  { "
      L"    ForceRemove %CLSID% = s '%DESCRIPTION%' "
      L"    { "
      L"      InProcServer32 = s '%MODULE%' "
      L"      { "
      L"        val ThreadingModel = s 'Apartment' "
      L"      } "
      L"    } "
      L"  } "
      L"} " ; 

   static LPCWSTR script_progid = 
      L"HKCR "
      L"{ "
      L"  %PROGID% =  s '%DESCRIPTION%' "
      L"  { "
      L"    CLSID = s '%CLSID%' "
      L"  } "
      L"  NoRemove CLSID "
      L"  { "
      L"    ForceRemove %CLSID% = s '%DESCRIPTION%' "
      L"    { "
      L"      ProgId = s '%PROGID%' "
      L"      InProcServer32 = s '%MODULE%' "
      L"      { "
      L"        val ThreadingModel = s 'Apartment' "
      L"      } "
      L"    } "
      L"  } "
      L"} " ; 

   USES_CONVERSION;

   LPOLESTR szclsid = NULL ; 
   StringFromCLSID ( clsid, &szclsid );

   wchar_t szModule[MAX_PATH];
   GetModuleFileNameW( _pModule -> GetModuleInstance (), szModule, MAX_PATH );

   CRegObject ro ;

#if _MSC_VER >= 1400
   ro.FinalConstruct();
#endif

   ro.AddReplacement( L"CLSID",       szclsid );
   ro.AddReplacement( L"PROGID",      T2CW(progid) );
   ro.AddReplacement( L"DESCRIPTION", T2CW(descr) );
   ro.AddReplacement( L"MODULE",      szModule );

   LPCWSTR script = *progid ? script_progid : script_noprogid ; 

   HRESULT hr = bRegister ? ro.StringRegister ( script ) : ro.StringUnregister ( script ); 

   if ( hr == S_OK && extra != NULL ) 
      hr = bRegister ? ro.StringRegister ( extra ) : ro.StringUnregister ( extra ); 

   CoTaskMemFree ( szclsid );

   return hr ;
}
