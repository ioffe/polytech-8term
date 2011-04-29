#pragma once

#include <vector>

// direct listener
template <class Event, class Base = Event>
   struct Listener : Base
{
   Listener() ; 

   bool Init  (IUnknown *environment, Event *pThis) ;
   void Init  (IUnknown *environment, IUnknownPtr pThis);
   void Deinit();

private:
   DWORD       m_dwCookie;
   IUnknownPtr m_Container;
};

// listener via proxy
namespace listener
{
   template< class Owner, class Event > struct Proxy ; 

   struct ProxyBase ; 
   struct NoEvent ; 

   struct Holder
   {
      ~Holder () ;
   protected: 
      struct ProxyBase * proxy_ ; 
   } ; 

   template<class Event1, class Event2 = NoEvent, class Event3 = NoEvent, class Event4 = NoEvent, class Event5 = NoEvent > 
      struct Subscriber : Holder
      {
         template<class Owner>
            Subscriber ( IUnknown * container, Owner * owner, REFIID iid1 = __uuidof(Event1) );
      } ; 

   template<class Event> 
      struct Subscriber<Event, NoEvent, NoEvent, NoEvent, NoEvent> : Holder
      {
         template<class Owner>
            Subscriber ( IUnknown * container, Owner * owner, REFIID iid = __uuidof(Event) );
      } ; 

} // namespace listener

//////////////////////////////////////////////////////////////////////////
// use case 
//      // implement proxy (once for event) 
//      namespace listener 
//      {
//         template<class Owner>
//            struct Proxy<Owner, IMyEvent> : impl::ImplBase<Owner, IMyEvent>
//         {
//            void __stdcall OnMyMethod () 
//            { 
//               owner_ -> OnMyMethod() ;
//            }
//         } ; 
//      }
//
//      // dynamic subscription 
//      struct Owner
//      {
//         void Init ( IUnknown * container )
//         {
//            subscriber_.reset( new listener::Subscriber<IMyEvent1, IMyEvent2, ...>( container, this ));
//         }
//         
//         void Deinit()
//         {
//            subscriber_.reset();
//         }
//         
//         void OnMyMethod()
//         { 
//         }
//
//      private:
//         boost::scoped_ptr<listener::Holder> subscriber_;
//      };
//
//      // static subscription 
//      struct Owner
//      {
//         Owner ( IUnknown *container )
//            : subscriber_ (container, this)
//         {
//         }
//         
//         void OnMyMethod()
//         { 
//         }
//
//      private:
//         Subscriber<IMyEvent1, IMyEvent2, ...> subscriber_;
//      };

//////////////////////////////////////////////////////////////////////////
// implementation 

template <class Event, class Base>
   Listener<Event,Base>::Listener()
      : m_dwCookie(0) 
   {}

template <class Event, class Base>
   bool Listener<Event,Base>::Init(IUnknown *environment, Event *pThis)
   {
      _com_util::CheckError( AtlAdvise(m_Container = environment, pThis, __uuidof(Event), &m_dwCookie));
      // todo: report if advise fails
      return (m_dwCookie != 0);
   }

template <class Event, class Base>
   void Listener<Event,Base>::Init(IUnknown *environment, IUnknownPtr pThis)
    {
      _com_util::CheckError( AtlAdvise(m_Container = environment, pThis, __uuidof(Event), &m_dwCookie));        
      // todo: report if advise fails
    }

template <class Event, class Base>
   void Listener<Event,Base>::Deinit()
   {
      _com_util::CheckError(AtlUnadvise(m_Container, __uuidof(Event), m_dwCookie));
      m_dwCookie = 0;
      m_Container = 0;
   }

namespace listener
{
   struct ProxyBase 
   {
      virtual void Deinit () = 0 ; 
   } ; 

   namespace impl
   {
      template< class Owner, class Event, class ComThreadModel = CComMultiThreadModel >
         struct ImplBase
            : public CComObjectRootEx< ComThreadModel >
            , public Event 
            , public ProxyBase
         {
         	HRESULT _InternalQueryInterface(REFIID iid, void** ppvObject) 
            {
	            if (ppvObject == NULL)
		            return E_POINTER;
	            *ppvObject = NULL;
	            if (InlineIsEqualUnknown(iid) || IsEqualIID(iid, iid_)) 
               {
			         Event * pUnk = this ;
			         pUnk->AddRef();
			         *ppvObject = pUnk;
			         return S_OK;
               }
	            return E_NOINTERFACE;
            }
            void Init ( IUnknown * container, Owner* owner, REFIID iid ) 
            {
               iid_ = iid ; 
               AddRef () ; 
               if ( AtlAdvise ( container, this, iid_, &cookie_) == S_OK ) 
               {
                  owner_     = owner ; 
                  container_ = container ; 
               }
            }

            void Deinit()
            {
               if ( container_ != NULL ) 
                  AtlUnadvise ( container_, iid_, cookie_ );

               owner_     = NULL ; 
               container_ = NULL ; 

               Release () ;
            }

         protected:
            Owner *     owner_ ;

         private:
            IUnknownPtr container_ ; 
            DWORD       cookie_ ; 
            IID         iid_ ; 
         };

      template< class Owner, class Event > 
         ProxyBase * CreateProxy ( IUnknown * container, Owner * owner, REFIID iid ) 
         {
            com_impl_obj<Proxy<Owner, Event> > proxy ; 

            proxy -> Init ( container, owner, iid ) ;
            return proxy ;
         }

      struct MultiProxy : ProxyBase
      {
         void Deinit () 
         {
            for ( size_t i = 0 ; i != proxy_.size() ; i ++ ) 
               proxy_[i] -> Deinit () ; 
            delete this ;
         }

         template<class Owner, class Event> void Add ( IUnknown * container, Owner * owner, Event *, REFIID iid = __uuidof(Event)) 
         {
            proxy_.push_back( CreateProxy<Owner,Event> ( container, owner, iid ) ) ; 
         }

         template<class Owner> void Add ( IUnknown * container, Owner * owner, NoEvent * ) 
         {
         }
      private:
         std::vector<ProxyBase *> proxy_ ; 
      } ; 
   } // namespace impl

   inline Holder::~Holder () 
   {
      proxy_ -> Deinit (); 
   } ; 

   template<class Event1, class Event2, class Event3, class Event4, class Event5 > template<class Owner>
      Subscriber<Event1,Event2,Event3,Event4,Event5>::Subscriber ( IUnknown * container, Owner * owner, REFIID iid1 )
      {
         impl::MultiProxy * proxy = new impl::MultiProxy ; 
         proxy -> Add ( container, owner, (Event1 *)NULL, iid1 ) ;
         proxy -> Add ( container, owner, (Event2 *)NULL ) ; 
         proxy -> Add ( container, owner, (Event3 *)NULL ) ; 
         proxy -> Add ( container, owner, (Event4 *)NULL ) ; 
         proxy -> Add ( container, owner, (Event5 *)NULL ) ; 

         proxy_ = proxy ; 
      }

   template<class Event> template<class Owner>
      Subscriber<Event, NoEvent, NoEvent, NoEvent, NoEvent>::Subscriber ( IUnknown * container, Owner * owner, REFIID iid )
      {
         proxy_ = impl::CreateProxy<Owner,Event> ( container, owner, iid ) ;  
      }
} // namespace listener

