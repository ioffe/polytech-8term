#pragma once 
#include "boost\noncopyable.hpp"

struct win_timer 
   : boost::noncopyable
{     
   template<class T>
      win_timer( T * client, void (T::*event)(), DWORD delay ) 
         : hWindow_ ( (new window<T>(client, event, delay)) -> m_hWnd ) 
      {
      }

   ~win_timer ()
   {
      ::DestroyWindow( hWindow_ ) ; 
   }
private:
   template<class T>
      struct window : CWindowImpl< window<T>, CWindow, CNullTraits >
      {     
         typedef void (T::*event_t)(); 

         window ( T * client, event_t event, DWORD delay ) 
            : pClient_ ( client )
            , pEvent_  ( event ) 
         {
            Create ( HWND_MESSAGE ) ;
            SetTimer ( 1, delay ) ; 
         }

         void OnFinalMessage ( HWND ) 
         {
            delete this ; 
         }

         LRESULT OnTimer ( UINT /*uMsg*/, WPARAM /*wParam*/, LPARAM /*lParam*/, BOOL& /*bHandled*/ ) 
         {
            (pClient_ ->*pEvent_) () ; 
            return 0 ; 
         }
      
         BEGIN_MSG_MAP(window)
            MESSAGE_HANDLER(WM_TIMER, OnTimer)
         END_MSG_MAP()

      private:
         T       * pClient_ ;   
         event_t   pEvent_ ;
      } ; 

private:
   HWND hWindow_ ; 
} ; 


