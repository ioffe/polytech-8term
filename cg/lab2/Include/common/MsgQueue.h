#pragma once 

#include <common\lock.h>

class CMessageQueue
{
   #pragma pack(push, 1)
   struct Stamp 
   {
      short type ; 
      DWORD size ; 
      DWORD tick ; 
      char  data[] ; 
   } ; 
   #pragma pack(pop)

public : 
   CMessageQueue ( DWORD size ) : 
      m_dwSize     ( size ), 
      m_dwLength   ( 0 ), 
      m_dwReadPos  ( 0 ), 
      m_dwWritePos ( 0 ), 
      m_dwDataSize ( 0 ), 
      m_pBuffer    ( NULL ), 
      m_hReadEvent ( NULL )
   {
      m_pBuffer = (LPBYTE)VirtualAlloc ( NULL, m_dwSize, MEM_COMMIT, PAGE_READWRITE ) ; 

      m_hReadEvent  = CreateEvent ( NULL, FALSE, FALSE, NULL ) ; 
      m_hWriteEvent = CreateEvent ( NULL, TRUE,  FALSE, NULL ) ; 
   }
   ~CMessageQueue ()
   {
      CloseHandle ( m_hReadEvent ) ; 
      CloseHandle ( m_hWriteEvent ) ; 
      
      VirtualFree ( m_pBuffer, 0, MEM_RELEASE ) ;
   }

public : 
   void WriteMessage ( short type, const void * data, DWORD size, DWORD tick ) 
   {
      while ( !DoWriteMessage ( type, data, size, tick ) ) 
         WaitForSingleObject ( m_hReadEvent, 1000 ) ; 
   }

   void WaitForEmpty () 
   {
      while ( m_dwDataSize != 0 ) 
         WaitForSingleObject ( m_hReadEvent, 1000 ) ; 
   }

public : 
   void WaitMessage ( DWORD delay ) 
   {
      if ( m_dwDataSize == 0 ) 
         WaitForSingleObject ( m_hWriteEvent, delay ) ; 
   }

   void MsgWaitMessage ( DWORD delay ) 
   {
      if ( m_dwDataSize == 0 ) 
         MsgWaitForMultipleObjects ( 1, &m_hWriteEvent, FALSE, delay, QS_ALLINPUT ) ; 
   }

   BOOL PeekMessage  ( short * type, const void ** data, DWORD * size, DWORD * tick ) 
   {
      lock::scoped_lock locker(m_cs) ; 

      if ( m_dwDataSize == 0 ) 
         return FALSE ; 

      if ( m_dwReadPos == m_dwLength )
      {
         m_dwLength  = m_dwWritePos ; 
         m_dwReadPos = 0 ; 
      }

      Stamp& stamp = *(Stamp *)(m_pBuffer + m_dwReadPos) ; 

      *type = stamp.type ;
      *size = stamp.size ;
      *tick = stamp.tick ;
      *data = stamp.data ; 

      return TRUE ; 
   }

   void ClearMessage  () 
   {
      lock::scoped_lock locker(m_cs) ; 

      Stamp& stamp = *(Stamp *)(m_pBuffer + m_dwReadPos) ; 

      DWORD stampsize = sizeof(Stamp) + stamp.size ; 

      m_dwDataSize -= stampsize ; 

      m_dwReadPos += stampsize ; 

      if ( m_dwReadPos == m_dwSize ) 
         m_dwReadPos = 0 ; 

      if ( m_dwDataSize == 0 ) 
      {
         m_dwReadPos  = 0 ;
         m_dwWritePos = 0 ;
         m_dwLength   = 0 ;
         ResetEvent ( m_hWriteEvent ) ; 
      }

      SetEvent ( m_hReadEvent ) ; 
   }

public : 
   void Touch () 
   {
      lock::scoped_lock locker(m_cs) ; 

      for ( DWORD i = 0 ; i < m_dwSize ; i ++ ) 
         m_pBuffer[i] = (BYTE)(i & 0xFF) ; 
   }

private : 
   DWORD            m_dwSize     ; 
   DWORD            m_dwLength   ; 
   DWORD            m_dwReadPos  ;
   DWORD            m_dwWritePos ;
   DWORD            m_dwDataSize ; 
   LPBYTE           m_pBuffer    ; 
   HANDLE           m_hReadEvent ;
   HANDLE           m_hWriteEvent ;

   lock::critsec    m_cs         ; 

   BOOL DoWriteMessage ( short type, const void * data, DWORD size, DWORD tick ) 
   {
      lock::scoped_lock locker(m_cs) ; 

      DWORD stampsize = sizeof(Stamp) + size ; 

      BOOL full ; 
      if ( m_dwWritePos < m_dwReadPos ) 
         full = stampsize > (m_dwReadPos - m_dwWritePos) ; 
      else if ( m_dwWritePos != m_dwReadPos || m_dwDataSize == 0 ) 
      {
         if ( stampsize <= m_dwSize - m_dwWritePos ) 
            full = FALSE ; 
         else if ( stampsize <= m_dwReadPos ) 
         {
            m_dwWritePos = 0 ; 
            full = FALSE ; 
         }
         else full = TRUE ; 
      }
      else 
         full = TRUE ; 

      if ( full ) 
         return FALSE ; 

      Stamp& stamp = *(Stamp *)(m_pBuffer + m_dwWritePos) ; 

      stamp.type = type ; 
      stamp.size = size ; 
      stamp.tick = tick ; 
      memcpy ( stamp.data, data, size ) ;  

      m_dwWritePos += stampsize ; 

      if ( m_dwWritePos > m_dwReadPos )
            m_dwLength = m_dwWritePos ; 

      if ( m_dwWritePos == m_dwSize ) 
            m_dwWritePos = 0 ; 

      if ( m_dwDataSize == 0 ) 
            SetEvent ( m_hWriteEvent ) ; 

      m_dwDataSize += stampsize ; 

      return TRUE ; 
   }
} ; 

