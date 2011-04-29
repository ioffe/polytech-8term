#pragma once 

#include <strstream>
#include <iomanip>

//struct ModuleHeapId  
//{
//   ModuleHeapId ( HMODULE hModule ) 
//   {
//      ptr_ = new char [256] ; 
//      strcpy ( ptr_, "ModuleHeapId:" ) ; 
//      GetModuleFileName ( hModule, ptr_ + 13, 256 - 13 ) ; 
//   }
//   ~ModuleHeapId () 
//   {
//      delete ptr_ ; 
//   }
//   LPSTR ptr_ ;
//} ; 
//
//inline void RegisterModule ( HMODULE hModule ) 
//{
//   static ModuleHeapId id ( hModule ) ; 
//}


inline void DumpMemoryBlocks(  LPCSTR type )
{
   //typedef std::map<LPVOID, std::string> heap_region_map ; 
   //heap_region_map heap_regions ; 

   //HANDLE heaps[1024] ; 
   //DWORD heapCount = GetProcessHeaps ( 1024, heaps ) ; 

   //for ( DWORD i = 0 ; i < heapCount ; i ++ ) 
   //{
   //   std::vector<LPVOID> regions ; 
   //   std::string         module ( "unknown" ); 

   //   PROCESS_HEAP_ENTRY entry = { 0 } ;
   //   while ( HeapWalk ( heaps[i], &entry ) ) 
   //   {
   //      if ( entry.wFlags == PROCESS_HEAP_REGION ) 
   //      {
   //         regions.push_back(entry.lpData) ; 
   //      }
   //      else if ( entry.wFlags == PROCESS_HEAP_ENTRY_BUSY ) 
   //      {
   //         LPCSTR data = (LPCSTR)entry.lpData + 32 ; 
   //         if ( entry.cbData > 32 + 13 + 1 && memcmp ( data, "ModuleHeapId:", 13 ) == 0 ) 
   //            module = data + 13 ; 
   //      }
   //   }

   //   for ( size_t j = 0 ; j < regions.size() ; j ++ ) 
   //      heap_regions[regions[j]] = module ; 
   //}


   std::ostrstream stream ; 
   
   DWORD freeCount = 0, freeTotal = 0, freeMax = 0 ; 

   DWORD privateTotal = 0, imageCount   = 0 ; 
   DWORD imageTotal   = 0, mappedCount  = 0 ; 
   DWORD mappedTotal  = 0, privateCount = 0 ; 
   DWORD sblTotal     = 0, sblCount     = 0 ; 
   DWORD dviewTotal   = 0, dviewCount   = 0, dviewWasted = 0  ; 
   

   DWORD ptr = 0 ;
   
   MEMORY_BASIC_INFORMATION mbi ;
   while  ( VirtualQuery ( (void *)ptr, & mbi, sizeof ( mbi ) ) == sizeof ( mbi ) ) 
   {
      DWORD size = mbi.RegionSize ; 
      
      stream << "Memory block:" << std::hex << std::setw(8) << (DWORD)mbi.BaseAddress << " size:" << std::dec << std::setw(10) << size ; 

      switch ( mbi.State ) 
      {
         case MEM_FREE    : stream << " free     " ; break ; 
         case MEM_RESERVE : stream << " reserved " ; break ; 
         case MEM_COMMIT  : stream << " commited " ; break ;      
      }     
      
      if ( mbi.State == MEM_FREE ) 
      {
         freeCount ++ ; 
         freeTotal += size ; 
         if ( freeMax < size ) 
            freeMax = size ;  
      }
      else 
      {
         switch ( mbi.Type ) 
         {
            case MEM_IMAGE    : stream << " image   " ; imageTotal   += size ; imageCount  ++ ; break ; 
            case MEM_MAPPED   : stream << " mapped  " ; mappedTotal  += size ; mappedCount ++ ; break ; 
            case MEM_PRIVATE  : stream << " private " ; privateTotal += size ; privateCount++ ; break ;      
         }     
         
         stream << "base:" << std::hex << std::setw(8) << (DWORD)mbi.AllocationBase ; 
            
         char buf[500] ; 
         if ( GetModuleFileName ( (HMODULE)mbi.AllocationBase, buf, sizeof ( buf ) ) != 0 )
         {
            stream << " module:" << buf ; 
         }

         if ( mbi.State == MEM_COMMIT && mbi.Type == MEM_MAPPED && mbi.AllocationBase == mbi.BaseAddress )
         {
            LPCBYTE base   = (LPCBYTE)mbi.BaseAddress ; 
            DWORD   dvsize = *(DWORD *)base ;

            if ( dvsize + sizeof(DWORD) + sizeof(DWORD) < size && *(DWORD *)(base + dvsize + sizeof(DWORD)) == 0x12345678 ) 
            { 
               LPCSTR caption = (LPCSTR)(base + dvsize + sizeof(DWORD) + sizeof(DWORD)) ; 

               stream << " dataview:" << std::dec << std::setw(10) << dvsize << " " << caption ; 

               dviewWasted += size - dvsize ; 
               dviewTotal  += size ; 
               dviewCount ++ ; 
            }
         }

         if ( mbi.State == MEM_COMMIT && mbi.Type == MEM_MAPPED &&  mbi.AllocationBase == mbi.BaseAddress )
         {
            LPCSTR sbl = "SeaGull Binary Library" ; 
            if ( memcmp ( mbi.BaseAddress, sbl, strlen(sbl) ) == 0 ) 
            { 
               stream << sbl; 
               sblTotal += size ; 
               sblCount ++ ; 
            }
         }

         //heap_region_map::const_iterator hi = heap_regions.find(mbi.AllocationBase) ; 
         //if ( hi != heap_regions.end () ) 
         //{
         //   stream << " module heap:" << hi -> second ; 
         //}
      }     
      
      stream << "\r\n" ; 
      
      ptr += size ; 
   }
   
   stream << std::ends;    

   std::ostrstream fstream ; 

   fstream << "Memory state\r\n" ; 
   
   fstream << "Free     bytes: total " << std::setw(12) << freeTotal    << ", count " << std::setw(8) << freeCount    << ", largest " << freeMax << "\r\n" ; 
   fstream << "Private  bytes: total " << std::setw(12) << privateTotal << ", count " << std::setw(8) << privateCount << "\r\n" ; 
   fstream << "Mapped   bytes: total " << std::setw(12) << mappedTotal  << ", count " << std::setw(8) << mappedCount  << "\r\n" ; 
   fstream << "Image    bytes: total " << std::setw(12) << imageTotal   << ", count " << std::setw(8) << imageCount   << "\r\n" ; 
   fstream << "Sbl      bytes: total " << std::setw(12) << sblTotal     << ", count " << std::setw(8) << sblCount     << "\r\n" ; 
   fstream << "DataView bytes: total " << std::setw(12) << dviewTotal   << ", count " << std::setw(8) << dviewCount   << ", wasted " << dviewWasted << "\r\n" ; 
   
   fstream << "Memory dump\r\n" << stream.str ();  
   
   fstream << std::ends;    
   
   IDebugMessageStreamPtr ( __uuidof(DebugMessageStream) ) -> InfoMessage ( type, fstream.str () ) ; 
}    


