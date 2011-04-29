#pragma once

#include <vector>
#include <string>
#include <map>

#include <boost/function.hpp>

#include "less_nocase.h"

struct SyncSettings
{
   std::vector<std::wstring> sync_exts;

   typedef boost::function<void (size_t, size_t)> progress_callback_t;
   progress_callback_t progress_callback;

   typedef boost::function<void (wchar_t const*)> message_callback_t;
   message_callback_t message_callback;

   typedef boost::function<bool (void)> terminate_callback_t;
   terminate_callback_t terminate_callback;


   void Trace ( wchar_t const* lpszFormat, ... ) const 
   {
      if (message_callback)
      {
         wchar_t buf[10240] ;                                         
         va_list argptr;                                         
         va_start(argptr, lpszFormat);                           
         int ret = _vsnwprintf ( buf, 10240, lpszFormat, argptr) ;  
         if ( ret == -1 )                                        
            buf[10239] = 0 ;                                     
         va_end(argptr);                                         
         
         message_callback(buf) ; 
      }
   }
};


namespace impl
{
   class FSItem
   {
   public:
      FSItem(): 
         isdir_(true), 
         fsize_(0),
         mtime_(0) 
      {}

      FSItem(unsigned size, FILETIME const& ftime):
         isdir_(false), 
         fsize_(size), 
         mtime_(unsigned __int64(ftime.dwLowDateTime) + (unsigned __int64(ftime.dwHighDateTime) << 32))
      {}

      bool is_dir() const
      {
         return isdir_;
      }

      unsigned size() const
      {
         return fsize_;
      }

      __int64 time() const
      {
         return mtime_;
      }

   private:
      bool     isdir_;  
      unsigned fsize_; 
      __int64  mtime_;
   };    

   // checks file size and modification time with 1 second granularity
   inline bool operator != (FSItem const& lhs, FSItem const& rhs)
   {
      return lhs.size() != rhs.size()            ||
             lhs.time() - rhs.time() >  10000000 ||
             lhs.time() - rhs.time() < -10000000;
   }

   typedef std::wstring FSPath;
   typedef std::map<FSPath, FSItem, less_nocase> FSItems ; 

   inline void get_contents(wchar_t const* folder, FSItems& contents)
   {
      contents.clear();

      wchar_t buf[MAX_PATH];
      swprintf(buf, L"%s\\*.*", folder); 

      WIN32_FIND_DATAW fdata; 
      HANDLE hFind = FindFirstFileW(buf, &fdata); 
      while (hFind != INVALID_HANDLE_VALUE) 
      {
         if (fdata.cFileName[0] != L'.') 
         {
            if (fdata.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) 
               contents[fdata.cFileName] = FSItem();
            else
               contents[fdata.cFileName] = FSItem(fdata.nFileSizeLow, fdata.ftLastWriteTime);
         }

         if (!FindNextFileW(hFind, &fdata)) 
         {
            FindClose(hFind); 
            hFind = INVALID_HANDLE_VALUE ; 
         }
      }
   }

   inline bool delete_folder(wchar_t const* folder, SyncSettings const& ss)  
   {
      FSItems items ; 
      get_contents(folder, items); 

      wchar_t path[MAX_PATH]; 
      for (FSItems::const_iterator i = items.begin(); i != items.end(); ++i) 
      {
         swprintf(path, L"%s\\%s", folder, i->first.c_str()); 

         if (i->second.is_dir())
         {
            if (!delete_folder(path, ss)) 
               return false;
         }
         else 
         {
            SetFileAttributesW(path, FILE_ATTRIBUTE_NORMAL); 
            if (!DeleteFileW(path)) 
            {
               ss.Trace ( L"Failed to delete file \"%s\"\r\n", path ) ; 
               return false;
            }
         }     
      }

      if ( ! RemoveDirectoryW(folder) ) 
      {
          ss.Trace ( L"Failed to remove directory \"%s\"\r\n", folder ) ; 
          return false ; 
      }
      return true ; 
   }

   inline std::wstring const extension(FSPath const& path)
   {
      std::wstring::size_type const ix = path.rfind(L'.');
      return ix == std::wstring::npos ? L"" : path.substr(ix + 1);
   }

   inline size_t nfiles_in(std::wstring const& path)
   {
      size_t result = 0;
      WIN32_FIND_DATAW fdata; 
      HANDLE hfind = FindFirstFileW((path + L"\\*.*").c_str(), &fdata); 
      while (hfind != INVALID_HANDLE_VALUE) 
      {
         if (fdata.cFileName[0] != L'.') 
         {
            if (fdata.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) 
               result += nfiles_in(path + L'\\' + fdata.cFileName);
            else
               ++result;
         }

         if (!FindNextFileW(hfind, &fdata)) 
         {
            FindClose(hfind); 
            hfind = INVALID_HANDLE_VALUE ; 
         }
      }
      return result;
   }

   inline bool sync_folders(wchar_t const* from, wchar_t const* to,
                            SyncSettings const& ss,
                            size_t* files_done, size_t files_total)
   {
      if (GetFileAttributesW(from) == INVALID_FILE_ATTRIBUTES) 
      {
         ss.Trace ( L"Failed to access \"%s\"\r\n", from ) ; 
         return false; 
      }

      if (!CreateDirectoryW(to, NULL) && GetLastError() != ERROR_ALREADY_EXISTS) 
      {
         ss.Trace ( L"Failed to create \"%s\"\r\n", to ) ; 
         return false; 
      }

      FSItems src, dst; 
      get_contents(from, src);
      get_contents(to, dst); 

      // first, remove everything, that has to be removed
      wchar_t dst_path[MAX_PATH]; 
      FSItems::iterator i = dst.begin(); 
      while (i != dst.end())
      {
         if (ss.terminate_callback && ss.terminate_callback() ) 
         {
            ss.Trace ( L"Terminated by user\r\n" ) ; 
            return false ; 
         }

         FSItems::const_iterator j = src.find(i->first); 
         if (j == src.end () || i -> second.is_dir() != j -> second.is_dir()) 
         {
            // item not found in source or has different type
            swprintf(dst_path, L"%s\\%s", to, i->first.c_str()); 
            if (i->second.is_dir())
            {
               if (!delete_folder(dst_path, ss))
               {
                  ss.Trace ( L"Failed to delete folder \"%s\"\r\n", dst_path ) ; 
                  return false;
               }
            }
            else 
            {
               SetFileAttributesW(dst_path, FILE_ATTRIBUTE_NORMAL); 
               if (!DeleteFileW(dst_path)) 
               {
                  ss.Trace ( L"Failed to delete file \"%s\"\r\n", dst_path ) ; 
                  return false ;
               }
            }
            ss.Trace ( L"\"%s\" removed\r\n", dst_path ) ; 
            i = dst.erase(i); 
         }
         else
            ++i; 
      }     

      wchar_t src_path[MAX_PATH]; 
      for (FSItems::const_iterator i = src.begin(); i != src.end(); ++i) 
      {
         if (ss.terminate_callback && ss.terminate_callback() ) 
         {
            ss.Trace ( L"Terminated by user\r\n" ) ; 
            return false ; 
         }

         swprintf(src_path, L"%s\\%s", from, i->first.c_str()); 
         swprintf(dst_path, L"%s\\%s", to, i->first.c_str()); 

         if (i->second.is_dir())
         {
            if (!sync_folders(src_path, dst_path, ss, files_done, files_total)) 
               return false; 
         }
         else 
         {
            if (ss.progress_callback)
               ss.progress_callback(*files_done, files_total);

            ++*files_done;

            if (!ss.sync_exts.empty() &&
                std::find(ss.sync_exts.begin(), ss.sync_exts.end(), extension(i->first)) == ss.sync_exts.end())
               continue;

            FSItems::const_iterator const j = dst.find(i->first); 
            if (j == dst.end() || i->second != j->second)
            {
               ss.Trace ( L"Copying \"%s\"...", src_path ) ; 

               SetFileAttributesW(dst_path, FILE_ATTRIBUTE_NORMAL); 
               if (!CopyFileW(src_path, dst_path, FALSE)) 
               {
                  ss.Trace ( L" failed\r\n" ) ; 
                  return false;
               }
               ss.Trace ( L" done\r\n" ) ; 
            }
         }
      }

      return true; 
   }
}


inline bool sync_folders(wchar_t const* from, wchar_t const* to, SyncSettings const& ss = SyncSettings())
{
   size_t cur = 0;
   return impl::sync_folders(from, to, ss, &cur, ss.progress_callback ? impl::nfiles_in(from) : 0 );
}
