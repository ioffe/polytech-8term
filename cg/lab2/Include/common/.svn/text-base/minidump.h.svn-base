#pragma once
#include <psapi.h>
#include <shlwapi.h>
#include <dbghelp.h>
#include <boost/format.hpp>
#include "common/on_scope_exit.h"

#pragma comment(lib, "psapi.lib")

namespace error
{
   namespace impl
   {
      typedef BOOL (WINAPI *minidump_write_func_t)(HANDLE process,
                                                   DWORD pid,
                                                   HANDLE file,
                                                   MINIDUMP_TYPE,
                                                   CONST PMINIDUMP_EXCEPTION_INFORMATION,
                                                   CONST PMINIDUMP_USER_STREAM_INFORMATION,
                                                   CONST PMINIDUMP_CALLBACK_INFORMATION);
   }


   template <bool ShowMessageBox>
   struct minidump_writer_t
   {
      minidump_writer_t()
      {
         SetUnhandledExceptionFilter(exception_filter);
      }

   private:
      typedef boost::wformat wfmt;

      static LONG WINAPI exception_filter(EXCEPTION_POINTERS* ex_pointers)
      {
         HMODULE const debug_dll = LoadLibrary(L"dbghelp.dll");
         if (debug_dll == NULL)
            return report_error(L"Can't load dbghelp.dll");

         ON_SCOPE_EXIT(&FreeLibrary, debug_dll);

         impl::minidump_write_func_t const dump_func = 
            reinterpret_cast<impl::minidump_write_func_t>(GetProcAddress(debug_dll, "MiniDumpWriteDump"));

         if (dump_func == NULL)
            return report_error(L"Can't find MiniDumpWriteDump function. Are you running XP?");

         std::wstring const path = get_minidump_path();

         HANDLE const file = CreateFile(path.c_str(),
                                        GENERIC_WRITE,
                                        FILE_SHARE_WRITE,
                                        NULL,
                                        CREATE_ALWAYS,
                                        FILE_ATTRIBUTE_NORMAL,
                                        NULL);
         if (file == INVALID_HANDLE_VALUE)
            return report_error(L"Can't create dump file " + path);

         ON_SCOPE_EXIT(&CloseHandle, file);

         MINIDUMP_EXCEPTION_INFORMATION ex_info = {0};
         ex_info.ThreadId = GetCurrentThreadId();
         ex_info.ExceptionPointers = ex_pointers;

         BOOL const ok = dump_func(GetCurrentProcess(),
                                   GetCurrentProcessId(),
                                   file,
                                   MiniDumpNormal,
                                   &ex_info,
                                   NULL,
                                   NULL);
         if (!ok)
            return report_error(L"Failed to write minidump");

         if (ShowMessageBox)
         {
            std::wstring const msg = L"Program has crashed.\n"
                                     L"Please consider sending the minidump to developers.\n"
                                     L"Minidump is located at\n" +
                                     path;
            MessageBox(0, msg.c_str(), L"Minidump saved", MB_ICONINFORMATION | MB_OK);
         }

         return EXCEPTION_EXECUTE_HANDLER;
      }

      static LONG report_error(std::wstring const& msg)
      {
         if (ShowMessageBox)
         {
            wfmt fmt(L"%s\nLast error 0x%08x");
            fmt.exceptions(boost::io::no_error_bits);

            MessageBox(NULL,
                       str(fmt % msg % HRESULT_FROM_WIN32(GetLastError())).c_str(),
                       L"Failed to save minidump",
                       MB_ICONERROR | MB_OK);
         }

         return EXCEPTION_CONTINUE_SEARCH;
      }

      static std::wstring get_minidump_path()
      {
         wchar_t dump_root[MAX_PATH] = {0};
         if (GetModuleFileName(NULL, dump_root, MAX_PATH))
         {
            BOOL const b = PathRemoveFileSpec(dump_root);
            assert(b);
         }
         else
         {
            DWORD const dw = GetTempPath(MAX_PATH, dump_root);
            assert(dw);
         }

         static const size_t bufsize = 64;

         wchar_t module_name[bufsize] = {0};
         DWORD const r1 = GetModuleBaseName(GetCurrentProcess(), GetModuleHandle(NULL), module_name, bufsize);
         if (r1 == 0)
            wcscpy(module_name, L"unknown-process");

         wchar_t date[bufsize] = {0};
         int const r2 = GetDateFormat(LOCALE_USER_DEFAULT, 0, NULL, L"yyyy'-'MM'-'dd", date, bufsize);
         if (r2 == 0)
            wcscpy(date, L"unknown-date");
         
         wchar_t time[bufsize] = {0};
         int const r3 = GetTimeFormat(LOCALE_USER_DEFAULT, TIME_FORCE24HOURFORMAT, NULL, L"HH'-'mm'-'ss", time, bufsize);
         if (r3 == 0)
            wcscpy(time, L"unknown-time");

         wfmt fmt(L"%s%s%s %s %s.mdmp");
         fmt.exceptions(boost::io::no_error_bits);

         return str(fmt % dump_root % (dump_root[0] == L'\0' ? L"" : L"\\") % module_name % date % time);
      }
   };


   typedef minidump_writer_t<true>         minidump_writer;
   typedef minidump_writer_t<false> silent_minidump_writer;
}
