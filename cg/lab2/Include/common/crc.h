#pragma once


inline DWORD cyclic_redundancy_check( LPCVOID data, DWORD size, DWORD initial_crc = 0UL )
{
   static DWORD crc_table[256];
   static bool initialized = false;

   if (!initialized)
   {
      for (size_t i = 0; i < 256; i++)
      {
         DWORD crc_table_accum = (DWORD)i << 24;
         for (size_t j = 0; j < 8; j++)
         {
            static const DWORD polynomal = 0x04C11DB7;
            if (crc_table_accum & 0x80000000L)
               crc_table_accum = (crc_table_accum << 1) ^ polynomal;
            else
               crc_table_accum = (crc_table_accum << 1);
         }
         crc_table[i] = crc_table_accum;
      }

      initialized = true;
   }

   LPCBYTE data_ptr = (LPCBYTE)data;
   DWORD crc_accum = initial_crc;

   while (size-- > 0)
   {
      size_t crc_table_idx = (crc_accum >> 24 ) ^ *data_ptr++;
      Assert(crc_table_idx < 256);
      crc_accum = (crc_accum << 8 ) ^ crc_table[crc_table_idx];  
   }

   return crc_accum;
}

inline DWORD cyclic_redundancy_check( LPCVOID begin, LPCVOID end, DWORD initial_crc = 0UL )
{
   return cyclic_redundancy_check(begin, (LPCBYTE)end - (LPCBYTE)begin, initial_crc);
}

