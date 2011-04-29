#pragma once 

namespace cg
{
   struct Empty {};

   template <class Stream> inline void write(Stream &, cg::Empty  ) {}
   template <class Stream> inline void read (Stream &, cg::Empty &) {}
} ;      
