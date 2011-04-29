#pragma once

#include <ostream>

struct Indent
{
   Indent(std::ostream &out, long ind = 2)
      :  out_(out)
      ,  old_(out_.iword(0))
   {
      out_.iword(0) += ind;
   }

   Indent& operator += (long ind)
   {
      out_.iword(0) += ind;
      return *this;
   }

   Indent& operator -= (long ind)
   {
      out_.iword(0) -= ind;
      return *this;
   }

   struct spaces
   {
      spaces() { std::fill(buf_, buf_ + 1000, ' ');}

      void put_symbol(long len, char sym)
      {
         buf_[len < 1000 ? len : 1000] = sym;
      }

      const char * get() const { return buf_; }

      struct setter
      {
         setter(spaces & s, long pos)
            :  s_(s), pos_(pos) 
         {
            s_.put_symbol(pos_, 0);
         }

         ~setter()
         {
            s_.put_symbol(pos_, ' ');
         }
      private:
         spaces & s_;
         long     pos_;
      };

   private:
      char buf_[1000];
   };

   template <class T>
   std::ostream& operator << (T const & x)
   {
      static spaces s;

      spaces::setter _ (s, out_.iword(0));

      return out_ << s.get() << x;
   }

   ~Indent()
   {
      out_.iword(0) = old_;
   };

private:
   std::ostream   & out_;
   long             old_;
};

