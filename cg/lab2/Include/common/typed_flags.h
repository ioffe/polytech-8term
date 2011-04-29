#pragma once


//
// example usage:
//
// enum my_flag_type
// {
//    mf_one   = 1 << 0,
//    mf_two   = 1 << 1,
//    mf_three = 1 << 2
// };
//
// typedef typed_flags<my_flag_type> my_flags_type;
//
// struct s
// {
//    my_flags_type flags;
// };
//


template <typename Enum, typename Storage = DWORD>
struct typed_flags
{
   typedef Enum enum_type;
   typedef Storage storage_type;

   typed_flags( Enum e ) : e_(e) { }
   explicit typed_flags( Storage s ) : s_(s) { }

   typed_flags & operator = ( Enum e ) { s_ = e; return *this; }

   typed_flags & operator &= ( Enum e ) { s_ &= e; return *this; }
   typed_flags & operator |= ( Enum e ) { s_ |= e; return *this; }
   typed_flags & operator ^= ( Enum e ) { s_ ^= e; return *this; }

   typed_flags & operator += ( Enum e ) { s_ |= e; return *this; }
   typed_flags & operator -= ( Enum e ) { s_ &= ~e; return *this; }

   typed_flags operator & ( Enum e ) const { return typed_flags(s_ & e); }
   typed_flags operator | ( Enum e ) const { return typed_flags(s_ | e); }
   typed_flags operator ^ ( Enum e ) const { return typed_flags(s_ ^ e); }

   operator Enum const& () const { return  e_; }


private:

   union
   {
      Storage s_;
      Enum e_;
   };
};
