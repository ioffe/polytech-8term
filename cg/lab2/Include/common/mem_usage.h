#pragma once 

struct mem_stat
{
   size_t bytes ; 
   size_t objects ; 

   mem_stat () : bytes(0), objects(0) {}
   mem_stat (size_t bytes, size_t objects = 1) : bytes(bytes), objects(objects) {}

   mem_stat& operator += (mem_stat const& usage) 
   {
      bytes   += usage.bytes ; 
      objects += usage.objects ; 

      return *this ; 
   }
} ; 

inline mem_stat operator + (mem_stat const& u1, mem_stat const& u2) 
{
   return mem_stat(u1) += u2 ; 
}

struct mem_usage 
{
   mem_stat cpu ; 
   mem_stat buffer ; 
   mem_stat texture ; 

   mem_usage () {} 

   mem_usage& operator += (mem_usage const& usage) 
   {
      cpu     += usage.cpu ; 
      buffer  += usage.buffer ; 
      texture += usage.texture ; 

      return *this ; 
   }
} ; 

inline mem_usage cpu_mem_usage( mem_stat const& stat )
{
   mem_usage usage ; 
   usage.cpu = stat ; 
   return usage ; 
}

inline mem_usage buffer_mem_usage( mem_stat const& stat )
{
   mem_usage usage ; 
   usage.buffer = stat ; 
   return usage ; 
}

inline mem_usage texture_mem_usage( mem_stat const& stat )
{
   mem_usage usage ; 
   usage.texture = stat ; 
   return usage ; 
}

inline mem_usage operator + (mem_usage const& u1, mem_usage const& u2) 
{
   return mem_usage(u1) += u2 ; 
}

template<class T>
   mem_usage get_full_mem_usage ( T const& t ) 
   {
      __if_exists(T::get_full_mem_usage)
      {
         return t.get_full_mem_usage(); 
      }
      __if_not_exists(T::get_full_mem_usage)
      {
         return cpu_mem_usage(sizeof(T)) + get_mem_usage(t); 
      }
   }

template<class T>
   mem_usage get_mem_usage ( T const& t ) 
   {
      __if_exists(T::get_mem_usage)
      {
         return t.get_mem_usage() ; 
      }
      __if_not_exists(T::get_mem_usage)
      {
         __if_exists(T::operator ->)
         {
            if ( t ) 
               return get_full_mem_usage(*t) ; 
            return mem_usage() ; 
         }
         __if_not_exists(T::operator ->)
         {
            return mem_usage() ; 
         }
      }
   } 

// forwards
namespace std 
{
   template <class Type, class Allocator> class vector ; 
   template <class CharType, class Traits, class Allocator > class basic_string ; 
   template <class Key, class Type, class Traits, class Allocator > class map ; 
   template <class Key, class Traits, class Allocator > class set ; 
   template <class Type1, class Type2> struct pair ; 
   template <class Type, class Allocator> class list ; 
   template <class Type, class Allocator> class deque ; 
   template <class Type> class auto_ptr ;
} ;   

namespace boost
{
   template <class Type> class scoped_array ;
} ;

namespace cg
{
   template <class T, class S, class D> struct array_2d ; 
} 

template<class First, class Second>
   mem_usage get_mem_usage ( std::pair<First, Second> const& pair ) 
   {
      return get_mem_usage(pair.first) + get_mem_usage(pair.second); 
   }

template<class T>
   mem_usage get_mem_usage ( boost::scoped_array<T> const& array, size_t size ) 
   {
      if ( T const * ptr = array.get() )
      {
         mem_usage usage = cpu_mem_usage(sizeof(T)*size) ;  

         for ( size_t i = 0 ; i != size ; ++i ) 
            usage += get_mem_usage(ptr[i]) ; 

         return usage ; 
      }

      return mem_usage() ; 
   }

template<class T, class A>
   mem_usage get_mem_usage ( std::vector<T, A> const& vector ) 
   {
      mem_usage usage = cpu_mem_usage(sizeof(T) * vector.capacity()) ; 

      for ( std::vector<T, A>::const_iterator i = vector.begin() ; i != vector.end() ; ++i ) 
         usage += get_mem_usage(*i) ; 

      return usage ; 
   }

template<class T, class S, class D>
   mem_usage get_mem_usage ( cg::array_2d<T, S, D> const& array) 
   {
      mem_usage usage = cpu_mem_usage(sizeof(T) * array.size()) ; 

      for ( cg::array_2d<T>::const_iterator i = array.begin() ; i != array.end() ; ++i ) 
         usage += get_mem_usage(*i) ; 

      return usage ; 

   }


