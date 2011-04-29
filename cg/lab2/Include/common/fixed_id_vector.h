#pragma once

#include <stack>
#include <boost/optional.hpp>

namespace util
{
   // TODO :: при необходимости можно написать bidirectional iterator.

   // Generator for indices for elements stored in vector
   template< class T >
      struct fixed_id_vector
   {
   private:
      typedef boost::optional< T >           optional_t ;
      typedef std::vector< optional_t >      container_t ;
      typedef std::vector< size_t >          indices_t ;

   private:
      template< class T, class C >
         struct iterator_impl
            : std::iterator<std::forward_iterator_tag, T>
      {
         iterator_impl()
            : c_(0)
            , id_(0)
         {}

         iterator_impl(C * c, size_t id)
            : c_(c)
            , id_(id)
         {
            if (id_ > c_->size())
               id_ = c_->size() ;

            if (c_->size() != id_ && !(*c_)[id_])
               find_next();
         }

         // Префиксный ++
         iterator_impl & operator++( )
         {
            find_next();
            return *this;
         }

         // Постфиксный ++
         iterator_impl operator++( int )
         {
            iterator_impl tmp( *this );
            ++*this;
            return tmp;
         }

         reference operator* ( ) const   { return  *(*c_)[id_]; }
         pointer   operator->( ) const   { return &*(*c_)[id_]; }

         operator iterator_impl<T const, C const>()
         {
            return iterator_impl<T const, C const>( c_, id_ );
         }

         friend bool operator == ( iterator_impl const& a, iterator_impl const& b )
         { 
            return a.id_ == b.id_ && a.c_ == b.c_; 
         }

         friend bool operator != ( iterator_impl const& a, iterator_impl const& b )
         { 
            return !(a == b);
         }

         size_t id() const { Assert ( id_ != c_->size() ); return id_; }

      private:
         void find_next()
         {
            Assert(id_ != c_->size());
            do { ++id_; }
            while( id_ != c_->size() && !(*c_)[id_] );
         }
         
      private:
         size_t id_ ;
         C * c_ ; 
      };            

   public:
      typedef typename iterator_impl< T      , container_t >        iterator;
      typedef typename iterator_impl< T const, container_t const >  const_iterator;

      iterator       begin()        { return iterator(&data_, 0); }
      iterator       end()          { return iterator(&data_, data_.size()); }

      const_iterator begin() const  { return const_iterator(&data_, 0); }
      const_iterator end()   const  { return const_iterator(&data_, data_.size()); }

   public:
      fixed_id_vector() {}

      size_t next_id() const
      {
         if ( indices_.empty() )
            return data_.size();
         
         return indices_.back() ;
      }

      size_t insert( T const& t ) 
      { 
         size_t id = -1;
         if ( indices_.empty() )
         {
            id = data_.size();
            data_.push_back(optional_t(t)) ;
         }
         else
         {
            id = indices_.back() ;
            data_[id] = t ;
            indices_.pop_back() ; // non throw
         }

         return id;
      }

      bool insert( size_t id, T const& t )   // slow
      { 
         if ( id < data_.size() )
         {
            if ( !data_[id] )
            {
               indices_t::iterator const 
                  it = std::find(indices_.begin(), indices_.end(), id);
               
               Assert( it != indices_.end() );
               
               std::iter_swap(it, indices_.end() - 1);
               indices_.pop_back();
            }
         }
         else
         {
            size_t const oldSize = data_.size() ;
            data_   .resize (id + 1) ;
            // TODO: Maybe reserve like this?
            // indices_.reserve(indices_.size() + id - oldSize);
            indices_.reserve(id + 1) ;
            for ( size_t i = oldSize; i != id; ++i )
               indices_.push_back(i);
         }

         bool res = data_[id];
         data_[id] = t;
         return res;
      }

      void erase( size_t id ) 
      { 
         Assert( data_.at(id) );
         data_.at(id).reset(); 
         indices_.push_back(id);
      }

      void clear() 
      { 
         data_    .clear() ; 
         indices_ .clear() ; 
      }

      void reserve( size_t size ) 
      { 
         data_   .reserve(size) ; 
         indices_.reserve(size) ;
      }
      
      size_t size() const 
      { 
         return data_.size() - indices_.size(); 
      }

      bool empty() const
      {
         return data_.size() == indices_.size();
      }

      bool valid( size_t id ) const 
      { 
         return (id < data_.size()) && (data_[id] ? true : false); 
      }

      size_t storage_size() const 
      { 
         return data_.size();
      }

      size_t free_size() const 
      { 
         return indices_.size();
      }

      size_t capacity() const 
      {
         return data_.capacity();
      }

      void swap( fixed_id_vector & v )
      {
         using std::swap;
         swap(data_,    v.data_   );
         swap(indices_, v.indices_);
      }

      T const& operator[]( size_t id ) const { return *data_[id]; }
      T      & operator[]( size_t id )       { return *data_[id]; }

      T const& at( size_t id ) const { return *data_.at(id); }
      T      & at( size_t id )       { return *data_.at(id); }
  
      iterator find( size_t id )
      {
         return iterator(&data_, id) ;
      }

      const_iterator find( size_t id ) const
      {
         return const_iterator(&data_, id) ;
      }

   private:
      container_t    data_ ;
      indices_t      indices_ ;
   };

   template< class T >
      void swap( fixed_id_vector<T> & v1, fixed_id_vector<T> & v2 )
   {
      v1.swap(v2);
   }

   template <class Stream, class T>
      void write(Stream & stream, fixed_id_vector<T> const & x)
      {
         std::vector<T> v;

         T *vdummy = (T *)_alloca( sizeof(T) ) ;
         memset ( vdummy, 0, sizeof( T ) );

         for(size_t i = 0, size = x.storage_size(); i != size; ++i)
            v.push_back(x.valid(i) ? x.at(i) : *vdummy);

         write(stream, v);
      }

   template <class Stream, class T>
      void read(Stream &stream, fixed_id_vector<T> & x)
      {
         T *vdummy = (T *)_alloca( sizeof(T) ) ;
         memset ( vdummy, 0, sizeof( T ) );

         size_t sz;
         read(stream, sz);

         x.reserve(sz);

         for (size_t i = 0; i != sz; ++i)
         {
            T  t;
            read(stream, t);

            x.insert(i, t);

            if (memcmp(vdummy, &t, sizeof(T)) == 0)
            {
               x.erase(i);
            }
         }
      }
}
