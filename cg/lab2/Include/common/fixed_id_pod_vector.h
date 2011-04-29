#pragma once

#include <stack>
#include <boost/optional.hpp>

namespace util
{
   template< class T >
   struct fixed_id_pod_vector
   {
   private:
      typedef std::vector<T>             container_t;
      typedef std::vector<size_t>        indices_t;
      typedef std::vector<unsigned char> presence_t;

   private:
      template< class T, class fixed_id_pod_vector_t >
      struct iterator_impl
         : public std::iterator<std::forward_iterator_tag, T>
      {
         iterator_impl()
            : vec_(NULL)
            , id_ (0)
         {}

         iterator_impl( fixed_id_pod_vector_t * vec, size_t id )
            : vec_(vec)
            , id_ (id)
         {
            if (id_ > vec_->size())
               id_ = vec_->size();

            if (vec_->size() != id_ && !vec->valid(id))
               find_next();
         }

         iterator_impl & operator ++ ()
         {
            find_next();
            return *this;
         }

         iterator_impl operator ++ ( int )
         {
            iterator_impl tmp(*this);
            ++*this;
            return tmp;
         }

         reference operator *  () const { return  (*vec_)[id_]; }
         pointer   operator -> () const { return &(*vec_)[id_]; }

         operator iterator_impl< T const, fixed_id_pod_vector_t const > ()
         {
            return iterator_impl< T const, fixed_id_pod_vector_t const >(vec_, id_);
         }

         friend bool operator == ( iterator_impl const & a, iterator_impl const & b )
         {
            return a.id_ == b.id_ && a.vec_ == b.vec_;
         }

         friend bool operator != ( iterator_impl const & a, iterator_impl const & b )
         {
            return !(a == b);
         }

         size_t id() const
         {
            Assert(id_ != vec_->size());
            return id_;
         }

      private:
         void find_next()
         {
            Assert(id_ != vec_->size());
            do
            {
               ++id_;
            } while(id_ != vec_->size() && !vec_->valid(id_));
         }
         
      private:
         size_t id_;
         fixed_id_pod_vector_t * vec_;
      };            

   public:
      typedef typename iterator_impl< T      , fixed_id_pod_vector >        iterator;
      typedef typename iterator_impl< T const, fixed_id_pod_vector const >  const_iterator;

      iterator       begin()        { return iterator(this, 0); }
      iterator       end()          { return iterator(this, data_.size()); }

      const_iterator begin() const  { return const_iterator(this, 0); }
      const_iterator end()   const  { return const_iterator(this, data_.size()); }

   public:
      fixed_id_pod_vector()
      {}

      size_t next_id() const
      {
         if (indices_.empty())
            return data_.size();

         return indices_.back();
      }

      size_t insert( T const& t )
      {
         size_t id = -1;
         if (indices_.empty())
         {
            id = data_.size();
            data_.push_back(t);
            presence_.push_back(true);
         }
         else
         {
            id = indices_.back();
            data_[id] = t;
            presence_[id] = true;
            indices_.pop_back();
         }

         return id;
      }

      bool insert( size_t id, T const& t )
      {
         if (id < data_.size())
         {
            if (!presence_[id])
            {
               indices_t::iterator const it = std::find(indices_.begin(), indices_.end(), id);
               Assert(it != indices_.end());

               std::iter_swap(it, indices_.end() - 1);
               indices_.pop_back();
            }
         }
         else
         {
            size_t const oldSize = data_.size();
            data_.resize(id + 1);
            presence_.resize(id + 1, false);
            indices_.reserve(id + 1);
            for (size_t i = oldSize; i < id; i++)
               indices_.push_back(i);
         }

         bool res = presence_[id];
         data_[id] = t;
         presence_[id] = true;
         return res;
      }

      void erase( size_t id )
      {
         Assert(presence_[id]);
         presence_[id] = false;
         indices_.push_back(id);
      }

      void clear()
      {
         data_.clear();
         indices_.clear();
         presence_.clear();
      }

      void reserve( size_t size )
      {
         data_.reserve(size);
         indices_.reserve(size);
         presence_.reserve(size);
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
         return (id < data_.size()) && presence_[id];
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

      void swap( fixed_id_pod_vector & v )
      {
         using std::swap;
         swap(data_, v.data_);
         swap(indices_, v.indices_);
         swap(presence_, v.presence_);
      }

      T const& operator[]( size_t id ) const { return data_[id]; }
      T      & operator[]( size_t id )       { return data_[id]; }

      T const& at( size_t id ) const { return data_.at(id); }
      T      & at( size_t id )       { return data_.at(id); }

      iterator find( size_t id )
      {
         return iterator(&data_, id);
      }

      const_iterator find( size_t id ) const
      {
         return const_iterator(&data_, id);
      }

   private:
      container_t data_;
      indices_t indices_;
      presence_t presence_;
   };

   //////////////////////////////////////////////////////////////////////////

   template< class T >
      void swap( fixed_id_pod_vector< T > & a, fixed_id_pod_vector< T > & b )
   {
      a.swap(b);
   }

   template< class Stream, class T >
      void write( Stream & stream, fixed_id_pod_vector< T > const & x )
   {
      T * vdummy = (T *)_alloca(sizeof(T));
      memset(vdummy, 0, sizeof(T));

      size_t size = x.storage_size();
      std::vector< T > v;
      v.reserve(size);
      for(size_t i = 0; i < size; i++)
         v.push_back(x.valid(i) ? x[i] : *vdummy);

      write(stream, v);
   }

   template< class Stream, class T >
      void read( Stream & stream, fixed_id_pod_vector< T > & x )
   {
      T * vdummy = (T *)_alloca(sizeof(T));
      memset(vdummy, 0, sizeof(T));

      size_t size;
      read(stream, size);
      x.reserve(size);

      for (size_t i = 0; i < size; i++)
      {
         T t;
         read(stream, t);

         x.insert(i, t);
         if (memcmp(vdummy, &t, sizeof(T)) == 0)
            x.erase(i);
      }
   }
}
