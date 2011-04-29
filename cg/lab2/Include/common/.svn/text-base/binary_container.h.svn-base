#pragma once

#pragma pack( push, 1 )

namespace utils
{

   // simple and fast vector replacement for objects w/o sensible constructor and destructor

   template<class T>
   class binary_container
   {

   public:

      binary_container()
         : data_(NULL)
         , reserved_(0)
         , size_(0)
      {
      }

      ~binary_container()
      {
         if (data_ != NULL)
            free(data_);
      }

      inline void resize( size_t new_size_ )
      {
         if (reserved_ < new_size_)
            reserve(new_size_);
         size_ = new_size_;
      }

      inline void reserve( size_t new_reserved_ )
      {
         if (reserved_ < new_reserved_)
         {
            const size_t next_reserved_step = __min(reserved_, 2048);
            reserved_ = __max(new_reserved_, reserved_ + next_reserved_step);
            data_ = (T *)realloc(data_, sizeof(T) * reserved_);
            Assert(data_);
         }
      }

      inline bool empty() const
      {
         return !size_;
      }

      inline size_t size() const
      {
         return size_;
      }

      inline void push_back( T const& val )
      {
         resize(size_ + 1);
         memcpy(data_ + size_ - 1, &val, sizeof(T));
      }

      inline void pop_back()
      {
         if (size_ > 0)
            size_--;
      }

      inline T const& back() const
      {
         return data_[size_ - 1];
      }

      inline T & back()
      {
         return data_[size_ - 1];
      }

      inline T const& operator[] ( size_t idx ) const
      {
         return data_[idx];
      }

      inline T & operator[] ( size_t idx )
      {
         return data_[idx];
      }

      inline T * begin()
      {
         return data_;
      }
      inline T const* begin() const
      {
         return data_;
      }

      inline T * end()
      {
         return data_ + size_;
      }
      inline T const* end() const
      {
         return data_ + size_;
      }

      size_t capacity() const
      {
         return reserved_;
      }

      void clear() 
      {
         size_ = 0;
      }

   private:

      T * data_;
      size_t reserved_, size_;

   };

   template < class T >
   size_t memory_used( binary_container< T > const & t )
   {
      return t.capacity() * sizeof(T);
   }

};

#pragma pack( pop )