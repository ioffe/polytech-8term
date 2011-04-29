#pragma once

namespace util
{
   // Simplify vector 'through COM' transfer
   template< class T >
      struct vector_wrapper
   {
      typedef T         value_type;
      typedef T *       iterator;
      typedef T const * const_iterator;

      vector_wrapper( T *data, size_t num_elements )
         : data (data)
         , num_elements (num_elements)
      {}

      vector_wrapper()
         : data( NULL )
         , num_elements( 0 )
      {}

      void reset( T *data, size_t num_elements )
      {
         this->data = data;
         this->num_elements = num_elements;
      }

      bool     valid()  const { return (num_elements == 0 && data == NULL) || (num_elements != 0 && data != NULL); }
      bool     empty()  const { return num_elements == 0; }
      size_t   size()   const { return num_elements; }

      T const & operator [] (size_t idx) const { return data[idx]; }
      T       & operator [] (size_t idx)       { return data[idx]; }

      T const & at(size_t idx) const { return data[idx]; }
      T       & at(size_t idx)       { return data[idx]; }

      T       * begin()       { return data; }
      T const * begin() const { return data; }
      T       * end()         { return data + num_elements; }
      T const * end()   const { return data + num_elements; }

      T const & back()  const { return (data + num_elements)[-1]; }
      T       & back()        { return (data + num_elements)[-1]; }
      T const & front() const { return *data; }
      T       & front()       { return *data; }

   private:
      T *data;
      size_t num_elements;
   };
   
   template< class T >
      struct vector_wrapper<T const>
   {
      typedef T value_type;
      typedef T * iterator;
      typedef T const * const_iterator;

      vector_wrapper( T const * data, size_t num_elements )
         : data (data)
         , num_elements (num_elements)
      {}

      vector_wrapper()
         : data( NULL )
         , num_elements( 0 )
      {}

      void reset( T const * data, size_t num_elements )
      {
         this->data = data;
         this->num_elements = num_elements;
      }

      bool     valid()  const { return (num_elements == 0 && data == NULL) || (num_elements != 0 && data != NULL); }
      bool     empty()  const { return num_elements == 0; }
      size_t   size()   const { return num_elements; }

      T const & operator [] (size_t idx) const { return data[idx]; }

      T const & at(size_t idx) const { return data[idx]; }

      T const * begin() const { return data; }
      T const * end()   const { return data + num_elements; }

      T const & back()  const { return (data + num_elements)[-1]; }
      T const & front() const { return *data; }

   private:
      T const * data;
      size_t num_elements;
   };
}