#pragma once

#pragma warning(push)
#pragma warning(disable : 4100)

namespace cg
{
    template <class T, size_t N>
        struct vector_fixed_capacity
    {
        typedef T           value_type;
        typedef T       &   reference;
        typedef T const &   const_reference;
        typedef T       *   pointer;
        typedef T const *   const_pointer;
        typedef T       *   iterator;
        typedef T const *   const_iterator;

        vector_fixed_capacity () : size_(0) {}

        explicit vector_fixed_capacity (size_t sz) 
           : size_(sz)
        {
           Assert(sz <= N);
           construct_n(begin(), size_);
        }

        vector_fixed_capacity (size_t sz, const_reference defval) 
            : size_(sz)
        {
            Assert(sz <= N);
            construct_n(begin(), size_, defval);
        }

        vector_fixed_capacity (vector_fixed_capacity const &v)
           : size_(v.size_)
        {
           construct(begin(), v.begin(), v.end());
        }

        template<size_t M>
        vector_fixed_capacity (vector_fixed_capacity<T, M> const &v)
            : size_(v.size())
        {
           Assert(v.size() <= N);
           construct(begin(), v.begin(), v.end());        
        }

        ~vector_fixed_capacity()
        {
           destruct_n(begin(), size_);
        }

        void clear() 
        {
            destruct_n(begin(), size_);
            size_ = 0;
        }

        void resize(size_t sz)
        {
            Assert(sz <= N);
            if ( size_ > sz )
               destruct_n(begin() + sz, size_ - sz);
            else
               construct_n(end(), sz - size_);

            size_ = sz;
        }

        void resize(size_t sz, value_type const& x)
        {
           Assert(sz <= N);
           if ( size_ > sz )
              destruct_n(begin() + sz, size_ - sz);
           else
              construct_n(end(), sz - size_, x);

           size_ = sz;
        }

        void push_back (value_type const & x)
        {
            Assert(!full());
            construct(end(), x);
            ++size_ ;
        }

        void pop_back() 
        {
            Assert(!empty());
            --size_;
            destruct(end());
        }

        void erase( iterator p ) 
        {
           Assert(!empty());
           Assert(p < end());
           --size_ ;
           iterator e = end();
           if ( p == e )
              destruct(p);
           else
           {
              using std::swap;
              swap(*p, *e);
              destruct(e);
           }
           // ToDo: implementation above don't save elements order
        }

        void erase( iterator p, iterator q )
        {
            Assert( begin( ) <= p );
            Assert( p <= q ) ;
            Assert( q <= end( ) );

            destruct_n(p, q - p);
            memmove(p, q, (end() - q) * sizeof(T));
            size_ -= q - p;
        }

        iterator  insert( iterator p, const_reference x )
        {
           Assert(!full());
           memmove(p + 1, p, (end() - p) * sizeof(T));
           construct(p, x);
           ++size_;
           return p;
        }

        vector_fixed_capacity & operator = (vector_fixed_capacity const &v)
        {
           return make_eq(v);
        }

        template<size_t M>
        vector_fixed_capacity & operator = (vector_fixed_capacity<T, M> const &v)
        {
           Assert(v.size() <= N);
           return make_eq(v);
        }                    

        size_t  size     () const { return size_; }
        size_t  capacity () const { return N;}
        bool    empty    () const { return 0 == size_; }
        bool    full     () const { return N == size_; }

        reference       operator [] (int idx)        { return data()[idx]; }
        const_reference operator [] (int idx) const  { return data()[idx]; }

        reference       at (int idx)       { check_valid(idx); return data()[idx]; } 
        const_reference at (int idx) const { check_valid(idx); return data()[idx]; } 

        bool contains (int idx) const { return 0 <= idx && idx < size_; } 

        void check_valid(int idx) const 
        {
            Assert (0 <= idx); 
            Assert (idx < (int)size_);
        }

              iterator  begin()       { return data(); }
        const_iterator  begin() const { return data(); }

              iterator  end  ()       { return data() + size_; }
        const_iterator  end  () const { return data() + size_; }

              reference back()        { Assert(!empty()); return *(end() - 1); }
        const_reference back()  const { Assert(!empty()); return *(end() - 1); }

              reference front()       { Assert(!empty()); return *begin(); }
        const_reference front() const { Assert(!empty()); return *begin(); }

        bool contains (const_iterator it) const { return data_ <= it && it < data_ + size_; }

    private:
       template<size_t M>
         vector_fixed_capacity & make_eq(vector_fixed_capacity<T, M> const &v)
       {
          if ( size_ >= v.size() )
          {
             iterator e = std::copy(v.begin(), v.end(), begin());
             destruct_n(e, size_ - v.size());
          }
          else
          {
             const_iterator med  = v.begin() + size_;
             iterator last = std::copy(v.begin(), med, begin());
             construct(last, med, v.end());
          }

          size_ = v.size();
          return *this;
       }

    private:
       void construct(iterator p)                      {  new(p) T() ; }
       void construct(iterator p, const_reference x)   {  new(p) T(x) ; }

       void destruct (iterator p)                      {  (*p). ~T() ; }

       void construct_n(iterator p, size_t count)
       {
          for ( iterator q = p + count; p != q; ++p)
             construct(p);
       }

       void construct(iterator p, const_iterator i, const_iterator j)
       {
          for ( ; i != j; ++i, ++p)
             construct(p, *i);
       }

       void construct_n(iterator p, size_t count, const_reference x)
       {
          for ( iterator q = p + count; p != q; ++p)
             construct(p, x);
       }

       void destruct_n(iterator p, size_t count)
       {
          for ( iterator q = p + count; p != q; ++p)
            destruct(p);
       }

    private:
       T      * data()       { return reinterpret_cast< T       * >(data_.bytes); }
       T const* data() const { return reinterpret_cast< T const * >(data_.bytes); }

    private:
       union
        {
           char bytes[ sizeof(T) * N ];

// Если будут проблемы с align-ом           
//           typename boost::type_with_alignment<
//              boost::alignment_of<value_type>::value >::type aligner;

        } data_ ;

       size_t          size_;
    };
}

#pragma warning(pop)