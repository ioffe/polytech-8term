#pragma once

#include "geometry\primitives\point.h"

#include "raster_2.h"

namespace cg 
{
    struct indexer2d 
    {
      point_2i    extents;
  
      indexer2d(point_2i const &extents) : extents(extents) {}
  
      int operator () (point_2i const &c) const {
        return c.y * extents.x + c.x;
      }
  
      void check_index(point_2i const &c) const {
        Assert(0 <= c.x);
        Assert(0 <= c.y);
        Assert(c.x < extents.x);
        Assert(c.y < extents.y);
      }
    };

    template <class T>
        struct Raster2D : raster_2
    {
	    typedef T Data;

	    Raster2D(raster_2_params const &rp, T defval = T(), double angle=0.0)
		    :	raster_2(rp.org(),rp.unit(),rp.extents(), angle)
		    ,	data_   (rp.width() * rp.height(), defval)
	    {}

	    Raster2D(point_2 const &origin = point_2(), 
			     point_2 const &Unit = point_2(1,1), 
			     point_2i const &ext = point_2i(),
                 double angle=0.0) 
                 : cg::raster_2 (origin, Unit, ext, angle)
		    , data_ (ext.x * ext.y)
	    {}
	    
	    raster_2_params rparam() const {
		    return raster_2_params(bounding(*this),unit());
	    }

	    Raster2D(raster_2 const &r)
		    :	raster_2(r)
		    ,	data_	(r.extents().x * r.extents().y)
	    {}


        void set_correct_size() {
            data_.resize( extents().x * extents().y );
        }

	    typename std::vector<Data>::reference operator [] (point_2i const &c) {
		    check_index(c);
		    std::vector<Data>::reference r = data_[index(c)];
		    return r;
	    }

	    typename std::vector<Data>::const_reference operator [] (point_2i const &c) const {
		    check_index(c);
		    return data_[index(c)];
	    }

	    Data & operator [] (point_2 const &pt) 
	    { return operator[](raster_2::operator()(pt)); }

	    Data const & operator [] (point_2 const &pt) const 
	    { return operator[](raster_2::operator()(pt)); }
	    
	    size_t allocated_memory_in_bytes () const {
		    return data_.size() * sizeof(Data) + sizeof(*this);
	    }

	    // сквозной итератор по клеткам
	    typedef typename std::vector<Data>::const_iterator const_iterator;

	    const_iterator begin() const { return data_.begin(); }
	    const_iterator end () const  { return data_.end();   }

        // возвращает линеаризованный массив
        std::vector<Data> const & getData() const {
            return data_;
        }

        // подменяет данные
        void swap(std::vector<Data> &v) {
            data_.swap(v);
        }

        void swap(std::vector<Data> &v, raster_2 const &r) {
            raster_2::assign(r);
            swap(v);
        }

    private:
	    std::vector<Data>	data_;

	    void check_index(point_2i const &c) const {
		    Assert(0 <= c.x);
		    Assert(0 <= c.y);
		    Assert(c.x < width());
		    Assert(c.y < height());
	    }

	    int  index(point_2i const &c) const {
		    return c.y * width() + c.x;
	    }
    };
}

