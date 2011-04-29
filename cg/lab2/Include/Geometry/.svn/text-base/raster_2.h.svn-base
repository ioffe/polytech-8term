#pragma once
#include "common/safe_bool.h"
#include "segment_2_intersection.h"

namespace cg {

    // подготавливает параметры инициализации растра
    struct raster_2_params {

            // domain - область пространства, дл€ которой будут хранитьс€ данные
            // step_size - размер €чейки по вертикали и горизонтали
            raster_2_params(rectangle_2 const &d, point_2 const &step_size)
                    :   domain_(d), unit_(step_size)
            {}

            point_2 org () const { return domain_.xy(); }
            point_2 unit() const { return unit_; }

            int         width () const { return int(domain_.x.size() / unit_.x) + 1; }
            int     height() const { return int(domain_.y.size() / unit_.y) + 1; }

            point_2i extents() const { return point_2i(width(),height()); }             

            raster_2_params& offset(point_2 const &pt) {
                    domain_.offset(pt); return *this;
            }
            
            raster_2_params& inflate(double d) {
                    domain_.inflate(d); return *this;
            }

    private:

            rectangle_2 domain_;
            point_2             unit_;
    };

    struct raster_2 
    {
            raster_2( point_2  const &origin = point_2 (), 
                              point_2  const &Unit   = point_2 (1,1), 
                              point_2i const &ext    = point_2i(),
                  double angle = 0.0 )
                    : org_(origin), unit_(Unit), ext_(ext)
            , angle(angle)
            {
            calc_sincos();
        }

            raster_2( rectangle_2 const &rc, point_2i const &ext,
                  double angle = 0.0 )
                    : org_(rc.lo()), ext_(ext), unit_(rc.size() / ext)
            , angle(angle)
            {
            calc_sincos();
        }
            
            raster_2( raster_2_params const &rp,
                  double angle = 0.0 )
                    : org_(rp.org()), unit_(rp.unit()), ext_(rp.extents())
            , angle(angle)
            {
            calc_sincos();
        }

        raster_2( double angle )
                    : org_(0,0), unit_(1,1), ext_(0,0)
            , angle(angle)
        {
            calc_sincos();
        }

        raster_2& operator = (raster_2 const &other) { 
            return assign(other);
            }

        raster_2 & assign(raster_2 const &other) {
                    org_  = other.org_; unit_ = other.unit_; ext_ = other.ext_;
            sina  = other.sina; cosa  = other.cosa;
            angle = other.angle;
                    return *this;
        }

            // ѕо точке получаем содержащую ее €чейку
            point_2i operator() (point_2 const &pt) const {
                    return floor(translate(pt));
            }
            
            // ѕо точке получаем ее относительную в сетке позицию
            point_2 translate(point_2 const &pt) const {
    //        return point_2( (pt.x - org_.x) / unit_.x, 
    //                        (pt.y - org_.y) / unit_.y );
            // вращаем точку
            double x =   pt.x * cosa  +  pt.y * sina;
            double y = - pt.x * sina  +  pt.y * cosa;

            return point_2( (x - org_.x) / unit_.x, 
                            (y - org_.y) / unit_.y );
            }

        point_2 translateback(point_2 const &pt) const
        {
            point_2 res_1 = (pt & unit_) + org_;

            point_2 res;
    
            // вращаем обратно
            res.x =   res_1.x * cosa  -  res_1.y * sina;
            res.y =   res_1.x * sina  +  res_1.y * cosa;

            return res;

    //        return pt & unit_ + org_;
        }
  
      // ѕо €чейке получаем левый нижний ее угол
            point_2 operator () (point_2i const &c) const {
    //        return translateback(c);
                    return org_ + (c & unit_);
            }

            // полуоткрытый пр€моугольник, содержащий эту клетку c
            rectangle_2 domain(point_2i const &c) const {
                    return rectangle_by_size(operator()(c),unit_);
            }

        point_2  const & origin () const { return org_; }
        point_2  const & unit   () const { return unit_;}
        point_2i const & extents() const { return ext_; }

        // сама€ лева€ верхн€€ клетка
        point_2i lo() const { return point_2i(0,0); }
        // сама€ права€ нижн€€ валидна€ клетка
        point_2i hi() const { return ext_ - point_2i(1,1); }

        // определ€ет, €вл€етс€ c валидной клеткой
        bool contains(point_2i const &c) const {
                  return 0 <= c.x && c.x < ext_.x && 0 <= c.y && c.y < ext_.y ;
        }
            
        bool is_valid(point_2 const &pt) const {
                    return contains((*this)(pt));
        }


        double getangle() const { return angle; }
        static unsigned size_to_save() { return 2*sizeof(point_2) + sizeof(point_2i) + sizeof(double); }

    private:
            point_2             org_;
            point_2             unit_;
            point_2i    ext_;

        // угол поворота растра
        double      sina;
        double      cosa;
        double      angle;

        void calc_sincos()
        {
            sina = sin(angle);
            cosa = cos(angle);
        }
    };

    inline raster_2_params createRaster(rectangle_2 const &rc, double step_size) {
            return raster_2_params(rc,point_2(step_size,step_size));
    }

    inline raster_2_params createRaster(rectangle_2 const &rc, point_2 step_size) {
            return raster_2_params(rc,step_size);
    }

    // пр€моугольник, который содержит все клетки растра
    inline rectangle_2 bounding(raster_2 const &r)      {
            return rectangle_2( r(r.lo()), r(r.extents()) );
    }

    // возвращает закрытый пр€моугольник, который содержит все клетки, на которые попал R
    inline rectangle_2i translate(raster_2 const &R, rectangle_2 const &B) {
            return rectangle_2i(R(B.lo()),R(B.hi()));
    }

    // ‘ункци€, котора€ возвращает итератор по сетке, перебирающий близкие к org €чейки.
    // ≈сли таких €чеек нет, возвращает невалидный итератор
    inline rectangle_2i::iterator 
            close_cells(raster_2 const &R, point_2 const &center, double radius)
    {
            rectangle_2i R_ascells = translate(R, rectangle_by_sphere(center,radius));
            rectangle_2i raster_bound = rectangle_by_extents(R.extents());
            return R_ascells & raster_bound;
    }

    struct uniform_grid_iterator
    {
            uniform_grid_iterator(point_2i ext, point_2 org, point_2 step_size)
                    :   raster_(org,step_size,ext)
                    ,   cur_(rectangle_by_extents(ext))
            {}

            uniform_grid_iterator(raster_2 r) : raster_(r), cur_(rectangle_by_extents(r.extents())) {}

            point_2 operator * () const { return raster_(*cur_); }

            uniform_grid_iterator& operator ++ () {
                    ++cur_; return *this;
            }

            SAFE_BOOL_OPERATOR(cur_)

    private:
            raster_2 const              raster_;
            rectangle_2i::iterator      cur_;
    };

    inline uniform_grid_iterator createSubdivision3(point_2 const &org, point_2 const &step)
    {
            return uniform_grid_iterator(point_2i(3,3), org - step / 2., step / 2.);
    }

    inline uniform_grid_iterator createGridIterator(point_2 org, point_2 step, int radius)
    {
            point_2 lt = point_2(org) - point_2(step * radius);
            raster_2 r(lt, step, point_2i(radius * 2 + 1, radius * 2 + 1));
            return uniform_grid_iterator(r);
    }

    // создает итератор по сетке из N*N €чеек на пр€моугольнике rc
    inline uniform_grid_iterator createGridIterator(rectangle_2 const &rc, int N) 
    {
            return uniform_grid_iterator(point_2i(N,N),rc.lo(),rc.size()/N);
    }


    inline rectangle_2 translate(rectangle_2 const &rect, raster_2 const &r)
    {
        return rectangle_2( r.translate( rect.lo() ),
                            r.translate( rect.hi() ) );
    }

}