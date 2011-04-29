#pragma once
#include "Geometry\Polygon_2.h"
#include "contours\common.h"

namespace cg
{
    namespace prs
    {
        // Ёти типы сделаны enum'ами только дл€ того, чтобы получать радость от перегрузки
        using contours::contour_id;
        using contours::point_id;
        using contours::segment_id;

#pragma pack(push,1)

        template <class ContourAttr>
        struct contour
        {
            // перва€ точка контура
            point_id startIdx() const { return startIdx_; }

            point_id stopIdx()  const { return static_cast<point_id>(startIdx_ + length_); }

            // количество точек и отрезков в контуре
            size_t size() const { return length_; }

            ContourAttr const & attr() const { return attr_; }

            contour (point_id start, size_t len, ContourAttr const & Attr) 
                : startIdx_(start)
                , length_(len)
                , attr_(Attr) 
            {}

        private:
            point_id startIdx_;
            size_t   length_;
            ContourAttr     attr_;
        };

        template <class Stream, class ContourAttr>
           inline void write(Stream & out, typename contour<ContourAttr> const & c)
        {
           write(out, c.startIdx());
           write(out, c.size());
           write(out, c.attr());
        }


#pragma pack(pop)
    }

    // TODO: заменить std::vector<... на отображение <key, value>

    //  ласс, который хранит контура
    // ѕозвол€ет добавл€ть в себ€ новые контура
    // все интересные функции доступа реализуютс€ наследниками
    template < class Point, class ContourAttr >
    struct ContoursHolderBuilder
    {
        typedef prs::contour_id contour_id;
        typedef prs::point_id   point_id;
        typedef prs::segment_id segment_id;

        typedef prs::contour<ContourAttr>  contour;

        typedef Point         point_type;
        typedef ContourAttr   contour_attr_type;

        // »спользуетс€ из Contours... должен быть только один метод
        // надо бы сделать это почище... но как заставить об€зательно передавать тип контура? 
        contour_id addContour (std::vector<Point> const & c, ContourAttr const & attr, bool FilterDuplicatePoints = true )
        {
            point_id startPointIdx = pointsCount();

            segments2points_.push_back (pointsCount());

            vertices_.push_back (Point(c[0]));

            points2contours_.push_back (contoursCount());            

            size_t curSize = 1;

            for (int k = c.size() - 1; k >= 0; --k)  
            {
                Point pt (c[k]);

                Assert ( pt.x > -1e6 );

                if ( ( !FilterDuplicatePoints && k == 0 ) || !cg::eq (pt, vertices_.back()) )
                {
                    segments2points_.push_back(pointsCount());

                    vertices_.push_back(pt);

                    points2contours_.push_back(contoursCount());

                    ++curSize;
                }
            }
            segments2points_.pop_back();

            contours_.push_back ( contour ( startPointIdx, curSize, attr ) );

            return points2contours_.back ();
        }

        // »спользуетс€ из CoastLines
        template <class FwdIter>
            contour_id addContour ( FwdIter first, FwdIter beyond, ContourAttr const & attr )
        {
            Assert (first != beyond);

            point_id startPointIdx = pointsCount();

            segments2points_.push_back (pointsCount());

            vertices_.push_back (*first);

            points2contours_.push_back (contoursCount());            

            size_t curSize = 1;

            FwdIter p = beyond;

            do 
            {
                --p;

                Point pt (*p);

                Assert ( pt.x > -1e6 );

                segments2points_.push_back(pointsCount());

                vertices_.push_back(pt);

                points2contours_.push_back(contoursCount());

                ++curSize;

            } while (p != first);

            segments2points_.pop_back();

            contours_.push_back ( contour ( startPointIdx, curSize, attr ) );

            return points2contours_.back();
        }

        template <class Stream>
            void serialize (Stream & out) const
        {
            write(out, vertices_);
            write(out, contours_);
            write(out, segments2points_);
            write(out, points2contours_);
        }
    
   private:
        typedef typename std::vector<Point>        VerticesVec;
        typedef typename std::vector<contour>      ContoursVec;
        typedef typename std::vector<point_id>     Segment2PointMap;
        typedef typename std::vector<contour_id>   Point2ContourMap;

    public:
        VerticesVec        const &  vertices() const { return vertices_; }
        VerticesVec              &  vertices()       { return vertices_; }

        ContoursVec        const &  contours() const { return contours_; }
        ContoursVec              &  contours()       { return contours_; }

        Segment2PointMap   const &  segments2points() const { return segments2points_; }
        Segment2PointMap         &  segments2points()       { return segments2points_; }

        Point2ContourMap   const &  points2contours() const { return points2contours_; }
        Point2ContourMap         &  points2contours()       { return points2contours_; }

        contour_id  contoursCount() const { return static_cast<contour_id>(contours_.size()); }
        point_id    pointsCount()   const { return static_cast<point_id>  (vertices_.size()); }
    private:

        // все точки полигонов
        // отображение є точки --> ее координаты
        VerticesVec        vertices_;

        // ќтображение є контура --> (начало контура, длина контура)
        ContoursVec        contours_;

        // отображение є отрезка --> є точки (первой)
        // втора€ точка получаетс€ прибавлением единицы
        Segment2PointMap   segments2points_;

        // отображение є точки --> є контура
        Point2ContourMap   points2contours_;
    };
}
