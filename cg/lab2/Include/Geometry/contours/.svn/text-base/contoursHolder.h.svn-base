#pragma once

namespace cg
{
#pragma pack ( push, 1 )
    //  Массив контуров. 
    //  Содержательная часть: заводит дополнительные массивы, для 
    //  быстрого нахождения по номеру отрезка контура, которому он принадлежит,
    //  а также для предоставления итератора по всем отрезкам (он нужен только при построении грида)
    //  getItem() - по индексу отрезка получить его координаты
    //  getContourBySegment() - по индексу отрезка получить остров, его содержащий
    //  distance() - растояние между двумя итераторами по отрезкам
    template <class Storage>
        struct ContoursHolder : Storage
    {
        typedef typename Storage::point_id   point_id;
        typedef typename Storage::segment_id segment_id;
        typedef typename Storage::contour_id contour_id;

        // получить по id отрезка сам отрезок
        // sid должен быть валидным
        __forceinline segment_2 getSegment (segment_id sid) const
        {
            return segment_2 (
                vertices()[getSegmentStartPoint(sid)], 
                vertices()[getSegmentEndPoint  (sid)]);
        }        

       
        // по отрезку получить индекс точки его начала
        __forceinline point_id getSegmentStartPoint (segment_id sid) const
        {
            Assert( valid( sid ) );
            return static_cast<point_id>(1 + segments2points()[sid]);
        }

        
        // по отрезку получить индекс точки его конца
        __forceinline point_id getSegmentEndPoint (segment_id sid) const
        {
            // Это верно, поскольку мы каждый контур дополняем еще одной точкой
            return segments2points()[sid];
        }

        
        // получить по точке, контур ее содержащий
        __forceinline contour_id getContourByPoint (point_id pid) const
        {
            Assert( valid( pid ) );
            return points2contours()[pid];
        }

        
        // получить по отрезку, контур его содержащий
        __forceinline contour_id getContourBySegment (segment_id sid) const
        {
            return getContourByPoint(getSegmentStartPoint(sid));
        }


        // id первого контура
        __forceinline contour_id contoursBegin() const { return (contour_id) 0; }

        // id контура, который следует за последним валидным
        __forceinline contour_id contoursEnd()   const { return (contour_id) contours().size(); }

        // id первой точки контура
        __forceinline point_id   pointsBegin (contour_id cid) const 
        {
            // первая точка отрезка принудительно добавлена
            return contours()[cid].startIdx();
        }

        // свозной итератор по точкам,указывающий на первую точку
        __forceinline point_id pointsBegin() const
        {
            return static_cast<point_id>(0);
        }

        __forceinline point_id pointsEnd() const
        {
            return static_cast<point_id>(vertices().size());
        }

        // id последней точки контура
        __forceinline point_id  lastPoint (contour_id cid) const
        {
            return static_cast<point_id>(contours()[cid].stopIdx() - 1);
        }

        // Антон Ковалев! Посмотри на эту ф-цию и на то, что было вместо нее до этого!
        __forceinline point_id  pointsEnd (contour_id cid) const
        {
            return static_cast<point_id>(contours()[cid].stopIdx());
        }

        // по id точки получить ее координаты
        __forceinline typename Storage::point_type const & getPoint (point_id pid) const { return vertices()[pid]; }

        // id невалидного сегмента
        __forceinline static segment_id defaultSegmentId () { return static_cast<segment_id>(-1); }

        // id контура по умолчанию
        __forceinline static contour_id defaultContourId () { return static_cast<contour_id>(-1); }

        // id контура, который содержит заданный контур
        __forceinline static contour_id getContainingContour(contour_id cid) { return defaultContourId(); }

        __forceinline bool valid( segment_id sid ) const
        {
          return 0 <= sid && static_cast< size_t >( sid ) < segments2points().size();
        }

        __forceinline bool valid( contour_id cid ) const
        {
          return 0 <= cid && static_cast< size_t >( cid ) < contours( ).size( );
        }

        __forceinline bool valid( point_id pid ) const
        {
          return 0 <= pid && static_cast< size_t >( pid ) < points2contours( ).size( );
        }

        // По id сегмента выдать id следующего за ним в контуре сегмента
        __forceinline segment_id getNextSegment (segment_id sid) const
        {
            Assert( valid( sid ) );
          
            segment_id nsid;

            if (sid == 0)
            {
                nsid = static_cast<segment_id>(contours()[0].size() - 2);
                Assert( valid( nsid ) );
                return nsid;
            }

            nsid = static_cast<segment_id>(sid - 1);
            Assert( valid( nsid ) );

            // Просто проверяем, принадлежит отрезок ли следующий отрезок тому же контуру...
            contour_id  curContour = getContourBySegment(sid);
            contour_id  nxtContour = getContourBySegment(nsid);

            if (curContour != nxtContour)
                nsid = static_cast<segment_id>(nsid + contours()[curContour].size() - 1);

            Assert( valid( nsid ) );
            return nsid;
        }

        // По id сегмента выдать id предыдущего за ним в контуре сегмента
        __forceinline segment_id getPrevSegment (segment_id sid) const
        {
            Assert( valid( sid ) );
            segment_id psid = static_cast<segment_id>(sid + 1);

            if (psid == static_cast<segment_id>(segments2points().size()))
            {
              segment_id res = static_cast<segment_id>(psid - contours().back().size() + 1); 
              Assert( valid( res ) );
              return res;
            }


            // опять же проверяем принадлежность отрезка тому же контуру
            contour_id  curContour = getContourBySegment(sid);
            contour_id  prvContour = getContourBySegment(psid);

            Assert( valid( curContour ) );


            if (curContour != prvContour)
                psid = static_cast<segment_id>(psid - contours()[curContour].size() + 1);

            Assert( valid( psid ) );

            return psid;
        }

        // По id отрезка получить смежный с ним в контуре отрезок
        // next задает направление: следующий или предыдущий
        __forceinline segment_id getAdjacentSegment (segment_id sid, bool prev) const
        {
            return !prev ? getNextSegment(sid) : getPrevSegment(sid);
        }

        __forceinline int distance ( segment_id p, segment_id q) const
        {
            return static_cast<int> ( q - p );
        }
        
        __forceinline cg::segment_2 getItem ( segment_id seg_idx ) const
        {
            return getSegment ( seg_idx );
        }

        __forceinline cg::point_2 getItem ( point_id pnt_idx ) const
        {
            return getPoint ( pnt_idx );
        }

        // сквозной итератор по отрезкам, указывающий на первый отрезок
        __forceinline segment_id segmentsBegin() const
        {
            return static_cast<segment_id>(0);
        }

        // сквозной итератор по отрезкам, указывающий за последний отрезок
        __forceinline segment_id segmentsEnd() const
        {
            return static_cast<segment_id>(segments2points().size());
        }

        __forceinline contour_id getExtContour ( contour_id con_id ) const
        {
//            return con_id;

            if ( con_id == defaultContourId() )
                return con_id;

            return ( contours()[ con_id ].attr().is_inner_of != defaultContourId() ? 
                     contours()[ con_id ].attr().is_inner_of : 
                     con_id );
        }


        __forceinline ContoursHolder const  & contoursHolder() const { return *this; }
        __forceinline ContoursHolder        & contoursHolder()       { return *this; }
    };

#pragma pack ( pop )
}



