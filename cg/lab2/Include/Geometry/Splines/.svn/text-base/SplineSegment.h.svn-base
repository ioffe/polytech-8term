//Сегмент сплайна + натуральная параметризация
//Пока только эрмитова
//Используется в логике Seagull "Birds"
//Написано: demi
#pragma once


namespace cg
{
#ifndef COUNTOF
#define COUNTOF(A) (sizeof(A) / sizeof*(A))
#endif

   namespace demi_only
   {
      template <class T>
      typename T::scalar_t CurveIntegral (T &f, typename T::scalar_t a, typename T::scalar_t b, int n = 16) //DEP: MakeNatural()
      {
         typename T::scalar_t dt = (b - a) / n;
         if (dt < static_cast< typename T::scalar_t >( 0.01 ))
            dt = static_cast< typename T::scalar_t >( 0.01 );

         typename T::scalar_t t = a;
         typename T::vector_t v1;
         f (t, v1);
         typename T::scalar_t l = 0;
         while (t < b)
         {
            t += dt;
            typename T::vector_t v2;
            f (t, v2);
            l += distance (v1, v2);
            v1 = v2;
         }
         return l;
      }
   }

   //Нам нужна такая штука потому что point_3 не шаблон,
   //но надо знать скаляр, породивший вектор.
   //Потенциально шаблон сплайна может параметризоваться
   //не только point_3.
   template <class T, class Y>
   struct vector_traits
   {
      typedef T scalar_t;
      typedef Y vector_t;
   };


   namespace _traits
   {
      typedef 
         vector_traits <double, cg::point_3>
         point_3;
      typedef
         vector_traits <float, cg::point_3f>
         point_3f;
   }


   //****
   //@ Spline3
   //****
   template <class T>
   //От типа T требуется:
   //  -тип vector_t
   //  -тип scalar_t
   class DECLSPEC_NOVTABLE/*due abstract*/ Spline3Segment: public T
   {
   public:
      typedef T traits;
      using typename T::scalar_t;
      using typename T::vector_t;

   public:
      //Вычислить коэфициенты смешивающих функций
      virtual void Evaluate (scalar_t t, scalar_t h[4]) const = 0;
      //Вычислить коэфициенты производных смешивающих функций
      virtual void Derivate (scalar_t t, scalar_t h[4]) const = 0;
      //Продолжение C(1) - ясно из прототипа (т.к. 2 параметра)
      virtual void Continue (Spline3Segment const &Spline, vector_t const &v, vector_t const &t) = 0;

      virtual void SetEnd0 (vector_t const &v) = 0;
      virtual void SetEnd1 (vector_t const &v) = 0;
      virtual void SetTan0 (vector_t const &v) = 0;
      virtual void SetTan1 (vector_t const &v) = 0;

   public:
      virtual vector_t End0 () const
      {
         vector_t v;
         Get (0, v);
         return v;
      }

      virtual vector_t End1 () const
      {
         vector_t v;
         Get (1, v);
         return v;
      }

      virtual vector_t Tan0 () const
      {
         vector_t v;
         Tangent (0, v);
         return v;
      }

      virtual vector_t Tan1 () const
      {
         vector_t v;
         Tangent (1, v);
         return v;
      }

      void Get (scalar_t x, vector_t &v) const 
      {
         scalar_t k[4];
         Evaluate (x, k);
         v = cp[0]*k[0] + cp[1]*k[1] + cp[2]*k[2] + cp[3]*k[3];
      }

      void Get (scalar_t x, vector_t &v, vector_t &d) const
      {
         Get (x, v);
         Tangent (x, d);
      }

      void Tangent (scalar_t x, vector_t &v) const
      {
         scalar_t k[4];
         Derivate(x, k);
         v = cp[0] * k[0] + cp[1]*k[1] + cp[2]*k[2] + cp[3]*k[3];
      }

      scalar_t Length () const 
      {
         return demi_only::CurveIntegral (*this, 0, 1);
      }

      vector_t& operator() (scalar_t t, vector_t &v) const
      {
         Get (t, v);
         return v;
      }

   protected:
      vector_t cp[4];
   };

   
   //****
   //@ BezierSpline3
   //****
   //TODO:


   //****
   //@ HermiteSpline
   //****
   //Расположение контрольных точек в сплайне Эрмита:
   //0 - Нач. точка
   //1 - Кон. точка
   //2 - Касат. в нач. точке
   //3 - Касат. в кон. точке
   template <class T>
   //T содержит определения типов:
   //  -vector_t
   //  -scalar_t
   class HermiteSplineSegment: public Spline3Segment <T>
   {
   public:
      typedef Spline3Segment<T> TSpline;
      using typename TSpline::vector_t;
      using typename TSpline::scalar_t;

   //Инлайнинга не происходит из-за виртуальности
   public:
      void Evaluate (scalar_t t, scalar_t h[4]) const
      {
         //Blending Functions:
         //H0(t) =  2t^3 - 3t^2 + 1
         //H1(t) = -2t^3 + 3t^2
         //H2(t) =   t^3 - 2t^2 + t
         //H3(t) =   t^3 -  t^2
         scalar_t u = t * t;
         scalar_t v = u * t;
         h[0] =  2*v -3*u + 1;
         h[1] = -2*v +3*u;
         h[2] =    v -2*u + t;
         h[3] =    v -  u;
      }

      void Derivate (scalar_t t, scalar_t h[4]) const
      {
         scalar_t v = t * t;
         h[0] =  6*v -6*t;
         h[1] = -6*v +6*t;
         h[2] =  3*v -4*t +1;
         h[3] =  3*v -2*t;
      }

      void Continue (Spline3Segment <traits> const &Spline, vector_t const &v, vector_t const &t)
      {
         cp[0] = Spline.End1();
         cp[2] = Spline.Tan1();
         cp[1] = v;
         cp[3] = t;
      }

   public:
      vector_t End0 () const
      {
         return cp[0];
      }

      vector_t End1 () const
      {
         return cp[1];
      }

      vector_t Tan0 () const
      {
         return cp[2];
      }

      vector_t Tan1 () const
      {
         return cp[3];
      }

   public:
      void SetEnd0 (vector_t const &v)
      {
         cp[0] = v;
      }

      void SetEnd1 (vector_t const &v)
      {
         cp[1] = v;
      }

      void SetTan0 (vector_t const &v)
      {
         cp[2] = v;
      }

      void SetTan1 (vector_t const &v)
      {
         cp[3] = v;
      }

   public:
      HermiteSplineSegment (int zero = 0)
      {
         if (zero)
         {
            cp[0].x = cp[0].y = cp[0].z = 0;
            cp[1].x = cp[1].y = cp[1].z = 0;
            cp[2].x = cp[2].y = cp[2].z = 0;
            cp[3].x = cp[3].y = cp[3].z = 0;
         }
      }
   };


   template <class T, int MAX_NODES = 16>
   //T - кривая, для которой требуется построить
   //натуральную параметризацию.
   //От T требуется:
   //  -тип scalar_t
   //  -тип vector_t
   //  -тип TSpline
   //  -функция Get(scalar_t, vector_t const &);
   //От T::vector_t:
   //  -distance(vector_t&, vector_t&)
   class NaturalCurve: public T
   {
   public:
      typedef typename T::TSpline   TSpline;
      typedef typename T::scalar_t  scalar_t;
      typedef typename T::vector_t  vector_t;

      //Фундаментально: NaturalLength() гарантированно не медленнее Length()
      scalar_t NaturalLength () const
      {
         return tab[COUNTOF(tab)-1];
      }

      scalar_t MakeNatural ()
      {
         scalar_t t = 0;
         scalar_t const dt = (1.0/MAX_NODES);

         scalar_t l = 0;
         vector_t v1;
         Get (t, v1);
         int i = 0;
         while (t < 1)
         {
            t += dt;
            vector_t v2;
            Get (t, v2);
            l += distance (v1, v2);
            tab[i++] = l;
            v1 = v2;
         }
         return tab[COUNTOF(tab)-1];
      }

      scalar_t Natural (scalar_t l) const
      {
         Assert (l>=0);
         int m = 0;
         int n = COUNTOF(tab)-1;
         if (l > tab[n])
            return 1;
         if (l < 0)
            return 0;
         while (m != n)
         {
            int c = (m + n)/2;
            if (tab[c] >= l)
               n = c;
            else
               m = c + 1;
         }
         Assert (tab[m] >= l);
         if (!m)
            return (l / tab[m]) * (1.0/MAX_NODES);
         else
            return ((l - tab[m-1]) / (tab[m] - tab[m-1]) + m) * (1.0/MAX_NODES);
      }

   protected:
      scalar_t tab [MAX_NODES];
   };


#if(0)
   template <class T>
   class SplineStrip
   {
   public:
      typedef T TSpline3;
      typedef TSpline3::vector_t vector_t;
      typedef TSpline3::scalar_t scalar_t;

   public:
      void Push (vector_t const &v, vector_t const &d)
      {
         pos.push_back (v);
         dir.push_back (d);
      }

      int SegmentNum ()
      {
         //(int) для случая cp.size()==0
         Assert (dir.size() == pos.size());
         return (int)dir.size() - 1;
      }

      int GetSegment (TSpline3 OUT &Seg, scalar_t t, scalar_t *tlocal)
      {
         //Реально есть хоть один сегмент
         Assert (cp.size() > 1);

         //Сплайн-кусочная кривая замкнута
         int m = roundd(t + 0.5);
         int n = m % pos.size();
         if (n == 0)
         {
            Seg.SetEnd0(pos[0]);
            Seg.SetTan0(dir[0]);
            Seg.SetEnd1(pos.back());
            Seg.SetTan1(pos.back());
         }
         else
         {
            Seg.SetEnd0 (pos[n-1]);
            Seg.SetEnd1 (pos[n]);
            Seg.SetTan0 (dir[n-1]);
            Seg.SetTan1 (dir[n]);
         }

         if (tlocal)
         {
            *tlocal = t - (m-1);
            double y = cg::mod (t, 1); //FIX: should be eq to *tlocal
            Assert (0<=*tlocal && *tlocal<=1);
         }

         return n;
      }

      void SetSegment (TSpline3 const &Seg, int n)
      {
         if (n)
         {
            pos[n-1] = Seg.End0();
            pos[n] = Seg.End1();
            dir[n-1] = Seg.Tan0();
            dir[n] = Seg.Tan1();
         }
         else
         {
            pos[0] = Seg.End1();
            dir[0] = Seg.Tan1();
            pos.back() = Seg.End0();
            pos.back() = Seg.Tan0();
         }
      }

      void Clear ()
      {
         dir.clear();
         pos.clear();
      }

   protected:
      std::vector<vector_t> pos;
      std::vector<vector_t> dir;
   };
#endif
}
