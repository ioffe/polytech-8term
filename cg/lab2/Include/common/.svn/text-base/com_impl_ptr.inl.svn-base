// to be included in com_impl_ptr.h

namespace com_impl
{
   template <class T>
   struct instancer
   {
      struct protecter
      {
         protecter( T * t ) : t_( t ) { t_->InternalAddRef (); }
        ~protecter()                  { t_->InternalRelease(); }

      private:
         protecter( protecter const& );
         protecter& operator=( protecter const& );

      private:
         T * t_;
      };

      // bool
      template<typename TT>
         static bool init(T * t, bool (TT::*pFn)())
         {
            protecter p ( t );
            return (t->*pFn)() ; 
         }
      template<typename TT, typename V1, 
               typename P1>
         static bool init(T * t, bool (TT::*pFn)(V1), 
                          P1 const& p1)
         {
            protecter p ( t );
            return (t->*pFn)(p1) ; 
         }
      template<typename TT, typename V1, typename V2, 
               typename P1, typename P2>
         static bool init(T * t, bool (TT::*pFn)(V1, V2), 
                          P1 const& p1, P2 const& p2)
         {
            protecter p ( t );
            return (t->*pFn)(p1, p2) ; 
         }
      template<typename TT, typename V1, typename V2, typename V3, 
               typename P1, typename P2, typename P3>
         static bool init(T * t, bool (TT::*pFn)(V1, V2, V3), 
                          P1 const& p1, P2 const& p2, P3 const& p3)
         {
            protecter p ( t );
            return (t->*pFn)(p1, p2, p3) ; 
         }
      template<typename TT, typename V1, typename V2, typename V3, typename V4, 
               typename P1, typename P2, typename P3, typename P4>
         static bool init(T * t, bool (TT::*pFn)(V1, V2, V3, V4), 
                          P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4)
         {
            protecter p ( t );
            return (t->*pFn)(p1, p2, p3, p4) ; 
         }
      template<typename TT, typename V1, typename V2, typename V3, typename V4, typename V5, 
               typename P1, typename P2, typename P3, typename P4, typename P5>
         static bool init(T * t, bool (TT::*pFn)(V1, V2, V3, V4, V5), 
                          P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5)
         {
            protecter p ( t );
            return (t->*pFn)(p1, p2, p3, p4, p5) ; 
         }
      template<typename TT, typename V1, typename V2, typename V3, typename V4, typename V5, typename V6, 
               typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
         static bool init(T * t, bool (TT::*pFn)(V1, V2, V3, V4, V5, V6), 
                          P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6)
         {
            protecter p ( t );
            return (t->*pFn)(p1, p2, p3, p4, p5, p6) ; 
         }
      template<typename TT, typename V1, typename V2, typename V3, typename V4, typename V5, typename V6, typename V7, 
               typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
         static bool init(T * t, bool (TT::*pFn)(V1, V2, V3, V4, V5, V6, V7), 
                          P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6, P7 const& p7)
         {
            protecter p ( t );
            return (t->*pFn)(p1, p2, p3, p4, p5, p6, p7) ; 
         }

      // void
      template<typename TT>
         static bool init(T * t, void (TT::*pFn)())
         {
            protecter p ( t );
            (t->*pFn)() ; 
            return true ; 
         }
      template<typename TT, typename V1, 
               typename P1>
         static bool init(T * t, void (TT::*pFn)(V1), 
                          P1 const& p1)
         {
            protecter p ( t );
            (t->*pFn)(p1) ; 
            return true ; 
         }
      template<typename TT, typename V1, typename V2, 
               typename P1, typename P2>
         static bool init(T * t, void (TT::*pFn)(V1, V2), 
                          P1 const& p1, P2 const& p2)
         {
            protecter p ( t );
            (t->*pFn)(p1, p2) ; 
            return true ; 
         }
      template<typename TT, typename V1, typename V2, typename V3, 
               typename P1, typename P2, typename P3>
         static bool init(T * t, void (TT::*pFn)(V1, V2, V3), 
                          P1 const& p1, P2 const& p2, P3 const& p3)
         {
            protecter p ( t );
            (t->*pFn)(p1, p2, p3) ; 
            return true ; 
         }
      template<typename TT, typename V1, typename V2, typename V3, typename V4, 
               typename P1, typename P2, typename P3, typename P4>
         static bool init(T * t, void (TT::*pFn)(V1, V2, V3, V4), 
                          P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4)
         {
            protecter p ( t );
            (t->*pFn)(p1, p2, p3, p4) ; 
            return true ; 
         }
      template<typename TT, typename V1, typename V2, typename V3, typename V4, typename V5, 
               typename P1, typename P2, typename P3, typename P4, typename P5>
         static bool init(T * t, void (TT::*pFn)(V1, V2, V3, V4, V5), 
                          P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5)
         {
            protecter p ( t );
            (t->*pFn)(p1, p2, p3, p4, p5) ; 
            return true ; 
         }
      template<typename TT, typename V1, typename V2, typename V3, typename V4, typename V5, typename V6, 
               typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
         static bool init(T * t, void (TT::*pFn)(V1, V2, V3, V4, V5, V6), 
                          P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6)
         {
            protecter p ( t );
            (t->*pFn)(p1, p2, p3, p4, p5, p6) ; 
            return true ; 
         }
      template<typename TT, typename V1, typename V2, typename V3, typename V4, typename V5, typename V6, typename V7, 
               typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
         static bool init(T * t, void (TT::*pFn)(V1, V2, V3, V4, V5, V6, V7), 
                          P1 const& p1, P2 const& p2, P3 const& p3, P4 const& p4, P5 const& p5, P6 const& p6, P7 const& p7)
         {
            protecter p ( t );
            (t->*pFn)(p1, p2, p3, p4, p5, p6, p7) ; 
            return true ; 
         }
   };
}




