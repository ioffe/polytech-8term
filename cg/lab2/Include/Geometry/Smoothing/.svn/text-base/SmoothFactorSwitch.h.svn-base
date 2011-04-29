#pragma once 

struct SmoothFactorSwitch
{
   SmoothFactorSwitch( double startFactor = 0., double startTime = 0. )
      : startFactor_( startFactor )
      , curFactor_  ( startFactor )
      , desFactor_  ( startFactor )
      , startTime_  ( startTime   )
      , duration_   ( 1.          )
   {
   }

   void SetDesiredFactor( double time, double desFactor, double duration ) // zero duration for immediately changing 
   {
      Update( time ) ;

      if( cg::le( duration, 0. ))
      {
         duration_  = 1. ;
         curFactor_ = desFactor ;
      }
      else 
         duration_ = duration ;

      startTime_  = time      ;
      startFactor_= curFactor_;
      desFactor_  = desFactor ;
   }       

   void SetDesiredFactorEx( double time, double desFactor, double speed ) 
   {
      Assert( !cg::le( speed, 0 )) ; 
      SetDesiredFactor( time, desFactor, cg::abs( desFactor - curFactor_ ) / speed ) ;
   }

   void Init( double startFactor = 0., double startTime = 0. )
   {
      startFactor_ = startFactor ;   
      curFactor_   = startFactor ;
      desFactor_   = startFactor ;
      startTime_   = startTime   ; 
      duration_    = 1. ; 
   }

   void Update ( double time ) 
   {
      curFactor_ = cg::clamp_d( startTime_, startTime_ + duration_, startFactor_, desFactor_ )( time ) ;
   }

   double GetFactor() const
   {
      return curFactor_ ;
   }

   double GetDesFactor() const
   {
      return desFactor_;
   }

   operator double() const 
   {
      return GetFactor() ;
   }

private:
   double         startTime_  ;
   double         startFactor_;
   double         desFactor_  ;  
   double         duration_   ;
   double         curFactor_  ;
};