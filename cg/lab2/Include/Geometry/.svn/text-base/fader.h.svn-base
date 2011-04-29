#pragma once

namespace cg
{

struct alpha_fader
{
   inline alpha_fader( float fade_in_time, float fade_out_time,
      float mode_last_time_stamp = std::numeric_limits<float>::infinity() );

   inline void fade_in ( float time_stamp, bool update_time_stamp = true );
   inline void fade_out( float time_stamp, bool update_time_stamp = true );

   inline float alpha() const;
   inline float update( float time_stamp );

private:
   enum fade_mode
   {
      FM_in,
      FM_out
   };

private:
   fade_mode fade_mode_;

   float fade_in_time_;
   float fade_out_time_;

   float mode_last_time_stamp_;

   float alpha_;
};

inline alpha_fader::alpha_fader( float fade_in_time, float fade_out_time, float mode_last_time_stamp )
   : fade_mode_           (FM_in)
   , fade_in_time_        (fade_in_time)
   , fade_out_time_       (fade_out_time)
   , mode_last_time_stamp_(mode_last_time_stamp)
   , alpha_               (0.0f)
{
   Assert(!cg::eq_zero(fade_in_time_));
   Assert(!cg::eq_zero(fade_out_time_));
}

inline void alpha_fader::fade_in ( float time_stamp, bool update_time_stamp )
{
   if (!update_time_stamp && fade_mode_ == FM_in)
      return;

   mode_last_time_stamp_ = time_stamp;
   fade_mode_ = FM_in;
}

inline void alpha_fader::fade_out( float time_stamp, bool update_time_stamp )
{
   if (!update_time_stamp && fade_mode_ == FM_out)
      return;

   mode_last_time_stamp_ = time_stamp;
   fade_mode_ = FM_out;
}

inline float alpha_fader::alpha() const
{
   return alpha_;
}

inline float alpha_fader::update( float time_stamp )
{
   if (time_stamp < mode_last_time_stamp_)
      return alpha_;

   if (fade_mode_ == FM_in)
      alpha_ += (time_stamp - mode_last_time_stamp_) / fade_in_time_;
   else
      alpha_ -= (time_stamp - mode_last_time_stamp_) / fade_out_time_;

   mode_last_time_stamp_ = time_stamp;
   alpha_ = cg::clamp01(alpha_);

   return alpha_;
}

}
