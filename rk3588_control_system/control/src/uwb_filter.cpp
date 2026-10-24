#include "manta/uwb_filter.hpp"
#include <stdexcept>

namespace manta {
void AngularFilter::reset() {
    angle=at=stable_since=missing; points.clear(); mode="acquiring";rate=0;
}
double AngularFilter::observe(double raw,double posterior,double now,double noise) {
    if(!std::isfinite(at)||now<=at||now-at>250) {reset();angle=posterior;at=now;}
    const double previous=at;
    double unwrapped=points.empty()?raw:points.back().z.x+wrap(raw-points.back().z.x);
    trim(points,now,500,31);points.push_back({now,{unwrapped,0}});
    bool moving=false;
    if(points.size()>=4&&now-points.front().at>=100) {
        const auto model=fit(points,now).value();rate=model.velocity.x;
        double residual=0;
        for(const auto& p:points) residual+=squared(p.z.x-model.position.x-rate*(p.at-now)/1000);
        residual=std::sqrt(residual/std::max(1.,double(points.size())-2));
        const double error=std::max(noise*.5,residual)/std::sqrt(model.denominator);
        moving=std::abs(rate)>std::max(4.,3.5*error);
        bool positive=true,negative=true,departure=true;
        for(size_t i=points.size()-3;i<points.size();++i) {
            const double d=wrap(points[i].z.x-angle);
            positive&=d>0;negative&=d<0;departure&=std::abs(d)>std::max(3.,2.5*noise);
        }
        if(now-points[points.size()-3].at>=30&&departure&&(positive||negative))moving=true;
    }
    if(moving){mode="moving";stable_since=missing;}
    else {if(!std::isfinite(stable_since))stable_since=now;if(now-stable_since>=600)mode="stationary";}
    const double tau=mode=="stationary"?stationary_tau:moving_tau;
    const double delta=wrap(posterior-angle),dt=std::max(0.,(now-previous)/1000);
    if(mode!="stationary"||std::abs(delta)>.5*noise)angle=wrap(angle+(1-std::exp(-dt/tau))*delta);
    at=now;return angle;
}
OcclusionGuard::OcclusionGuard(double speed,double dn,double an,double recovery):
    speed_limit(speed),step_margin(std::max(.24,2*dn)),angle_margin(std::max(12.,3*an)),recovery_ms(recovery){}
void OcclusionGuard::reset(){anchor.reset();history.clear();velocity={};reference={};bad_count=0;interrupt_confirmation();diagnostics={{"state","clear"},{"reason",nullptr}};}
void OcclusionGuard::interrupt_confirmation(){return_at=return_last_at=missing;return_hits=0;}
void OcclusionGuard::radio_loss(){if(anchor){++bad_count;interrupt_confirmation();diagnostics={{"state",bad_count>=2?"quarantined":"suspect"},{"reason","radio_loss"},{"bad_count",bad_count}};}}
bool OcclusionGuard::check(Vec position,double now) {
    if(!anchor)return true;
    const Vec prior=anchor->z;
    const double dt=(now-anchor->at)/1000,step=(position-prior).norm();
    const double angle=std::abs(wrap(position.angle()-prior.angle()));
    const double budget=speed_limit*std::max(0.,dt);
    const bool jump=dt>0&&dt<=.3&&(step>step_margin+budget||angle>angle_margin+budget/std::max(.15,std::min(position.norm(),prior.norm()))/radians);
    if(!bad_count&&!jump){diagnostics={{"state","clear"},{"reason",nullptr}};return true;}
    const Vec expected=reference+velocity*clamp(dt,0,.25);
    const double error=(position-expected).norm(),angle_error=std::abs(wrap(position.angle()-expected.angle()));
    if(!(error<=std::max(.30,step_margin)&&angle_error<=angle_margin&&!jump)) {
        ++bad_count;interrupt_confirmation();
        diagnostics={{"state",bad_count>=2?"quarantined":"suspect"},{"reason","raw_position_jump"},{"step_m",step},{"return_error_m",error},{"return_angle_error_deg",angle_error},{"bad_count",bad_count}};
        return false;
    }
    if(bad_count<2){diagnostics={{"state","clear"},{"reason","isolated_spike_returned"}};return true;}
    if(!std::isfinite(return_last_at)||now-return_last_at>150){return_at=now;return_hits=0;}
    return_last_at=now;++return_hits;
    diagnostics={{"state","recovering"},{"reason","confirming_original_track"},{"return_error_m",error},{"return_angle_error_deg",angle_error},{"hits",return_hits},{"confirmed_ms",now-return_at},{"required_ms",recovery_ms}};
    return now-return_at>=recovery_ms&&return_hits>=3;
}
void OcclusionGuard::accept(Vec position,double now) {
    const bool recovering=bad_count>=2;
    if(recovering)history.clear();
    trim(history,now,250,31);history.push_back({now,position});velocity={};reference=position;
    if(history.size()>=3&&now-history.front().at>=100) {
        const auto model=fit(history,now).value();reference=model.position;
        const double speed=model.velocity.norm();
        if(speed>2.5*(step_margin/2)/std::sqrt(model.denominator))velocity=model.velocity*std::min(1.,speed_limit/std::max(speed,1e-9));
    }
    anchor=Point{now,position};bad_count=0;interrupt_confirmation();
    diagnostics={{"state","clear"},{"reason",recovering?Json("original_track_confirmed"):Json(nullptr)}};
}
Axis Axis::predicted(double dt,double q) const {
    Axis a=*this;const double density=q*.1;
    a.position=position+velocity*dt;
    a.p00=p00+2*dt*p01+dt*dt*p11+density*dt*dt*dt/3;
    a.p01=p01+dt*p11+density*dt*dt/2;a.p11=p11+density*dt;return a;
}
void Axis::correct(double measurement,double variance) {
    const double a=p00,b=p01,c=p11,s=a+variance,k0=a/s,k1=b/s,residual=measurement-position;
    position+=k0*residual;velocity+=k1*residual;
    p00=std::max(1e-9,squared(1-k0)*a+k0*k0*variance);
    p01=(1-k0)*(b-k1*a)+k0*k1*variance;
    p11=std::max(1e-9,c-2*k1*b+k1*k1*(a+variance));
    const double limit=std::sqrt(p00*p11);p01=clamp(p01,-limit,limit);
}
static Json defaults(const Json& options) {
    Json c={{"enabled",true},{"distance_std_m",.12},{"angle_std_deg",2.},{"acceleration_std_mps2",2.},{"initial_velocity_std_mps",2.},{"nis_threshold",9.21},{"absolute_gate_m",.6},{"max_relative_speed_mps",6.},{"rejection_covariance_scale",1.25},{"adaptive_enabled",true},{"stationary_time_constant_s",.4},{"moving_time_constant_s",.01},{"occlusion_enabled",true}};
    c.update(options.value("config",Json::object()));
    for(const auto& [key,value]:c.items()) {
        if(key=="enabled"||key=="adaptive_enabled"||key=="occlusion_enabled") {
            if(!value.is_boolean())throw std::invalid_argument("invalid boolean: "+key);
        } else if(!value.is_number()||!std::isfinite(value.get<double>())||value.get<double>()<=0)throw std::invalid_argument("invalid filter parameter: "+key);
    }
    if(number(c,"moving_time_constant_s")<.01||number(c,"moving_time_constant_s")>number(c,"stationary_time_constant_s")||number(c,"stationary_time_constant_s")>2||number(c,"rejection_covariance_scale")<1)throw std::invalid_argument("invalid filter bounds");
    return c;
}
UwbFilter::UwbFilter(const Json& o):config(defaults(o)),minimum(number(o,"minimum_distance_m",.15)),maximum(number(o,"maximum_distance_m",20)),sign(number(o,"bearing_sign",1)),scale(number(o,"bearing_scale",1)),offset(number(o,"bearing_offset_deg",0)),mount(number(o,"mount_yaw_deg",0)),angular(number(config,"stationary_time_constant_s"),number(config,"moving_time_constant_s")),occlusion(number(config,"max_relative_speed_mps"),number(config,"distance_std_m"),number(config,"angle_std_deg")*scale) {
    if(scale<=0||(sign!=1&&sign!=-1))throw std::invalid_argument("invalid bearing calibration");
}
void UwbFilter::predict(double now){if(!axes)return;const double dt=clamp((now-at)/1000,0,10);for(auto& a:*axes)a=a.predicted(dt,squared(number(config,"acceleration_std_mps2")));at=now;}
Json UwbFilter::reject(double now,const std::string& why,double heading) {
    if(config["occlusion_enabled"]==true&&(why=="radio_invalid"||why=="distance_out_of_range"))occlusion.radio_loss();
    if(why!="innovation_outlier"&&why!="occlusion_suspected")occlusion.interrupt_confirmation();
    predict(now);++rejected_count;status="suspect";reason=why;return result(now,heading);
}
std::optional<UwbFilter::Candidate> UwbFilter::raw_candidate(Vec z,double variance,double now) {
    trim(raw_window,now,500,5);Point point{now,z,variance};
    if(!raw_window.empty()&&std::floor(raw_window.back().at/75)==std::floor(now/75))raw_window.back()=point;
    else raw_window.push_back(point);
    trim(raw_window,now,500,5);
    recovery_diagnostics={{"reason","insufficient_hits"},{"hits",raw_window.size()}};
    if(raw_window.size()<3)return {};
    std::optional<Candidate> best;
    for(size_t i=0;i+2<raw_window.size();++i)for(size_t j=i+1;j+1<raw_window.size();++j)for(size_t k=j+1;k<raw_window.size();++k){
        auto model=fit({raw_window[i],raw_window[j],raw_window[k]},now);
        if(!model||model->velocity.norm()>number(config,"max_relative_speed_mps"))continue;
        std::vector<Point> inliers;
        for(const auto& p:raw_window)if((p.z-model->position-model->velocity*((p.at-now)/1000)).norm()<=.24)inliers.push_back(p);
        if(inliers.size()<3||inliers.back().at!=now||now-inliers.front().at<180)continue;
        bool gap=false;for(size_t n=1;n<inliers.size();++n)gap|=inliers[n].at-inliers[n-1].at>450;
        if(gap)continue;
        model=fit(inliers,now);if(!model)continue;
        double residual=0,mean_variance=0;
        for(const auto& p:inliers){residual=std::max(residual,(p.z-model->position-model->velocity*((p.at-now)/1000)).norm());mean_variance+=p.variance;}
        if(residual>.24||model->velocity.norm()>number(config,"max_relative_speed_mps"))continue;
        mean_variance/=inliers.size();
        const double p00=mean_variance*(1./inliers.size()+squared(model->mean_time)/model->denominator)+residual*residual;
        Candidate candidate{int(inliers.size()),residual,*model,p00,mean_variance/model->denominator,2*std::sqrt(2*p00)};
        if(!best||candidate.hits>best->hits||(candidate.hits==best->hits&&candidate.residual<best->residual))best=candidate;
    }
    if(!best){recovery_diagnostics={{"reason","inconsistent_window"},{"hits",0}};return {};}
    recovery_diagnostics={{"reason",best->uncertainty<=.5?"candidate_ready":"candidate_uncertain"},{"hits",best->hits},{"residual_m",best->residual},{"uncertainty_m",best->uncertainty}};
    return best;
}
void UwbFilter::update_angle(Vec measurement,double now){if(config["adaptive_enabled"]==true)angular.observe(measurement.angle(),Vec{(*axes)[0].position,(*axes)[1].position}.angle(),now,number(config,"angle_std_deg")*scale);}
Json UwbFilter::update(const Json& o) {
    const double now=number(o,"now_ms"),heading=number(o,"heading_deg"),distance=number(o,"distance_m"),azimuth=number(o,"azimuth_deg"),elevation=number(o,"elevation_deg");
    nis=innovation=missing;
    if(!std::isfinite(now)||now<=0)throw std::invalid_argument("measurement time must be positive");
    auto reset_track=[&](){axes.reset();at=last_accepted=missing;raw_window.clear();angular.reset();occlusion.reset();};
    if(std::isfinite(at)&&now<at){reset_track();return reject(now,"measurement_clock_reset",heading);}
    if(!std::isfinite(distance)||distance<minimum||distance>maximum)return reject(now,"distance_out_of_range",heading);
    if(!std::isfinite(azimuth)||std::abs(azimuth)>180)return reject(now,"azimuth_out_of_range",heading);
    if(o.contains("elevation_deg")&&!o["elevation_deg"].is_null()&&(!std::isfinite(elevation)||std::abs(elevation)>90))return reject(now,"elevation_out_of_range",heading);
    if(o.value("valid",true)==false)return reject(now,"radio_invalid",heading);
    if(!std::isfinite(heading))return reject(now,"heading_unavailable",heading);
    const Json sequence=o.value("sequence_id",Json(nullptr)),identity=o.value("tag_id",Json(nullptr));
    if((!sequence.is_null()&&sequence==last_sequence)||now==last_received){auto r=result(now,heading);r["duplicate"]=true;return r;}
    last_received=now;last_sequence=sequence;
    const bool has_tag=!identity.is_null()&&identity!=0;
    if(has_tag&&!tag.is_null()&&tag!=0&&tag!=identity){reset_track();tag=identity;return reject(now,"target_identity_changed",heading);}
    if(has_tag)tag=identity;
    const double a=wrap((azimuth+mount)*sign*scale+offset+heading)*radians;
    const Vec z{distance*std::cos(a),distance*std::sin(a)};
    const double variance=squared(number(config,"distance_std_m"))+squared(distance*number(config,"angle_std_deg")*radians);
    bool outlier=false;
    if(axes){
        predict(now);const Vec residual=z-Vec{(*axes)[0].position,(*axes)[1].position};
        nis=squared(residual.x)/((*axes)[0].p00+variance)+squared(residual.y)/((*axes)[1].p00+variance);innovation=residual.norm();
        outlier=nis>number(config,"nis_threshold")||innovation>number(config,"absolute_gate_m")+number(config,"max_relative_speed_mps")*std::max(0.,(now-last_accepted)/1000);
    }
    if(config["occlusion_enabled"]==true&&!occlusion.check(z,now)){
        raw_window.clear();recovery_diagnostics={{"reason","occlusion_quarantine"},{"hits",0}};
        return reject(now,outlier?"innovation_outlier":"occlusion_suspected",heading);
    }
    const auto candidate=raw_candidate(z,variance,now);
    if(!axes){axes=std::array<Axis,2>{Axis(z.x,variance,squared(number(config,"initial_velocity_std_mps"))),Axis(z.y,variance,squared(number(config,"initial_velocity_std_mps")))};at=now;}
    else {
        if(outlier){
            if(candidate&&now-last_accepted>=200){
                const auto& c=*candidate;axes=std::array<Axis,2>{Axis(c.model.position.x,c.p00,c.p11),Axis(c.model.position.y,c.p00,c.p11)};
                (*axes)[0].velocity=c.model.velocity.x;(*axes)[1].velocity=c.model.velocity.y;
                at=last_accepted=now;++accepted_count;++recovery_count;status="trusted";reason="raw_window_reacquired";
                occlusion.accept(z,now);angular.reset();update_angle(c.model.position,now);
                auto r=result(now,heading);r["accepted"]=true;return r;
            }
            return reject(now,"innovation_outlier",heading);
        }
        (*axes)[0].correct(z.x,variance);(*axes)[1].correct(z.y,variance);
    }
    const double speed=std::hypot((*axes)[0].velocity,(*axes)[1].velocity),limit=number(config,"max_relative_speed_mps");
    if(speed>limit)for(auto& axis:*axes)axis.velocity*=limit/speed;
    last_accepted=now;++accepted_count;status="trusted";reason="kalman_updated";occlusion.accept(z,now);update_angle(z,now);
    auto r=result(now,heading);r["accepted"]=true;return r;
}
Json UwbFilter::result(double now,double heading) const {
    Json output={{"admission_version",1},{"accepted",false},{"duplicate",false},{"status",status},{"reason",reason},{"accepted_at",nullable(last_accepted)},{"accepted_count",accepted_count},{"rejected_count",rejected_count},{"nis",nullable(nis)},{"innovation_m",nullable(innovation)},{"uncertainty_m",nullptr},{"distance_m",nullptr},{"azimuth_deg",nullptr},{"recovery_count",recovery_count},{"recovery_diagnostics",recovery_diagnostics},{"occlusion_diagnostics",config["occlusion_enabled"]==true?occlusion.diagnostics:Json{{"state","disabled"},{"reason",nullptr}}}};
    if(!axes)return output;
    const double dt=clamp((now-at)/1000,0,10),q=squared(number(config,"acceleration_std_mps2"));
    const auto x=(*axes)[0].predicted(dt,q),y=(*axes)[1].predicted(dt,q);
    const Vec position{x.position,y.position};const double distance=position.norm(),world=position.angle();
    const bool adaptive=config["adaptive_enabled"]==true;
    const double smooth=adaptive&&std::isfinite(angular.angle)?angular.angle:world;
    const double displacement=2*distance*std::abs(std::sin(wrap(smooth-world)*radians/2));
    const double uncertainty=2*std::sqrt(std::max(0.,x.p00+y.p00));
    output.update(Json{{"base_uncertainty_m",uncertainty},{"uncertainty_m",uncertainty+displacement},{"angular_filter_mode",adaptive?angular.mode:"disabled"},{"angular_filter_rate_deg_s",angular.rate/scale},{"smoothing_displacement_m",displacement}});
    if(std::isfinite(heading))output.update(Json{{"distance_m",distance},{"unsmoothed_azimuth_deg",wrap(wrap(world-heading-offset)/(sign*scale)-mount)},{"azimuth_deg",wrap(wrap(smooth-heading-offset)/(sign*scale)-mount)}});
    return output;
}
}
