#include "soarm_localization/fixed_map_registration.hpp"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
using namespace soarm_localization;

static std::vector<Eigen::Vector3d> line(double x0, double y0, double x1, double y1) { std::vector<Eigen::Vector3d> out; int n=static_cast<int>(std::round(std::hypot(x1-x0,y1-y0)/.02)); for(int i=0;i<=n;++i) out.emplace_back(x0+(x1-x0)*i/n,y0+(y1-y0)*i/n,0); return out; }
static void add(std::vector<Eigen::Vector3d>& a,const std::vector<Eigen::Vector3d>& b){a.insert(a.end(),b.begin(),b.end());}
static std::vector<Eigen::Vector3d> inverse(const std::vector<Eigen::Vector3d>& points, Se2 pose) { std::vector<Eigen::Vector3d> out; double c=std::cos(pose.yaw),s=std::sin(pose.yaw); for(auto q:points){double x=q.x()-pose.x,y=q.y()-pose.y;out.emplace_back(c*x+s*y,-s*x+c*y,0);}return out; }
static bool same(const PreRefinementResult& a,const PreRefinementResult& b){if(a.peaks.size()!=b.peaks.size())return false;for(size_t i=0;i<a.peaks.size();++i)if(a.peaks[i].score!=b.peaks[i].score||a.peaks[i].pose.x!=b.peaks[i].pose.x||a.peaks[i].pose.y!=b.peaks[i].pose.y||a.peaks[i].pose.yaw!=b.peaks[i].pose.yaw)return false;return true;}
int main() {
  Options options; options.voxel_resolution=.01; options.min_cells=20; options.max_cells=1200; options.threads=1;
  SearchProfile tracking;
  std::vector<Eigen::Vector3d> corridor; add(corridor,line(-2,-1,2,-1));add(corridor,line(-2,1,2,1));std::vector<Eigen::Vector3d> corridor_source;add(corridor_source,line(-1,-1,1,-1));add(corridor_source,line(-1,1,1,1));
  PreRefinementResult corridor_reference;
  for(int threads: {1,2}) { options.threads=threads; FixedMapRegistration reg(options);assert(reg.set_map(corridor));for(int repeat=0;repeat<20;++repeat){auto value=reg.search(corridor_source,tracking);if(repeat==0&&threads==1)corridor_reference=value;assert(same(value,corridor_reference));if (threads == 1 && repeat == 0) { std::cerr<<"corridor diagnostic threads="<<threads<<" repeat="<<repeat<<" peaks="<<value.peaks.size()<<" gap="<<value.score_gap;for(const auto& peak:value.peaks)std::cerr<<" ["<<peak.pose.x<<","<<peak.pose.y<<","<<peak.pose.yaw<<":"<<peak.score<<"]";std::cerr<<"\n"; } assert(value.score_gap<=1e-12);}}
  std::cout<<"corridor peaks="<<corridor_reference.peaks.size()<<" gap="<<corridor_reference.score_gap<<" ambiguous=1\n";
  std::vector<Eigen::Vector3d> corner;add(corner,line(0,0,1.8,0));add(corner,line(0,0,0,1.6));add(corner,line(.65,0,.65,.55)); Se2 truth{.12,-.08,.06};auto corner_source=inverse(corner,truth);
  options.threads=1;FixedMapRegistration corner_reg(options);assert(corner_reg.set_map(corner));PreRefinementResult corner_reference;
  for(int threads: {1,2}) { options.threads=threads; FixedMapRegistration reg(options);assert(reg.set_map(corner));for(int repeat=0;repeat<20;++repeat){auto value=reg.search(corner_source,tracking);if(repeat==0&&threads==1)corner_reference=value;assert(same(value,corner_reference));}}
  assert(corner_reference.peaks.size()>1&&corner_reference.score_gap>0.0);auto best=corner_reference.peaks.front();assert(std::hypot(best.pose.x-truth.x,best.pose.y-truth.y)<=.05&&std::abs(best.pose.yaw-truth.yaw)<=.03);auto refined=corner_reg.refine_selected(corner_source,{best});assert(refined.size()==1&&refined[0].accepted_input);std::cout<<"corner gap="<<corner_reference.score_gap<<" best="<<best.pose.x<<","<<best.pose.y<<","<<best.pose.yaw<<" refine="<<refined[0].transform.x<<","<<refined[0].transform.y<<","<<refined[0].transform.yaw<<"\n";
  // Explicit fixture-only policy applied by the caller, never by the core.
  constexpr double fixture_rank_tolerance=1e-5;std::cerr<<"corner diagnostic pose="<<refined[0].transform.x<<","<<refined[0].transform.y<<","<<refined[0].transform.yaw<<" support="<<refined[0].observability.valid_support<<" radius="<<std::sqrt(std::accumulate(corner_source.begin(),corner_source.end(),0.0,[](double sum,const auto& point){return sum+point.template head<2>().squaredNorm();})/corner_source.size())<<" info="<<refined[0].observability.information<<" eigen="<<refined[0].observability.eigenvalues.transpose()<<" ratio="<<refined[0].observability.min_trace_ratio<<" rank="<<refined[0].observability.rank_for_relative_tolerance(fixture_rank_tolerance)<<"\n";assert(refined[0].observability.rank_for_relative_tolerance(fixture_rank_tolerance)>=1);
  std::vector<Eigen::Vector3d> wall=line(0,0,3,0);FixedMapRegistration wall_reg(options);assert(wall_reg.set_map(wall));auto wall_result=wall_reg.refine_selected(wall,{{{0,0,0},0}}).front();assert(wall_result.accepted_input);std::cout<<"observability corner="<<refined[0].observability.eigenvalues.transpose()<<" wall="<<wall_result.observability.eigenvalues.transpose()<<"\n";
  assert(!corner_reg.search({Eigen::Vector3d(NAN,0,0)},tracking).accepted_input);assert(!corner_reg.search({},tracking).accepted_input);SearchProfile invalid=tracking;invalid.xy_step=0;assert(!corner_reg.search(corner_source,invalid).accepted_input);
  std::vector<double> samples;for(int i=0;i<40;++i){auto search=corner_reg.search(corner_source,tracking);samples.push_back(search.metrics.wall_seconds);}std::sort(samples.begin(),samples.end());std::cout<<"tracking_ms p50="<<samples[20]*1e3<<" p95="<<samples[38]*1e3<<" max="<<samples.back()*1e3<<" cells="<<corner_reg.cell_count()<<"\n";assert(samples[38]<.020);SearchProfile recovery=tracking;recovery.xy_extent=.5;recovery.yaw_extent=.3;auto recovery_result=corner_reg.search(corner_source,recovery);std::cout<<"recovery_ms="<<recovery_result.metrics.wall_seconds*1e3<<" refinement_ms="<<refined[0].wall_seconds*1e3<<"\n";
}
