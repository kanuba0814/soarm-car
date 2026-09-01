#include "soarm_localization/fixed_map_registration.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <numeric>
#include <new>
#include <stdexcept>
#include <tuple>

namespace soarm_localization {
namespace {
constexpr std::size_t kMaxSourceCells = 1200;
constexpr std::size_t kMaxFieldPayloadBytes = 64U * 1024U * 1024U;
constexpr std::size_t kMaxModelBytes = 128U * 1024U * 1024U;
constexpr std::size_t kMaxLatticeEvaluations = 1000000U;
constexpr int kMaxIterations = 10000;
constexpr int kTileWidth = 32;
constexpr std::size_t kMaxModelCells = kMaxModelBytes / 256U;

bool checked_int(double value, double (*rounding)(double), int* out) {
  if (!std::isfinite(value)) return false;
  const double rounded = rounding(value);
  if (rounded < static_cast<double>(std::numeric_limits<int>::min()) || rounded > static_cast<double>(std::numeric_limits<int>::max())) return false;
  *out = static_cast<int>(rounded); return true;
}
bool checked_floor(double v, int* o) { return checked_int(v, std::floor, o); }
bool checked_ceil(double v, int* o) { return checked_int(v, std::ceil, o); }
bool checked_round(double v, int* o) { return checked_int(v, std::round, o); }
std::uint64_t tile_key(int x, int y) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
         static_cast<std::uint32_t>(y);
}
std::uint64_t mix_key(std::uint64_t value) {
  value ^= value >> 33U;
  value *= 0xff51afd7ed558ccdULL;
  value ^= value >> 33U;
  value *= 0xc4ceb9fe1a85ec53ULL;
  return value ^ (value >> 33U);
}
bool checked_multiply(std::size_t a, std::size_t b, std::size_t* out) { if (a && b > std::numeric_limits<std::size_t>::max() / a) return false; *out=a*b; return true; }
bool finite_points(const std::vector<Eigen::Vector3d>& points) { return !points.empty() && std::all_of(points.begin(), points.end(), [](const auto& p) { return p.allFinite(); }); }
bool finite_pose(const Se2& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw); }
bool valid_options(const Options& o) { return std::isfinite(o.voxel_resolution) && o.voxel_resolution > 0 && std::isfinite(o.distance_field_resolution) && o.distance_field_resolution > 0 && std::isfinite(o.capped_squared_loss) && o.capped_squared_loss > 0 && std::isfinite(o.max_correspondence_distance) && o.max_correspondence_distance > 0 && o.min_cells > 0 && o.max_cells >= o.min_cells && static_cast<std::size_t>(o.max_cells) <= kMaxSourceCells && o.max_iterations > 0 && o.max_iterations <= kMaxIterations && o.threads > 0 && o.max_model_cells > 0 && o.max_model_cells <= kMaxModelCells && o.max_model_bytes > 0 && o.max_model_bytes <= kMaxModelBytes && o.max_field_payload_bytes > 0 && o.max_field_payload_bytes <= kMaxFieldPayloadBytes; }
bool valid_profile(const SearchProfile& p) { return std::isfinite(p.xy_extent) && std::isfinite(p.yaw_extent) && std::isfinite(p.xy_step) && std::isfinite(p.yaw_step) && std::isfinite(p.nms_radius) && p.xy_extent >= 0 && p.yaw_extent >= 0 && p.xy_step > 0 && p.yaw_step > 0 && p.nms_radius >= 0 && p.peak_count > 0; }
bool valid_refinement_profile(const RefinementProfile& p) { return std::isfinite(p.max_relative_x) && std::isfinite(p.max_relative_y) && std::isfinite(p.max_relative_yaw) && p.max_relative_x >= 0 && p.max_relative_y >= 0 && p.max_relative_yaw >= 0; }
bool within(const Se2& p, const RefinementProfile& b) { return finite_pose(p) && std::abs(p.x) <= b.max_relative_x && std::abs(p.y) <= b.max_relative_y && std::abs(p.yaw) <= b.max_relative_yaw; }
bool checked_lattice(const SearchProfile& p, int* xy, int* yaw, std::size_t* evaluations) { const double xq=p.xy_extent/p.xy_step, yq=p.yaw_extent/p.yaw_step; if (!std::isfinite(xq)||!std::isfinite(yq)||!checked_floor(std::floor(xq+1e-9),xy)||!checked_floor(std::floor(yq+1e-9),yaw)||*xy<0||*yaw<0||*xy>(std::numeric_limits<int>::max()-1)/2||*yaw>(std::numeric_limits<int>::max()-1)/2) return false; std::size_t plane=0; return checked_multiply(2U*static_cast<std::size_t>(*xy)+1U,2U*static_cast<std::size_t>(*xy)+1U,&plane)&&checked_multiply(plane,2U*static_cast<std::size_t>(*yaw)+1U,evaluations)&&*evaluations<=kMaxLatticeEvaluations; }
Se2 compose(const Se2& a, const Se2& b) { const double c=std::cos(a.yaw),s=std::sin(a.yaw); return {a.x+c*b.x-s*b.y,a.y+s*b.x+c*b.y,a.yaw+b.yaw}; }
Eigen::Isometry3d as_isometry(const Se2& p) { Eigen::Isometry3d out=Eigen::Isometry3d::Identity(); out.translation()=Eigen::Vector3d(p.x,p.y,0); out.linear()=Eigen::AngleAxisd(p.yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix(); return out; }
Se2 as_se2(const Eigen::Isometry3d& p) { return {p.translation().x(),p.translation().y(),std::atan2(p.rotation()(1,0),p.rotation()(0,0))}; }
}  // namespace

struct FixedMapRegistration::Model {
  struct Tile { std::vector<double> values; };
  struct TileSlot { std::uint64_t key=0; bool occupied=false; Tile tile; };
  struct Node { std::size_t cell; int axis; int left=-1, right=-1; };
  std::vector<Eigen::Vector3d> cells;
  std::vector<Eigen::Vector2d> normals;
  std::vector<bool> normal_valid;
  // Fixed open-address table: lookup never iterates and cannot affect score ordering.
  std::vector<TileSlot> tile_table;
  std::size_t tile_count=0;
  std::vector<Node> index;
  int root=-1;
  std::shared_ptr<small_gicp::PointCloud> cloud;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> tree;
};

int Observability::rank_for_relative_tolerance(double tolerance) const { if (!std::isfinite(tolerance)||tolerance<0||eigenvalues.maxCoeff()<=std::numeric_limits<double>::epsilon()) return 0; int rank=0; for(int i=0;i<3;++i) if(eigenvalues[i]>eigenvalues.maxCoeff()*tolerance) ++rank; return rank; }
FixedMapRegistration::FixedMapRegistration(Options options) : options_(options) {}

bool FixedMapRegistration::set_map(const std::vector<Eigen::Vector3d>& points) {
  const auto start=std::chrono::steady_clock::now();
  try {
  if (!finite_points(points) || !valid_options(options_)) return false;
  std::map<std::tuple<int,int,int>,std::pair<Eigen::Vector3d,std::size_t>> bins;
  for(const auto& point: points) { int x,y,z; if(!checked_floor(point.x()/options_.voxel_resolution,&x)||!checked_floor(point.y()/options_.voxel_resolution,&y)||!checked_floor(point.z()/options_.voxel_resolution,&z)) return false; auto key=std::make_tuple(x,y,z); auto it=bins.find(key); if(it==bins.end()) { if(bins.size()>=std::min(options_.max_model_cells,kMaxModelCells)) return false; it=bins.emplace(key,std::make_pair(Eigen::Vector3d::Zero(),std::size_t{0})).first; } if(it->second.second==std::numeric_limits<std::size_t>::max()) return false; const auto sum=it->second.first+point; if(!sum.allFinite()) return false; it->second.first=sum; ++it->second.second; }
  if(bins.size()<static_cast<std::size_t>(options_.min_cells) || bins.size()>std::min(options_.max_model_cells,kMaxModelCells)) return false;
  auto model=std::make_shared<Model>(); model->cells.reserve(bins.size()); for(const auto& bin:bins) model->cells.push_back(bin.second.first/bin.second.second);
  // Canonical cells are map-bin order.  The persistent KD index uses that index as every equal-distance tie breaker.
  // Reserve before recursive child assignment so index-node storage never relocates mid-build.
  model->index.reserve(model->cells.size());
  std::vector<std::size_t> order(model->cells.size()); std::iota(order.begin(),order.end(),0);
  auto build_index=[&](auto&& self, std::vector<std::size_t>& ids, int depth)->int { if(ids.empty()) return -1; const int axis=depth%2; std::stable_sort(ids.begin(),ids.end(),[&](std::size_t a,std::size_t b){ const double av=model->cells[a][axis],bv=model->cells[b][axis]; if(av!=bv)return av<bv; const double ao=model->cells[a][1-axis],bo=model->cells[b][1-axis]; if(ao!=bo)return ao<bo; return a<b; }); const std::size_t mid=ids.size()/2, cell=ids[mid]; const int node=static_cast<int>(model->index.size()); model->index.push_back({cell,axis}); std::vector<std::size_t> left(ids.begin(),ids.begin()+mid),right(ids.begin()+mid+1,ids.end()); const int left_node=self(self,left,depth+1); const int right_node=self(self,right,depth+1); model->index[node].left=left_node; model->index[node].right=right_node; return node; }; model->root=build_index(build_index,order,0);
  model->normals.assign(model->cells.size(),Eigen::Vector2d::Zero()); model->normal_valid.assign(model->cells.size(),false);
  for(std::size_t i=0;i<model->cells.size();++i) { std::vector<std::pair<double,std::size_t>> near; for(std::size_t j=0;j<model->cells.size();++j)if(i!=j){const double d=(model->cells[i].head<2>()-model->cells[j].head<2>()).squaredNorm();if(d<=16*options_.voxel_resolution*options_.voxel_resolution)near.emplace_back(d,j);} if(near.size()<4)continue; std::partial_sort(near.begin(),near.begin()+4,near.end()); Eigen::Vector2d mean=Eigen::Vector2d::Zero();for(int k=0;k<4;++k)mean+=model->cells[near[k].second].head<2>();mean/=4;Eigen::Matrix2d cov=Eigen::Matrix2d::Zero();for(int k=0;k<4;++k){auto d=model->cells[near[k].second].head<2>()-mean;cov+=d*d.transpose();}Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);if(solver.eigenvalues()[1]>std::numeric_limits<double>::epsilon()&&solver.eigenvalues()[0]<=.10*solver.eigenvalues()[1]){model->normals[i]=solver.eigenvectors().col(0);model->normal_valid[i]=true;} }
  const double radius=std::sqrt(options_.capped_squared_loss); if(!std::isfinite(radius)) return false;
  std::map<std::uint64_t,Model::Tile> tiles;
  for(const auto& cell:model->cells) { int minx,maxx,miny,maxy; if(!checked_ceil((cell.x()-radius)/options_.distance_field_resolution,&minx)||!checked_floor((cell.x()+radius)/options_.distance_field_resolution,&maxx)||!checked_ceil((cell.y()-radius)/options_.distance_field_resolution,&miny)||!checked_floor((cell.y()+radius)/options_.distance_field_resolution,&maxy)||minx>maxx||miny>maxy||maxx==std::numeric_limits<int>::max()||maxy==std::numeric_limits<int>::max())return false; for(int iy=miny;iy<=maxy;++iy)for(int ix=minx;ix<=maxx;++ix) { const int tx=ix>=0?ix/kTileWidth:(ix-(kTileWidth-1))/kTileWidth, ty=iy>=0?iy/kTileWidth:(iy-(kTileWidth-1))/kTileWidth; const int lx=ix-tx*kTileWidth,ly=iy-ty*kTileWidth; const auto key=tile_key(tx,ty); auto it=tiles.find(key);if(it==tiles.end()){std::size_t bytes=0;std::size_t cell_bytes=0;if(!checked_multiply(model->cells.size(),std::size_t{256},&cell_bytes)||!checked_multiply(tiles.size()+1,static_cast<std::size_t>(kTileWidth*kTileWidth*sizeof(double)),&bytes)||bytes>options_.max_field_payload_bytes||cell_bytes>options_.max_model_bytes||bytes>options_.max_model_bytes-cell_bytes)return false; it=tiles.emplace(key,Model::Tile{std::vector<double>(kTileWidth*kTileWidth,options_.capped_squared_loss)}).first;} const double dx=ix*options_.distance_field_resolution-cell.x(),dy=iy*options_.distance_field_resolution-cell.y();it->second.values[static_cast<std::size_t>(ly*kTileWidth+lx)]=std::min(it->second.values[static_cast<std::size_t>(ly*kTileWidth+lx)],dx*dx+dy*dy); } }
  std::size_t payload=0;if(!checked_multiply(tiles.size(),static_cast<std::size_t>(kTileWidth*kTileWidth*sizeof(double)),&payload))return false; std::size_t estimated_cells=0;if(!checked_multiply(model->cells.size(),std::size_t{256},&estimated_cells)||estimated_cells>options_.max_model_bytes||payload>options_.max_model_bytes-estimated_cells)return false;
  std::size_t required_slots=0;if(!checked_multiply(tiles.size(),std::size_t{2},&required_slots))return false;std::size_t table_capacity=1;while(table_capacity<required_slots){if(table_capacity>std::numeric_limits<std::size_t>::max()/2)return false;table_capacity*=2;}std::size_t table_bytes=0;if(!checked_multiply(table_capacity,sizeof(Model::TileSlot),&table_bytes)||table_bytes>options_.max_model_bytes-estimated_cells-payload)return false;model->tile_table.resize(table_capacity);const std::size_t mask=table_capacity-1;for(auto& entry:tiles){std::size_t slot=static_cast<std::size_t>(mix_key(entry.first))&mask;while(model->tile_table[slot].occupied)slot=(slot+1)&mask;model->tile_table[slot].key=entry.first;model->tile_table[slot].occupied=true;model->tile_table[slot].tile=std::move(entry.second);}model->tile_count=tiles.size();

  auto processed=small_gicp::preprocess_points(model->cells,options_.voxel_resolution,10,options_.threads); model->cloud=processed.first;model->tree=processed.second;if(!model->tree)return false;
  model_=std::move(model); ++build_count_;last_build_seconds_=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();return true;
  } catch (const std::bad_alloc&) { return false; } catch (const std::length_error&) { return false; }
}

double FixedMapRegistration::field_at(double x,double y) const { if(!model_||model_->tile_table.empty()||!std::isfinite(x)||!std::isfinite(y))return options_.capped_squared_loss;int ix,iy;if(!checked_round(x/options_.distance_field_resolution,&ix)||!checked_round(y/options_.distance_field_resolution,&iy))return options_.capped_squared_loss;const int tx=ix>=0?ix/kTileWidth:(ix-(kTileWidth-1))/kTileWidth,ty=iy>=0?iy/kTileWidth:(iy-(kTileWidth-1))/kTileWidth;const auto key=tile_key(tx,ty);const std::size_t mask=model_->tile_table.size()-1;std::size_t slot=static_cast<std::size_t>(mix_key(key))&mask;for(std::size_t probe=0;probe<model_->tile_table.size();++probe){const auto& entry=model_->tile_table[slot];if(!entry.occupied)return options_.capped_squared_loss;if(entry.key==key)return entry.tile.values[static_cast<std::size_t>((iy-ty*kTileWidth)*kTileWidth+(ix-tx*kTileWidth))];slot=(slot+1)&mask;}return options_.capped_squared_loss; }
double FixedMapRegistration::field_squared_distance(double x,double y) const{return field_at(x,y);}
std::size_t FixedMapRegistration::nearest_cell_index(double x,double y) const { if(!model_||!std::isfinite(x)||!std::isfinite(y))return std::numeric_limits<std::size_t>::max();const Eigen::Vector2d query(x,y);std::size_t best=std::numeric_limits<std::size_t>::max();double best2=std::numeric_limits<double>::infinity();auto visit=[&](auto&& self,int node_index)->void{if(node_index<0)return;const auto& node=model_->index[static_cast<std::size_t>(node_index)];const auto point=model_->cells[node.cell].head<2>();const double d2=(point-query).squaredNorm();if(d2<best2||(d2==best2&&node.cell<best)){best2=d2;best=node.cell;}const double difference=query[node.axis]-point[node.axis];const int near=difference<0.0?node.left:node.right;const int far=difference<0.0?node.right:node.left;self(self,near);if(difference*difference<=best2)self(self,far);};visit(visit,model_->root);return best; }
ModelStats FixedMapRegistration::model_stats() const { ModelStats out;out.build_count=build_count_;out.build_seconds=last_build_seconds_;if(model_){out.map_cells=model_->cells.size();out.tiles=model_->tile_count;out.field_payload_bytes=out.tiles*kTileWidth*kTileWidth*sizeof(double);}return out; }
std::size_t FixedMapRegistration::cell_count() const{return model_?model_->cells.size():0;}

PreRefinementResult FixedMapRegistration::search(const std::vector<Eigen::Vector3d>& source,const SearchProfile& profile) const{return search_around(source,{},profile);}
PreRefinementResult FixedMapRegistration::search_around(
    const std::vector<Eigen::Vector3d>& source,
    const Se2& anchor,
    const SearchProfile& profile) const {
    PreRefinementResult out;
    const auto start = std::chrono::steady_clock::now();
    int xy;
    int yaw;
    std::size_t evaluations;
    if (!model_ || !valid_options(options_) || !finite_points(source) ||
        source.size() < static_cast<std::size_t>(options_.min_cells) ||
        source.size() > static_cast<std::size_t>(options_.max_cells) ||
        !finite_pose(anchor) || !valid_profile(profile) ||
        !checked_lattice(profile, &xy, &yaw, &evaluations)) {
        return out;
    }

    out.accepted_input = true;
    out.metrics.source_cells = source.size();
    std::vector<Peak> raw;
    raw.reserve(evaluations);
    for (int ia = -yaw; ia <= yaw; ++ia) {
        for (int iy = -xy; iy <= xy; ++iy) {
            for (int ix = -xy; ix <= xy; ++ix) {
                Peak peak{
                    compose(anchor,
                            {ix * profile.xy_step,
                             iy * profile.xy_step,
                             ia * profile.yaw_step}),
                    0.0};
                const double cosine = std::cos(peak.pose.yaw);
                const double sine = std::sin(peak.pose.yaw);
                for (const auto& point : source) {
                    peak.score += field_at(
                        peak.pose.x + cosine * point.x() - sine * point.y(),
                        peak.pose.y + sine * point.x() + cosine * point.y());
                }
                peak.score /= static_cast<double>(source.size());
                raw.push_back(peak);
            }
        }
    }

    out.metrics.lattice_evaluations = raw.size();
    std::sort(raw.begin(), raw.end(), [](const Peak& a, const Peak& b) {
        if (a.score != b.score) return a.score < b.score;
        if (a.pose.x != b.pose.x) return a.pose.x < b.pose.x;
        if (a.pose.y != b.pose.y) return a.pose.y < b.pose.y;
        return a.pose.yaw < b.pose.yaw;
    });
    const double radius_squared = profile.nms_radius * profile.nms_radius;
    for (const auto& peak : raw) {
        bool distinct = true;
        for (const auto& kept : out.peaks) {
            if (std::pow(peak.pose.x - kept.pose.x, 2) +
                    std::pow(peak.pose.y - kept.pose.y, 2) <
                radius_squared) {
                distinct = false;
                break;
            }
        }
        if (distinct) out.peaks.push_back(peak);
        if (out.peaks.size() == profile.peak_count) break;
    }
    if (out.peaks.size() > 1) {
        out.score_gap = out.peaks[1].score - out.peaks[0].score;
    }
    out.metrics.wall_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return out;
}

std::vector<Result> FixedMapRegistration::refine_selected(const std::vector<Eigen::Vector3d>& source,const std::vector<Peak>& selected) const{std::vector<Result> out;out.reserve(selected.size());for(const auto&p:selected)out.push_back(refine(source,p.pose));return out;}
Result FixedMapRegistration::refine_bounded(const std::vector<Eigen::Vector3d>& scan,const Se2& initial,const RefinementProfile& profile) const{return refine_bounded_around(scan,{},initial,profile);}
Result FixedMapRegistration::refine_bounded_around(const std::vector<Eigen::Vector3d>& scan,const Se2& anchor,const Se2& initial_delta,const RefinementProfile& profile) const { Result result;const auto start=std::chrono::steady_clock::now();if(!model_||!valid_options(options_)||!finite_points(scan)||scan.size()<static_cast<std::size_t>(options_.min_cells)||scan.size()>static_cast<std::size_t>(options_.max_cells)||!finite_pose(anchor)||!finite_pose(initial_delta)||!valid_refinement_profile(profile))return result;result.accepted_input=true;result.bounded_refinement_used=true;struct Eval{double error=0;int inliers=0;Eigen::Matrix3d h=Eigen::Matrix3d::Zero();Eigen::Vector3d b=Eigen::Vector3d::Zero();};const double max2=options_.max_correspondence_distance*options_.max_correspondence_distance;const double ca=std::cos(anchor.yaw),sa=std::sin(anchor.yaw);auto candidate_local=[&](const Se2& relative){return Se2{initial_delta.x+relative.x,initial_delta.y+relative.y,initial_delta.yaw+relative.yaw};};auto pose_for=[&](const Se2& relative){return compose(anchor,candidate_local(relative));};auto evaluate=[&](const Se2& relative,bool linearize,Eval* e){if(!within(relative,profile)){result.trust_region_limited=true;return false;}*e=Eval{};++result.bounded_evaluations;const Se2 local=candidate_local(relative);const double c=std::cos(local.yaw),s=std::sin(local.yaw);for(const auto&p:scan){const Eigen::Vector2d local_q(c*p.x()-s*p.y()+local.x,s*p.x()+c*p.y()+local.y);const Eigen::Vector2d q(ca*local_q.x()-sa*local_q.y()+anchor.x,sa*local_q.x()+ca*local_q.y()+anchor.y);const auto target=nearest_cell_index(q.x(),q.y());if(target==std::numeric_limits<std::size_t>::max())continue;const auto residual=q-model_->cells[target].head<2>();if(residual.squaredNorm()>max2)continue;e->error+=residual.squaredNorm();++e->inliers;if(linearize){Eigen::Matrix<double,2,3> j;const double local_dx=-s*p.x()-c*p.y(),local_dy=c*p.x()-s*p.y();j<<ca,-sa,ca*local_dx-sa*local_dy,sa,ca,sa*local_dx+ca*local_dy;e->h.noalias()+=j.transpose()*j;e->b.noalias()+=j.transpose()*residual;}}e->error=e->inliers?e->error/e->inliers:std::numeric_limits<double>::infinity();return std::isfinite(e->error);};Se2 relative;result.transform=pose_for(relative);Eval current;if(!evaluate(relative,true,&current)){result.all_evaluations_within_bounds=false;return result;}double damping=1e-3;for(int i=0;i<options_.max_iterations;++i){if(current.inliers<3)break;Eigen::Matrix3d system=current.h;for(int axis=0;axis<3;++axis)system(axis,axis)+=damping*std::max(1.0,current.h(axis,axis));const Eigen::Vector3d increment=(-system.ldlt().solve(current.b)).eval();if(!increment.allFinite())break;if(increment.norm()<1e-6&&current.error<1e-5){result.converged=true;break;}Se2 candidate{relative.x+increment.x(),relative.y+increment.y(),relative.yaw+increment.z()};Eval trial;if(!evaluate(candidate,false,&trial)){damping*=10;continue;}if(trial.error<current.error){relative=candidate;if(increment.norm()<1e-6){result.converged=true;current=trial;break;}if(!evaluate(relative,true,&current)){result.all_evaluations_within_bounds=false;break;}damping=std::max(1e-9,damping*.5);}else damping*=10;}result.transform=pose_for(relative);result.mean_squared_error=current.error;result.inlier_fraction=current.inliers/static_cast<double>(scan.size());result.observability.information=current.h;result.observability.valid_support=current.inliers;Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(current.h);result.observability.eigenvalues=solver.eigenvalues();const double trace=result.observability.eigenvalues.sum(),top=result.observability.eigenvalues.maxCoeff();if(top>std::numeric_limits<double>::epsilon()){result.observability.min_trace_ratio=result.observability.eigenvalues.minCoeff()/std::max(trace,std::numeric_limits<double>::epsilon());result.observability.condition_number=top/std::max(result.observability.eigenvalues.minCoeff(),std::numeric_limits<double>::epsilon());}result.wall_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();return result; }

Result FixedMapRegistration::refine(const std::vector<Eigen::Vector3d>& scan,const Se2& initial) const { Result result;const auto start=std::chrono::steady_clock::now();if(!model_||!valid_options(options_)||!finite_points(scan)||scan.size()<static_cast<std::size_t>(options_.min_cells)||scan.size()>static_cast<std::size_t>(options_.max_cells)||!finite_pose(initial))return result;result.accepted_input=true;small_gicp::RegistrationSetting settings;settings.type=small_gicp::RegistrationSetting::GICP;settings.num_threads=options_.threads;settings.max_iterations=options_.max_iterations;settings.max_correspondence_distance=options_.max_correspondence_distance;settings.downsampling_resolution=options_.voxel_resolution;auto source=small_gicp::preprocess_points(scan,options_.voxel_resolution,10,options_.threads);auto alignment=small_gicp::align(*model_->cloud,*source.first,*model_->tree,as_isometry(initial),settings);result.converged=alignment.converged;result.transform=as_se2(alignment.T_target_source);result.mean_squared_error=alignment.error;result.inlier_fraction=alignment.num_inliers/static_cast<double>(std::max<std::size_t>(1,source.first->size()));double radius2=0;for(const auto& point:scan)radius2+=point.head<2>().squaredNorm();const double radius=std::max(options_.voxel_resolution,std::sqrt(radius2/scan.size()));const double c=std::cos(result.transform.yaw),s=std::sin(result.transform.yaw),max2=options_.max_correspondence_distance*options_.max_correspondence_distance;for(const auto& point:scan){Eigen::Vector2d q(c*point.x()-s*point.y()+result.transform.x,s*point.x()+c*point.y()+result.transform.y);const auto target=nearest_cell_index(q.x(),q.y());if(target==std::numeric_limits<std::size_t>::max()||(q-model_->cells[target].head<2>()).squaredNorm()>max2||!model_->normal_valid[target])continue;const auto& normal=model_->normals[target];Eigen::Vector2d dtheta(-s*point.x()-c*point.y(),c*point.x()-s*point.y());Eigen::Vector3d jacobian(normal.x(),normal.y(),normal.dot(dtheta)/radius);result.observability.information+=jacobian*jacobian.transpose();++result.observability.valid_support;}Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(result.observability.information);result.observability.eigenvalues=solver.eigenvalues();const double trace=result.observability.eigenvalues.sum(),top=result.observability.eigenvalues.maxCoeff();if(top>std::numeric_limits<double>::epsilon()){result.observability.min_trace_ratio=result.observability.eigenvalues.minCoeff()/std::max(trace,std::numeric_limits<double>::epsilon());result.observability.condition_number=top/std::max(result.observability.eigenvalues.minCoeff(),std::numeric_limits<double>::epsilon());}result.wall_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();return result; }
}  // namespace soarm_localization
