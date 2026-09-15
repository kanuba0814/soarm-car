#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <memory>
using namespace rtabmap;
int main(int argc,char**argv){
 if(argc!=3 && argc!=4) return 2;
 const bool reset3s=argc==4; double lostSince=-1; int epoch=0;
 cv::setNumThreads(1);cv::setRNGSeed(17);
 ParametersMap params;params["Odom/Strategy"]="0";params["Odom/ResetCountdown"]="0";
 params["Vis/FeatureType"]="8";params["Kp/DetectorStrategy"]="8";
 params["Vis/MaxDepth"]="5";params["Vis/MinDepth"]="0.3";
 params["Grid/FromDepth"]="false";params["Rtabmap/WorkingDirectory"]=argv[2];
 auto defaults=Parameters::getDefaultParameters();for(auto p:params) defaults[p.first]=p.second;
 std::ofstream config(std::string(argv[2])+"/parameters.txt");for(auto p:defaults) config<<p.first<<"="<<p.second<<"\n";
 std::unique_ptr<Odometry> odom(Odometry::create(params));Rtabmap slam;slam.init(params,std::string(argv[2])+"/map.db");
 CameraModel camera(488.09033203125,488.09033203125,320.88531494140625,211.70437622070312,Transform::getIdentity(),0,cv::Size(640,400));
 std::ifstream in(std::string(argv[1])+"/associations.txt");std::ofstream out(std::string(argv[2])+"/frames.csv");std::ofstream errors(std::string(argv[2])+"/rejections.txt");
 out<<"i,timestamp,ok,features,inliers,matches,odom_ms,backend_ms,backend_processed,loop_id,proximity_id,x,y,z,map_x,map_y,map_z,map_epoch\n"<<std::setprecision(17);
 double t,td,lastBackend=-1e30;std::string rgb,depth;int i=0;
 while(in>>t>>rgb>>td>>depth){
  auto color=cv::imread(std::string(argv[1])+"/"+rgb);auto d=cv::imread(std::string(argv[1])+"/"+depth,-1);
  if(color.empty()||d.empty()) return 3;color=color(cv::Rect(0,0,640,400)).clone();
  if(reset3s && lostSince>=0 && t-lostSince>3.0){odom->reset();slam.triggerNewMap();epoch++;lostSince=-1;lastBackend=-1e30;}
  SensorData data(color,d,camera,++i,t);OdometryInfo info;
  auto start=std::chrono::steady_clock::now();auto pose=odom->process(data,&info);double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  bool ok=!pose.isNull(); if(ok) lostSince=-1; else if(lostSince<0) lostSince=t;bool processed=false;int loop=0,prox=0;double backendMs=0;
  if(ok&&t-lastBackend>=1.0){start=std::chrono::steady_clock::now();processed=slam.process(data,pose,info.reg.covariance.empty()?cv::Mat::eye(6,6,CV_64FC1):info.reg.covariance);backendMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();lastBackend=t;loop=slam.getStatistics().loopClosureId();prox=slam.getStatistics().proximityDetectionId();}
  auto mapped=ok?slam.getMapCorrection()*pose:Transform();
  out<<i-1<<','<<t<<','<<ok<<','<<info.features<<','<<info.reg.inliers<<','<<info.reg.matches<<','<<ms<<','<<backendMs<<','<<processed<<','<<loop<<','<<prox;
  if(ok) out<<','<<pose.x()<<','<<pose.y()<<','<<pose.z()<<','<<mapped.x()<<','<<mapped.y()<<','<<mapped.z();else out<<",nan,nan,nan,nan,nan,nan";
  out<<','<<epoch<<std::endl;if(!ok) errors<<i-1<<' '<<t<<' '<<info.reg.rejectedMsg<<'\n';
  if(i%100==0) std::cout<<"VALIDATION_FRAME "<<i<<" ok="<<ok<<std::endl;
 }
 slam.close();std::ofstream(std::string(argv[2])+"/complete.json")<<"{\"frames\":"<<i<<"}";
}
