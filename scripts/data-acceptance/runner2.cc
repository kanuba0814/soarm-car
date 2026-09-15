// Generalized ORB-SLAM3 RGB-D runner: crop window (offset, height, width) is
// configurable so TUM 640x480 and DC1 640x400 windows share one binary.
// Same per-frame CSV contract as the frozen orbslam3-validation runner.
#include <System.h>
#include <MapPoint.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <sstream>
int main(int argc,char**argv){
 if(argc!=8){std::cerr<<"usage: runner2 vocab config inputdir outdir row_offset rows cols\n";return 2;}
 cv::setNumThreads(1);
 ORB_SLAM3::System slam(argv[1],argv[2],ORB_SLAM3::System::RGBD,false);
 const int offset=std::stoi(argv[5]),rows=std::stoi(argv[6]),cols=std::stoi(argv[7]);
 std::ifstream in(std::string(argv[3])+"/associations.txt");
 std::ofstream out(std::string(argv[4])+"/tracking.csv");
 out<<"timestamp,state,features,map_points,track_ms,x,y,z,qx,qy,qz,qw\n"<<std::setprecision(17);
 double t,td; std::string rgb,depth; int index=0;
 while(in>>t>>rgb>>td>>depth){
  auto im=cv::imread(std::string(argv[3])+"/"+rgb); auto d=cv::imread(std::string(argv[3])+"/"+depth,-1);
  if(im.empty()||d.empty()) return 3;
  if(im.rows<offset+rows||im.cols<cols||d.rows!=rows||d.cols!=cols){
   std::cerr<<"shape mismatch rgb "<<im.size()<<" depth "<<d.size()<<" window "<<cols<<"x"<<rows<<"+"<<offset<<std::endl;return 4;}
  im=im(cv::Rect(0,offset,cols,rows)).clone();
  auto start=std::chrono::steady_clock::now();
  auto pose=slam.TrackRGBD(im,d,t); int state=slam.GetTrackingState();
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  int points=0; for(auto p:slam.GetTrackedMapPoints()) if(p&&!p->isBad()&&p->Observations()>0) points++;
  auto twc=pose.inverse(); auto v=twc.translation(); auto q=twc.unit_quaternion();
  out<<t<<','<<state<<','<<slam.GetTrackedKeyPointsUn().size()<<','<<points<<','<<ms<<','<<v.x()<<','<<v.y()<<','<<v.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<std::endl;
  if(++index%100==0) std::cout<<"VALIDATION_FRAME "<<index<<" state="<<state<<std::endl;
  auto pos=in.tellg(); double next; if(in>>next){double wait=(next-t)*1000-ms; if(wait>0&&wait<500) std::this_thread::sleep_for(std::chrono::duration<double,std::milli>(wait));} in.clear(); in.seekg(pos);
 }
 slam.Shutdown();
 slam.SaveTrajectoryTUM(std::string(argv[4])+"/CameraTrajectory.txt");
 slam.SaveKeyFrameTrajectoryTUM(std::string(argv[4])+"/KeyFrameTrajectory.txt");
}
