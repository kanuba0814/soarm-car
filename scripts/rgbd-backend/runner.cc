// Offline external-odometry experiment. Identity information is an uncalibrated
// graph weight, NOT a pose uncertainty estimate or an operational health signal.
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Link.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace rtabmap;

static void writePose(std::ostream &out, const Transform &p) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) out << ',' << p(r, c);
}

int main(int argc, char **argv) try {
    if (argc < 12) {
        std::cerr << "runner images keyframes output fx fy cx cy rows depth_scale max_depth refine [parameter=value ...]\n";
        return 2;
    }
    cv::setNumThreads(1); cv::setRNGSeed(17);
    const std::string images=argv[1], output=argv[3];
    const int rows=std::stoi(argv[8]);
    const double depthScale=std::stod(argv[9]);
    ParametersMap params;
    params["Vis/FeatureType"]="8"; params["Kp/DetectorStrategy"]="8";
    params["Vis/MinDepth"]="0.3"; params["Vis/MaxDepth"]=argv[10];
    params["Kp/MinDepth"]="0.3"; params["Kp/MaxDepth"]=argv[10];
    params["RGBD/CreateOccupancyGrid"]="false";
    params["Mem/GenerateIds"]="false";
    params["RGBD/LinearUpdate"]="0"; params["RGBD/AngularUpdate"]="0";
    // Position proximity is unreliable between disconnected odometry epochs.
    params["RGBD/ProximityBySpace"]="false";
    params["RGBD/ProximityByTime"]="false";
    params["RGBD/NeighborLinkRefining"]=argv[11];
    params["Optimizer/VarianceIgnored"]="true";
    params["Rtabmap/WorkingDirectory"]=output;
    for(int i=12;i<argc;++i) {
        const std::string option=argv[i];
        const auto pos=option.find('=');
        if(pos==std::string::npos) throw std::runtime_error("Expected parameter=value");
        params[option.substr(0,pos)]=option.substr(pos+1);
    }
    auto defaults=Parameters::getDefaultParameters();
    for (const auto &p:params) {
        if (!defaults.count(p.first)) throw std::runtime_error("Unknown parameter "+p.first);
        defaults[p.first]=p.second;
    }
    std::ofstream config(output+"/parameters.txt");
    for (const auto &p:defaults) config << p.first << '=' << p.second << '\n';
    Rtabmap slam; slam.init(params, output+"/map.db");
    CameraModel camera(std::stod(argv[4]),std::stod(argv[5]),std::stod(argv[6]),std::stod(argv[7]),
                       Transform::getIdentity(),0,cv::Size(640,rows));
    std::ifstream input(argv[2]);
    if (!input) throw std::runtime_error("missing keyframes");
    std::ofstream online(output+"/online.csv");
    online << "id,timestamp,segment,processed,loop_id,backend_ms";
    for (const auto &prefix:{"raw", "online"})
        for (int r=0;r<3;++r) for(int c=0;c<4;++c) online<<','<<prefix<<r<<c;
    online << '\n' << std::setprecision(17);
    int id,segment,lastSegment=-1,count=0; double t;
    std::string rgbName,depthName;
    while (input>>id>>t>>segment>>rgbName>>depthName) {
        cv::Mat matrix(3,4,CV_32FC1);
        for (int r=0;r<3;++r) for (int c=0;c<4;++c)
            if (!(input>>matrix.at<float>(r,c))) throw std::runtime_error("invalid pose");
        if (lastSegment>=0 && segment!=lastSegment) slam.triggerNewMap();
        lastSegment=segment;
        auto color=cv::imread(images+"/"+rgbName);
        auto rawDepth=cv::imread(images+"/"+depthName,-1);
        if (color.empty() || rawDepth.empty() || color.cols!=640 || color.rows<rows ||
            rawDepth.cols!=640 || rawDepth.rows!=rows || rawDepth.type()!=CV_16UC1)
            throw std::runtime_error("invalid RGB-D input");
        color=color(cv::Rect(0,0,640,rows)).clone();
        cv::Mat depth; rawDepth.convertTo(depth,CV_32FC1,depthScale);
        depth.setTo(0,(depth<0.3)|(depth>std::stod(argv[10])));
        const Transform pose(matrix);
        SensorData data(color,depth,camera,id,t);
        auto start=std::chrono::steady_clock::now();
        bool processed=slam.process(data,pose,cv::Mat::eye(6,6,CV_64FC1));
        double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        online<<id<<','<<t<<','<<segment<<','<<processed<<','<<slam.getLoopClosureId()<<','<<ms;
        writePose(online,pose); writePose(online,slam.getMapCorrection()*pose); online<<std::endl;
        if (++count%40==0) std::cout<<"KEYFRAMES "<<count<<std::endl;
    }
    std::map<int,Transform> poses; std::multimap<int,Link> links;
    slam.getGraph(poses,links,true,true);
    std::ofstream graph(output+"/optimized.csv");
    graph<<"id"; for(int r=0;r<3;++r) for(int c=0;c<4;++c) graph<<",t"<<r<<c;
    graph<<'\n'<<std::setprecision(17);
    for (const auto &p:poses) {graph<<p.first; writePose(graph,p.second);graph<<'\n';}
    std::ofstream edges(output+"/links.csv");
    edges<<"from,to,type"; for(int r=0;r<3;++r) for(int c=0;c<4;++c) edges<<",t"<<r<<c;
    edges<<'\n'<<std::setprecision(17);
    for (const auto &p:links) {
        edges<<p.second.from()<<','<<p.second.to()<<','<<int(p.second.type());
        writePose(edges,p.second.transform()); edges<<'\n';
    }
    slam.close();
    std::ofstream(output+"/complete.json")<<"{\"keyframes\":"<<count<<",\"optimized_nodes\":"<<poses.size()<<"}";
    return 0;
} catch (const std::exception &e) {std::cerr<<e.what()<<'\n';return 3;}
