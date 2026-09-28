// Re-optimize each disconnected component separately using the same backend
// parameters. RTAB getGraph(optimized=true) only exports the active component.
// Read the completed database read-only so no earlier segment disappears.
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/SensorData.h>
#include <fstream>
#include <iomanip>
#include <memory>
#include <set>
#include <tuple>
#include <stdexcept>
#include <iostream>

using namespace rtabmap;

static void writePose(std::ostream &out, const Transform &p) {
    for(int r=0;r<3;++r) for(int c=0;c<4;++c) out<<','<<p(r,c);
}

int main(int argc,char **argv) try {
    if(argc!=2) return 2;
    const std::string dir=argv[1];
    ParametersMap params;
    std::ifstream config(dir+"/parameters.txt"); std::string line;
    while(std::getline(config,line)) {
        auto pos=line.find('=');
        if(pos!=std::string::npos) params[line.substr(0,pos)]=line.substr(pos+1);
    }
    if(params.empty()) throw std::runtime_error("missing parameters");
    std::unique_ptr<DBDriver> db(DBDriver::create());
    if(!db->openConnection(dir+"/map.db",false,true)) throw std::runtime_error("cannot open database read-only");
    std::set<int> ids; db->getAllNodeIds(ids);
    std::map<int,Transform> raw, optimized;
    for(int id:ids) {
        Transform pose,groundTruth; int map,weight; std::string label; double stamp;
        std::vector<float> velocity; GPS gps; EnvSensors sensors;
        if(!db->getNodeInfo(id,pose,map,weight,label,stamp,groundTruth,velocity,gps,sensors))
            throw std::runtime_error("missing node");
        raw[id]=pose;
    }
    std::multimap<int,Link> stored,links;
    db->getAllLinks(stored);
    std::set<std::tuple<int,int,int>> unique;
    for(const auto &p:stored) {
        auto link=p.second;
        auto key=std::make_tuple(std::min(link.from(),link.to()),std::max(link.from(),link.to()),int(link.type()));
        if(unique.insert(key).second) links.insert(p);
    }
    std::unique_ptr<Optimizer> optimizer(Optimizer::create(params));
    int componentCount=0;
    for(const auto &p:raw) {
        if(optimized.count(p.first)) continue;
        std::map<int,Transform> component;
        std::multimap<int,Link> componentLinks;
        optimizer->getConnectedGraph(p.first,raw,links,component,componentLinks);
        if(component.empty()) component[p.first]=p.second;
        auto result=component.size()>1?optimizer->optimize(p.first,component,componentLinks):component;
        if(result.size()!=component.size()) throw std::runtime_error("component optimization failed");
        optimized.insert(result.begin(),result.end()); ++componentCount;
    }
    std::ofstream out(dir+"/optimized-all.csv");
    out<<"id"; for(int r=0;r<3;++r) for(int c=0;c<4;++c) out<<",t"<<r<<c;
    out<<'\n'<<std::setprecision(17);
    for(const auto &p:optimized) {out<<p.first;writePose(out,p.second);out<<'\n';}
    std::ofstream edges(dir+"/links-all.csv");
    edges<<"from,to,type"; for(int r=0;r<3;++r) for(int c=0;c<4;++c) edges<<",t"<<r<<c;
    edges<<'\n'<<std::setprecision(17);
    for(const auto &p:links) {
        edges<<p.second.from()<<','<<p.second.to()<<','<<int(p.second.type());
        writePose(edges,p.second.transform());edges<<'\n';
    }
    std::ofstream(dir+"/export-complete.json")<<"{\"nodes\":"<<optimized.size()
        <<",\"components\":"<<componentCount<<",\"read_only_database\":true}";
    return 0;
} catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 3;}
