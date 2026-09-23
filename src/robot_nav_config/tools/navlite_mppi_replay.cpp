// Offline only: no rclcpp::init, ROS node, topic, action, service or motion command.
#include <fstream>
#include <iomanip>
#include <iostream>
#include "robot_nav_config/navlite_snapshot_replay.hpp"

namespace ns=robot_nav_config::navlite_snapshot;
namespace fs=std::filesystem;
using Json=nlohmann::json;

int main(int argc,char ** argv)
{
  try {
    if(argc!=5 || std::string(argv[1])!="--input" || std::string(argv[3])!="--output") {
      std::cerr<<"usage: navlite_mppi_replay --input mppi_instance_directory --output new_directory\n";return 2;
    }
    const fs::path input=argv[2], output=argv[4];
    Json schema; {std::ifstream f(input/"schema.json"); f>>schema;}
    const uint16_t endian=1;
    if(schema.at("schema")!=1 || schema.at("byte_order")!=
      (*reinterpret_cast<const uint8_t *>(&endian)?"little":"big")) {
      throw std::runtime_error("unsupported schema/byte order");
    }
    if(!fs::create_directory(output)) {throw std::runtime_error("output must be a new directory");}
    std::ifstream index(input/"frames.jsonl"), payload(input/"payload.bin",std::ios::binary);
    if(!index || !payload) {throw std::runtime_error("missing frames/payload");}
    std::ofstream csv(output/"trajectory.csv");
    csv<<"compute_seq,index,time_sec,x,y,yaw,predicted_vx,predicted_wz,command_offset\n"<<std::setprecision(17);
    Json status{{"schema",1},{"processed",0},{"skipped",0},{"expected_no_return",0},{"reasons",Json::array()},
      {"semantics","native Ranger response prediction of captured final pre-shift sequence; not measured motion"}};
    auto frame=std::make_unique<ns::Frame>(); std::string line; uint64_t line_number=0;
    while(std::getline(index,line)) {
      ++line_number;
      try {
        const auto j=Json::parse(line); auto & f=*frame;
        if(!j.at("command_returned").get<bool>()) {
          status["expected_no_return"]=status["expected_no_return"].get<unsigned>()+1; continue;
        }
        static_cast<ns::Metadata &>(f)=ns::Metadata{};
        f.compute_seq=j.at("compute_seq"); f.incomplete=j.at("incomplete");
        f.model_enabled=j.at("model_enabled"); f.sequence_valid=j.at("sequence_valid");
        f.command_returned=j.at("command_returned"); f.command_offset=j.at("command_offset");
        f.sequence_original_count=j.at("sequence_original_count");
        f.sequence_count=j.at("sequence").at("shape").at(0);
        f.model_dt=j.at("model_dt"); f.min_turning_radius=j.at("min_turning_radius");
        f.pose=j.at("pose").get<std::array<double,7>>(); f.odom=j.at("odom").get<std::array<double,6>>();
        f.command=j.at("command").get<std::array<double,6>>();
        f.dynamics=j.at("dynamics").get<std::array<double,8>>();
        f.smoother_limits=j.at("smoother_limits").get<std::array<double,4>>();
        f.constraints=j.at("constraints").get<std::array<double,4>>();
        f.smoother_valid=j.at("smoother_valid");
        f.smoother_linear=j.at("smoother_initial").at(0); f.smoother_angular=j.at("smoother_initial").at(1);
        const auto & issued=j.at("issued"); f.history_count=issued.size();
        if(f.history_count>ns::kHistory || f.sequence_count>ns::kSteps || f.command_offset>=f.sequence_count) {
          throw std::runtime_error("invalid capacities/offset");
        }
        for(unsigned i=0;i<f.history_count;++i) {f.issued[i]=issued[i].get<std::array<double,3>>();}
        const auto & block=j.at("sequence");
        if(block.at("dtype")!="f8" || block.at("shape").at(1)!=3 ||
          block.at("length")!=f.sequence_count*3*sizeof(double)) {throw std::runtime_error("invalid sequence block");}
        payload.clear(); payload.seekg(block.at("offset").get<uint64_t>());
        payload.read(reinterpret_cast<char *>(f.sequence.data()),f.sequence_count*3*sizeof(double));
        if(!payload) {throw std::runtime_error("truncated sequence payload");}
        const auto & selected=f.sequence[f.command_offset];
        if(selected[0]!=f.command[0] || selected[1]!=f.command[1] || selected[2]!=f.command[5]) {
          throw std::runtime_error("selected sequence does not equal actual returned command");
        }
        const auto prediction=ns::replay(f);
        for(unsigned t=0;t<f.sequence_count;++t) {
          csv<<f.compute_seq<<','<<t<<','<<(t+1)*f.model_dt<<','<<prediction.trajectory.x(0,t)<<','
            <<prediction.trajectory.y(0,t)<<','<<prediction.trajectory.yaws(0,t)<<','
            <<prediction.state.vx(0,t)<<','<<prediction.state.wz(0,t)<<','<<f.command_offset<<'\n';
        }
        status["processed"]=status["processed"].get<unsigned>()+1;
      } catch(const std::exception & e) {
        status["skipped"]=status["skipped"].get<unsigned>()+1;
        status["reasons"].push_back({{"frames_line",line_number},{"reason",e.what()}});
      }
    }
    csv.close(); if(!csv) {throw std::runtime_error("trajectory write failed");}
    status["trajectory_available"]=status["processed"].get<unsigned>()>0;
    status["complete"]=status["trajectory_available"].get<bool>() && status["skipped"]==0;
    std::ofstream summary(output/"replay_status.json"); summary<<status.dump(2);
    std::cout<<status.dump()<<'\n'; return !summary?1:(status["trajectory_available"].get<bool>()?0:3);
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
