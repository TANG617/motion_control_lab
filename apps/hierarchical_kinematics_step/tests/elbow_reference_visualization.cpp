#include "../elbow_reference_visualization.hpp"
#include <cmath>
#include <iostream>
#include <json/json.h>
#include <stdexcept>
namespace app=motion_control_lab::hierarchical_kinematics_step;
void require(bool v,const char *m) { if(!v) throw std::runtime_error(m); }
Json::Value decode(const motion_control::viz::EncodedMessageSample &sample) {
  Json::CharReaderBuilder b;std::unique_ptr<Json::CharReader> r(b.newCharReader());
  Json::Value value;std::string error;
  const auto *p=reinterpret_cast<const char*>(sample.data.data());
  require(r->parse(p,p+sample.data.size(),&value,&error),"JSON decode");return value;
}
int main() {
 try {
  constexpr double pi=3.141592653589793;
  std::array<app::ElbowReferenceAngleTracker,2> trackers;
  app::ElbowPrediction p;
  for(int side=0;side<2;++side) {
    p.arm_angles[side]={std::cos(pi-.01),std::sin(pi-.01)};
    require(std::abs(trackers[side].update(p,side)-(pi-.01))<1e-12,"initial angle");
    ++p.sequence;p.arm_angles[side]={std::cos(-pi+.01),std::sin(-pi+.01)};
    require(std::abs(trackers[side].update(p,side)-(pi+.01))<1e-12,"unwrap");
    require(std::abs(trackers[side].update(p,side)-(pi+.01))<1e-12,"pause/repeat");
  }
  app::ArmPoses ee{Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity()};
  ee[0].translation()<<.4,.2,0;ee[1].translation()<<.4,-.2,0;
  std::array<Eigen::Vector3d,2> shoulders{Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
  std::array<app::ElbowGeometry,2> geometry;
  for(auto &g:geometry) {g.upper_length=.3;g.lower_length=.3;}
  Eigen::Isometry3d ref=Eigen::Isometry3d::Identity();
  ref.linear()=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitY()).toRotationMatrix();
  ref.translation()<<.1,.02,.03;
  for(const std::string source:{"harp","recorded"})
  for(int mask:{1,2,3}) {
    app::ElbowConsumption record;
    record.prediction.generation=++p.generation;
    record.time_s=.01;record.selected_priority=1;
    for(int a=0;a<2;++a) {
      record.prediction.arm_angles[a]={std::cos(.3+a),std::sin(.3+a)};
      record.arms[a].enabled=(mask&(1<<a))!=0;
      const Eigen::Vector3d target=ref.inverse()*geometry[a].target(ref*shoulders[a],ref*ee[a],record.prediction.arm_angles[a][0],record.prediction.arm_angles[a][1]);
      for(int i=0;i<3;++i) record.arms[a].target[i]=target[i];
    }
    auto snap=app::makeElbowReferenceVisualizationSnapshot(record,geometry,shoulders,ee,ref,trackers);
    motion_control::viz::RenderBatch batch;batch.timestamp_ns=123;
    app::appendElbowReferenceVisualization(batch,"base_link",source,snap);
    require(batch.encoded_messages.size()==2,"both predicted angles published");
    require(batch.line_strips.size()==(mask==3?16:8),"only enabled circles displayed");
    for(int a=0;a<2;++a) {
      const auto j=decode(batch.encoded_messages[a]);
      const std::string topic=(source=="harp"?"/mcl/harp/":"/mcl/elbow_reference/")+std::string(a==0?"left":"right");
      require(batch.encoded_messages[a].channel==topic+"/angle","symmetric topic");
      require(j["task_enabled"].asBool()==record.arms[a].enabled,"prediction vs task participation");
      require(std::abs(j["angle_rad"].asDouble()-(.3+a))<1e-12,"arm order");
      require(!j["secondary_selected"].asBool(),"Primary-only is not posture completion");
      for(const auto &line:batch.line_strips) {
        if(line.channel!=topic+"/scene") continue;
        require(line.frame_id=="base_link","scene frame");
        if(line.entity_id==(source=="harp"?"harp_direction":"elbow_reference_direction"))
          require(line.points_m.back()==record.arms[a].target,"exact consumed elbow endpoint");
        if(line.entity_id=="elbow_circle") for(const auto &x:line.points_m) {
          Eigen::Vector3d q(x[0],x[1],x[2]);
          require(std::abs((q-shoulders[a]).norm()-.3)<1e-12,"rotated upper circle");
          require(std::abs((q-ee[a].translation()).norm()-.3)<1e-12,"rotated lower circle");
        }
      }
    }
    motion_control::viz::RenderBatch empty;
    app::appendElbowReferenceVisualization(empty,"base_link","manual",snap);
    require(empty.encoded_messages.empty(),"manual has no prediction");
  }
  return 0;
 } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
