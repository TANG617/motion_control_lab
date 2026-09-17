#include "../elbow_reference.hpp"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

using namespace motion_control_lab::hierarchical_kinematics_step;
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
int main(int argc, char** argv) {
  try {
    if (argc!=3 && argc!=4) throw std::runtime_error("usage: harp_lifecycle MODEL_DIR RECORD_PATH [cpu|cuda]");
    ElbowReferenceOptions options;
    options.source="harp"; options.harp_model_directory=argv[1];
    options.record_path=argv[2];
    if(argc==4)options.harp_device=argv[3];
    auto pose=Eigen::Isometry3d::Identity();
    pose.translation()=Eigen::Vector3d(.35,.25,.1);
    {
      ElbowReference reference(options,1000);
      reference.initialize({pose,pose},17);
      auto initial=reference.consume(0);
      require(initial.sequence==0 && initial.generation==17,"initial identity");
      // Wall time advances during pause; active reference age remains frozen.
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      auto paused=reference.consume(0);
      require(paused.sequence==0 && paused.arm_angles==initial.arm_angles,"pause freezes reference time");
      for (int i=1; i<=100; ++i) {
        pose.translation().x()=.35+i*.00001;
        reference.sampleAccepted(i*.01,{pose,pose});
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      auto last=reference.consume(1.0);
      require(last.sequence==100 && last.generation==17,"latest full window survives skipped requests");
      bool expired=false;
      try { reference.consume(1.101); }
      catch(const std::runtime_error& e) { expired=std::string(e.what()).find("maximum active-time age")!=std::string::npos; }
      require(expired,"stale output terminates rather than falling back");
      reference.finish();
    }
    // Each run owns its worker, buffers and Predictor. Destruction joins first.
    options.record_path.clear();
    {
      ElbowReference restarted(options,1000);
      restarted.initialize({pose,pose},18);
      const auto value=restarted.consume(0);
      require(value.generation==18 && value.sequence==0,"restart clears history and old results");
      pose.translation().x()=std::numeric_limits<double>::quiet_NaN();
      restarted.sampleAccepted(.01,{pose,pose});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      bool propagated=false;
      try { restarted.consume(.01); }
      catch(const std::runtime_error& e) { propagated=std::string(e.what()).find("not finite")!=std::string::npos; }
      require(propagated,"inference input failure must propagate unchanged from worker");
      restarted.finish();
    }
    // TEST-ONLY graph selects faulty outputs by the oldest left wrist x value.
    // A normal robot pose does not trigger these deliberate faults.
    if (const char* fixture=std::getenv("MCL_HARP_INVALID_MODEL")) {
      auto faulty_options=options;
      faulty_options.harp_model_directory=fixture;
      faulty_options.harp_device="cpu";
      for (double marker : {1.,2.,3.,4.,5.,6.}) {
        auto bad_pose=Eigen::Isometry3d::Identity();
        bad_pose.translation().x()=marker;
        bool rejected=false;
        try {
          ElbowReference faulty(faulty_options,1000);
          faulty.initialize({bad_pose,bad_pose},20);
        } catch (const std::runtime_error&) { rejected=true; }
        require(rejected,"invalid predicted pair or raw norm must fail startup");
      }
      auto safe_pose=Eigen::Isometry3d::Identity();
      ElbowReference faulty(faulty_options,1000);
      faulty.initialize({safe_pose,safe_pose},21);
      safe_pose.translation().x()=3.;
      for (int i=1;i<=30;++i) faulty.sampleAccepted(i*.01,{safe_pose,safe_pose});
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      bool propagated=false;
      try { faulty.consume(.30); }
      catch (const std::runtime_error& e) {
        propagated=std::string(e.what()).find("nonfinite")!=std::string::npos;
      }
      require(propagated,"non-finite output must propagate unchanged from worker");
      faulty.finish();
    }
    options.harp_model_directory += "/missing";
    bool failed=false;
    try { ElbowReference invalid(options,1000); invalid.initialize({pose,pose},19); }
    catch(const std::runtime_error&) { failed=true; }
    require(failed,"load failure must propagate");
    std::cout << "HARP pause, skip, expiration, restart and load failure passed\n";
    return 0;
  } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
