#include "motion_control_lab/execution_request.hpp"
#include <cassert>
#include <iostream>
#include <chrono>
int main(int argc,char** argv) {
 using namespace motion_control_lab;using namespace execution;
 std::filesystem::path d=std::filesystem::path(argv[1])/std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());std::filesystem::create_directories(d);
 Json::Value input;input["samples"]=Json::Value(Json::arrayValue);writeJson(d/"input.json",input);
 Json::Value r;r["schema_version"]="execution_request.v1";r["app_id"]="contract-test";r["execution_structure"]="snapshot";
 r["input"]["path"]=(d/"input.json").string();r["input"]["sha256"]=sha256_file(d/"input.json");r["input"]["format"]="json";r["output_dir"]=(d/"out").string();
 for(auto k:{"app_config","execution","observation","tracking"})r[k]=Json::Value(Json::objectValue);
 writeJson(d/"request.json",r);auto loaded=readRequest(d/"request.json","contract-test");assert(loaded.input==input);assert(!std::filesystem::exists(d/"out"));
 auto reject=[&](auto f){bool failed=false;try{f();}catch(const std::exception&){failed=true;}assert(failed);};
 reject([&]{readRequest(d/"request.json","wrong");});
 beginOutput(loaded,r["app_config"],Json::Value());reject([&]{beginOutput(loaded,Json::Value(),Json::Value());});
 writeJson(d/"input.json",Json::Value("changed"));reject([&]{readRequest(d/"request.json","contract-test");});
 std::cout<<"identity, input hash, read-only load, append-only output: passed\n";
}
