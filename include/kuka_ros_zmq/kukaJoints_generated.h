// automatically generated by the FlatBuffers compiler, do not modify

#ifndef FLATBUFFERS_GENERATED_KUKAJOINTS_KUKA_JOINTS_FLATBUFFER_H_
#define FLATBUFFERS_GENERATED_KUKAJOINTS_KUKA_JOINTS_FLATBUFFER_H_

#include "flatbuffers/flatbuffers.h"

namespace kuka_joints {
namespace flatbuffer {

struct kukaJoints;

struct kukaJoints FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum {
    VT_ANGLEVALUE = 4,
    VT_ROBOTNAME = 6
  };
  const flatbuffers::Vector<double> *angleValue() const { return GetPointer<const flatbuffers::Vector<double> *>(VT_ANGLEVALUE); }
  const flatbuffers::String *robotName() const { return GetPointer<const flatbuffers::String *>(VT_ROBOTNAME); }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyField<flatbuffers::uoffset_t>(verifier, VT_ANGLEVALUE) &&
           verifier.Verify(angleValue()) &&
           VerifyField<flatbuffers::uoffset_t>(verifier, VT_ROBOTNAME) &&
           verifier.Verify(robotName()) &&
           verifier.EndTable();
  }
};

struct kukaJointsBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_angleValue(flatbuffers::Offset<flatbuffers::Vector<double>> angleValue) { fbb_.AddOffset(kukaJoints::VT_ANGLEVALUE, angleValue); }
  void add_robotName(flatbuffers::Offset<flatbuffers::String> robotName) { fbb_.AddOffset(kukaJoints::VT_ROBOTNAME, robotName); }
  kukaJointsBuilder(flatbuffers::FlatBufferBuilder &_fbb) : fbb_(_fbb) { start_ = fbb_.StartTable(); }
  kukaJointsBuilder &operator=(const kukaJointsBuilder &);
  flatbuffers::Offset<kukaJoints> Finish() {
    auto o = flatbuffers::Offset<kukaJoints>(fbb_.EndTable(start_, 2));
    return o;
  }
};

inline flatbuffers::Offset<kukaJoints> CreatekukaJoints(flatbuffers::FlatBufferBuilder &_fbb,
   flatbuffers::Offset<flatbuffers::Vector<double>> angleValue = 0,
   flatbuffers::Offset<flatbuffers::String> robotName = 0) {
  kukaJointsBuilder builder_(_fbb);
  builder_.add_robotName(robotName);
  builder_.add_angleValue(angleValue);
  return builder_.Finish();
}

inline const kuka_joints::flatbuffer::kukaJoints *GetkukaJoints(const void *buf) { return flatbuffers::GetRoot<kuka_joints::flatbuffer::kukaJoints>(buf); }

inline bool VerifykukaJointsBuffer(flatbuffers::Verifier &verifier) { return verifier.VerifyBuffer<kuka_joints::flatbuffer::kukaJoints>(); }

inline void FinishkukaJointsBuffer(flatbuffers::FlatBufferBuilder &fbb, flatbuffers::Offset<kuka_joints::flatbuffer::kukaJoints> root) { fbb.Finish(root); }

}  // namespace flatbuffer
}  // namespace kuka_joints

#endif  // FLATBUFFERS_GENERATED_KUKAJOINTS_KUKA_JOINTS_FLATBUFFER_H_
