// automatically generated by the FlatBuffers compiler, do not modify

#ifndef FLATBUFFERS_GENERATED_KUKAMONITOR_KUKA_JOINTS_FLATBUFFER_H_
#define FLATBUFFERS_GENERATED_KUKAMONITOR_KUKA_JOINTS_FLATBUFFER_H_

#include "flatbuffers/flatbuffers.h"

namespace kuka_joints {
namespace flatbuffer {

struct Vector3;

struct Quaternion;

struct kukaJoints;

MANUALLY_ALIGNED_STRUCT(8) Vector3 FLATBUFFERS_FINAL_CLASS {
 private:
  double x_;
  double y_;
  double z_;

 public:
  Vector3(double _x, double _y, double _z)
    : x_(flatbuffers::EndianScalar(_x)), y_(flatbuffers::EndianScalar(_y)), z_(flatbuffers::EndianScalar(_z)) { }

  double x() const { return flatbuffers::EndianScalar(x_); }
  double y() const { return flatbuffers::EndianScalar(y_); }
  double z() const { return flatbuffers::EndianScalar(z_); }
};
STRUCT_END(Vector3, 24);

MANUALLY_ALIGNED_STRUCT(8) Quaternion FLATBUFFERS_FINAL_CLASS {
 private:
  double x_;
  double y_;
  double z_;
  double w_;

 public:
  Quaternion(double _x, double _y, double _z, double _w)
    : x_(flatbuffers::EndianScalar(_x)), y_(flatbuffers::EndianScalar(_y)), z_(flatbuffers::EndianScalar(_z)), w_(flatbuffers::EndianScalar(_w)) { }

  double x() const { return flatbuffers::EndianScalar(x_); }
  double y() const { return flatbuffers::EndianScalar(y_); }
  double z() const { return flatbuffers::EndianScalar(z_); }
  double w() const { return flatbuffers::EndianScalar(w_); }
};
STRUCT_END(Quaternion, 32);

struct kukaJoints FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum {
    VT_ANGLEVALUE = 4,
    VT_POSVALUE = 6,
    VT_ROTVALUE = 8,
    VT_ROBOTNAME = 10
  };
  const flatbuffers::Vector<double> *angleValue() const { return GetPointer<const flatbuffers::Vector<double> *>(VT_ANGLEVALUE); }
  const Vector3 *posValue() const { return GetStruct<const Vector3 *>(VT_POSVALUE); }
  const Quaternion *rotValue() const { return GetStruct<const Quaternion *>(VT_ROTVALUE); }
  const flatbuffers::String *robotName() const { return GetPointer<const flatbuffers::String *>(VT_ROBOTNAME); }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyField<flatbuffers::uoffset_t>(verifier, VT_ANGLEVALUE) &&
           verifier.Verify(angleValue()) &&
           VerifyField<Vector3>(verifier, VT_POSVALUE) &&
           VerifyField<Quaternion>(verifier, VT_ROTVALUE) &&
           VerifyField<flatbuffers::uoffset_t>(verifier, VT_ROBOTNAME) &&
           verifier.Verify(robotName()) &&
           verifier.EndTable();
  }
};

struct kukaJointsBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_angleValue(flatbuffers::Offset<flatbuffers::Vector<double>> angleValue) { fbb_.AddOffset(kukaJoints::VT_ANGLEVALUE, angleValue); }
  void add_posValue(const Vector3 *posValue) { fbb_.AddStruct(kukaJoints::VT_POSVALUE, posValue); }
  void add_rotValue(const Quaternion *rotValue) { fbb_.AddStruct(kukaJoints::VT_ROTVALUE, rotValue); }
  void add_robotName(flatbuffers::Offset<flatbuffers::String> robotName) { fbb_.AddOffset(kukaJoints::VT_ROBOTNAME, robotName); }
  kukaJointsBuilder(flatbuffers::FlatBufferBuilder &_fbb) : fbb_(_fbb) { start_ = fbb_.StartTable(); }
  kukaJointsBuilder &operator=(const kukaJointsBuilder &);
  flatbuffers::Offset<kukaJoints> Finish() {
    auto o = flatbuffers::Offset<kukaJoints>(fbb_.EndTable(start_, 4));
    return o;
  }
};

inline flatbuffers::Offset<kukaJoints> CreatekukaJoints(flatbuffers::FlatBufferBuilder &_fbb,
   flatbuffers::Offset<flatbuffers::Vector<double>> angleValue = 0,
   const Vector3 *posValue = 0,
   const Quaternion *rotValue = 0,
   flatbuffers::Offset<flatbuffers::String> robotName = 0) {
  kukaJointsBuilder builder_(_fbb);
  builder_.add_robotName(robotName);
  builder_.add_rotValue(rotValue);
  builder_.add_posValue(posValue);
  builder_.add_angleValue(angleValue);
  return builder_.Finish();
}

inline const kuka_joints::flatbuffer::kukaJoints *GetkukaJoints(const void *buf) { return flatbuffers::GetRoot<kuka_joints::flatbuffer::kukaJoints>(buf); }

inline bool VerifykukaJointsBuffer(flatbuffers::Verifier &verifier) { return verifier.VerifyBuffer<kuka_joints::flatbuffer::kukaJoints>(); }

inline void FinishkukaJointsBuffer(flatbuffers::FlatBufferBuilder &fbb, flatbuffers::Offset<kuka_joints::flatbuffer::kukaJoints> root) { fbb.Finish(root); }

}  // namespace flatbuffer
}  // namespace kuka_joints

#endif  // FLATBUFFERS_GENERATED_KUKAMONITOR_KUKA_JOINTS_FLATBUFFER_H_
