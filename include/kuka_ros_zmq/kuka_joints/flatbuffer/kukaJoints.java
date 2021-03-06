// automatically generated, do not modify

package kuka_joints.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class kukaJoints extends Table {
  public static kukaJoints getRootAskukaJoints(ByteBuffer _bb) { return getRootAskukaJoints(_bb, new kukaJoints()); }
  public static kukaJoints getRootAskukaJoints(ByteBuffer _bb, kukaJoints obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public kukaJoints __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public double angleValue(int j) { int o = __offset(4); return o != 0 ? bb.getDouble(__vector(o) + j * 8) : 0; }
  public int angleValueLength() { int o = __offset(4); return o != 0 ? __vector_len(o) : 0; }
  public ByteBuffer angleValueAsByteBuffer() { return __vector_as_bytebuffer(4, 8); }
  public Vector3 posValue() { return posValue(new Vector3()); }
  public Vector3 posValue(Vector3 obj) { int o = __offset(6); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  public Quaternion rotValue() { return rotValue(new Quaternion()); }
  public Quaternion rotValue(Quaternion obj) { int o = __offset(8); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  public String robotName() { int o = __offset(10); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer robotNameAsByteBuffer() { return __vector_as_bytebuffer(10, 1); }

  public static void startkukaJoints(FlatBufferBuilder builder) { builder.startObject(4); }
  public static void addAngleValue(FlatBufferBuilder builder, int angleValueOffset) { builder.addOffset(0, angleValueOffset, 0); }
  public static int createAngleValueVector(FlatBufferBuilder builder, double[] data) { builder.startVector(8, data.length, 8); for (int i = data.length - 1; i >= 0; i--) builder.addDouble(data[i]); return builder.endVector(); }
  public static void startAngleValueVector(FlatBufferBuilder builder, int numElems) { builder.startVector(8, numElems, 8); }
  public static void addPosValue(FlatBufferBuilder builder, int posValueOffset) { builder.addStruct(1, posValueOffset, 0); }
  public static void addRotValue(FlatBufferBuilder builder, int rotValueOffset) { builder.addStruct(2, rotValueOffset, 0); }
  public static void addRobotName(FlatBufferBuilder builder, int robotNameOffset) { builder.addOffset(3, robotNameOffset, 0); }
  public static int endkukaJoints(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
  public static void finishkukaJointsBuffer(FlatBufferBuilder builder, int offset) { builder.finish(offset); }
};

