// automatically generated, do not modify

package ros_kuka.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class kukaDesPose extends Table {
  public static kukaDesPose getRootAskukaDesPose(ByteBuffer _bb) { return getRootAskukaDesPose(_bb, new kukaDesPose()); }
  public static kukaDesPose getRootAskukaDesPose(ByteBuffer _bb, kukaDesPose obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public kukaDesPose __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public Vector3 position() { return position(new Vector3()); }
  public Vector3 position(Vector3 obj) { int o = __offset(4); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  public Quaternion orientation() { return orientation(new Quaternion()); }
  public Quaternion orientation(Quaternion obj) { int o = __offset(6); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  public RPY rotation() { return rotation(new RPY()); }
  public RPY rotation(RPY obj) { int o = __offset(8); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  public byte control() { int o = __offset(10); return o != 0 ? bb.get(o + bb_pos) : 1; }
  public double joints(int j) { int o = __offset(12); return o != 0 ? bb.getDouble(__vector(o) + j * 8) : 0; }
  public int jointsLength() { int o = __offset(12); return o != 0 ? __vector_len(o) : 0; }
  public ByteBuffer jointsAsByteBuffer() { return __vector_as_bytebuffer(12, 8); }

  public static void startkukaDesPose(FlatBufferBuilder builder) { builder.startObject(5); }
  public static void addPosition(FlatBufferBuilder builder, int positionOffset) { builder.addStruct(0, positionOffset, 0); }
  public static void addOrientation(FlatBufferBuilder builder, int orientationOffset) { builder.addStruct(1, orientationOffset, 0); }
  public static void addRotation(FlatBufferBuilder builder, int rotationOffset) { builder.addStruct(2, rotationOffset, 0); }
  public static void addControl(FlatBufferBuilder builder, byte control) { builder.addByte(3, control, 1); }
  public static void addJoints(FlatBufferBuilder builder, int jointsOffset) { builder.addOffset(4, jointsOffset, 0); }
  public static int createJointsVector(FlatBufferBuilder builder, double[] data) { builder.startVector(8, data.length, 8); for (int i = data.length - 1; i >= 0; i--) builder.addDouble(data[i]); return builder.endVector(); }
  public static void startJointsVector(FlatBufferBuilder builder, int numElems) { builder.startVector(8, numElems, 8); }
  public static int endkukaDesPose(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
  public static void finishkukaDesPoseBuffer(FlatBufferBuilder builder, int offset) { builder.finish(offset); }
};

