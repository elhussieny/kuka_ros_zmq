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
  public RPY rotation() { return rotation(new RPY()); }
  public RPY rotation(RPY obj) { int o = __offset(8); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }

  public static void startkukaDesPose(FlatBufferBuilder builder) { builder.startObject(3); }
  public static void addPosition(FlatBufferBuilder builder, int positionOffset) { builder.addStruct(0, positionOffset, 0); }
  public static void addRotation(FlatBufferBuilder builder, int rotationOffset) { builder.addStruct(2, rotationOffset, 0); }
  public static int endkukaDesPose(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
  public static void finishkukaDesPoseBuffer(FlatBufferBuilder builder, int offset) { builder.finish(offset); }
};

