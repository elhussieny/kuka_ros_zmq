// automatically generated, do not modify

package ros_kuka.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class Vector3 extends Struct {
  public Vector3 __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public double x() { return bb.getDouble(bb_pos + 0); }
  public double y() { return bb.getDouble(bb_pos + 8); }
  public double z() { return bb.getDouble(bb_pos + 16); }

  public static int createVector3(FlatBufferBuilder builder, double x, double y, double z) {
    builder.prep(8, 24);
    builder.putDouble(z);
    builder.putDouble(y);
    builder.putDouble(x);
    return builder.offset();
  }
};

