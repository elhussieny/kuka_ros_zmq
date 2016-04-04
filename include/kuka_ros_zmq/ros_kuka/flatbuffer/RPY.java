// automatically generated, do not modify

package ros_kuka.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class RPY extends Struct {
  public RPY __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public double alpha() { return bb.getDouble(bb_pos + 0); }
  public double beta() { return bb.getDouble(bb_pos + 8); }
  public double gamma() { return bb.getDouble(bb_pos + 16); }

  public static int createRPY(FlatBufferBuilder builder, double alpha, double beta, double gamma) {
    builder.prep(8, 24);
    builder.putDouble(gamma);
    builder.putDouble(beta);
    builder.putDouble(alpha);
    return builder.offset();
  }
};

