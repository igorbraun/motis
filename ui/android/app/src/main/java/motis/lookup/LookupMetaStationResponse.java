// automatically generated by the FlatBuffers compiler, do not modify

package motis.lookup;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class LookupMetaStationResponse extends Table {
  public static LookupMetaStationResponse getRootAsLookupMetaStationResponse(ByteBuffer _bb) { return getRootAsLookupMetaStationResponse(_bb, new LookupMetaStationResponse()); }
  public static LookupMetaStationResponse getRootAsLookupMetaStationResponse(ByteBuffer _bb, LookupMetaStationResponse obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public LookupMetaStationResponse __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public motis.Station equivalent(int j) { return equivalent(new motis.Station(), j); }
  public motis.Station equivalent(motis.Station obj, int j) { int o = __offset(4); return o != 0 ? obj.__init(__indirect(__vector(o) + j * 4), bb) : null; }
  public int equivalentLength() { int o = __offset(4); return o != 0 ? __vector_len(o) : 0; }

  public static int createLookupMetaStationResponse(FlatBufferBuilder builder,
      int equivalentOffset) {
    builder.startObject(1);
    LookupMetaStationResponse.addEquivalent(builder, equivalentOffset);
    return LookupMetaStationResponse.endLookupMetaStationResponse(builder);
  }

  public static void startLookupMetaStationResponse(FlatBufferBuilder builder) { builder.startObject(1); }
  public static void addEquivalent(FlatBufferBuilder builder, int equivalentOffset) { builder.addOffset(0, equivalentOffset, 0); }
  public static int createEquivalentVector(FlatBufferBuilder builder, int[] data) { builder.startVector(4, data.length, 4); for (int i = data.length - 1; i >= 0; i--) builder.addOffset(data[i]); return builder.endVector(); }
  public static void startEquivalentVector(FlatBufferBuilder builder, int numElems) { builder.startVector(4, numElems, 4); }
  public static int endLookupMetaStationResponse(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

