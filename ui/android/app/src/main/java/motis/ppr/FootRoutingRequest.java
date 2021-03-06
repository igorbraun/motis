// automatically generated by the FlatBuffers compiler, do not modify

package motis.ppr;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class FootRoutingRequest extends Table {
  public static FootRoutingRequest getRootAsFootRoutingRequest(ByteBuffer _bb) { return getRootAsFootRoutingRequest(_bb, new FootRoutingRequest()); }
  public static FootRoutingRequest getRootAsFootRoutingRequest(ByteBuffer _bb, FootRoutingRequest obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public FootRoutingRequest __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public motis.Position start() { return start(new motis.Position()); }
  public motis.Position start(motis.Position obj) { int o = __offset(4); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  public motis.Position destinations(int j) { return destinations(new motis.Position(), j); }
  public motis.Position destinations(motis.Position obj, int j) { int o = __offset(6); return o != 0 ? obj.__init(__vector(o) + j * 16, bb) : null; }
  public int destinationsLength() { int o = __offset(6); return o != 0 ? __vector_len(o) : 0; }
  public motis.ppr.SearchOptions searchOptions() { return searchOptions(new motis.ppr.SearchOptions()); }
  public motis.ppr.SearchOptions searchOptions(motis.ppr.SearchOptions obj) { int o = __offset(8); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }
  public byte searchDirection() { int o = __offset(10); return o != 0 ? bb.get(o + bb_pos) : 0; }
  public boolean includeSteps() { int o = __offset(12); return o != 0 ? 0!=bb.get(o + bb_pos) : false; }
  public boolean includeEdges() { int o = __offset(14); return o != 0 ? 0!=bb.get(o + bb_pos) : false; }
  public boolean includePath() { int o = __offset(16); return o != 0 ? 0!=bb.get(o + bb_pos) : false; }

  public static void startFootRoutingRequest(FlatBufferBuilder builder) { builder.startObject(7); }
  public static void addStart(FlatBufferBuilder builder, int startOffset) { builder.addStruct(0, startOffset, 0); }
  public static void addDestinations(FlatBufferBuilder builder, int destinationsOffset) { builder.addOffset(1, destinationsOffset, 0); }
  public static void startDestinationsVector(FlatBufferBuilder builder, int numElems) { builder.startVector(16, numElems, 8); }
  public static void addSearchOptions(FlatBufferBuilder builder, int searchOptionsOffset) { builder.addOffset(2, searchOptionsOffset, 0); }
  public static void addSearchDirection(FlatBufferBuilder builder, byte searchDirection) { builder.addByte(3, searchDirection, 0); }
  public static void addIncludeSteps(FlatBufferBuilder builder, boolean includeSteps) { builder.addBoolean(4, includeSteps, false); }
  public static void addIncludeEdges(FlatBufferBuilder builder, boolean includeEdges) { builder.addBoolean(5, includeEdges, false); }
  public static void addIncludePath(FlatBufferBuilder builder, boolean includePath) { builder.addBoolean(6, includePath, false); }
  public static int endFootRoutingRequest(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

