// automatically generated by the FlatBuffers compiler, do not modify

package motis.intermodal;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class Car extends Table {
  public static Car getRootAsCar(ByteBuffer _bb) { return getRootAsCar(_bb, new Car()); }
  public static Car getRootAsCar(ByteBuffer _bb, Car obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public Car __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public int maxDuration() { int o = __offset(4); return o != 0 ? bb.getInt(o + bb_pos) : 0; }

  public static int createCar(FlatBufferBuilder builder,
      int max_duration) {
    builder.startObject(1);
    Car.addMaxDuration(builder, max_duration);
    return Car.endCar(builder);
  }

  public static void startCar(FlatBufferBuilder builder) { builder.startObject(1); }
  public static void addMaxDuration(FlatBufferBuilder builder, int maxDuration) { builder.addInt(0, maxDuration, 0); }
  public static int endCar(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

