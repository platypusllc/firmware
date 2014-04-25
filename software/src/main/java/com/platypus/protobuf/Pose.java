// Code generated by Wire protocol buffer compiler, do not edit.
// Source file: ././platypus.proto
package com.platypus.protobuf;

import com.squareup.wire.Message;
import com.squareup.wire.ProtoField;

import static com.squareup.wire.Message.Label.REQUIRED;

public final class Pose extends Message {

  @ProtoField(tag = 1, label = REQUIRED)
  public final Vector3 position;

  @ProtoField(tag = 2, label = REQUIRED)
  public final Vector3 rotation;

  private Pose(Builder builder) {
    super(builder);
    this.position = builder.position;
    this.rotation = builder.rotation;
  }

  @Override
  public boolean equals(Object other) {
    if (other == this) return true;
    if (!(other instanceof Pose)) return false;
    Pose o = (Pose) other;
    return equals(position, o.position)
        && equals(rotation, o.rotation);
  }

  @Override
  public int hashCode() {
    int result = hashCode;
    if (result == 0) {
      result = position != null ? position.hashCode() : 0;
      result = result * 37 + (rotation != null ? rotation.hashCode() : 0);
      hashCode = result;
    }
    return result;
  }

  public static final class Builder extends Message.Builder<Pose> {

    public Vector3 position;
    public Vector3 rotation;

    public Builder() {
    }

    public Builder(Pose message) {
      super(message);
      if (message == null) return;
      this.position = message.position;
      this.rotation = message.rotation;
    }

    public Builder position(Vector3 position) {
      this.position = position;
      return this;
    }

    public Builder rotation(Vector3 rotation) {
      this.rotation = rotation;
      return this;
    }

    @Override
    public Pose build() {
      checkRequiredFields();
      return new Pose(this);
    }
  }
}