import React, { useEffect, useRef } from "react";
import { TransformControls } from "@react-three/drei";

export function TargetGizmo({ position, onChange, onDragging }) {
  const meshRef = useRef();
  const controlRef = useRef();
  const pending = useRef(null);
  const draggingRef = useRef(false);

  useEffect(() => {
    if (controlRef.current && meshRef.current) {
      controlRef.current.setMode("translate");
      controlRef.current.attach(meshRef.current);
      controlRef.current.enabled = true;
    }
  }, []);

  useEffect(() => {
    if (meshRef.current) {
      meshRef.current.position.set(position.x, position.y, position.z);
    }
    if (controlRef.current) {
      controlRef.current.position.set(position.x, position.y, position.z);
    }
  }, [position]);

  const notifyChange = () => {
    if (!meshRef.current) return;
    if (pending.current) cancelAnimationFrame(pending.current);
    pending.current = requestAnimationFrame(() => {
      const { x, y, z } = meshRef.current.position;
      onChange?.({ x, y, z });
    });
  };

  useEffect(() => {
    const release = () => {
      if (draggingRef.current) {
        draggingRef.current = false;
        onDragging?.(false);
      }
    };
    window.addEventListener("pointerup", release);
    window.addEventListener("pointercancel", release);
    return () => {
      window.removeEventListener("pointerup", release);
      window.removeEventListener("pointercancel", release);
    };
  }, [onDragging]);

  return (
    <TransformControls
      ref={controlRef}
      showX
      showY
      showZ
      mode="translate"
      size={0.55}
      translationSnap={0.01}
      dragRotation={false}
      onPointerDown={() => {
        draggingRef.current = true;
        onDragging?.(true);
      }}
      onMouseDown={() => {
        draggingRef.current = true;
        onDragging?.(true);
      }}
      onMouseUp={() => {
        draggingRef.current = false;
        onDragging?.(false);
      }}
      onPointerUp={() => {
        draggingRef.current = false;
        onDragging?.(false);
      }}
      onPointerCancel={() => {
        draggingRef.current = false;
        onDragging?.(false);
      }}
      onPointerMissed={() => {
        draggingRef.current = false;
        onDragging?.(false);
      }}
      onMouseLeave={() => {
        draggingRef.current = false;
        onDragging?.(false);
      }}
      onObjectChange={notifyChange}
    >
      <mesh ref={meshRef}>
        <sphereGeometry args={[0.03, 24, 24]} />
        <meshStandardMaterial color="#6dd3ff" emissive="#0a4d78" />
      </mesh>
    </TransformControls>
  );
}
